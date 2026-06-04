#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "dual_arm_soem_bridge/soem_master.hpp"

namespace dual_arm_soem_bridge
{

using JTCState = control_msgs::msg::JointTrajectoryControllerState;

class SoemBridgeNode : public rclcpp::Node
{
public:
  SoemBridgeNode()
  : Node("soem_bridge_node"), master_(std::make_unique<SoemCsvMaster>())
  {
    // 参数由 launch/yaml 注入；默认 dry_run=true，避免误连真实硬件。
    ifname_ = declare_parameter<std::string>("ifname", ""); // declare_parameter 是模板函数，<> 用来指定返回值类型
    dry_run_ = declare_parameter<bool>("dry_run", true);
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/dual_arm/csv_waypoints");
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());

    // 第 3 步 waypoint 生成：从控制器 controller_state 低频采样 reference.positions。
    sample_from_controllers_ = declare_parameter<bool>("sample_from_controllers", true);
    left_state_topic_ = declare_parameter<std::string>(
      "left_state_topic", "/left_arm_controller/controller_state");
    right_state_topic_ = declare_parameter<std::string>(
      "right_state_topic", "/right_arm_controller/controller_state");
    sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 10.0);
    feedback_rate_hz_ = declare_parameter<double>("feedback_rate_hz", 20.0);

    // 第 4 步轴标定参数(编码器位数/减速比共享标量，从站/方向/零点用数组)。
    load_axis_configs();

    // 订阅低频 CSV waypoint(也可由外部直接发布)，并提供发布器供采样器使用。
    waypoint_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      waypoint_topic_, rclcpp::QoS(10),
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { on_waypoints(msg); });
    waypoint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      waypoint_topic_, rclcpp::QoS(10));

    // 真实机械臂反馈单独发布，不覆盖仿真的 /joint_states。
    real_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/real_joint_states", rclcpp::QoS(10));

    // 显式使能服务，真实硬件不随节点启动自动运动。
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        handle_enable(request, response);
      });

    // 停止服务，后续会扩展为清空队列和安全停机。
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_stop(request, response);
      });

    // 故障复位服务，后续映射到 CiA402 fault reset。
    clear_fault_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_fault",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_clear_fault(request, response);
      });

    // 第 3 步：订阅左右控制器状态并定时采样发布 waypoint。
    if (sample_from_controllers_) {
      left_state_sub_ = create_subscription<JTCState>(
        left_state_topic_, rclcpp::QoS(10),
        [this](JTCState::SharedPtr msg) { on_controller_state(msg); });
      right_state_sub_ = create_subscription<JTCState>(
        right_state_topic_, rclcpp::QoS(10),
        [this](JTCState::SharedPtr msg) { on_controller_state(msg); });
      double rate = sample_rate_hz_ > 0.1 ? sample_rate_hz_ : 10.0;
      sample_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate), [this] { sample_and_publish(); });
    }

    // 真实状态发布定时器。
    double frate = feedback_rate_hz_ > 0.1 ? feedback_rate_hz_ : 20.0;
    feedback_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / frate), [this] { publish_feedback(); });

    RCLCPP_INFO(
      get_logger(),
      "SOEM bridge ready (CSV mode): dry_run=%s ifname='%s' topic='%s' axes=%zu sample=%s@%.1fHz ctx=%zu",
      dry_run_ ? "true" : "false", ifname_.c_str(), waypoint_topic_.c_str(),
      axis_configs_.size(), sample_from_controllers_ ? "on" : "off", sample_rate_hz_,
      master_->soem_context_size());
  }

private:
  static std::vector<std::string> default_joint_names()
  {
    // 默认顺序：左臂 7 轴 + 右臂 7 轴。
    return {
      "laxis1_joint", "laxis2_joint", "laxis3_joint", "laxis4_joint", "laxis5_joint",
      "laxis6_joint", "laxis7_joint", "raxis1_joint", "raxis2_joint", "raxis3_joint",
      "raxis4_joint", "raxis5_joint", "raxis6_joint", "raxis7_joint"};
  }

  // 从参数构造每轴标定配置。数组参数若长度不足则用默认值填充。
  void load_axis_configs()
  {
    const size_t n = joint_names_.size();
    int enc_bits = static_cast<int>(declare_parameter<int>("enc_bits", 19));
    double gear_ratio = declare_parameter<double>("gear_ratio", 100.0);
    double pos_min_rad = declare_parameter<double>("pos_limit_min_rad", -3.2);
    double pos_max_rad = declare_parameter<double>("pos_limit_max_rad", 3.2);

    std::vector<int64_t> default_slaves;
    for (size_t i = 0; i < n; i++) default_slaves.push_back(static_cast<int64_t>(i + 1));
    auto slaves = declare_parameter<std::vector<int64_t>>("axis_slaves", default_slaves);
    auto directions = declare_parameter<std::vector<int64_t>>(
      "axis_directions", std::vector<int64_t>(n, 1));
    auto zero_offsets = declare_parameter<std::vector<int64_t>>(
      "axis_zero_offsets", std::vector<int64_t>(n, 0));

    axis_configs_.clear();
    for (size_t i = 0; i < n; i++) {
      AxisConfig cfg;
      cfg.joint_name = joint_names_[i];
      cfg.slave = (i < slaves.size()) ? static_cast<uint16_t>(slaves[i])
                                      : static_cast<uint16_t>(i + 1);
      cfg.enc_bits = enc_bits;
      cfg.gear_ratio = gear_ratio;
      cfg.direction = (i < directions.size() && directions[i] < 0) ? -1 : 1;
      cfg.zero_offset_counts =
        (i < zero_offsets.size()) ? static_cast<int32_t>(zero_offsets[i]) : 0;
      // 此时 cfg.min/max 仍为默认极大值，rad_to_counts 不会裁剪，得到纯换算限位。
      int32_t lo = master_->rad_to_counts(cfg, pos_min_rad);
      int32_t hi = master_->rad_to_counts(cfg, pos_max_rad);
      if (lo > hi) std::swap(lo, hi);
      cfg.min_counts = lo;
      cfg.max_counts = hi;
      axis_configs_.push_back(cfg);
    }
  }

  // 缓存控制器期望位置(reference.positions)，按关节名存。
  void on_controller_state(const JTCState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cache_mtx_);
    for (size_t i = 0; i < msg->joint_names.size() && i < msg->reference.positions.size(); i++) {
      ref_cache_[msg->joint_names[i]] = msg->reference.positions[i];
    }
  }

  // 定时按统一关节顺序组装单点 waypoint 并发布到 waypoint_topic。
  void sample_and_publish()
  {
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    {
      std::lock_guard<std::mutex> lk(cache_mtx_);
      if (ref_cache_.empty()) return;  // 还没收到控制器状态
      for (const auto & jn : joint_names_) {
        auto it = ref_cache_.find(jn);
        if (it == ref_cache_.end()) return;  // 关节未集齐，跳过本次
        traj.joint_names.push_back(jn);
        pt.positions.push_back(it->second);
      }
    }
    pt.time_from_start = rclcpp::Duration(0, 0);
    traj.points.push_back(std::move(pt));
    waypoint_pub_->publish(traj);
  }

  // 发布真实电机反馈到 ~/real_joint_states。
  void publish_feedback()
  {
    if (dry_run_ || !master_->enabled()) return;
    auto fb = master_->feedback();
    if (fb.empty()) return;
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    for (const auto & f : fb) {
      js.name.push_back(f.joint_name);
      js.position.push_back(f.position_rad);
      js.velocity.push_back(f.velocity_rad_s);
      js.effort.push_back(static_cast<double>(f.torque));
    }
    real_joint_state_pub_->publish(js);
  }

  void on_waypoints(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    // 将 ROS JointTrajectory 转成内部 CSV waypoint 结构。
    std::vector<CsvWaypoint> waypoints;
    waypoints.reserve(msg->points.size());

    for (const auto & point : msg->points) {
      CsvWaypoint waypoint;
      waypoint.joint_names = msg->joint_names;
      waypoint.positions = point.positions;
      // 如果有速度信息则使用，否则默认为 0
      if (!point.velocities.empty()) {
        waypoint.velocities = point.velocities;
      } else {
        waypoint.velocities.resize(point.positions.size(), 0.0);
      }
      waypoint.time_from_start = rclcpp::Duration(point.time_from_start).seconds();
      waypoints.push_back(std::move(waypoint));
    }

    if (dry_run_) {
      // dry-run：打印收到的目标和首关节 rad->counts 换算，便于核对标定。
      int32_t c0 = 0;
      int32_t v0 = 0;
      if (!msg->joint_names.empty() && !axis_configs_.empty() && !msg->points.empty() &&
        !msg->points.back().positions.empty())
      {
        c0 = master_->rad_to_counts(axis_configs_[0], msg->points.back().positions[0]);
        if (!msg->points.back().velocities.empty()) {
          v0 = master_->vel_to_counts(axis_configs_[0], msg->points.back().velocities[0]);
        }
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "dry-run waypoint: joints=%zu points=%zu axis0_pos_counts=%d axis0_vel_counts=%d",
        msg->joint_names.size(), msg->points.size(), c0, v0);
      return;
    }

    // 非 dry-run 时必须先调用 enable 服务。
    if (!master_->enabled()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "trajectory ignored: SOEM master not enabled");
      return;
    }

    if (!master_->submit_waypoints(waypoints)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "failed to submit SOEM CSV waypoints");
    }
  }

  void handle_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // 请求 false 表示关闭桥接。
    if (!request->data) {
      master_->stop();
      response->success = true;
      response->message = "SOEM bridge disabled";
      return;
    }

    // dry-run 模式下不打开网卡，只验证 ROS 流程。
    if (dry_run_) {
      response->success = true;
      response->message = "dry-run enabled without opening EtherCAT";
      return;
    }

    // 真实模式：下发网卡名与轴标定后启动。
    if (!master_->configure(ifname_, axis_configs_)) {
      response->success = false;
      response->message = "configure failed";
      return;
    }

    response->success = master_->start();
    response->message = response->success ? "SOEM bridge enabled" : "failed to enable SOEM bridge";
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    // 当前骨架只停止主站标志。
    master_->stop();
    response->success = true;
    response->message = "SOEM bridge stopped";
  }

  void handle_clear_fault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    // CiA402 fault 在 RT 状态机里自动复位，这里仅作占位回执。
    response->success = true;
    response->message = dry_run_ ? "dry-run clear_fault accepted"
                                 : "fault reset handled by RT state machine";
  }

  std::unique_ptr<SoemCsvMaster> master_;
  std::string ifname_;
  std::string waypoint_topic_;
  std::vector<std::string> joint_names_; // 一个vector动态数组，元素类型是字符串
  std::vector<AxisConfig> axis_configs_;
  bool dry_run_{true};

  bool sample_from_controllers_{true};
  std::string left_state_topic_;
  std::string right_state_topic_;
  double sample_rate_hz_{10.0};
  double feedback_rate_hz_{20.0};

  std::mutex cache_mtx_;
  std::map<std::string, double> ref_cache_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr waypoint_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr real_joint_state_pub_;
  rclcpp::Subscription<JTCState>::SharedPtr left_state_sub_;
  rclcpp::Subscription<JTCState>::SharedPtr right_state_sub_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
};

}  // namespace dual_arm_soem_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dual_arm_soem_bridge::SoemBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
