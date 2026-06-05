// soem_bridge_node.cpp
// ROS2 节点：桥接 MoveIt 轨迹与 SOEM EtherCAT 电机控制。
//
// 数据流：
//   MoveIt → joint_trajectory_controller → controller_state (采样)
//       → waypoint_topic → 本节点 → SoemCsvMaster → EtherCAT 电机
//
// 服务：
//   ~/enable       使能/关闭电机控制
//   ~/stop         紧急停止
//   ~/clear_fault  故障复位(RT 线程自动处理，此服务仅回执)

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
    // =====================================================================
    // 参数声明（从 yaml 注入）
    // =====================================================================

    // 基础配置
    ifname_ = declare_parameter<std::string>("ifname", "");
    dry_run_ = declare_parameter<bool>("dry_run", true);
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/dual_arm/csv_waypoints");
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "~/real_joint_states");
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());

    // 控制器采样配置：从左右臂 controller_state 话题采样 reference.positions 生成 waypoint
    sample_from_controllers_ = declare_parameter<bool>("sample_from_controllers", true);
    left_state_topic_ = declare_parameter<std::string>(
      "left_state_topic", "/left_arm_controller/controller_state");
    right_state_topic_ = declare_parameter<std::string>(
      "right_state_topic", "/right_arm_controller/controller_state");
    sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 10.0);
    feedback_rate_hz_ = declare_parameter<double>("feedback_rate_hz", 20.0);

    // 轴标定参数（每轴独立配置：从站号/方向/零点/减速比/限位）
    load_axis_configs();

    // =====================================================================
    // 话题 & 服务
    // =====================================================================

    // waypoint 订阅 & 发布（外部可直接发布 waypoint，也可由采样器自动生成）
    waypoint_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      waypoint_topic_, rclcpp::QoS(10),
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { on_waypoints(msg); });
    waypoint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      waypoint_topic_, rclcpp::QoS(10));

    // 真实电机反馈（可通过 feedback_topic 参数配置话题名）
    // 设为 "/joint_states" 可让 RViz 直接显示真实编码器数据
    real_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      feedback_topic_, rclcpp::QoS(10));

    // 使能/停止/故障复位服务
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        handle_enable(req, res);
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        handle_stop(req, res);
      });
    clear_fault_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_fault",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        handle_clear_fault(req, res);
      });

    // =====================================================================
    // 定时器
    // =====================================================================

    // 控制器状态采样：定时从 ref_cache_ 组装 waypoint 并发布
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

    // 电机反馈发布
    double frate = feedback_rate_hz_ > 0.1 ? feedback_rate_hz_ : 20.0;
    feedback_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / frate), [this] { publish_feedback(); });

    RCLCPP_INFO(
      get_logger(),
      "SOEM bridge ready (CSV): dry_run=%s ifname='%s' waypoint='%s' feedback='%s' axes=%zu",
      dry_run_ ? "true" : "false", ifname_.c_str(), waypoint_topic_.c_str(),
      feedback_topic_.c_str(), axis_configs_.size());
  }

private:
  // =====================================================================
  // 默认关节名（左臂 7 轴 + 右臂 7 轴）
  // =====================================================================
  static std::vector<std::string> default_joint_names()
  {
    return {
      "laxis1_joint", "laxis2_joint", "laxis3_joint", "laxis4_joint",
      "laxis5_joint", "laxis6_joint", "laxis7_joint",
      "raxis1_joint", "raxis2_joint", "raxis3_joint", "raxis4_joint",
      "raxis5_joint", "raxis6_joint", "raxis7_joint"};
  }

  // =====================================================================
  // 轴标定参数加载
  // 从 yaml 读取每轴配置，构造 AxisConfig 数组。
  // 数组参数长度不足时用默认值填充。
  // =====================================================================
  void load_axis_configs()
  {
    const size_t n = joint_names_.size();
    int enc_bits = static_cast<int>(declare_parameter<int>("enc_bits", 19));

    // lambda：读取 double 数组参数，不足 n 个时用 default_val 补全
    auto read_double_array = [this, n](const std::string & name, double default_val) {
      auto vec = declare_parameter<std::vector<double>>(name, std::vector<double>(n, default_val));
      if (vec.size() < n) vec.resize(n, default_val);
      return vec;
    };

    auto gear_ratios = read_double_array("gear_ratio", 100.0);
    auto pos_min_vec = read_double_array("pos_limit_min_rad", -3.2);
    auto pos_max_vec = read_double_array("pos_limit_max_rad", 3.2);

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
      cfg.gear_ratio = gear_ratios[i];
      cfg.direction = (i < directions.size() && directions[i] < 0) ? -1 : 1;
      cfg.zero_offset_counts =
        (i < zero_offsets.size()) ? static_cast<int32_t>(zero_offsets[i]) : 0;
      // 软限位：rad → counts，确保 lo < hi
      int32_t lo = master_->rad_to_counts(cfg, pos_min_vec[i]);
      int32_t hi = master_->rad_to_counts(cfg, pos_max_vec[i]);
      if (lo > hi) std::swap(lo, hi);
      cfg.min_counts = lo;
      cfg.max_counts = hi;
      axis_configs_.push_back(cfg);
    }
  }

  // =====================================================================
  // 控制器状态采样
  // 订阅左右臂 controller_state，缓存 reference.positions 到 ref_cache_。
  // sample_and_publish() 定时从缓存组装 waypoint 发布。
  // =====================================================================
  void on_controller_state(const JTCState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(cache_mtx_);
    for (size_t i = 0; i < msg->joint_names.size() && i < msg->reference.positions.size(); i++) {
      ref_cache_[msg->joint_names[i]] = msg->reference.positions[i];
    }
  }

  void sample_and_publish()
  {
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    {
      std::lock_guard<std::mutex> lk(cache_mtx_);
      if (ref_cache_.empty()) return;
      for (const auto & jn : joint_names_) {
        auto it = ref_cache_.find(jn);
        if (it == ref_cache_.end()) return;  // 某关节未收到状态，跳过本周期
        traj.joint_names.push_back(jn);
        pt.positions.push_back(it->second);
      }
    }
    pt.time_from_start = rclcpp::Duration(0, 0);
    traj.points.push_back(std::move(pt));
    waypoint_pub_->publish(traj);
  }

  // =====================================================================
  // 电机反馈发布
  // 将 SoemCsvMaster 的反馈（位置/速度/力矩）发布到 ~/real_joint_states。
  // =====================================================================
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

  // =====================================================================
  // waypoint 接收
  // 将 ROS JointTrajectory 转成内部 CsvWaypoint，提交给 SoemCsvMaster。
  // dry_run 模式下只打印换算结果，不发送给电机。
  // =====================================================================
  void on_waypoints(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    std::vector<CsvWaypoint> waypoints;
    waypoints.reserve(msg->points.size());

    for (const auto & point : msg->points) {
      CsvWaypoint waypoint;
      waypoint.joint_names = msg->joint_names;
      waypoint.positions = point.positions;
      // 有速度信息则使用，否则填 0（CSV 模式速度为 0 时电机保持位置）
      waypoint.velocities = point.velocities.empty()
        ? std::vector<double>(point.positions.size(), 0.0)
        : point.velocities;
      waypoint.time_from_start = rclcpp::Duration(point.time_from_start).seconds();
      waypoints.push_back(std::move(waypoint));
    }

    // dry-run：打印首轴换算结果，便于核对标定参数
    if (dry_run_) {
      int32_t c0 = 0, v0 = 0;
      if (!msg->joint_names.empty() && !axis_configs_.empty() &&
        !msg->points.empty() && !msg->points.back().positions.empty())
      {
        c0 = master_->rad_to_counts(axis_configs_[0], msg->points.back().positions[0]);
        if (!msg->points.back().velocities.empty()) {
          v0 = master_->vel_to_counts(axis_configs_[0], msg->points.back().velocities[0]);
        }
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "dry-run: joints=%zu points=%zu axis0_pos=%d axis0_vel=%d",
        msg->joint_names.size(), msg->points.size(), c0, v0);
      return;
    }

    // 真实模式：必须先调用 enable 服务
    if (!master_->enabled()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "trajectory ignored: SOEM master not enabled");
      return;
    }

    if (!master_->submit_waypoints(waypoints)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "failed to submit CSV waypoints");
    }
  }

  // =====================================================================
  // 服务处理
  // =====================================================================

  // enable(true)  → 配置并启动 EtherCAT 主站
  // enable(false) → 停止主站
  void handle_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!request->data) {
      master_->stop();
      response->success = true;
      response->message = "SOEM bridge disabled";
      return;
    }

    if (dry_run_) {
      response->success = true;
      response->message = "dry-run enabled without opening EtherCAT";
      return;
    }

    if (!master_->configure(ifname_, axis_configs_)) {
      response->success = false;
      response->message = "configure failed";
      return;
    }

    response->success = master_->start();
    response->message = response->success ? "SOEM bridge enabled" : "failed to enable SOEM bridge";
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    master_->stop();
    response->success = true;
    response->message = "SOEM bridge stopped";
  }

  // CiA402 fault 由 RT 线程自动复位，此服务仅回执。
  void handle_clear_fault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = true;
    response->message = dry_run_ ? "dry-run clear_fault accepted"
                                 : "fault reset handled by RT state machine";
  }

  // =====================================================================
  // 成员变量
  // =====================================================================

  // EtherCAT 主站
  std::unique_ptr<SoemCsvMaster> master_;
  std::string ifname_;
  bool dry_run_{true};

  // waypoint 配置
  std::string waypoint_topic_;
  std::string feedback_topic_;
  std::vector<std::string> joint_names_;
  std::vector<AxisConfig> axis_configs_;

  // 控制器采样配置
  bool sample_from_controllers_{true};
  std::string left_state_topic_;
  std::string right_state_topic_;
  double sample_rate_hz_{10.0};
  double feedback_rate_hz_{20.0};
  std::mutex cache_mtx_;
  std::map<std::string, double> ref_cache_;  // joint_name → reference.position

  // ROS 话题/服务/定时器
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
