// soem_bridge_node.cpp
// ROS2 节点：桥接 MoveIt 轨迹与 SOEM EtherCAT 电机控制。
//
// 数据流：
//   MoveIt → joint_trajectory_controller → controller_state
//       → 本节点 → SoemCsvMaster → EtherCAT 电机
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

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"


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
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "/joint_states");
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());

    // 控制器采样配置：从左右臂 controller_state 采样 reference.positions
    left_state_topic_ = declare_parameter<std::string>(
      "left_state_topic", "/left_arm_controller/controller_state");
    right_state_topic_ = declare_parameter<std::string>(
      "right_state_topic", "/right_arm_controller/controller_state");
    feedback_rate_hz_ = declare_parameter<double>("feedback_rate_hz", 20.0);

    // 轴标定参数（每轴独立配置：从站号/方向/零点/减速比/限位）
    load_axis_configs();

    // =====================================================================
    // 话题 & 服务
    // =====================================================================

    // 真实电机反馈（可通过 feedback_topic 参数配置话题名）
    // 设为 "/joint_states" 可让 RViz 直接显示真实编码器数据
    real_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      feedback_topic_, rclcpp::QoS(10));

    // 订阅左右臂 controller_state，直接处理并发送给电机
    left_state_sub_ = create_subscription<JTCState>(
      left_state_topic_, rclcpp::QoS(10),
      [this](JTCState::SharedPtr msg) { on_controller_state(msg); });
    right_state_sub_ = create_subscription<JTCState>(
      right_state_topic_, rclcpp::QoS(10),
      [this](JTCState::SharedPtr msg) { on_controller_state(msg); });

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

    // 电机反馈发布
    double frate = feedback_rate_hz_ > 0.1 ? feedback_rate_hz_ : 20.0;
    feedback_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / frate), [this] { publish_feedback(); });

    // =====================================================================
    // dry_run=false 时自动启动 EtherCAT（用于读取编码器）
    // =====================================================================
    if (!dry_run_) 
    {
      if (ifname_.empty()) {
        RCLCPP_ERROR(get_logger(), "dry_run=false but ifname is empty, cannot start EtherCAT");
      } else if (!master_->configure(ifname_, axis_configs_)) {
        RCLCPP_ERROR(get_logger(), "Failed to configure SOEM master");
      } else if (!master_->start()) {
        RCLCPP_ERROR(get_logger(), "Failed to start SOEM master");
      } else {
        RCLCPP_INFO(get_logger(), "EtherCAT started automatically (dry_run=false)");
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "SOEM bridge ready (CSV): dry_run=%s ifname='%s' feedback='%s' axes=%zu",
      dry_run_ ? "true" : "false", ifname_.c_str(),
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

    // 生成默认从站号 [1, 2, 3, ..., n]，即第 0 个轴对应从站 1
    std::vector<int64_t> default_slaves;
    for (size_t i = 0; i < n; i++) default_slaves.push_back(static_cast<int64_t>(i + 1));
    // axis_slaves: 每个轴对应的 EtherCAT 从站号，YAML 可覆盖，如 axis_slaves: [3, 5]
    auto slaves = declare_parameter<std::vector<int64_t>>("axis_slaves", default_slaves);
    // axis_directions: 旋转方向，1=正向，-1=反向，默认全 1
    auto directions = declare_parameter<std::vector<int64_t>>(
      "axis_directions", std::vector<int64_t>(n, 1));
    // axis_zero_offsets: 零位偏移（编码器计数），用于校准机械臂零位与编码器零位的偏差
    auto zero_offsets = declare_parameter<std::vector<int64_t>>(
      "axis_zero_offsets", std::vector<int64_t>(n, 0));

    // 遍历每个轴，组装完整的 AxisConfig
    axis_configs_.clear();
    for (size_t i = 0; i < n; i++) {
      AxisConfig cfg;
      cfg.joint_name = joint_names_[i];                    // 关节名，如 "laxis1"
      // 从站号：数组越界时用默认值 i+1
      cfg.slave = (i < slaves.size()) ? static_cast<uint16_t>(slaves[i])
                                      : static_cast<uint16_t>(i + 1);
      cfg.enc_bits = enc_bits;                             // 编码器位数，如 19
      cfg.gear_ratio = gear_ratios[i];                     // 减速比，如 100.0
      cfg.direction = (i < directions.size() && directions[i] < 0) ? -1 : 1;
      cfg.zero_offset_counts =
        (i < zero_offsets.size()) ? static_cast<int32_t>(zero_offsets[i]) : 0;
      // 软限位：rad → counts，确保 lo < hi
      int32_t lo = master_->rad_to_counts(cfg, pos_min_vec[i]);
      int32_t hi = master_->rad_to_counts(cfg, pos_max_vec[i]);
      if (lo > hi) std::swap(lo, hi);                      // 防止 min > max
      cfg.min_counts = lo;
      cfg.max_counts = hi;
      axis_configs_.push_back(cfg); // push_back 是在末尾追加一个元素
    }
  }

  // =====================================================================
  // 控制器状态处理
  // 订阅左右臂 controller_state，直接将 reference.positions 转成电机指令。
  // dry_run 模式下只打印换算结果，不发送给电机。
  // =====================================================================
  void on_controller_state(const JTCState::SharedPtr msg)
  {
    // 构造 CsvWaypoint
    std::vector<CsvWaypoint> waypoints;
    CsvWaypoint waypoint;
    waypoint.joint_names = msg->joint_names;
    waypoint.positions = msg->reference.positions;
    // controller_state 通常没有速度信息，填 0（CSV 模式速度为 0 时电机保持位置）
    waypoint.velocities.resize(msg->reference.positions.size(), 0.0);
    waypoints.push_back(std::move(waypoint));

    // dry-run：打印首轴换算结果，便于核对标定参数
    if (dry_run_) {
      int32_t c0 = 0;
      if (!msg->joint_names.empty() && !axis_configs_.empty() &&
        !msg->reference.positions.empty())
      {
        c0 = master_->rad_to_counts(axis_configs_[0], msg->reference.positions[0]);
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "dry-run: joints=%zu axis0_pos=%d",
        msg->joint_names.size(), c0);
      return;
    }

    // 真实模式：必须先调用 enable 服务允许发送
    if (!send_enabled_.load()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "trajectory ignored: send not enabled (call ~/enable)");
      return;
    }

    if (!master_->submit_waypoints(waypoints)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "failed to submit CSV waypoints");
    }
  }

  // =====================================================================
  // 电机反馈发布
  // 将 SoemCsvMaster 的反馈（位置/速度/力矩）发布到 feedback_topic。
  // dry_run=false 时自动启动 EtherCAT，无需 enable 即可读取编码器。
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
  // 服务处理
  // =====================================================================

  // enable(true)  → 允许发送速度指令
  // enable(false) → 禁止发送速度指令
  void handle_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (dry_run_) {
      response->success = true;
      response->message = "dry-run: enable service has no effect";
      return;
    }

    if (!master_->enabled()) {
      response->success = false;
      response->message = "EtherCAT not started";
      return;
    }

    send_enabled_.store(request->data);
    response->success = true;
    response->message = request->data ? "send enabled" : "send disabled";
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    send_enabled_.store(false);
    response->success = true;
    response->message = "send disabled (stop)";
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
  std::atomic<bool> send_enabled_{false};  // 是否允许发送速度指令（enable 服务控制）

  // 配置
  std::string feedback_topic_;
  std::vector<std::string> joint_names_;
  std::vector<AxisConfig> axis_configs_;

  // 控制器采样配置
  std::string left_state_topic_;
  std::string right_state_topic_;
  double feedback_rate_hz_{20.0};

  // ROS 话题/服务/定时器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr real_joint_state_pub_;
  rclcpp::Subscription<JTCState>::SharedPtr left_state_sub_;
  rclcpp::Subscription<JTCState>::SharedPtr right_state_sub_;
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
