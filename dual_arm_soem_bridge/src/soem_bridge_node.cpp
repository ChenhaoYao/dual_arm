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
//
// 测试话题：
//   ~/test_axis    单电机测试 (std_msgs/Float64MultiArray: [joint_index, velocity_rad_s])

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
  ~SoemBridgeNode()
  {
    if (trajectory_file_.is_open()) {
      trajectory_file_.close();
      RCLCPP_INFO(get_logger(), "Trajectory log file closed");
    }
  }

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

    // 控制器采样配置：同时订阅左右臂 controller_state
    left_state_topic_ = declare_parameter<std::string>(
      "left_state_topic", "/left_arm_controller/controller_state");
    right_state_topic_ = declare_parameter<std::string>(
      "right_state_topic", "/right_arm_controller/controller_state");
    feedback_rate_hz_ = declare_parameter<double>("feedback_rate_hz", 20.0);

    // 速度限制（正反转都限制）
    max_velocity_ = declare_parameter<double>("max_velocity", 1.0);

    // 逐个使能间隔时间(ms)
    axis_enable_interval_ms_ = declare_parameter<int>("axis_enable_interval_ms", 100);

    // 轨迹日志保存间隔(ms)
    log_interval_ms_ = declare_parameter<int>("log_interval_ms", 100);

    // 轴标定参数（每轴独立配置：从站号/方向/零点/减速比/限位）
    load_axis_configs();

    // =====================================================================
    // 话题 & 服务
    // =====================================================================

    // 真实电机反馈（可通过 feedback_topic 参数配置话题名）
    // 设为 "/joint_states" 可让 RViz 直接显示真实编码器数据
    real_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      feedback_topic_, rclcpp::QoS(10).best_effort());

    // 同时订阅左右臂 controller_state，双臂独立控制
    left_state_sub_ = create_subscription<JTCState>(
      left_state_topic_, rclcpp::QoS(10),
      [this](JTCState::SharedPtr msg) { on_controller_state(msg); });
    RCLCPP_INFO(get_logger(), "Subscribed to LEFT arm: %s", left_state_topic_.c_str());

    right_state_sub_ = create_subscription<JTCState>(
      right_state_topic_, rclcpp::QoS(10),
      [this](JTCState::SharedPtr msg) { on_controller_state(msg); });
    RCLCPP_INFO(get_logger(), "Subscribed to RIGHT arm: %s", right_state_topic_.c_str());

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

    // 单电机测试话题：~/test_axis (std_msgs/Float64MultiArray: [joint_index, velocity_rad_s])
    test_axis_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/test_axis", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        on_test_axis(msg);
      });

    // =====================================================================
    // 定时器
    // =====================================================================

    // 电机反馈发布
    double frate = feedback_rate_hz_ > 0.1 ? feedback_rate_hz_ : 20.0;
    feedback_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / frate), [this] { publish_feedback(); });

    // =====================================================================
    // 启动 EtherCAT（用于读取编码器，无论 dry_run 与否）
    // dry_run=true 时只读编码器，不发送速度命令
    // =====================================================================
    if (ifname_.empty()) {
      RCLCPP_WARN(get_logger(), "ifname is empty, EtherCAT not started");
    } else if (!master_->configure(ifname_, axis_configs_)) {
      RCLCPP_ERROR(get_logger(), "Failed to configure SOEM master");
    } else {
      // 设置逐个使能间隔时间
      master_->set_enable_delay_ms(axis_enable_interval_ms_);
      if (!master_->start()) {
        RCLCPP_ERROR(get_logger(), "Failed to start SOEM master");
      } else {
        RCLCPP_INFO(get_logger(), "EtherCAT started (dry_run=%s, axis_enable_interval=%dms)", 
                    dry_run_ ? "true" : "false", axis_enable_interval_ms_);
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
    // axis_zero_offsets: 零位偏移（角度），用于校准机械臂零位与编码器零位的偏差
    auto zero_offsets = read_double_array("axis_zero_offsets", 0.0);

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
      // 将角度转换为 counts: counts = round(deg * π/180 * scale)
      // 其中 scale = 2^enc_bits * gear_ratio / (2π)
      double scale = (double)((int64_t)1 << enc_bits) * gear_ratios[i] / (2.0 * M_PI);
      double offset_deg = (i < zero_offsets.size()) ? zero_offsets[i] : 0.0;
      cfg.zero_offset_counts = static_cast<int32_t>(
        std::llround(offset_deg * M_PI / 180.0 * scale));
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
  // dry_run 模式下保存轨迹到文件，不发送给电机。
  // =====================================================================
  void on_controller_state(const JTCState::SharedPtr msg)
  {
    // 构造 CsvWaypoint
    std::vector<CsvWaypoint> waypoints;
    CsvWaypoint waypoint;
    waypoint.joint_names = msg->joint_names;

    // velocity 模式：使用 output.velocities (PID 输出的速度命令)
    // 保存原始速度用于日志
    std::vector<double> raw_velocities;
    if (!msg->output.velocities.empty()) {
      raw_velocities = msg->output.velocities;
      waypoint.velocities = msg->output.velocities;
      // 速度限制（正反转都限制）
      for (auto& vel : waypoint.velocities) {
        vel = std::clamp(vel, -max_velocity_, max_velocity_);
      }
    } else {
      // fallback: 如果没有 output.velocities，填 0
      waypoint.velocities.resize(msg->joint_names.size(), 0.0);
      raw_velocities = waypoint.velocities;
    }

    // positions 用于反馈显示（非必需，但保持结构完整）
    if (!msg->reference.positions.empty()) {
      waypoint.positions = msg->reference.positions;
    }

    // 在 move 之前保存限速后的速度，用于日志记录
    std::vector<double> limited_velocities = waypoint.velocities;

    waypoints.push_back(std::move(waypoint));

    // 保存轨迹到文件（enable 后才保存，限频10Hz）
    if (send_enabled_.load() || dry_run_) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time_).count();
      if (elapsed >= log_interval_ms_) {
        save_trajectory_to_file(msg, raw_velocities, limited_velocities);
        last_log_time_ = now;
      }
    }

    // dry-run：不发送给电机，直接返回
    if (dry_run_) {
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
  // 保存轨迹到文件（双臂 14 个关节）
  // 每次启动生成带时间戳的文件，避免覆盖
  // 使用CSV格式，逗号分隔，固定宽度对齐
  // =====================================================================
  void save_trajectory_to_file(const JTCState::SharedPtr & msg, 
                                const std::vector<double> & raw_velocities,
                                const std::vector<double> & limited_velocities)
  {
    // 判断是左臂还是右臂（根据关节名前缀）
    bool is_left = !msg->joint_names.empty() && msg->joint_names[0].find("laxis") == 0;
    std::string prefix = is_left ? "laxis" : "raxis";

    // 首次调用时打开文件
    if (!trajectory_file_.is_open()) {
      // 生成带时间戳的文件名
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch()) % 1000;
      
      std::stringstream ss;
      ss << "/home/dell/dual_arm/dual_arm_soem_bridge/log/trajectory_"
         << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
         << "." << std::setfill('0') << std::setw(3) << ms.count()
         << ".csv";
      std::string filename = ss.str();
      
      // 确保log目录存在
      std::filesystem::create_directories("/home/dell/dual_arm/dual_arm_soem_bridge/log");
      
      trajectory_file_.open(filename, std::ios::out | std::ios::trunc);
      if (!trajectory_file_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open trajectory log file: %s", filename.c_str());
        return;
      }
      
      // 写入表头（双臂 14 个关节，每个关节 4 列）
      trajectory_file_ << std::left 
                       << std::setw(20) << "timestamp_ns";
      // 左臂
      for (int i = 1; i <= 7; i++) {
        trajectory_file_ << "," << std::setw(12) << ("laxis" + std::to_string(i) + "_ref")
                         << "," << std::setw(12) << ("laxis" + std::to_string(i) + "_pid")
                         << "," << std::setw(12) << ("laxis" + std::to_string(i) + "_out")
                         << "," << std::setw(12) << ("laxis" + std::to_string(i) + "_fb");
      }
      // 右臂
      for (int i = 1; i <= 7; i++) {
        trajectory_file_ << "," << std::setw(12) << ("raxis" + std::to_string(i) + "_ref")
                         << "," << std::setw(12) << ("raxis" + std::to_string(i) + "_pid")
                         << "," << std::setw(12) << ("raxis" + std::to_string(i) + "_out")
                         << "," << std::setw(12) << ("raxis" + std::to_string(i) + "_fb");
      }
      trajectory_file_ << "\n";
      
      RCLCPP_INFO(get_logger(), "Trajectory logging to: %s", filename.c_str());
    }

    // 获取真实电机位置反馈
    std::vector<double> fb_positions(14, 0.0);
    if (master_->enabled()) {
      auto fb = master_->feedback();
      for (size_t i = 0; i < fb.size() && i < fb_positions.size(); i++) {
        fb_positions[i] = fb[i].position_rad;
      }
    }

    // 根据左右臂确定偏移量（左臂 0-6，右臂 7-13）
    size_t offset = is_left ? 0 : 7;

    // 写入数据行
    trajectory_file_ << std::left << std::setw(20) 
                     << (msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec);
    
    // 写入所有 14 个关节的数据（当前臂写实际值，另一臂填 0）
    for (size_t i = 0; i < 14; i++) {
      double ref_pos = 0.0, pid_vel = 0.0, out_vel = 0.0, fb_pos = 0.0;
      if (i >= offset && i < offset + msg->joint_names.size()) {
        size_t j = i - offset;
        ref_pos = (j < msg->reference.positions.size()) ? msg->reference.positions[j] : 0.0;
        pid_vel = (j < raw_velocities.size()) ? raw_velocities[j] : 0.0;
        out_vel = (j < limited_velocities.size()) ? limited_velocities[j] : 0.0;
      }
      fb_pos = (i < fb_positions.size()) ? fb_positions[i] : 0.0;
      trajectory_file_ << std::fixed << std::setprecision(4)
                       << "," << std::setw(12) << ref_pos
                       << "," << std::setw(12) << pid_vel
                       << "," << std::setw(12) << out_vel
                       << "," << std::setw(12) << fb_pos;
    }
    trajectory_file_ << "\n";
    trajectory_file_.flush();
  }

  // =====================================================================
  // 电机反馈发布
  // 将 SoemCsvMaster 的反馈（位置/速度/力矩）发布到 feedback_topic。
  // dry_run=false 时自动启动 EtherCAT，无需 enable 即可读取编码器。
  // =====================================================================
  void publish_feedback()
  {
    if (!master_->enabled()) return;
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

    // disable 时关闭轨迹日志文件
    if (!request->data && trajectory_file_.is_open()) {
      trajectory_file_.close();
      RCLCPP_INFO(get_logger(), "Trajectory log file closed");
    }
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    send_enabled_.store(false);
    response->success = true;
    response->message = "send disabled (stop)";

    // 关闭轨迹日志文件
    if (trajectory_file_.is_open()) {
      trajectory_file_.close();
      RCLCPP_INFO(get_logger(), "Trajectory log file closed");
    }
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

  // 单电机测试：~/test_axis (std_msgs/Float64MultiArray: [joint_index, velocity_rad_s])
  void on_test_axis(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(get_logger(), "test_axis: need [joint_index, velocity_rad_s], got %zu values",
                  msg->data.size());
      return;
    }

    size_t idx = static_cast<size_t>(msg->data[0]);
    double vel_rad_s = msg->data[1];

    if (idx >= axis_configs_.size()) {
      RCLCPP_WARN(get_logger(), "test_axis: index %zu out of range (max %zu)",
                  idx, axis_configs_.size() - 1);
      return;
    }

    if (!send_enabled_.load()) {
      RCLCPP_WARN(get_logger(), "test_axis: send not enabled (call ~/enable first)");
      return;
    }

    // 构造单轴 waypoint
    std::vector<CsvWaypoint> waypoints;
    CsvWaypoint wp;
    wp.joint_names.push_back(axis_configs_[idx].joint_name);
    wp.velocities.push_back(vel_rad_s);
    waypoints.push_back(std::move(wp));

    if (master_->submit_waypoints(waypoints)) {
      RCLCPP_INFO(get_logger(), "test_axis: %s vel=%.3f rad/s",
                  axis_configs_[idx].joint_name.c_str(), vel_rad_s);
    } else {
      RCLCPP_WARN(get_logger(), "test_axis: failed to submit");
    }
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

  // 速度限制
  double max_velocity_{1.0};  // rad/s

  // 逐个使能间隔时间(ms)
  int axis_enable_interval_ms_{100};

  // dry_run 模式：轨迹日志文件
  std::ofstream trajectory_file_;
  std::chrono::steady_clock::time_point last_log_time_;
  int log_interval_ms_{100};  // 日志保存间隔，默认100ms (10Hz)

  // ROS 话题/服务/定时器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr real_joint_state_pub_;
  rclcpp::Subscription<JTCState>::SharedPtr left_state_sub_;
  rclcpp::Subscription<JTCState>::SharedPtr right_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr test_axis_sub_;
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
