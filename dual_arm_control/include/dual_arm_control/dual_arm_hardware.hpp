/**
 * @file dual_arm_hardware.hpp
 * @brief 双臂机器人硬件接口头文件
 *
 * 实现 ros2_control 的 SystemInterface，用于桥接 JTC 和 soem_bridge_node。
 *
 * 设计思路：
 *   - 本接口不直接控制硬件，只负责状态传递
 *   - read():  订阅 /joint_states 话题，更新 state_interface（供 JTC 读取）
 *   - write(): 不做任何操作，命令由 soem_bridge 通过 topic 发送
 *
 * 数据流：
 *   JTC → command_interface.velocity → (本接口不处理) → soem_bridge 订阅 controller_state
 *   soem_bridge → /joint_states → 本接口 → state_interface.position → JTC
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

// ros2_control 硬件接口基类
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"

// ROS2 核心
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// 消息类型：用于订阅 /joint_states
#include "sensor_msgs/msg/joint_state.hpp"

// 插件可见性控制（Windows DLL 导出）
#include "visibility_control.h"

namespace dual_arm_control
{

/**
 * @class DualArmHardware
 * @brief 双臂机器人硬件接口插件
 *
 * 继承自 hardware_interface::SystemInterface，实现以下生命周期回调：
 *   - on_init:      初始化存储空间，验证 URDF 接口
 *   - on_configure: 创建 ROS2 节点，订阅 /joint_states
 *   - on_activate:  激活硬件（本项目无特殊操作）
 *   - on_deactivate: 停用硬件（本项目无特殊操作）
 *
 * 导出接口：
 *   - StateInterface:   position, velocity, effort（供 JTC 读取）
 *   - CommandInterface: position, velocity（供 JTC 写入）
 */
class DualArmHardware : public hardware_interface::SystemInterface
{
public:
  // 生成 shared_ptr 相关类型定义（必须宏）
  RCLCPP_SHARED_PTR_DEFINITIONS(DualArmHardware)

  // ==================== 生命周期回调 ====================

  /**
   * @brief 初始化回调
   *
   * 在插件加载时调用。完成以下工作：
   * 1. 调用父类初始化
   * 2. 根据 URDF 关节数量分配存储空间
   * 3. 保存关节名称列表
   * 4. 验证所有关节支持 position/velocity 接口
   *
   * @param params 硬件参数（包含 URDF 解析信息和 executor）
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /**
   * @brief 配置回调
   *
   * 在 Unconfigured → Inactive 转换时调用。
   * 创建内部 ROS2 节点并订阅 /joint_states 话题。
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 激活回调
   *
   * 在 Inactive → Active 转换时调用。
   * 本项目中电机使能由 soem_bridge 的 enable 服务控制，此处无操作。
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 停用回调
   *
   * 在 Active → Inactive 转换时调用。
   * 本项目中无特殊操作。
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ==================== 接口导出 ====================

  /**
   * @brief 导出状态接口
   *
   * 为每个关节创建 position, velocity, effort 三个状态接口。
   * 控制器（如 JTC）通过这些接口读取关节实际状态。
   * 接口内存地址在生命周期内保持不变。
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief 导出命令接口
   *
   * 为每个关节创建 position, velocity 两个命令接口。
   * 控制器通过这些接口写入命令（本项目只用 velocity）。
   * 注意：write() 不处理这些命令，实际发送由 soem_bridge 完成。
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ==================== 读写循环 ====================

  /**
   * @brief 读取硬件状态
   *
   * 由 controller_manager 以 update_rate (100Hz) 调用。
   * 调用 spin_some() 处理 /joint_states 话题回调，
   * 更新 hw_position_states_ 等数据。
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief 写入硬件命令
   *
   * 由 controller_manager 以 update_rate (100Hz) 调用。
   * 本项目中不做任何操作，命令由 soem_bridge 通过订阅
   * controller_state 话题直接发送给电机。
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // ==================== 常量 ====================

  /// 双臂关节数量：左臂 7 轴 + 右臂 7 轴 = 14 轴
  static constexpr size_t NUM_JOINTS = 14;

  // ==================== 命令/状态存储 ====================
  // 这些数组的地址会被导出为 StateInterface/CommandInterface
  // 控制器通过指针直接读写，无需额外拷贝

  /// 位置命令（本项目未使用，velocity 模式下由 JTC 写入但不处理）
  std::vector<double> hw_position_commands_;

  /// 速度命令（JTC 的 PID 输出写入此处，但 write() 不处理）
  std::vector<double> hw_velocity_commands_;

  /// 位置状态（从 /joint_states 读取，JTC 用它计算位置误差）
  std::vector<double> hw_position_states_;

  /// 速度状态（从 /joint_states 读取，JTC 可选使用）
  std::vector<double> hw_velocity_states_;

  /// 力矩状态（从 /joint_states 读取，本项目未使用）
  std::vector<double> hw_effort_states_;

  // ==================== ROS2 订阅 ====================

  /// 内部 ROS2 节点，用于订阅 /joint_states 话题
  rclcpp::Node::SharedPtr node_;

  /// /joint_states 话题订阅者
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // ==================== 关节配置 ====================

  /// 关节名称列表（顺序与 URDF 一致）
  std::vector<std::string> joint_names_;

  /**
   * @brief /joint_states 话题回调
   *
   * 当 soem_bridge_node 发布新的关节状态时调用。
   * 将消息中的 position/velocity/effort 更新到 hw_xxx_states_ 数组。
   *
   * @param msg joint_states 消息
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace dual_arm_control
