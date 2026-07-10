/**
 * @file dual_arm_hardware.cpp
 * @brief 双臂机器人硬件接口实现
 *
 * 本文件实现了 ros2_control 的硬件接口，用于桥接 JTC (JointTrajectoryController)
 * 和 soem_bridge_node。
 *
 * 数据流：
 *   JTC → command_interface (velocity) → 本接口（不做处理）→ soem_bridge 通过 topic 发送
 *   soem_bridge → /joint_states → 本接口 → state_interface → JTC 读取
 *
 * 关键点：
 *   - write() 不做任何操作，因为实际命令由 soem_bridge 通过订阅 controller_state 发送
 *   - read() 通过订阅 /joint_states 话题更新 state_interface，供 JTC 读取位置反馈
 */

#include "dual_arm_control/dual_arm_hardware.hpp"

namespace dual_arm_control
{

/**
 * @brief 硬件接口初始化
 *
 * ros2_control 加载硬件插件时调用。完成以下工作：
 * 1. 调用父类初始化
 * 2. 根据 URDF 中定义的关节数量分配存储空间
 * 3. 保存关节名称列表
 * 4. 验证所有关节都支持 position 和 velocity 接口
 *
 * @param params 硬件参数，包含 URDF 解析后的硬件信息
 * @return SUCCESS 或 ERROR
 */
hardware_interface::CallbackReturn DualArmHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // 调用父类初始化，必须成功才能继续
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 从 URDF 解析的硬件信息
  const auto & info = params.hardware_info;

  // 根据关节数量分配存储空间
  // hw_xxx_commands_: 存储 JTC 输出的命令（本项目中只用 velocity）
  // hw_xxx_states_:  存储实际状态（从 /joint_states 读取）
  hw_position_commands_.resize(info.joints.size(), 0.0);
  hw_velocity_commands_.resize(info.joints.size(), 0.0);
  hw_position_states_.resize(info.joints.size(), 0.0);
  hw_velocity_states_.resize(info.joints.size(), 0.0);
  hw_effort_states_.resize(info.joints.size(), 0.0);

  // 保存关节名称，用于后续与 /joint_states 话题匹配
  joint_names_.clear();
  for (const auto & joint : info.joints)
  {
    joint_names_.push_back(joint.name);
  }

  // 验证所有关节都支持 position 和 velocity 命令接口
  // 如果有不支持的接口，返回错误
  for (const auto & joint : info.joints)
  {
    for (const auto & cmd_interface : joint.command_interfaces)
    {
      if (cmd_interface.name != "position" && cmd_interface.name != "velocity")
      {
        RCLCPP_ERROR(rclcpp::get_logger("DualArmHardware"),
                     "Joint '%s' has unsupported command interface '%s'",
                     joint.name.c_str(), cmd_interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 配置阶段回调
 *
 * 在硬件从 Unconfigured 转到 Inactive 时调用。
 * 创建内部 ROS2 节点并订阅 /joint_states 话题，
 * 用于接收 soem_bridge_node 发布的真实编码器位置。
 *
 * @param previous_state 前一个生命周期状态（未使用）
 * @return SUCCESS 或 ERROR
 */
hardware_interface::CallbackReturn DualArmHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 创建内部 ROS2 节点，用于订阅话题
  // 注意：这个节点不参与 ros2_control 的生命周期管理
  node_ = std::make_shared<rclcpp::Node>("dual_arm_hardware_node");

  // 订阅 /joint_states 话题
  // soem_bridge_node 发布真实编码器数据到这个话题
  // QoS 必须与 soem_bridge_node 的 /joint_states 发布端一致(best_effort)，
  // 否则 RELIABLE 订阅者无法接收 BEST_EFFORT 发布者的数据，
  // 导致 hw_position_states_ 永远停在初始值 0，JTC 误差恒等于 ref、输出恒为 p*ref 失控。
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10).best_effort(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      joint_state_callback(msg);
    });

  RCLCPP_INFO(rclcpp::get_logger("DualArmHardware"),
              "Configured: subscribing to /joint_states");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 激活阶段回调
 *
 * 在硬件从 Inactive 转到 Active 时调用。
 * 本项目中不需要特殊操作，因为实际电机使能由 soem_bridge 的 enable 服务控制。
 *
 * @param previous_state 前一个生命周期状态（未使用）
 * @return SUCCESS
 */
hardware_interface::CallbackReturn DualArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DualArmHardware"), "Activating hardware...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 停用阶段回调
 *
 * 在硬件从 Active 转到 Inactive 时调用。
 * 本项目中不需要特殊操作。
 *
 * @param previous_state 前一个生命周期状态（未使用）
 * @return SUCCESS
 */
hardware_interface::CallbackReturn DualArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DualArmHardware"), "Deactivating hardware...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 导出状态接口
 *
 * 创建并返回状态接口列表，供控制器（如 JTC）读取。
 * 每个关节导出 3 个状态接口：position, velocity, effort
 *
 * 这些接口的内存地址在节点生命周期内保持不变，
 * 控制器通过指针直接读取最新值。
 *
 * @return 状态接口列表
 */
std::vector<hardware_interface::StateInterface>
DualArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // position 状态：JTC 用它计算位置误差
    interfaces.emplace_back(
      info_.joints[i].name, "position", &hw_position_states_[i]);
    // velocity 状态：JTC 可选使用（用于前馈或监控）
    interfaces.emplace_back(
      info_.joints[i].name, "velocity", &hw_velocity_states_[i]);
    // effort 状态：本项目未使用，但 ros2_control 要求导出
    interfaces.emplace_back(
      info_.joints[i].name, "effort", &hw_effort_states_[i]);
  }
  return interfaces;
}

/**
 * @brief 导出命令接口
 *
 * 创建并返回命令接口列表，供控制器写入。
 * 每个关节导出 2 个命令接口：position, velocity
 *
 * 本项目使用 velocity 接口，JTC 的 PID 输出写入 hw_velocity_commands_。
 * 但 write() 函数不会处理这些命令，因为实际发送由 soem_bridge 完成。
 *
 * @return 命令接口列表
 */
std::vector<hardware_interface::CommandInterface>
DualArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // position 命令：本项目未使用（velocity 模式）
    interfaces.emplace_back(
      info_.joints[i].name, "position", &hw_position_commands_[i]);
    // velocity 命令：JTC 的 PID 输出写入此处
    // 但实际不会被 write() 处理，命令由 soem_bridge 通过 topic 发送
    interfaces.emplace_back(
      info_.joints[i].name, "velocity", &hw_velocity_commands_[i]);
  }
  return interfaces;
}

/**
 * @brief /joint_states 话题回调
 *
 * 当 soem_bridge_node 发布新的关节状态时调用。
 * 将话题中的位置、速度、力矩数据更新到 hw_xxx_states_ 数组，
 * 供 JTC 通过 state_interface 读取。
 *
 * @param msg joint_states 消息，包含所有关节的 name, position, velocity, effort
 */
void DualArmHardware::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 遍历本接口管理的关节
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    // 在消息中查找同名关节
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (msg->name[j] == joint_names_[i])
      {
        // 更新位置（必须有）
        hw_position_states_[i] = msg->position[j];
        // 更新速度（可选，消息中可能没有）
        if (j < msg->velocity.size())
        {
          hw_velocity_states_[i] = msg->velocity[j];
        }
        // 更新力矩（可选，消息中可能没有）
        if (j < msg->effort.size())
        {
          hw_effort_states_[i] = msg->effort[j];
        }
        break;  // 找到后跳出内层循环
      }
    }
  }
}

/**
 * @brief 读取硬件状态
 *
 * 由 controller_manager 以 update_rate (100Hz) 调用。
 * 本项目中，此函数调用 spin_some() 处理 /joint_states 话题的回调，
 * 更新 hw_position_states_ 等数据。
 *
 * 注意：实际的数据更新在 joint_state_callback() 中完成，
 * 这里只是触发回调处理。
 *
 * @param time 当前时间（未使用）
 * @param period 距上次调用的时间间隔（未使用）
 * @return 总是返回 OK
 */
hardware_interface::return_type DualArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 处理待处理的 ROS2 回调（主要是 joint_state_callback）
  if (node_)
  {
    rclcpp::spin_some(node_);
  }

  // hw_position_states_ 已在 joint_state_callback 中更新
  // JTC 会通过 state_interface 指针直接读取最新值
  return hardware_interface::return_type::OK;
}

/**
 * @brief 写入硬件命令
 *
 * 由 controller_manager 以 update_rate (100Hz) 调用。
 * 本项目中，此函数不做任何操作。
 *
 * 原因：实际的电机命令由 soem_bridge_node 发送。
 * soem_bridge 订阅 controller_state 话题，读取 JTC 的 output.velocities，
 * 然后通过 EtherCAT 发送给电机。
 *
 * 这种设计避免了命令传递的延迟和复杂性。
 *
 * @param time 当前时间（未使用）
 * @param period 距上次调用的时间间隔（未使用）
 * @return 总是返回 OK
 */
hardware_interface::return_type DualArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 不做任何操作
  // 命令由 soem_bridge_node 通过订阅 controller_state 话题发送
  return hardware_interface::return_type::OK;
}

}  // namespace dual_arm_control

// 注册插件，让 ros2_control 能够通过 pluginlib 加载本硬件接口
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dual_arm_control::DualArmHardware,
  hardware_interface::SystemInterface)
