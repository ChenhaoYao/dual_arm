#include "dual_arm_control/dual_arm_hardware.hpp"

namespace dual_arm_control
{

hardware_interface::CallbackReturn DualArmHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;

  // Resize storage based on joints defined in URDF
  hw_position_commands_.resize(info.joints.size(), 0.0);
  hw_velocity_commands_.resize(info.joints.size(), 0.0);
  hw_position_states_.resize(info.joints.size(), 0.0);
  hw_velocity_states_.resize(info.joints.size(), 0.0);
  hw_effort_states_.resize(info.joints.size(), 0.0);

  // Store joint names from URDF
  joint_names_.clear();
  for (const auto & joint : info.joints)
  {
    joint_names_.push_back(joint.name);
  }

  // Validate that all joints have the required interfaces
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

hardware_interface::CallbackReturn DualArmHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create ROS2 node for subscribing to /joint_states
  node_ = std::make_shared<rclcpp::Node>("dual_arm_hardware_node");

  // Subscribe to /joint_states (published by soem_bridge_node)
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      joint_state_callback(msg);
    });

  RCLCPP_INFO(rclcpp::get_logger("DualArmHardware"),
              "Configured: subscribing to /joint_states");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DualArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DualArmHardware"), "Activating hardware...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DualArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DualArmHardware"), "Deactivating hardware...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DualArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name, "position", &hw_position_states_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "velocity", &hw_velocity_states_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "effort", &hw_effort_states_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
DualArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name, "position", &hw_position_commands_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "velocity", &hw_velocity_commands_[i]);
  }
  return interfaces;
}

void DualArmHardware::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update position states from /joint_states
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (msg->name[j] == joint_names_[i])
      {
        hw_position_states_[i] = msg->position[j];
        if (j < msg->velocity.size())
        {
          hw_velocity_states_[i] = msg->velocity[j];
        }
        if (j < msg->effort.size())
        {
          hw_effort_states_[i] = msg->effort[j];
        }
        break;
      }
    }
  }
}

hardware_interface::return_type DualArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Spin to process incoming /joint_states messages
  if (node_)
  {
    rclcpp::spin_some(node_);
  }

  // Position states are updated in joint_state_callback
  // No need to copy here - they are already in hw_position_states_
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Commands are handled by soem_bridge_node via controller_state topic
  // No direct hardware write needed here
  return hardware_interface::return_type::OK;
}

}  // namespace dual_arm_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dual_arm_control::DualArmHardware,
  hardware_interface::SystemInterface)
