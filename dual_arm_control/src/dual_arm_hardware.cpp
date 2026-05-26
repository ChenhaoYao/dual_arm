#include "dual_arm_control/dual_arm_hardware.hpp"

namespace dual_arm_control
{

hardware_interface::CallbackReturn DualArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Resize storage based on joints defined in URDF
  hw_position_commands_.resize(info_.joints.size(), 0.0);
  hw_velocity_commands_.resize(info_.joints.size(), 0.0);
  hw_position_states_.resize(info_.joints.size(), 0.0);
  hw_velocity_states_.resize(info_.joints.size(), 0.0);
  hw_effort_states_.resize(info_.joints.size(), 0.0);

  // Validate that all joints have the required interfaces
  for (const auto & joint : info_.joints)
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
  // Here: open serial port, connect to motor controllers, etc.
  // For real hardware: initialize CAN bus, EtherCAT, serial, etc.
  RCLCPP_INFO(rclcpp::get_logger("DualArmHardware"), "Configuring hardware...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DualArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Enable motors, set initial positions
  RCLCPP_INFO(rclcpp::get_logger("DualArmHardware"), "Activating hardware...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DualArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Disable motors safely
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

hardware_interface::return_type DualArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // REAL HARDWARE: read encoder positions from motor controllers
  // SIMULATION: just copy commands to states (passthrough)
  for (size_t i = 0; i < hw_position_states_.size(); ++i)
  {
    hw_position_states_[i] = hw_position_commands_[i];
    hw_velocity_states_[i] = hw_velocity_commands_[i];
    hw_effort_states_[i] = 0.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DualArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // REAL HARDWARE: send commands to motor controllers
  // SIMULATION: no-op (read() already does passthrough)
  return hardware_interface::return_type::OK;
}

}  // namespace dual_arm_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dual_arm_control::DualArmHardware,
  hardware_interface::SystemInterface)
