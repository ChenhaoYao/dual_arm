#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

namespace dual_arm_control
{

class DualArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DualArmHardware)

  // Lifecycle methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Interface export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read/Write loop (called at update_rate Hz)
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Storage for 14 joints (7 left + 7 right)
  static constexpr size_t NUM_JOINTS = 14;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
};

}  // namespace dual_arm_control
