cmake_minimum_required(VERSION 3.8)
project(dual_arm_control)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(dual_arm_control SHARED
  src/dual_arm_hardware.cpp
)
target_include_directories(dual_arm_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dual_arm_control
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface hardware_plugin_description.xml)

install(TARGETS dual_arm_control
  EXPORT export_dual_arm_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_dual_arm_control HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
)
ament_package()
