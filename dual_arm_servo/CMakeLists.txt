cmake_minimum_required(VERSION 3.8)
project(dual_arm_servo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  src/keyboard_teleop.py
  src/trajectory_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
