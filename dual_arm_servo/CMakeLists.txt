cmake_minimum_required(VERSION 3.8)
project(dual_arm_servo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS dual_arm_servo/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_teleop)
install(PROGRAMS dual_arm_servo/rviz_servo_marker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rviz_servo_marker)
install(FILES dual_arm_servo/__init__.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
