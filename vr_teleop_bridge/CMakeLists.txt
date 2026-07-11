cmake_minimum_required(VERSION 3.8)
project(vr_teleop_bridge)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  vr_teleop_bridge/vr_pose_to_servo_node.py
  vr_teleop_bridge/vr_pose_to_move_group_node.py
  vr_teleop_bridge/trajectory_logger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
