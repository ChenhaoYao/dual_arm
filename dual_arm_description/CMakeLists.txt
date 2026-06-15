cmake_minimum_required(VERSION 3.8)
project(dual_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES urdf.rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  joint_control_panel.py
  scripts/move_to_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
