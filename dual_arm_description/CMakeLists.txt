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
  DESTINATION lib/${PROJECT_NAME}
)

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/move_to_pose.py
  \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/move_to_pose.py
)")

ament_package()
