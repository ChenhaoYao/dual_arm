cmake_minimum_required(VERSION 3.8)
project(dual_arm_soem_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)

set(SOEM_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SOEM" CACHE PATH "Path to SOEM source")
set(SOEM_BUILD_SAMPLES OFF CACHE BOOL "Build SOEM samples" FORCE)
add_subdirectory("${SOEM_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/soem")

add_library(soem_pp_master
  src/soem_pp_master.cpp
)
target_include_directories(soem_pp_master PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(soem_pp_master PUBLIC soem)

add_executable(soem_bridge_node
  src/soem_bridge_node.cpp
)
target_link_libraries(soem_bridge_node soem_pp_master)
ament_target_dependencies(soem_bridge_node
  rclcpp
  trajectory_msgs
  sensor_msgs
  std_srvs
  control_msgs
)

install(TARGETS soem_pp_master soem_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
