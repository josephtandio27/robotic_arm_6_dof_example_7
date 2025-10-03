cmake_minimum_required(VERSION 3.8)
project(r6dof_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
# find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(send_trajectory reference_generator/send_trajectory.cpp)
ament_target_dependencies(send_trajectory rclcpp kdl_parser trajectory_msgs)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  send_trajectory
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
