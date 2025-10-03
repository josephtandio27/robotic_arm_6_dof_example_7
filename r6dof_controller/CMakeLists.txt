cmake_minimum_required(VERSION 3.16)
project(r6dof_controller LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
    "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)

add_library(
  r6dof_controller
  SHARED
  src/r6bot_hardware.cpp
  src/r6bot_controller.cpp
)

target_compile_features(r6dof_controller PUBLIC cxx_std_17)
target_include_directories(r6dof_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/r6dof_controller>
)
target_link_libraries(r6dof_controller PUBLIC
  ${trajectory_msgs_TARGETS}
  ${control_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface r6dof_controller.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface r6dof_controller.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS r6dof_controller
  EXPORT export_r6dof_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_r6dof_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  pluginlib
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
  controller_interface
  realtime_tools
  control_msgs
)
ament_package()
