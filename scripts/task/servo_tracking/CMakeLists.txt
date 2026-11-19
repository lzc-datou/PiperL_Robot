cmake_minimum_required(VERSION 3.16)
project(piper_servo_tracking)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanJointTarget.srv" "srv/JointPlanCommand.srv" "srv/SelectGamepadControl.srv" "msg/GamepadInput.msg"
  DEPENDENCIES std_msgs)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
ament_python_install_package(servo_tracking PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
add_library(piper_servo_panel SHARED rviz_panel/panel.cpp rviz_panel/panel.hpp
  rviz_panel/workspace_display.cpp rviz_panel/target_ghost_display.cpp
  "${CMAKE_CURRENT_SOURCE_DIR}/../common/rviz_theme.qrc")
ament_target_dependencies(piper_servo_panel rclcpp rviz_common rviz_default_plugins pluginlib std_msgs std_srvs)
target_link_libraries(piper_servo_panel Qt5::Widgets
  moveit_ros_visualization::moveit_robot_state_rviz_plugin_core)
if(rviz_common_VERSION VERSION_EQUAL "14.1.23")
  target_compile_definitions(piper_servo_panel PRIVATE PIPER_RVIZ_NANOSECOND_DELTA)
endif()
pluginlib_export_plugin_description_file(rviz_common rviz_panel/plugins.xml)
install(TARGETS piper_servo_panel LIBRARY DESTINATION lib)
install(PROGRAMS bin/servo_backend bin/servo_guard bin/gamepad_backend
  "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/gamepad_control/bin/gamepad_input"
  DESTINATION lib/${PROJECT_NAME})
install(FILES task.launch.py DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/gamepad_control/config/config.yaml"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/gamepad_control/config/flydigi_direwolf2.yaml"
  DESTINATION share/${PROJECT_NAME}/config/gamepad)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
