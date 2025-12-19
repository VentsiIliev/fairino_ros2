cmake_minimum_required(VERSION 3.22)
project(fairino5_v6_moveit2_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ApplyIPP.srv"
  DEPENDENCIES trajectory_msgs
)

# Python scripts
install(PROGRAMS
  scripts/velocity_monitor.py
  scripts/gui_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# C++ IPP helper node
add_executable(ipp_helper src_cpp/ipp_helper_node.cpp)

# Make sure the service headers are visible
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include MoveIt headers
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

target_include_directories(ipp_helper PUBLIC
  $<BUILD_INTERFACE:${moveit_core_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${moveit_ros_planning_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${moveit_ros_planning_interface_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)

# Link ROS 2 and service dependencies
ament_target_dependencies(ipp_helper
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  trajectory_msgs
  geometry_msgs
  shape_msgs
)

# Link the generated service types
target_link_libraries(ipp_helper ${cpp_typesupport_target})

# Install the executable
install(TARGETS ipp_helper DESTINATION lib/${PROJECT_NAME})

ament_package()
