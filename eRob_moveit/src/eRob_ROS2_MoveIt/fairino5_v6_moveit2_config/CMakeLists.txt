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
  DEPENDENCIES trajectory_msgs moveit_msgs
)

# Python scripts - install to lib for executable access
install(PROGRAMS
  scripts/main.py
  scripts/simple_monitor_gui.py
  scripts/monitor_window.py
  scripts/robot_controller.py
  scripts/rest_server.py
  scripts/enums.py
  scripts/fairino_ros2_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules to lib so they can be imported
install(DIRECTORY scripts/utils
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY scripts/safety
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY scripts/status
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY scripts/motion
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY scripts/tools
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY scripts/workspace
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

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

# Set RPATH to find libraries at runtime
set_target_properties(ipp_helper PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../lib:$ORIGIN/../../../../fairino_msgs/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Install the executable
install(TARGETS ipp_helper DESTINATION lib/${PROJECT_NAME})

# C++ Robot State Publisher node
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fairino_msgs REQUIRED)

add_executable(robot_state_publisher src_cpp/robot_state_publisher.cpp)

target_include_directories(robot_state_publisher PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(robot_state_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  fairino_msgs
)

# Set RPATH to find libraries at runtime
set_target_properties(robot_state_publisher PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../lib:$ORIGIN/../../../../fairino_msgs/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS robot_state_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
