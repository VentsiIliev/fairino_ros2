cmake_minimum_required(VERSION 3.22)
project(erob_moveit_runtime)

# Avoid rosidl_generator_py colliding with symlink-install in this package.
set(AMENT_CMAKE_SYMLINK_INSTALL OFF CACHE BOOL "" FORCE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(ruckig REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ApplyIPP.srv"
  "srv/ComputeContourIK.srv"
  "srv/ComputeLinkedLin.srv"
  "srv/ComputePtp.srv"
  "srv/ValidateTrajectoryStates.srv"
  DEPENDENCIES trajectory_msgs moveit_msgs sensor_msgs geometry_msgs
)

add_library(trajectory_validation
  src_cpp/trajectory_validation.cpp)
target_include_directories(trajectory_validation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_validation
  builtin_interfaces
  trajectory_msgs)

add_executable(trajectory_validation_smoke
  src_cpp/trajectory_validation_smoke.cpp)
target_link_libraries(trajectory_validation_smoke
  trajectory_validation)
ament_target_dependencies(trajectory_validation_smoke
  trajectory_msgs)

# Install the shared Python package so robot-specific publishers can import it
ament_python_install_package(erob_state_publisher)

install(PROGRAMS
  scripts/main.py
  scripts/simple_monitor_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/__init__.py
  scripts/config.py
  scripts/enums.py
  scripts/monitor_window.py
  scripts/runtime_initializer.py
  scripts/robot_controller.py
  scripts/robot_runtime_context.py
  DESTINATION lib/${PROJECT_NAME}
)

foreach(module_dir api backend motion rest runtime_api runtime_gateway runtime_websockets safety status utils)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${module_dir})
    install(DIRECTORY scripts/${module_dir}
      DESTINATION lib/${PROJECT_NAME}
      FILES_MATCHING PATTERN "*.py" PATTERN "*.txt")
  endif()
endforeach()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/docs)
  install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
  install(FILES README.md DESTINATION share/${PROJECT_NAME})
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS trajectory_validation
  EXPORT export_trajectory_validation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_include_directories(include)
ament_export_targets(export_trajectory_validation HAS_LIBRARY_TARGET)
ament_export_dependencies(builtin_interfaces trajectory_msgs)

add_executable(ipp_helper src_cpp/ipp_helper_node.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(ipp_helper
  rclcpp moveit_core moveit_ros_planning_interface moveit_ros_planning
  moveit_msgs trajectory_msgs geometry_msgs shape_msgs)
target_link_libraries(ipp_helper ${cpp_typesupport_target})
set_target_properties(ipp_helper PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../lib:$ORIGIN/../../../../fairino_msgs/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS ipp_helper DESTINATION lib/${PROJECT_NAME})

add_executable(linked_lin_helper src_cpp/linked_lin_helper_node.cpp)
ament_target_dependencies(linked_lin_helper
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs)
target_link_libraries(linked_lin_helper
  trajectory_validation
  ${cpp_typesupport_target})
set_target_properties(linked_lin_helper PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../lib:$ORIGIN/../../../../fairino_msgs/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS linked_lin_helper DESTINATION lib/${PROJECT_NAME})

add_executable(trajectory_state_validator src_cpp/trajectory_state_validator_node.cpp)
ament_target_dependencies(trajectory_state_validator
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs)
target_link_libraries(trajectory_state_validator ${cpp_typesupport_target})
set_target_properties(trajectory_state_validator PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../lib:$ORIGIN/../../../../fairino_msgs/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS trajectory_state_validator DESTINATION lib/${PROJECT_NAME})

add_executable(ruckig_helper src_cpp/ruckig_helper_node.cpp)
ament_target_dependencies(ruckig_helper
  rclcpp moveit_core moveit_ros_planning_interface moveit_ros_planning
  moveit_msgs trajectory_msgs geometry_msgs shape_msgs ruckig)
target_link_libraries(ruckig_helper ${cpp_typesupport_target})
set_target_properties(ruckig_helper PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../lib:$ORIGIN/../../../../fairino_msgs/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS ruckig_helper DESTINATION lib/${PROJECT_NAME})

add_executable(
  contour_ik_helper
  src_cpp/contour_ik_helper_node.cpp)

ament_target_dependencies(
    contour_ik_helper
    rclcpp
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_msgs
    sensor_msgs
    trajectory_msgs
    geometry_msgs)

target_link_libraries(contour_ik_helper ${cpp_typesupport_target})
set_target_properties(contour_ik_helper PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../lib:$ORIGIN/../../../../fairino_msgs/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS contour_ik_helper DESTINATION lib/${PROJECT_NAME})


add_executable(ptp_helper src_cpp/ptp_helper_node.cpp)
ament_target_dependencies(ptp_helper
  rclcpp moveit_core moveit_ros_planning_interface moveit_ros_planning
  moveit_msgs sensor_msgs trajectory_msgs geometry_msgs)
target_link_libraries(ptp_helper ${cpp_typesupport_target})
set_target_properties(ptp_helper PROPERTIES
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../../../lib:$ORIGIN/../../../../fairino_msgs/lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS ptp_helper DESTINATION lib/${PROJECT_NAME})

ament_package()
