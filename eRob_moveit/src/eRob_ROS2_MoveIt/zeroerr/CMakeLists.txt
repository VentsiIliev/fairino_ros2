cmake_minimum_required(VERSION 3.22)
project(zeroerr)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(zeroerr_servo_node src/zeroerr_servo_node.cpp)
ament_target_dependencies(zeroerr_servo_node
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_servo
  rclcpp
  std_msgs
  std_srvs
  trajectory_msgs
)

install(TARGETS zeroerr_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/zeroerr_state_publisher.py
  scripts/zeroerr_error_monitor.py
  scripts/zeroerr_drive_diagnostics.py
  scripts/zeroerr_runtime.py
  DESTINATION lib/${PROJECT_NAME}
)

# GitHub's contents API creates new files without the executable bit. Under
# colcon --symlink-install an install(PROGRAMS) entry would then point directly
# at that non-executable source file, so ros2 run would not discover it.
# Stage an executable build-tree copy for the installed twin test entrypoint.
configure_file(
  scripts/twin_test.py
  ${CMAKE_CURRENT_BINARY_DIR}/twin_test.py
  COPYONLY
)
file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/twin_test.py
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/twin_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/EtherCatStart.sh
  scripts/PollEthercat.sh
  scripts/WaitForSlavesOp.sh
  scripts/StartMoveIt.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()