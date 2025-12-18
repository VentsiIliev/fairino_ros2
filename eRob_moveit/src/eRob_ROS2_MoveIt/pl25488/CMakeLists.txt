cmake_minimum_required(VERSION 3.22)
project(pl25488)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


install(PROGRAMS
  scripts/gui_bridge.py
  #scripts/publish_initial_state.py
  #scripts/auto_home_on_launch
  scripts/cartesian_gui.py
  scripts/collision_monitor.py
  scripts/test_collision.py
  #scripts/cobot-soft-new-v4/cobot-glue-dispensing-v3/src/main.py
  #scripts/set_start_state_from_joints.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE OWNER_READ GROUP_READ WORLD_READ
)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
