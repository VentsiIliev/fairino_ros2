cmake_minimum_required(VERSION 3.22)
project(zeroerr)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/zeroerr_state_publisher.py
  scripts/zeroerr_error_monitor.py
  scripts/zeroerr_torque_sensor_probe.py
  scripts/zeroerr_torque_model_fit.py
  scripts/zeroerr_collision_monitor.py
  scripts/zeroerr_collision_monitor_gui.py
  scripts/zeroerr_runtime.py
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
