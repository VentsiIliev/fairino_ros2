cmake_minimum_required(VERSION 3.22)
project(fairino5_v6_moveit2_config)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/fairino_state_publisher.py
  scripts/fairino_runtime.py
  scripts/fairino_oscilloscope.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/docs)
  install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
  install(FILES README.md DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant)
  install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
