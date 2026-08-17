cmake_minimum_required(VERSION 3.8)
project(ethercat_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# rosidl_generator_py creates a generated python package directory for this
# interface package. With workspace-wide symlink-install enabled, the ament
# python helper tries to create a symlink at the same path and fails because
# that path is already a directory. Disable symlink-install locally so the
# generated package is copied instead.
set(AMENT_CMAKE_SYMLINK_INSTALL OFF CACHE BOOL "" FORCE)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetSdo.srv"
  "srv/GetSdo.srv"
  DEPENDENCIES
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
