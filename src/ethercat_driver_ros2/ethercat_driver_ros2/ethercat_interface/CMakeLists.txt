cmake_minimum_required(VERSION 3.8)
project(ethercat_interface)

# Compiler flags for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)  # Ensure yaml_cpp_vendor is found

# EtherLab Setup
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

# Export include directories
ament_export_include_directories(
  include
  ${ETHERLAB_DIR}/include
)

# Create shared library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ec_master.cpp
)

# Include directories for the target
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${ETHERLAB_DIR}/include
)

# Link ethercat library and yaml-cpp to the project
target_link_libraries(${PROJECT_NAME} ${ETHERCAT_LIB} yaml-cpp)

# Dependencies for the project
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

# Enable RTTI (Run-Time Type Information)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")

# C++ Standard (optional but recommended)
set(CMAKE_CXX_STANDARD 14)

# Install the target and header files
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# If testing is enabled
if(BUILD_TESTING)
  # Find testing dependencies and linter
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test PdoChannelManager using Google Test
  ament_add_gmock(
    test_ec_pdo_channel_manager
    test/test_ec_pdo_channel_manager.cpp
  )

  # Include directories for the test
  target_include_directories(test_ec_pdo_channel_manager PRIVATE include ${ETHERLAB_DIR}/include)

  # Link test dependencies and yaml_cpp_vendor
  ament_target_dependencies(test_ec_pdo_channel_manager
    yaml_cpp_vendor
  )
  target_link_libraries(test_ec_pdo_channel_manager yaml-cpp)  # Explicitly link yaml-cpp
endif()

## Exported dependencies
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp
)

ament_package()

