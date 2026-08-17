cmake_minimum_required(VERSION 3.8)
project(ethercat_generic_slave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Enable RTTI (run-time type information)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")

# Gather source files for plugins
file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)

# Create the ethercat_generic_slave library
add_library(ethercat_generic_slave ${PLUGINS_SRC})
target_compile_features(ethercat_generic_slave PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Include directories
target_include_directories(ethercat_generic_slave PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(
  ethercat_generic_slave
  ethercat_interface
  pluginlib
  yaml_cpp_vendor
)

# Explicitly link yaml-cpp to ethercat_generic_slave
target_link_libraries(ethercat_generic_slave yaml-cpp)

# Export the plugin description file for pluginlib
pluginlib_export_plugin_description_file(ethercat_interface ethercat_plugins.xml)

# Install directives
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ethercat_generic_slave
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# If BUILD_TESTING is enabled, add tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(ethercat_interface REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test Load EtherCAT modules
  ament_add_gmock(
    test_load_generic_plugins
    test/test_load_ec_modules.cpp
  )
  target_include_directories(test_load_generic_plugins PRIVATE include)
  ament_target_dependencies(test_load_generic_plugins
    pluginlib
    ethercat_interface
  )

  # Test Generic EtherCAT Slave Plugin
  ament_add_gmock(
    test_generic_ec_slave
    test/test_generic_ec_slave.cpp
  )
  target_include_directories(test_generic_ec_slave PRIVATE include)
  target_link_libraries(test_generic_ec_slave
    ethercat_generic_slave
    yaml-cpp  # Ensure yaml-cpp is linked to the test
  )
  ament_target_dependencies(test_generic_ec_slave
    pluginlib
    ethercat_interface
  )
endif()

# Export include directories, libraries, and targets
ament_export_include_directories(
  include
)
ament_export_libraries(
  ethercat_generic_slave
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

