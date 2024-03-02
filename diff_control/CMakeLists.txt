cmake_minimum_required(VERSION 3.16)
project(diff_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav_interfaces
  rclcpp_components
  serial
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  diff_control
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(diff_control PUBLIC cxx_std_17)
target_include_directories(diff_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/diff_control>
)

ament_target_dependencies(
  diff_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFF_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diff_control.xml)

# INSTALL
install(
  DIRECTORY world
  DESTINATION share/diff_control
)
install(
  DIRECTORY hardware/include/
  DESTINATION include/diff_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/diff_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diff_control
)
install(TARGETS diff_control
  EXPORT export_diff_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_diff_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
