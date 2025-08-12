cmake_minimum_required(VERSION 3.8)
project(ros2_control_diffdrive_encoder_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 2. compile directive
add_library(
  ros2_control_diffdrive_encoder_plugin
  SHARED
  src/diffdrive_system.cpp
)

target_compile_features(ros2_control_diffdrive_encoder_plugin PUBLIC cxx_std_17)

# 3. include directories
target_include_directories(ros2_control_diffdrive_encoder_plugin PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ros2_control_diffdrive_encoder_plugin>
)

# 4. ament dependencies 
ament_target_dependencies(ros2_control_diffdrive_encoder_plugin PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# 5. export pluginlib
pluginlib_export_plugin_description_file(hardware_interface ros2_control_diffdrive_encoder_plugin.xml)

# link gpio library
target_link_libraries(ros2_control_diffdrive_encoder_plugin
  PRIVATE -lpigpiod_if2
)

# 6. install directive

install(
  DIRECTORY include/
  DESTINATION include/ros2_control_diffdrive_encoder_plugin
)

install(
  DIRECTORY bringup/launch bringup/config description
  DESTINATION share/ros2_control_diffdrive_encoder_plugin
)

install(TARGETS ros2_control_diffdrive_encoder_plugin
  EXPORT ros2_control_diffdrive_encoder_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
