cmake_minimum_required(VERSION 3.16)
project(steveros_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(steveros_hardware SHARED
  src/steveros_hardware.cpp
)
target_compile_features(steveros_hardware PUBLIC cxx_std_17)
target_include_directories(steveros_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/steveros_hardware>
)
ament_target_dependencies(steveros_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_compile_definitions(steveros_hardware PRIVATE "STEVEROS_HARDWARE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface steveros_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/steveros_hardware
)
install(TARGETS steveros_hardware
  EXPORT export_steveros_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_steveros_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
