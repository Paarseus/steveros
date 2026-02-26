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
  src/robstride_can_driver.cpp
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_robstride_protocol test/test_robstride_protocol.cpp)
  target_include_directories(test_robstride_protocol PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_robstride_can_driver test/test_robstride_can_driver.cpp)
  target_include_directories(test_robstride_can_driver PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_robstride_can_driver steveros_hardware)

  ament_add_gtest(test_can_bus_inspection test/test_can_bus_inspection.cpp)
  target_include_directories(test_can_bus_inspection PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_can_bus_inspection steveros_hardware)

  add_executable(test_motor21_move test/test_motor21_move.cpp)
  target_include_directories(test_motor21_move PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_motor21_move steveros_hardware)

  add_executable(test_motor21_defaults test/test_motor21_defaults.cpp)
  target_include_directories(test_motor21_defaults PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_motor21_defaults steveros_hardware)

  add_executable(test_motor21_diag test/test_motor21_diag.cpp)
  target_include_directories(test_motor21_diag PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_motor21_diag steveros_hardware)

  # Work around conda libstdc++ shadowing the system version
  if(EXISTS "/usr/lib/x86_64-linux-gnu/libstdc++.so.6")
    set_tests_properties(test_robstride_can_driver PROPERTIES
      ENVIRONMENT "LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
    )
    set_tests_properties(test_can_bus_inspection PROPERTIES
      ENVIRONMENT "LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
    )
  endif()
endif()

ament_package()
