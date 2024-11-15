add_library(diffdrive_andino_esp32 SHARED diffdrive_andino_esp32.cpp wheel.cpp)

target_include_directories(diffdrive_andino_esp32 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(diffdrive_andino_esp32 PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(
  TARGETS diffdrive_andino_esp32
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(diffdrive_andino_esp32)
ament_export_targets(diffdrive_andino_esp32 HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
