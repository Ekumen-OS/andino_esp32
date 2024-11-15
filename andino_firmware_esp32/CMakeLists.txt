cmake_minimum_required(VERSION 3.8)
project(andino_firmware_esp32)

find_package(ament_cmake REQUIRED)

install(
  FILES
    andino_firmware_esp32.ino
    platformio.ini
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  DIRECTORY
    src
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
