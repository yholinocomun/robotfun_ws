cmake_minimum_required(VERSION 3.8)
project(robotfun_firmware)

find_package(ament_cmake REQUIRED)

# El .ino se compila/sube con Arduino IDE o PlatformIO; aqui solo se instalan
# el codigo fuente y el launch del Agent para tenerlos versionados en el workspace.
install(DIRECTORY
  firmware
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
