cmake_minimum_required(VERSION 3.14)
project(self_driving_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(self_driving_controller
  src/self_driving_controller.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
