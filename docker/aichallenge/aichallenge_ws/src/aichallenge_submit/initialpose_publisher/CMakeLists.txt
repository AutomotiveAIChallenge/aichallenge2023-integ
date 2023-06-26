cmake_minimum_required(VERSION 3.14)
project(initialpose_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(initialpose_publisher
  src/initialpose_publisher.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch config)
