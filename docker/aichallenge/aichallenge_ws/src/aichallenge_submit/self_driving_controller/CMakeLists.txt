cmake_minimum_required(VERSION 3.14)
project(self_driving_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()
# find_package(autoware_auto_system_msgs REQUIRED)

ament_auto_add_executable(self_driving_controller
  src/self_driving_controller.cpp
)

# ament_target_dependencies(self_driving_controller autoware_auto_system_msgs)

ament_auto_package(INSTALL_TO_SHARE launch)

# find_package(autoware_auto_vehicle_msgs REQUIRED)
# find_package(autoware_auto_perception_msgs REQUIRED)



# install(TARGETS
#   self_driving_controller
#   DESTINATION lib/${PROJECT_NAME})

# ament_package()
