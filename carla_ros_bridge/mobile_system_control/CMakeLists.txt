cmake_minimum_required(VERSION 3.5)
project(mobile_system_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mobile_system_control
  src/mobile_system_control/main.cpp
  src/mobile_system_control/mobile_system_control.cpp
)

include_directories(include)

ament_target_dependencies(mobile_system_control
  rclcpp
  carla_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS
  mobile_system_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
