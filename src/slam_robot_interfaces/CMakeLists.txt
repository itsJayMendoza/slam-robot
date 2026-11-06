cmake_minimum_required(VERSION 3.8)
project(slam_robot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ApReading.msg"
  "msg/RawReading.msg"
  "action/ScanAtWaypoint.action"
)
ament_package()
