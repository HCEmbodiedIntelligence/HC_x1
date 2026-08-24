cmake_minimum_required(VERSION 3.8)
project(io_msgs2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VrData.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
