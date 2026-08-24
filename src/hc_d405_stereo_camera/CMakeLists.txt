cmake_minimum_required(VERSION 3.8)
project(hc_d405_stereo_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(realsense2 REQUIRED)

add_executable(d405_stereo_camera_node src/d405_stereo_camera_node.cpp)
ament_target_dependencies(
  d405_stereo_camera_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(
  d405_stereo_camera_node
  ${OpenCV_LIBS}
  realsense2::realsense2
)

install(
  TARGETS d405_stereo_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/run_d405_stereo_camera.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
