cmake_minimum_required(VERSION 3.8)
project(hc_tj_camera_bridge)

find_package(ament_cmake REQUIRED)
find_package(camera_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc)

add_executable(image_jpeg_compressor src/image_jpeg_compressor.cpp)
ament_target_dependencies(
  image_jpeg_compressor
  rclcpp
  sensor_msgs
)
target_link_libraries(image_jpeg_compressor ${OpenCV_LIBS})

add_executable(depth_png_compressor src/depth_png_compressor.cpp)
ament_target_dependencies(
  depth_png_compressor
  rclcpp
  sensor_msgs
)
target_link_libraries(depth_png_compressor ${OpenCV_LIBS})

add_executable(camera_calibration_json_bridge src/camera_calibration_json_bridge.cpp)
ament_target_dependencies(
  camera_calibration_json_bridge
  camera_msgs
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
    image_jpeg_compressor
    depth_png_compressor
    camera_calibration_json_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
