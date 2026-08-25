cmake_minimum_required(VERSION 3.8)
project(camera_driver)

get_filename_component(HC_X1_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(camera_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIREDIS REQUIRED hiredis)
find_path(REDIS_PLUS_PLUS_INCLUDE_DIR NAMES sw/redis++/redis++.h)
find_library(REDIS_PLUS_PLUS_LIBRARY NAMES redis++)

if(DEFINED ENV{BERXEL_SDK_LIBRARY} AND NOT "$ENV{BERXEL_SDK_LIBRARY}" STREQUAL "")
  set(BERXEL_SDK_ROOT "$ENV{BERXEL_SDK_LIBRARY}")
else()
  set(BERXEL_SDK_ROOT "${HC_X1_ROOT}/vendor/berxel")
endif()
set(BERXEL_SDK_INCLUDE ${BERXEL_SDK_ROOT}/Include)
set(BERXEL_SDK_LIB ${BERXEL_SDK_ROOT}/libs)
include_directories(${BERXEL_SDK_INCLUDE})
link_directories(${BERXEL_SDK_LIB})

if(DEFINED ENV{ANGSTRONG_SDK_LIBRARY} AND NOT "$ENV{ANGSTRONG_SDK_LIBRARY}" STREQUAL "")
  set(ANGSTRONG_SDK_ROOT "$ENV{ANGSTRONG_SDK_LIBRARY}")
else()
  set(ANGSTRONG_SDK_ROOT "${HC_X1_ROOT}/vendor/angstrong")
endif()
set(ANGSTRONG_SDK_INCLUDE ${ANGSTRONG_SDK_ROOT}/include)
set(ANGSTRONG_SDK_LIB ${ANGSTRONG_SDK_ROOT}/libs)
include_directories(${ANGSTRONG_SDK_INCLUDE})
link_directories(${ANGSTRONG_SDK_LIB})

# 创建可执行文件
add_executable(${PROJECT_NAME}
  src/camera_angstrong.cpp
  src/camera_berxel.cpp
  src/camera_driver.cpp
  src/axis_transform.cpp
  src/pose_estimator.cpp
  src/rediswrapper.cpp
  src/pose3d.cpp
  src/custom_log.cpp
  src/config.cpp
  src/main.cpp
)

# 添加包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${camera_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${HIREDIS_INCLUDE_DIRS}
  ${REDIS_PLUS_PLUS_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
# 链接库
target_link_libraries(${PROJECT_NAME} 
  ${HIREDIS_LIBRARIES}
  ${REDIS_PLUS_PLUS_LIBRARY}
  BerxelHawk
  nlohmann_json::nlohmann_json
  yaml-cpp
  spdlog::spdlog
  ${OpenCV_LIBS}
)
target_link_libraries(${PROJECT_NAME}
  AngstrongCameraSdk
  zmq
  Filt
  asuvc
  asusb
  turbojpeg
  alg_kunlun
  jpeg
  AngKondyorArith
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  camera_msgs
)

# 安装目标
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  # config
  DESTINATION share/${PROJECT_NAME}
)

# 安装包含文件
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
