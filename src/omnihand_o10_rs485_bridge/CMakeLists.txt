cmake_minimum_required(VERSION 3.8)
project(omnihand_o10_rs485_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(io_msgs2 REQUIRED)

get_filename_component(HC_X1_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
if(NOT DEFINED OMNIHAND_SDK_ROOT)
  if(DEFINED ENV{OMNIHAND_SDK_ROOT} AND NOT "$ENV{OMNIHAND_SDK_ROOT}" STREQUAL "")
    set(OMNIHAND_SDK_ROOT "$ENV{OMNIHAND_SDK_ROOT}")
  else()
    set(OMNIHAND_SDK_ROOT "${HC_X1_ROOT}/vendor/omnihand")
  endif()
endif()

if(NOT EXISTS "${OMNIHAND_SDK_ROOT}/include/omnihand/omnihand_2025.h")
  message(FATAL_ERROR
    "OmniHand SDK not found at ${OMNIHAND_SDK_ROOT}. "
    "Set -DOMNIHAND_SDK_ROOT=/path/to/agillink_omnihand_sdk/linux/aarch64/cpp")
endif()

add_executable(omnihand_o10_rs485_bridge_node
  src/omnihand_o10_rs485_bridge_node.cpp
  src/omnihand_o10_rs485_bridge_node_main.cpp
)
target_include_directories(omnihand_o10_rs485_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OMNIHAND_SDK_ROOT}/include
)
target_link_directories(omnihand_o10_rs485_bridge_node PRIVATE
  ${OMNIHAND_SDK_ROOT}/lib
)
target_link_libraries(omnihand_o10_rs485_bridge_node omnihand)
set_target_properties(omnihand_o10_rs485_bridge_node PROPERTIES
  BUILD_RPATH "${OMNIHAND_SDK_ROOT}/lib"
  INSTALL_RPATH "$ORIGIN/.."
)
ament_target_dependencies(omnihand_o10_rs485_bridge_node
  rclcpp geometry_msgs sensor_msgs std_msgs io_msgs2)

add_executable(test_finger_mapping test/test_finger_mapping.cpp)
target_include_directories(test_finger_mapping PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS omnihand_o10_rs485_bridge_node test_finger_mapping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${OMNIHAND_SDK_ROOT}/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

install(PROGRAMS scripts/test_finger_cmd_debug.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
