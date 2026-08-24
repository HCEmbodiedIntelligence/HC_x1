cmake_minimum_required(VERSION 3.8)
project(hc_tj_marvin_bridge_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(HC_X1_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
if(NOT DEFINED MARVIN_SDK_PATH)
  if(DEFINED ENV{MARVIN_SDK_PATH} AND NOT "$ENV{MARVIN_SDK_PATH}" STREQUAL "")
    set(MARVIN_SDK_PATH "$ENV{MARVIN_SDK_PATH}")
  else()
    set(MARVIN_SDK_PATH "${HC_X1_ROOT}/vendor/marvin")
  endif()
endif()

set(MARVIN_CONTRL_SDK_DIR "${MARVIN_SDK_PATH}/contrlSDK")
set(MARVIN_KINEMATICS_SDK_DIR "${MARVIN_SDK_PATH}/kinematicsSDK")
if(NOT EXISTS "${MARVIN_CONTRL_SDK_DIR}/MarvinSDK.h")
  message(FATAL_ERROR "Marvin SDK not found at ${MARVIN_CONTRL_SDK_DIR}")
endif()

if(NOT EXISTS "${MARVIN_CONTRL_SDK_DIR}/libMarvinSDK.so")
  message(STATUS "libMarvinSDK.so not found, building Marvin SDK...")
  execute_process(
    COMMAND make
    WORKING_DIRECTORY "${MARVIN_CONTRL_SDK_DIR}"
    RESULT_VARIABLE MARVIN_BUILD_RESULT
  )
  if(NOT MARVIN_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to build Marvin SDK in ${MARVIN_CONTRL_SDK_DIR}")
  endif()
endif()

if(NOT EXISTS "${MARVIN_KINEMATICS_SDK_DIR}/FxRobot.h" OR
   NOT EXISTS "${MARVIN_KINEMATICS_SDK_DIR}/libKine.so")
  message(FATAL_ERROR "Marvin kinematics SDK not found at ${MARVIN_KINEMATICS_SDK_DIR}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# redis-plus-plus 装在 /usr/local，没有导出 CMake 包配置，直接找库
find_library(HIREDIS_LIB hiredis REQUIRED)
find_library(REDIS_PLUS_PLUS_LIB redis++ REQUIRED)
find_path(REDIS_PLUS_PLUS_INCLUDE sw/redis++/redis++.h REQUIRED)

set(HC_TJ_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MARVIN_CONTRL_SDK_DIR}
  ${MARVIN_KINEMATICS_SDK_DIR}
  ${REDIS_PLUS_PLUS_INCLUDE}
)

add_library(hc_tj_marvin_bridge_core STATIC
  src/hc_tj_marvin_bridge_node.cpp
)
target_include_directories(hc_tj_marvin_bridge_core PUBLIC ${HC_TJ_INCLUDES})
target_compile_definitions(hc_tj_marvin_bridge_core PRIVATE CMPL_LIN)
ament_target_dependencies(hc_tj_marvin_bridge_core
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  ament_index_cpp
)
target_link_libraries(hc_tj_marvin_bridge_core
  yaml-cpp
  nlohmann_json::nlohmann_json
  ${REDIS_PLUS_PLUS_LIB}
  ${HIREDIS_LIB}
  ${MARVIN_CONTRL_SDK_DIR}/libMarvinSDK.so
  ${MARVIN_KINEMATICS_SDK_DIR}/libKine.so
  pthread
  rt
)

add_executable(hc_tj_marvin_bridge_node
  src/hc_tj_marvin_bridge_node_main.cpp
)
target_link_libraries(hc_tj_marvin_bridge_node hc_tj_marvin_bridge_core)
set_target_properties(hc_tj_marvin_bridge_node PROPERTIES
  BUILD_RPATH "${MARVIN_CONTRL_SDK_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)

install(TARGETS hc_tj_marvin_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${MARVIN_CONTRL_SDK_DIR}/libMarvinSDK.so
  ${MARVIN_KINEMATICS_SDK_DIR}/libKine.so
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY ../config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY ../launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
