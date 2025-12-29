cmake_minimum_required(VERSION 3.8)
project(segwayrmp)

# C++17 필수 (Humble 이상)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 경고 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(segway_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# include 및 외부 라이브러리 설정
include_directories(include)

set(EXTERNAL_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/include)
message("-- Segway Robotics Notice: External include path: ${EXTERNAL_INCLUDE_PATH}")

set(EXTERNAL_LIB_PATH ${CMAKE_SOURCE_DIR}/lib)
message("-- Segway Robotics Notice: External lib path: ${EXTERNAL_LIB_PATH}")

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64")
  message("-- Segway Robotics Notice: Detected CPU architecture: aarch64")
  set(EXTERNAL_LIB ${EXTERNAL_LIB_PATH}/libctrl_arm64-v8a.so)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
  message("-- Segway Robotics Notice: Detected CPU architecture: x86_64")
  set(EXTERNAL_LIB ${EXTERNAL_LIB_PATH}/libctrl_x86_64.so)
else()
  message(FATAL_ERROR "-- Segway Robotics Notice: Only x86_64 and aarch64 architectures are supported!")
endif()

message("-- Segway Robotics Notice: External library: ${EXTERNAL_LIB}")

# robot core library
add_library(robot_lib STATIC
  src/robot.cpp
)
set_target_properties(robot_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(robot_lib PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_lib ${EXTERNAL_LIB})
set_target_properties(robot_lib PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH ${EXTERNAL_LIB_PATH}
)
ament_target_dependencies(robot_lib
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  segway_msgs
)

# Component Node
add_library(rmp401_driver_component SHARED
  src/rmp401_driver_node.cpp
)
target_link_libraries(rmp401_driver_component robot_lib)
set_target_properties(rmp401_driver_component PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH ${EXTERNAL_LIB_PATH}
)
ament_target_dependencies(rmp401_driver_component
  rclcpp
  rclcpp_components
)
rclcpp_components_register_nodes(rmp401_driver_component "segwayrmp::Rmp401DriverNode")

# Standalone Executable
add_executable(rmp401_driver_node
  src/main.cpp
)
target_link_libraries(rmp401_driver_node
  rmp401_driver_component
)
ament_target_dependencies(rmp401_driver_node
  rclcpp
)

# install targets
install(TARGETS
  robot_lib
  rmp401_driver_component
  rmp401_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
