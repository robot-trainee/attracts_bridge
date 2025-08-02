cmake_minimum_required(VERSION 3.8)
project(attracts_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(attracts_msgs REQUIRED)
find_package(rabcl REQUIRED)
find_package(rabcl_ros2 REQUIRED)

include_directories(include)

add_executable(stm32_bridge_node
  src/stm32_bridge_node.cpp
)
ament_target_dependencies(stm32_bridge_node
  rclcpp
  attracts_msgs
  rabcl
  rabcl_ros2
)

add_executable(transceiver_module_bridge_node
  src/transceiver_module_bridge_node.cpp
)
ament_target_dependencies(transceiver_module_bridge_node
  rclcpp
  attracts_msgs
)

install(TARGETS
  stm32_bridge_node
  transceiver_module_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
