cmake_minimum_required(VERSION 3.8)
project(attracts_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(stm32_bridge_node src/stm32_bridge_node.cpp)
ament_target_dependencies(stm32_bridge_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  stm32_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
