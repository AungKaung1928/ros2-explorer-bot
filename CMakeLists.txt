cmake_minimum_required(VERSION 3.8)
project(autonomous_explorer)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Explorer node - simplified without SLAM dependencies
add_executable(explorer_node src/explorer_node.cpp)
ament_target_dependencies(explorer_node 
  rclcpp 
  geometry_msgs 
  sensor_msgs
)

ament_package()