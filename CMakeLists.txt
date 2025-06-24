cmake_minimum_required(VERSION 3.8)
project(autonomous_explorer)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# Explorer node
add_executable(explorer_node src/explorer_node.cpp src/slam_processor.cpp src/navigation_controller.cpp)
ament_target_dependencies(explorer_node rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros)

install(TARGETS explorer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch worlds urdf DESTINATION share/${PROJECT_NAME})

ament_package()