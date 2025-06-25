# ROS2 Explorer Bot 🤖

A simple autonomous robot that explores unknown environments using SLAM in ROS2 + Gazebo.

## Features
- **Autonomous Navigation** - Obstacle avoidance with laser scanning
- **Real-time SLAM** - Builds occupancy grid maps while exploring
- **Simple & Educational** - Clean C++17 code for learning robotics

📁 Package Structure
```
autonomous_explorer/
├── package.xml
├── CMakeLists.txt
├── src/
│   ├── explorer_node.cpp
│   ├── slam_processor.cpp
│   └── navigation_controller.cpp
├── include/
│   └── autonomous_explorer/
│       ├── explorer_node.hpp
│       ├── slam_processor.hpp
│       └── navigation_controller.hpp
├── launch/
│   ├── gazebo_world.launch.py
│   ├── slam_launch.py
│   └── explorer_launch.py
├── worlds/
│   └── maze_world.world
├── urdf/
│   └── explorer_robot.urdf
└── README.md
```

## Quick Start

```bash
# Clone and build
git clone https://github.com/yourusername/ros2-explorer-bot.git
cd ros2-explorer-bot
colcon build --packages-select autonomous_explorer

# Install dependencies
sudo apt install ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs

# Run the explorer
source install/setup.bash
ros2 launch autonomous_explorer explorer_launch.py
```

## What it does
1. Spawns a robot in Gazebo with laser scanner
2. Robot moves autonomously avoiding obstacles  
3. Creates a map using SLAM as it explores
4. Publishes map data to `/map` topic

## Visualize
Open RViz and add:
- **LaserScan** topic: `/scan`
- **Map** topic: `/map` 
- Set Fixed Frame to `map`

## Tech Stack
- **ROS2 Humble** - Robot framework
- **Gazebo Classic** - Physics simulation
- **C++17** - Modern C++ code
- **SLAM Toolbox** - Mapping algorithm

## Project Structure
```
autonomous_explorer/
├── package.xml
├── CMakeLists.txt
├── src/
│   ├── explorer_node.cpp
│   ├── slam_processor.cpp
│   └── navigation_controller.cpp
├── include/
│   └── autonomous_explorer/
│       ├── explorer_node.hpp
│       ├── slam_processor.hpp
│       └── navigation_controller.hpp
├── launch/
│   ├── gazebo_world.launch.py
│   ├── slam_launch.py
│   └── explorer_launch.py
├── worlds/
│   └── maze_world.world
├── urdf/
│   └── explorer_robot.urdf
└── README.md
```

## Learning Goals
Perfect for understanding:
- ROS2 node communication
- Sensor data processing
- Basic autonomous navigation
- SLAM concepts
- Launch file systems

---
**Requirements:** Ubuntu 22.04 + ROS2 Humble + Gazebo Classic
