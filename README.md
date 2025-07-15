# Autonomous Explorer 🤖

A ROS2-based autonomous robot navigation system that enables a TurtleBot3 to explore environments while avoiding obstacles intelligently.

## Features ✨

- **Smart Obstacle Avoidance**: Uses laser scan data to detect and avoid obstacles
- **Lane Width Analysis**: Measures available space before turning to ensure safe navigation
- **Dynamic Path Planning**: Chooses the optimal direction based on available space
- **Real-time Navigation**: Continuous movement with 10Hz update rate
- **Custom Environment Support**: Works with custom Gazebo worlds

## Project Structure 📁

```
auto_explore_ws/
├── package.xml
├── CMakeLists.txt
├── src/
│   ├── explorer_node.cpp
├── include/
│   └── autonomous_explorer/
│       ├── explorer_node.hpp
├── launch/
│   └── explorer_launch.py
├── worlds/
└── README.md
```

## How It Works 🧠

The `explorer_node.cpp` implements an intelligent navigation algorithm:

1. **Laser Scan Processing**: Analyzes 360° laser data to detect obstacles in front, left, and right directions
2. **Safety Zones**: Maintains minimum distances (50cm) and lane widths (80cm) for safe navigation
3. **Decision Making**: 
   - If front is clear → Move forward at normal speed
   - If obstacle detected → Analyze left/right lanes
   - Choose the wider, safer lane for turning
   - If no safe lanes → Rotate to find better options
4. **Smooth Movement**: Reduced speeds for stable navigation (0.3 m/s linear, 0.3 rad/s angular)

## Key Components 🔧

- **Laser Callback**: Processes sensor data and updates robot state
- **Movement Controller**: Publishes velocity commands based on sensor analysis
- **Safety Checks**: Validates path clearance before movement
- **Lane Width Measurement**: Ensures sufficient space for safe turning

## Getting Started 🚀

1. **Build the package**:
   ```bash
   cd ~/auto_explore_ws
   colcon build --packages-select autonomous_explorer
   source install/setup.bash
   ```

2. **Launch with custom world**:(You can change your custom map here)
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py
   ```

3. **Launch the file**:
   ```bash
   ros2 launch autonomous_explorer explorer_launch.py
   ```

## Custom World Setup 🌍

The project includes setup instructions for creating custom Gazebo environments:
- Import floor plans and adjust scale
- Add walls, doors, and furniture
- Configure TurtleBot3 spawn position
- Test navigation in realistic environments

## Dependencies 📦

- ROS2 Humble
- TurtleBot3 Gazebo packages
- geometry_msgs
- sensor_msgs

## Safety Features 🛡️

- Obstacle detection with configurable minimum distances
- Lane width validation before turning
- Error handling for sensor data processing
- Emergency stop capability

---
