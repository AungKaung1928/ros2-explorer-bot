# Autonomous Explorer (Lifecycle Node Edition)

ROS2-based autonomous robot navigation system with lifecycle node architecture, YAML configuration, and intelligent obstacle avoidance for safe exploration.

## Features

- **Lifecycle Node Architecture**: Managed state transitions (configure → activate → deactivate → cleanup)
- **YAML Parameter Configuration**: All tuning parameters externalized to config file
- **Smart Obstacle Avoidance**: Laser scan-based detection with configurable safety zones
- **Lane Width Analysis**: Measures available space before turning to ensure safe navigation
- **Dynamic Path Planning**: Chooses optimal direction based on available clearance
- **Configurable Control Loop**: Adjustable update rate and behavior tuning

## Quick Start

### Installation

```bash
cd ~/auto_explore_ws
colcon build --packages-select autonomous_explorer
source install/setup.bash
```

### Run with TurtleBot3 Simulation

**Terminal 1 - Launch Gazebo:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Start Explorer:**
```bash
cd ~/auto_explore_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch autonomous_explorer simple_explorer_launch.py
```

## Parameter Tuning

### Quick Parameter Changes (No File Editing)

```bash
# Change parameters at launch
ros2 launch autonomous_explorer simple_explorer_launch.py \
    normal_speed:=0.4 \
    turn_speed:=0.4 \
    min_distance:=0.7 \
    min_lane_width:=0.6
```

### Parameter Reference

| Parameter | Default | Range | What It Does |
|-----------|---------|-------|--------------|
| `normal_speed` | 0.3 | 0.1-0.5 | Forward movement speed (m/s) |
| `turn_speed` | 0.3 | 0.2-0.6 | Rotation speed (rad/s) |
| `min_distance` | 0.5 | 0.3-1.0 | Stop distance from obstacles (m) |
| `min_lane_width` | 0.8 | 0.5-1.2 | Minimum gap width to enter (m) |
| `control_frequency` | 10.0 | 5.0-20.0 | Command publishing rate (Hz) |

### Scan Angle Parameters (Advanced)

| Parameter | Default | Range | What It Does |
|-----------|---------|-------|--------------|
| `front_scan_start` | 340 | 320-350 | Front detection cone start (degrees) |
| `front_scan_end` | 20 | 10-40 | Front detection cone end (degrees) |
| `left_scan_start` | 60 | 45-90 | Left side detection start (degrees) |
| `left_scan_end` | 120 | 90-135 | Left side detection end (degrees) |
| `right_scan_start` | 240 | 225-270 | Right side detection start (degrees) |
| `right_scan_end` | 300 | 270-315 | Right side detection end (degrees) |
| `left_wide_start` | 45 | 30-60 | Left lane width measurement start |
| `left_wide_end` | 135 | 120-150 | Left lane width measurement end |
| `right_wide_start` | 225 | 210-240 | Right lane width measurement start |
| `right_wide_end` | 315 | 300-330 | Right lane width measurement end |

### Common Tuning Scenarios

**Robot moving too fast:**
```bash
ros2 launch autonomous_explorer simple_explorer_launch.py \
    normal_speed:=0.2 \
    turn_speed:=0.25
```

**Robot too cautious (not entering spaces):**
```bash
ros2 launch autonomous_explorer simple_explorer_launch.py \
    min_distance:=0.4 \
    min_lane_width:=0.6
```

**Robot hitting obstacles:**
```bash
ros2 launch autonomous_explorer simple_explorer_launch.py \
    min_distance:=0.7 \
    min_lane_width:=0.9
```

**Getting stuck in narrow corridors:**
```bash
ros2 launch autonomous_explorer simple_explorer_launch.py \
    min_lane_width:=0.6 \
    turn_speed:=0.4
```

**Slow, jerky turning:**
```bash
ros2 launch autonomous_explorer simple_explorer_launch.py \
    turn_speed:=0.5
```

**For tight indoor environments:**
```bash
ros2 launch autonomous_explorer simple_explorer_launch.py \
    normal_speed:=0.2 \
    min_lane_width:=0.6 \
    min_distance:=0.4
```

**For open warehouse spaces:**
```bash
ros2 launch autonomous_explorer simple_explorer_launch.py \
    normal_speed:=0.4 \
    min_distance:=0.6 \
    turn_speed:=0.5
```

**More responsive control (higher frequency):**
```bash
ros2 launch autonomous_explorer simple_explorer_launch.py \
    control_frequency:=15.0
```

### Change Parameters While Running

```bash
# List all parameters
ros2 param list /explorer

# Change speed on-the-fly (requires node restart to take effect)
ros2 param set /explorer normal_speed 0.35

# Check current value
ros2 param get /explorer min_distance
```

## Lifecycle Management

### Automatic State Transitions

The node automatically transitions through states on launch:
1. **Unconfigured** → `configure()` → **Inactive**
2. **Inactive** → `activate()` → **Active** (robot starts moving)

### Manual Lifecycle Control (Optional)

```bash
# Check current state
ros2 lifecycle get /explorer

# Manual state transitions
ros2 lifecycle set /explorer configure
ros2 lifecycle set /explorer activate
ros2 lifecycle set /explorer deactivate  # Stops robot
ros2 lifecycle set /explorer cleanup
```

## Monitoring

```bash
# Watch robot velocity commands
ros2 topic echo /cmd_vel

# Monitor laser scan data
ros2 topic echo /scan

# Check lifecycle state
ros2 lifecycle get /explorer

# View current parameters
ros2 param list /explorer
```

## How It Works

1. **Laser Scan Processing**: Analyzes 360° laser data using configurable angle ranges
2. **Safety Evaluation**: Compares distances to `min_distance` threshold in front/left/right sectors
3. **Lane Width Measurement**: Uses wider scan ranges to measure available turning space
4. **Decision Logic**:
   - **Front clear** → Move forward at `normal_speed`
   - **Obstacle detected** → Stop, evaluate left/right lanes
   - **Both lanes safe** → Choose wider lane (must exceed `min_lane_width`)
   - **One lane safe** → Turn toward safe side
   - **No safe lanes** → Rotate in place to find opening
5. **Command Publishing**: Publishes velocity commands at `control_frequency` rate

## Troubleshooting

### Robot doesn't move

```bash
# Check laser data is arriving
ros2 topic hz /scan

# Verify lifecycle state is active
ros2 lifecycle get /explorer
# Should show: active [id=3]

# Manually activate if needed
ros2 lifecycle set /explorer activate
```

### Robot keeps spinning / won't go forward

```bash
# Reduce safety thresholds
ros2 launch autonomous_explorer simple_explorer_launch.py \
    min_distance:=0.4 \
    min_lane_width:=0.6
```

### Robot enters too-narrow spaces

```bash
# Increase lane width requirement
ros2 launch autonomous_explorer simple_explorer_launch.py \
    min_lane_width:=1.0
```

### Robot movements too jerky

```bash
# Reduce speeds for smoother motion
ros2 launch autonomous_explorer simple_explorer_launch.py \
    normal_speed:=0.2 \
    turn_speed:=0.25
```

### Gazebo crashes on startup

```bash
# Kill existing Gazebo processes
killall -9 gazebo gzserver gzclient

# Clear Gazebo temp files
rm -rf /tmp/.gazebo-*

# Relaunch
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### "Address already in use" error

You launched Gazebo twice. Kill all Gazebo processes:
```bash
killall -9 gzserver gzclient
```

## Make Changes Permanent

Edit the config file for permanent parameter changes:

```bash
nano ~/auto_explore_ws/src/autonomous_explorer/config/explorer_params.yaml
```

Then rebuild:
```bash
cd ~/auto_explore_ws
colcon build --packages-select autonomous_explorer
source install/setup.bash
```

## Project Structure

```
autonomous_explorer/
├── package.xml                     # ROS2 package dependencies
├── CMakeLists.txt                  # Build configuration
├── config/
│   └── explorer_params.yaml        # Parameter configuration
├── src/
│   └── explorer_node.cpp           # Lifecycle node implementation
├── include/
│   └── autonomous_explorer/
│       └── explorer_node.hpp       # Header with lifecycle callbacks
├── launch/
│   └── simple_explorer_launch.py   # Launch file with parameter loading
└── README.md
```

## Advanced: Testing Different Configurations

Create test config files without modifying the original:

```bash
# Copy default config
cp config/explorer_params.yaml config/test_cautious.yaml

# Edit test_cautious.yaml:
# - Set min_distance to 1.0
# - Set min_lane_width to 1.0
# - Set normal_speed to 0.2

# Launch with test config
ros2 launch autonomous_explorer simple_explorer_launch.py \
    params_file:=config/test_cautious.yaml
```

## Dependencies

- ROS2 Humble
- TurtleBot3 Gazebo packages (`sudo apt install ros-humble-turtlebot3-gazebo`)
- rclcpp_lifecycle
- lifecycle_msgs
- geometry_msgs
- sensor_msgs
