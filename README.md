# Homework 3 
## Multi-Language ROS 2 Mobile Robot Simulation

A ROS 2 workspace implementing a 3-DOF omnidirectional robot simulator with autonomous square navigation and manual joystick control.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)
![Python](https://img.shields.io/badge/Python-3.10-blue)

## 🎯 Features

- **🤖 Autonomous Navigation**: State machine-based controller with proportional control for precise 2m × 2m square path following
- **🎮 Manual Teleoperation**: Full 3-DOF joystick control (forward/backward, strafe, rotation, vertical)
- **📡 Real-time Odometry**: C++ odometry node with TF2 transform broadcasting
- **🔧 Custom Service**: Reset position service for instant robot repositioning
- **📊 Visualization**: Pre-configured RViz setup with TF display and trajectory tracking
- **🔄 Closed-loop Control**: Uses TF2 pose feedback for accurate waypoint navigation


## 🚀 Quick Start

### Prerequisites

- **ROS 2 Humble** (or compatible distro)
- **Ubuntu 22.04** (recommended) or **Microsoft WSL**
- **Build tools**: `colcon`, `rosdep`

### Installation

```bash
# 1. Install dependencies
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep2 \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-teleop-twist-joy \
  ros-humble-joy

# 2. Initialize rosdep (if not already done)
sudo rosdep init
rosdep update

# 3. Clone and navigate to workspace
cd /home/rex/Documents/ROBE313/ros2_ws_herter_mcallister_ward

# 4. Install package dependencies
rosdep install --from-paths src --ignore-src -r -y

# 5. Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 6. Source workspace
source install/setup.bash
```

## 🎮 Usage

### Autonomous Square Navigation

Launch the autonomous controller to watch the robot drive a precise 2m × 2m square:

```bash
ros2 launch robot_bringup robot_simulation.launch.py
```

**What's happening:**
- Robot starts at origin and navigates through 4 waypoints
- Uses proportional control for smooth acceleration/deceleration
- Turns 90° at each corner
- Continuously loops the square path

**Key parameters** (in `controller_node.py`):
- `side_length = 2.0` - Size of square (meters)
- `kp_linear = 0.5` - Linear velocity gain
- `kp_angular = 2.0` - Angular velocity gain
- `position_tolerance = 0.02` - Waypoint precision (2cm)
- `angle_tolerance = 0.02` - Turn precision (~1.15°)

### Manual Joystick Control

Launch teleoperation mode for manual driving:

```bash
ros2 launch robot_bringup teleop.launch.py
```

**Controls** (Xbox/PlayStation controller):

| Input | Action |
|-------|--------|
| Left Stick ↑↓ | Forward / Backward |
| Left Stick ←→ | Strafe Left / Right |
| Right Stick ←→ | Rotate Left / Right |
| Y Button | **Hold to Enable** |

> ⚠️ **Safety**: You must hold the Y button to enable movement

**No physical controller?** Use a software joystick emulator:

```bash
# Install AntiMicroX
sudo apt install antimicrox

# Launch and map keyboard keys to joystick axes
antimicrox
```

### Testing Without Hardware

```bash
# View available topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Watch TF transforms
ros2 run tf2_ros tf2_echo odom base_link

# Call reset service (teleport robot)
ros2 service call /reset_position custom_interfaces/srv/ResetPosition \
  "{pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## 🔧 Development

### Modifying Python Code

Thanks to `--symlink-install`, Python changes are instant:

```bash
# 1. Edit controller_node.py
# 2. Save file
# 3. Restart launch (Ctrl+C, then relaunch)
ros2 launch robot_bringup robot_simulation.launch.py
```

**No rebuild needed!**

### Modifying C++ Code

C++ requires rebuilding:

```bash
# Edit odometry_node.cpp, then:
colcon build --packages-select robot_simulator_cpp
source install/setup.bash
ros2 launch robot_bringup robot_simulation.launch.py
```

### Adjusting Square Navigation

Edit `src/robot_simulator_py/robot_simulator_py/controller_node.py`:

```python
# Make a bigger square
self.side_length = 5.0  # 5m × 5m

# Go faster
self.max_linear_speed = 1.0
self.kp_linear = 0.8

# Higher precision
self.position_tolerance = 0.01  # 1cm tolerance
```

Save and relaunch (no rebuild needed).

### Tuning Joystick Sensitivity

Edit `src/robot_bringup/launch/teleop.launch.py`:

```python
'scale_linear.x': 1.0,    # Faster forward/back (was 0.5)
'scale_linear.y': 1.0,    # Faster strafe (was 0.5)
'scale_angular.yaw': 2.0, # Faster rotation (was 1.0)
```

Relaunch to apply (no rebuild needed).

## 📚 Technical Details

### Proportional Control

**Linear velocity:**
```
v = kp_linear × distance_to_target
v = clamp(v, min_speed, max_speed)
```

**Angular velocity:**
```
ω = kp_angular × angle_error
ω = clamp(ω, -max_angular, max_angular)
```

**Heading correction** (while driving):
```
desired_heading = atan2(Δy, Δx)
ω_correction = kp_angular × (desired_heading - current_heading)
```

### State Machine

```
   ┌─────────┐  distance < tolerance    ┌──────┐
   │ FORWARD │─────────────────────────>│ TURN │
   └─────────┘                          └──────┘
        ^                                   |
        |      angle_error < tolerance      |
        └───────────────────────────────────┘
```



### Wrong joystick axes

Find correct axis indices:

```bash
jstest /dev/input/js0
# Move sticks and note which axes[] values change
# Update teleop.launch.py with correct indices
```

### Build errors

```bash
# Clean build
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## 👥 Authors

- Jordyn Herter
- Rex McAllister  
- Ava Ward

---

**ROBE313 - Robot Operating Systems**  
*November 2025*