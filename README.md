# Final Project: Autonomous Warehouse Inspection Robot

ROS 2 implementation for ROBE 313 Fall 2025 final project by Luis Escobar and Giacomo Marani.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5-green)

## 🎯 Features
- **🔄 Differential Drive**: ẋ=vcosθ, ẏ=vsinθ, θ̇=ω from `/cmd_vel`
- **🎯 ArUco Detection**: Camera search (60s max)
- **🧠 Mission FSM**: Full sequence handling
- **📸 Image Analysis**: Motion + feature counting
- **⚡ TF**: `camera_link` → `base_link`
- **📡 Interface**: `/missions` → `/robot_status` + `/robot_report`

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
cd /home/user/Documents/ROBE313/ros2_ws_herter_mcallister_ward

# 4. Install package dependencies
rosdep install --from-paths src --ignore-src -r -y

# 5. Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 6. Source workspace
source install/setup.bash
```
## 🚀 Launching the Student System

To run the autonomous warehouse inspection system, launch the student-developed
nodes using the final mission launch file:

```bash
ros2 launch robot_bringup final_mission.launch.py
```

## 🧠 System Architecture & State Machine Design

---
The autonomous warehouse inspection robot is implemented using a **modular ROS 2
architecture**, where each major responsibility is handled by a dedicated node.
A centralized **mission control state machine** coordinates robot behavior in
response to commands from the external Evaluator.


### 🏗 Overall System Architecture

The system is composed of the following core components:

#### 1. Mission Control Node (Python)
- Acts as the **central decision-making unit**
- Subscribes to `/missions` for evaluator commands
- Publishes robot state updates to `/robot_status`
- Publishes mission results to `/robot_report`
- Transitions between mission states based on:
  - Evaluator commands
  - Sensor feedback
  - Task completion conditions

#### 2. Odometry Node (C++)
- Simulates **differential-drive kinematics**
- Integrates robot pose `[x, y, θ]` from `/cmd_vel`
- Publishes odometry used for navigation and docking
- Does **not** rely on global ground truth

#### 3. Perception Modules (Python)
- **ArUco Detection**
  - Processes camera images to identify marker IDs and relative positions
- **Image Analysis**
  - Motion detection between two images (`image_analysis`)
  - Feature detection and counting (`image_analysis2`, bonus)

#### 4. Navigation Controller
- Generates velocity commands on `/cmd_vel`
- Uses odometry feedback to:
  - Navigate toward detected ArUco markers
  - Perform precision docking
  - Return to the origin

#### 5. TF System
- Maintains a consistent transform tree
- Includes a required `camera_link` frame attached to `base_link`
- Enables correct interpretation of visual data in the robot frame


## 🔧 Development

### Modifying Python Code

Thanks to `--symlink-install`, Python changes are instant:

```bash
# 1. Edit controller_node.py
# 2. Save file
# 3. Restart launch (Ctrl+C, then relaunch)
ros2 launch robot_bringup final_mission.launch.py
```

**No rebuild needed!**

### Modifying C++ Code

C++ requires rebuilding:

```bash
# Edit odometry_node.cpp, then:
colcon build --packages-select robot_simulator_cpp
source install/setup.bash
ros2 launch robot_bringup final_mission.launch.py
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
*December 2025*
