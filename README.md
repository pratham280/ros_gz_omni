# ROS 2 Gazebo Omnidirectional Drive Simulation

A comprehensive robotics simulation project featuring omnidirectional (omni) wheel drive systems integrated with ROS 2 and Gazebo Harmonic. This project includes multiple robot models (omni drive, mecanum drive, and differential drive) with custom Gazebo plugins for kinematics and dynamics simulation.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Included Packages](#included-packages)
3. [System Requirements & Dependencies](#system-requirements--dependencies)
4. [Installation & Setup](#installation--setup)
5. [Launch Files & Nodes](#launch-files--nodes)
6. [ROS Topics & Communication](#ros-topics--communication)
7. [Gazebo Omni Drive Plugin](#gazebo-omni-drive-plugin)
8. [Robot Models & Design Considerations](#robot-models--design-considerations)
9. [Running the Simulation](#running-the-simulation)
10. [Visualization with RViz](#visualization-with-rviz)
11. [Joystick Teleoperation](#joystick-teleoperation)
12. [Troubleshooting](#troubleshooting)
13. [Miscellaneous Models](#miscellaneous-models)

---

## Project Overview

This project provides a complete simulation environment for testing omnidirectional mobile robots using ROS 2 and Gazebo. The primary focus is on the **OmniDrive** system, which uses four omnidirectional wheels capable of simultaneous linear and angular motion. The project also includes support for mecanum-drive and differential-drive robots for comparative studies.

### Key Features

- **Custom Gazebo Plugin System**: OmniDrive, BasicSystem, and FullSystem plugins
- **Multi-Wheel Support**: Four independently controlled wheels with omnidirectional motion
- **Odometry Publishing**: Real-time odometry data with TF transforms
- **ROS 2 Integration**: Full integration with ROS 2 ecosystem (ros_gz bridge)
- **Joystick Control**: Teleoperator interface for manual robot control
- **RViz Visualization**: Real-time visualization of robot state and sensor data

---

## Included Packages

### 1. `ros_gz_omni_description`
**Purpose**: Robot model definitions and assets

- Contains SDF (SDFormat) model files for robot descriptions
- Stores 3D mesh assets (STL files) for visual and collision geometry
- Includes model configurations and URDF/SDF conversions
- **Directory**: `models/`
  - `omni_drive/`: Primary omnidirectional robot model
  - `diff_drive/`: Differential drive robot (two-wheel control)
  - `mecanum_drive/`: Mecanum wheel robot (four-wheel with slip control)
  - `rrbot/`: Revolute-revolute robot (articulated arm)
  - `old_version/`: Previous iterations of omni drive models for reference

### 2. `ros_gz_omni_gazebo`
**Purpose**: Gazebo-specific implementations and plugins

- **Plugins** (C++ compiled to shared libraries):
  - `OmniDrive`: Omnidirectional wheel controller plugin
  - `BasicSystem`: Basic robot dynamics system
  - `FullSystem`: Advanced system with complete physics
- **Worlds**: SDF world definitions with physics and environment setup
  - `omni_drive.sdf`: Omnidirectional robot world
  - `mecanum_drive.sdf`: Mecanum-drive world
  - `diff_drive.sdf`: Differential-drive world
- **Header Files**: Plugin interfaces and kinematic math utilities

### 3. `ros_gz_omni_bringup`
**Purpose**: Launch files and configuration utilities

- **Launch Files** (Python-based):
  - `omni_drive.launch.py`: Main omnidirectional robot launch
  - `mecanum_drive.launch.py`: Mecanum drive robot launch
  - `diff_drive.launch.py`: Differential drive robot launch
  - `rrbot_setup.launch.py`: Articulated arm setup
- **Configuration Files** (YAML):
  - `ros_gz_omni_bridge.yaml`: ROS ↔ Gazebo topic mapping
  - `teleop_twist_joy.yaml`: Joystick teleoperator settings
  - `rrbot.rviz`, `omni_drive.rviz`, `mecanum_drive.rviz`: RViz layouts

### 4. `ros_gz_omni_application`
**Purpose**: ROS 2-specific application code

- Custom ROS 2 nodes (planned/extensible)
- Application-level logic separate from simulation

---

## System Requirements & Dependencies

### ROS 2 & Gazebo Compatibility

This project targets **ROS 2 Jazzy** with **Gazebo Harmonic** (gz-sim8).

| Component | Version | Note |
|-----------|---------|------|
| ROS 2 | Jazzy | LTS support recommended |
| Gazebo | Harmonic (gz-sim8) | Main branch default |
| Ubuntu | 24.04 LTS | Recommended for Jazzy |

For **Gazebo Fortress**, switch to the `fortress` branch.

### System Dependencies

```bash
sudo apt update && sudo apt install -y \
  python3-vcstool \
  python3-colcon-common-extensions \
  git \
  wget \
  build-essential \
  cmake
```

### ROS 2 Core Dependencies

| Package | Purpose |
|---------|---------|
| `ros_gz` | ROS ↔ Gazebo bridge interface |
| `sdformat_urdf` | URDF ↔ SDFormat conversion |
| `robot_state_publisher` | Publish robot kinematics to TF |
| `rviz2` | 3D visualization |
| `joy` | Joystick input handling |
| `teleop_twist_joy` | Joystick-to-Twist converter |
| `ros_gz_sim` | ROS 2 Gazebo integration |
| `ros_gz_bridge` | Topic/service bridge |

### Gazebo Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| `gz-sim` | 8.x | Physics simulator |
| `gz-plugin` | 2.x | Plugin system |
| `gz-common` | 5.x | Common utilities |
| `gz-cmake` | 3.x | CMake helpers |
| `gz-transport` | - | Message transport |

---

## Installation & Setup

### Step 1: Create ROS 2 Workspace

```bash
mkdir -p ~/ros_gz_ws/src
cd ~/ros_gz_ws
```

### Step 2: Clone Repository

```bash
cd src
git clone https://github.com/gazebosim/ros_gz_omni.git
# Or clone your fork
cd ~/ros_gz_ws
```

### Step 3: Install ROS 2 Jazzy

Follow the [official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html).

```bash
source /opt/ros/jazzy/setup.bash
```

### Step 4: Initialize ROS Dependencies

```bash
cd ~/ros_gz_ws
sudo rosdep init  # Only if first time
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro jazzy
```

### Step 5: Build Workspace

```bash
cd ~/ros_gz_ws
colcon build --cmake-args -DBUILD_TESTING=ON
```

**Output**: Built packages appear in `install/` directory

### Step 6: Source Environment

Add to your shell profile (`~/.bashrc` or `~/.zshrc`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_gz_ws/install/setup.bash
export GZ_VERSION=harmonic  # Optional: explicit version
```

Or source for current session:

```bash
source ~/ros_gz_ws/install/setup.bash
```

### Verification

```bash
# Check if packages are available
ros2 pkg list | grep ros_gz_omni

# Expected output:
# ros_gz_omni_application
# ros_gz_omni_bringup
# ros_gz_omni_description
# ros_gz_omni_gazebo
```

---

## Launch Files & Nodes

### Main Launch: OmniDrive (`omni_drive.launch.py`)

**Location**: `ros_gz_omni_bringup/launch/omni_drive.launch.py`

#### Launch Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `rviz` | bool | `true` | Launch RViz visualization |
| `teleop` | bool | `true` | Enable joystick teleoperation |

#### Launched Nodes

```
Node: gz_sim (Gazebo Harmonic)
├── World: omni_drive.sdf
├── Robot: omni_drive model
└── Plugins: JointStatePublisher, PosePublisher, OdometryPublisher

Node: robot_state_publisher
├── Publishes robot kinematics to /tf
└── Parameters: use_sim_time=true, robot_description

Node: ros_gz_bridge (parameter_bridge)
├── Config: ros_gz_omni_bridge.yaml
└── Bridges ROS ↔ Gazebo topics

Node: rviz2 (if rviz:=true)
├── Config: omni_drive.rviz
└── Visualizes simulation state

Node: joy_node (if teleop:=true)
├── Device: /dev/input/js0
├── Deadzone: 0.05
└── Publish rate: 20 Hz

Node: teleop_twist_joy (if teleop:=true)
├── Config: teleop_twist_joy.yaml
├── Remaps: cmd_vel → /omni_drive/cmd_vel
└── Publishes Twist messages
```

#### Example Launch Commands

**Full simulation with RViz and joystick**:
```bash
ros2 launch ros_gz_omni_bringup omni_drive.launch.py
```

**Headless simulation (no RViz)**:
```bash
ros2 launch ros_gz_omni_bringup omni_drive.launch.py rviz:=false
```

**No joystick control**:
```bash
ros2 launch ros_gz_omni_bringup omni_drive.launch.py teleop:=false
```

**Both disabled**:
```bash
ros2 launch ros_gz_omni_bringup omni_drive.launch.py rviz:=false teleop:=false
```

### Alternative Launch Files

**Mecanum Drive** (`mecanum_drive.launch.py`):
- Similar structure, loads `mecanum_drive.sdf` world
- Four wheels with horizontal-axis rollers for lateral slip

**Differential Drive** (`diff_drive.launch.py`):
- Two independent wheels with no lateral motion capability
- Classic robot platform

---

## ROS Topics & Communication

### Topic Bridge Configuration

**File**: `ros_gz_omni_bringup/config/ros_gz_omni_bridge.yaml`

The bridge enables bidirectional communication between ROS 2 and Gazebo:

#### ROS → Gazebo (Command Topics)

| ROS Topic | Type | Gazebo Topic | Gazebo Type | Purpose |
|-----------|------|--------------|-------------|---------|
| `/omni_drive/cmd_vel` | `geometry_msgs/Twist` | `/model/omni_drive/cmd_vel` | `gz.msgs.Twist` | Velocity commands (linear + angular) |

**Message Structure** (Twist):
```
linear:
  x: forward velocity (m/s)
  y: lateral velocity (m/s)
  z: vertical velocity (ignored)
angular:
  x, y: ignored
  z: rotation rate (rad/s)
```

#### Gazebo → ROS (Sensor Topics)

| Gazebo Topic | Gazebo Type | ROS Topic | Type | Frequency | Purpose |
|--------------|-------------|-----------|------|-----------|---------|
| `/clock` | `gz.msgs.Clock` | `/clock` | `rosgraph_msgs/Clock` | 1000 Hz | Simulation time sync |
| `/model/omni_drive/odometry` | `gz.msgs.Odometry` | `/omni_drive/odometry` | `nav_msgs/Odometry` | 50 Hz | Odometry (pose + velocity) |
| `/scan` | `gz.msgs.LaserScan` | `/omni_drive/scan` | `sensor_msgs/LaserScan` | - | LIDAR data (if equipped) |
| `/world/omni_drive/model/omni_drive/joint_state` | `gz.msgs.Model` | `/joint_states` | `sensor_msgs/JointState` | - | Joint angles/velocities |
| `/model/omni_drive/pose` | `gz.msgs.Pose_V` | `/tf` | `tf2_msgs/TFMessage` | - | Transform frames (dynamic) |
| `/model/omni_drive/pose_static` | `gz.msgs.Pose_V` | `/tf_static` | `tf2_msgs/TFMessage` | 1 Hz | Static transforms (frame hierarchy) |

### Transform (TF) Frame Structure

```
world (global frame)
  └── omni_drive/odom (odometry reference)
      └── omni_drive (robot base)
          ├── chassis (main body)
          ├── wheel_FR (front-right wheel)
          ├── wheel_FL (front-left wheel)
          ├── wheel_BR (back-right wheel)
          └── wheel_BL (back-left wheel)
```

### Example Topic Communication

**Subscribe to odometry**:
```bash
ros2 topic echo /omni_drive/odometry
```

**Publish velocity command**:
```bash
# Move forward 1 m/s with 0.1 rad/s rotation
ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

# Strafe left 0.5 m/s (lateral motion)
ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place at 0.5 rad/s
ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

**View joint states**:
```bash
ros2 topic echo /joint_states
```

---

## Gazebo Omni Drive Plugin

### Plugin Architecture

The OmniDrive plugin is a custom Gazebo system plugin that implements omnidirectional kinematic control.

**File**: `ros_gz_omni_gazebo/src/OmniDrive.cc`

**Header**: `ros_gz_omni_gazebo/include/ros_gz_omni_gazebo/OmniDrive.hh`

### Plugin Interfaces

The OmniDrive class implements:

```cpp
class OmniDrive : 
  public gz::sim::System,
  public gz::sim::ISystemConfigure,    // Initialization
  public gz::sim::ISystemPreUpdate,    // Pre-physics updates
  public gz::sim::ISystemPostUpdate    // Post-physics callbacks
```

### Key Methods

#### `Configure()`
- **Called**: Once at plugin initialization
- **Tasks**:
  - Parse SDF parameters (wheel joints, dimensions)
  - Create subscribers for velocity commands
  - Initialize odometry publishers
  - Set up transport nodes

#### `PreUpdate()`
- **Called**: Before physics simulation step
- **Tasks**:
  - Update wheel velocity commands from ROS
  - Apply velocity limiters (acceleration/deceleration)
  - Set joint velocity targets

#### `PostUpdate()`
- **Called**: After physics simulation step
- **Tasks**:
  - Read wheel positions and velocities
  - Update odometry calculations
  - Publish odometry and transform messages

### Plugin Parameters (SDF)

**World file location**: `ros_gz_omni_gazebo/worlds/omni_drive.sdf`

```xml
<plugin filename="ros_gz_omni_gazebo::OmniDrive"
        name="ros_gz_omni_gazebo::OmniDrive">
  <!-- Joint names for wheel control -->
  <front_left_joint>wheel_FL</front_left_joint>
  <front_right_joint>wheel_FR</front_right_joint>
  <back_left_joint>wheel_BL</back_left_joint>
  <back_right_joint>wheel_BR</back_right_joint>
  
  <!-- Geometric parameters (meters) -->
  <wheelbase>0.148492</wheelbase>  <!-- Front-to-back wheel distance -->
  <wheel_separation>0.148492</wheel_separation>  <!-- Left-to-right distance -->
  <wheel_radius>0.0525</wheel_radius>  <!-- Wheel radius -->
  
  <!-- Control parameters -->
  <odom_publish_frequency>50</odom_publish_frequency>  <!-- Hz -->
  
  <!-- Topic names -->
  <topic>/model/omni_drive/cmd_vel</topic>
  <odom_topic>/model/omni_drive/odometry</odom_topic>
  <tf_topic>/model/omni_drive/pose</tf_topic>
  
  <!-- TF frame names -->
  <frame_id>omni_drive/odom</frame_id>
  <child_frame_id>omni_drive</child_frame_id>
</plugin>
```

### OmniMath Library

**File**: `ros_gz_omni_gazebo/src/math/OmniMath.cc`

**Header**: `ros_gz_omni_gazebo/include/ros_gz_omni_gazebo/OmniMath.hh`

Computes forward/inverse kinematics for omnidirectional wheels:

```cpp
class OmniMath {
  // Initialize odometry at startup
  void Init(const clock::time_point &_time);
  
  // Update with new wheel positions
  bool Update(
    const gz::math::Angle &_frontLeftPos,
    const gz::math::Angle &_frontRightPos,
    const gz::math::Angle &_backLeftPos,
    const gz::math::Angle &_backRightPos,
    const clock::time_point &_time);
  
  // Get computed odometry values
  double X() const;              // X position
  double Y() const;              // Y position
  double LinearVelocity() const;   // Forward velocity
  double LateralVelocity() const;  // Strafe velocity
  gz::math::Angle Heading() const;           // Robot heading
  gz::math::Angle AngularVelocity() const;   // Rotation rate
};
```

#### Kinematic Equations

For a four-wheel omnidirectional platform with wheels at 45° angles:

**Forward Kinematics** (wheel velocities → robot velocity):
```
v_x = (ω_FR + ω_FL + ω_BR + ω_BL) / 4 * r
v_y = (-ω_FR + ω_FL + ω_BR - ω_BL) / 4 * r
ω_z = (-ω_FR - ω_FL + ω_BR + ω_BL) / (4 * L) * r
```

**Inverse Kinematics** (robot velocity → wheel commands):
```
ω_FR = (v_x - v_y - L·ω_z) / r
ω_FL = (v_x + v_y + L·ω_z) / r
ω_BR = (v_x + v_y - L·ω_z) / r
ω_BL = (v_x - v_y + L·ω_z) / r
```

Where:
- $r$ = wheel radius (0.0525 m)
- $L$ = characteristic length (wheelbase/2)
- $\omega$ = wheel angular velocity
- $v_x, v_y$ = linear velocities
- $\omega_z$ = angular velocity

---

## Robot Models & Design Considerations

### OmniDrive Model

**Location**: `ros_gz_omni_description/models/omni_drive/`

#### Physical Specifications

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Wheelbase (front-to-back) | 0.1485 | m | Distance between wheel pairs |
| Wheel separation (left-to-right) | 0.1485 | m | Distance between left/right wheels |
| Wheel radius | 0.0525 | m | Individual wheel size |
| Chassis mass | 0.8247 | kg | Main body weight |
| Wheel mass | 0.2147 | kg | Each wheel (×4) |
| Total mass | ~1.7 | kg | Approximate total |
| Height (CG) | 0.0125 | m | Center of gravity location |

#### Model Files

- **`model.sdf`**: Main SDF model (395 lines)
  - Generated from Onshape CAD model
  - Contains 4 wheel links + 1 chassis link
  - Full collision geometry for physics
  - Visual meshes (STL format)
  - Inertial properties per link

- **`model.sdf.old`**: Previous version (backup)

- **`meshes/`**: 3D model assets
  - `chassis.part`, `body_1.part`, `body_2.part`: Body meshes
  - `tyre.part`, `couple.part`, `shaft.part`: Wheel components
  - `config.json`: Mesh configuration
  - `merged/`: Precomputed merged geometry

#### Wheel Configuration

Each wheel is modeled as a **cylinder with directional friction**:

**Front-Right (FR) & Back-Left (BL) wheels** - Friction direction `(1, 1, 0)`:
```xml
<friction>
  <ode>
    <mu>1.0</mu>          <!-- Primary friction coefficient (X-direction) -->
    <mu2>0.0</mu2>        <!-- Secondary friction (Y-direction) = no slip -->
    <fdir1 gz:expressed_in="chassis">1 1 0</fdir1>  <!-- Omni wheel direction -->
  </ode>
</friction>
```

**Front-Left (FL) & Back-Right (BR) wheels** - Friction direction `(1, -1, 0)`:
```xml
<friction>
  <ode>
    <mu>1.0</mu>          <!-- Primary friction coefficient -->
    <mu2>0.0</mu2>        <!-- No lateral slip -->
    <fdir1 gz:expressed_in="chassis">1 -1 0</fdir1>  <!-- Opposite direction -->
  </ode>
</friction>
```

#### Design Considerations

**Friction Model**:
- **μ (mu)**: Coefficient of friction along omni wheel axis = **1.0**
  - Controls rolling resistance along wheel direction
  - Higher values increase traction along intended direction
  
- **μ2 (mu2)**: Lateral friction coefficient = **0.0**
  - **Critical for omni wheel function**: Zero means no friction perpendicular to wheel
  - Allows free lateral sliding at wheel contact point
  - This is the key to omnidirectional motion

- **fdir1**: Primary friction direction in world frame
  - Defines axis along which friction applies
  - At 45° angles for diagonal wheel arrangement
  - Enables coordinated motion in any direction

**Wheel Arrangement**:
```
     FL(-45°)    FR(+45°)
        \          /
         \________/
         |        |
         |________|
         /        \
        /          \
     BL(+45°)     BR(-45°)
```

**Physics Parameters**:
```
- Joint dynamics: friction=0.0, damping=0.0
- Joint velocity limits: ±10 rad/s per wheel
- Joint effort limits: 10 N·m per wheel
- ODE solver: 1ms timestep, real-time factor 1.0
```
---

## Running the Simulation

### Complete Step-by-Step Guide

#### Terminal Setup (Run in separate terminals)

**Terminal 1**: Source environment and launch
```bash
source ~/ros_gz_ws/install/setup.bash
export GZ_VERSION=harmonic
ros2 launch ros_gz_omni_bringup omni_drive.launch.py
```

**Terminal 2**: Monitor odometry
```bash
source ~/ros_gz_ws/install/setup.bash
ros2 topic echo /omni_drive/odometry
```

**Terminal 3**: Test velocity commands
```bash
source ~/ros_gz_ws/install/setup.bash
# Wait 5 seconds, then send forward command
sleep 5 && ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Simulation Scenarios

#### Scenario 1: Forward Motion
```bash
ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Expected**: Robot moves forward, odometry X position increases

#### Scenario 2: Lateral Motion (Strafe)
```bash
ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.8, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Expected**: Robot moves left (positive Y), omnidirectional capability demonstrated

#### Scenario 3: Rotation in Place
```bash
ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

**Expected**: Robot spins counterclockwise, heading angle changes

#### Scenario 4: Combined Motion (Circle)
```bash
ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.7, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

**Expected**: Robot follows curved path with increasing angle offset

#### Scenario 5: Stop Command
```bash
ros2 topic pub /omni_drive/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Expected**: All wheel velocities go to zero, robot decelerates to stop

### Performance Monitoring

**Check simulation frequency**:
```bash
ros2 topic hz /omni_drive/odometry
```

**Monitor CPU usage**:
```bash
# In separate terminal
top -p $(pgrep -f "gz sim")
```

**View detailed joint information**:
```bash
ros2 topic echo /joint_states --once
```

---

## Visualization with RViz

### Launching RViz

RViz is automatically launched with default configuration. To manually launch:

```bash
ros2 launch ros_gz_omni_bringup omni_drive.launch.py rviz:=true
```

Or directly:
```bash
ros2 run rviz2 rviz2 -d ~/ros_gz_ws/src/ros_gz_omni/ros_gz_omni_bringup/config/omni_drive.rviz
```

### Pre-configured Display Elements

**File**: `ros_gz_omni_bringup/config/omni_drive.rviz`

| Display | Type | Topic | Purpose |
|---------|------|-------|---------|
| Grid | Grid | - | Reference plane |
| RobotModel | RobotModel | /robot_description | 3D robot mesh |
| TF | TF | /tf, /tf_static | Frame visualization |
| Odometry | Odometry | /omni_drive/odometry | Pose trajectory |

### RViz Interaction

**Mouse Controls**:
- **Middle-click drag**: Rotate view
- **Scroll**: Zoom in/out
- **Right-click drag**: Pan camera

**Keyboard**:
- **`R`**: Reset view to home
- **`S`**: Enable/disable selection
- **`M`**: Switch interaction tool

### Troubleshooting RViz

**"Could not find the package"**:
```bash
source ~/ros_gz_ws/install/setup.bash
```

**"TF transforms not appearing"**:
- Check: `ros2 topic echo /tf` shows messages
- Verify: `ros2 tf2_tools view_frames` generates tree

**"Robot model not visible"**:
- Check RobotModel display is enabled
- Verify `/robot_description` topic exists:
  ```bash
  ros2 param get /robot_state_publisher robot_description | head -20
  ```

---

## Joystick Teleoperation

### Setup

**Prerequisites**:
- USB joystick/gamepad connected
- `joy` and `teleop_twist_joy` packages installed

**Verify joystick is detected**:
```bash
ls /dev/input/js*
# Should show /dev/input/js0 (or js1, js2, etc.)
```

**Test joystick input**:
```bash
sudo apt install joystick
jstest /dev/input/js0
# Move sticks, press buttons - should show input changes
```

### Control Mapping

**Configuration**: `ros_gz_omni_bringup/config/teleop_twist_joy.yaml`

| Control | Mapping | Effect | Range |
|---------|---------|--------|-------|
| **Left Stick Vertical** | Axis 1 (Y) | Forward/Backward velocity | ±1.5 m/s |
| **Left Stick Horizontal** | Axis 2 (X) | Lateral (strafe) velocity | ±1.5 m/s |
| **Right Stick Horizontal** | Axis 3 (Z-rot) | Angular velocity | ±1.5 rad/s |
| **LT Button** (Enable) | Button 5 | Must hold for commands | - |
| **RT Button** (Turbo) | Button 7 | Doubles speed limits | ×2.5 |

### Typical Joystick Layout

```
        ____Y____
       /          \
     LB(5)    [TGT](7)RB
     /              \
    /                \
   |  [LS]  [RS]     |
   |                  |
   |   LT(4)  RT(6)  |
    \                /
     \              /
       \__________/
       X  [DPAD]  B
       
[LS] = Left Stick (Axis 0-1)
[RS] = Right Stick (Axis 3-4)
LT = Left Trigger (Axis 2)
RT = Right Trigger (Axis 5)
```

### Operating Sequence

1. **Start simulation** (Terminal 1):
   ```bash
   ros2 launch ros_gz_omni_bringup omni_drive.launch.py teleop:=true
   ```

2. **Connect joystick** if not already connected

3. **Enable control** by holding `LT` button

4. **Move left stick** to control velocity (forward/strafe)

5. **Move right stick horizontal** to rotate

6. **Hold RT** for turbo (faster motion)

### Velocity Tuning

Edit `teleop_twist_joy.yaml` to adjust:

```yaml
scale_linear:
  x: 1.5      # Forward speed scaling
  y: 1.5      # Lateral speed scaling
scale_angular:
  yaw: 1.5    # Rotation speed scaling
scale_linear_turbo:
  x: 2.5      # Turbo forward
  y: 2.5      # Turbo lateral
scale_angular_turbo:
  yaw: 2.5    # Turbo rotation
```

Higher values = faster motion per stick input

---

## Troubleshooting

For comprehensive troubleshooting guidance, including 9+ common issues and solutions, please refer to the [TROUBLESHOOTING.md](TROUBLESHOOTING.md) file.

Common topics covered:
- ROS 2 command not found issues
- Package discovery problems
- Gazebo plugin loading failures
- SDF file loading errors
- Transform (TF) frame issues
- Joystick device recognition
- Segmentation faults and crashes
- Model mesh and collision geometry problems
- RViz visualization issues
- Performance optimization tips

---

## Miscellaneous Models

### Additional Models Included

#### RRBot (Revolute-Revolute Robot)

**Location**: `models/rrbot/`

**Type**: Articulated arm with 2 revolute joints

**Files**:
- `model.sdf`: Complete arm definition with joint limits

**Use Case**:
- Separate arm simulation
- Joint control testing
- Kinematics validation

**Launch**:
```bash
ros2 launch ros_gz_omni_bringup rrbot_setup.launch.py
```

#### Model Versioning Archive

**Location**: `models/old_version/`

**Contents**:

| Directory | Version | Status | Notes |
|-----------|---------|--------|-------|
| `omni_drive_old/` | v0 (initial) | Deprecated | Prototype, basic geometry |
| `omni_drive_v0/` | v0.1 | Archived | Includes robot.urdf conversion |
| `omni_drive_v1/` | v1 | Previous stable | Improved inertia, better meshes |
| `omni_drive/` | **v2 (current)** | **Active** | Onshape-generated, production |

**Mesh Assets by Version**:

- `v0`:
  - Frame, wheels, basic parts
  - Low-poly meshes

- `v1`:
  - Separate body sections (body_1, body_2)
  - Wheel components (shaft, holder, tyre)
  - Higher fidelity than v0

- **`v2` (current)**:
  - Optimized merged geometries
  - Config-driven mesh selection
  - Production-quality parts

---

## Summary

This project provides a complete, production-ready simulation environment for omnidirectional mobile robots. Key capabilities include:

✅ **Full ROS 2 / Gazebo Integration** - Modern robotics stack  
✅ **Custom OmniDrive Plugin** - Specialized kinematic controller  
✅ **Multi-Robot Support** - Omni, mecanum, differential drive, arm  
✅ **Real-time Visualization** - RViz integration with TF transforms  
✅ **Hardware Simulation** - Joystick control and odometry publishing  
✅ **Comprehensive Documentation** - This README and inline code comments  

For questions or contributions, refer to the official [ROS-Gazebo integration guide](https://gazebosim.org/docs/latest/ros_installation).

---

**Last Updated**: 2025-12-28  
**Gazebo Version**: Harmonic (gz-sim8)  
**ROS 2 Distribution**: Jazzy
