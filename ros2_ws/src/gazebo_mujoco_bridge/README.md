# Gazebo MuJoCo Bridge

## Overview
This ROS2 package provides cross-simulator validation tools for migrating from MuJoCo to Gazebo Harmonic (Gazebo Sim 8.10). It includes launch files, world configurations, and bridge utilities to compare simulation results.

## Prerequisites

### System Requirements
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS2 Humble or later
- Gazebo Harmonic (Gazebo Sim 8.10)

### Installation

#### Install Gazebo Harmonic
```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install Gazebo Harmonic
sudo apt update
sudo apt install gz-harmonic
```

#### Install ROS2-Gazebo Bridge
```bash
sudo apt install ros-humble-ros-gz
```

## Building the Workspace

```bash
# Navigate to workspace root
cd ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Package Structure

```
gazebo_mujoco_bridge/
├── launch/          # Launch files for Gazebo and ROS2 nodes
├── worlds/          # Gazebo world files (.sdf)
├── models/          # Robot and object models
├── urdf/            # Robot URDF/XACRO descriptions
├── config/          # Configuration files (YAML, etc.)
├── scripts/         # Python scripts for validation and utilities
├── package.xml      # Package manifest
├── CMakeLists.txt   # Build configuration
└── README.md        # This file
```

## Usage

### Launch Gazebo Harmonic
```bash
ros2 launch gazebo_mujoco_bridge gazebo.launch.py
```

### Run Bridge Nodes
```bash
ros2 run gazebo_mujoco_bridge bridge_node
```

## Cross-Simulator Validation

This package is designed to facilitate comparison between MuJoCo and Gazebo simulations by:
- Providing equivalent world/model configurations
- Recording and comparing state trajectories
- Analyzing physics discrepancies
- Validating control strategies across simulators

## License
MIT License
