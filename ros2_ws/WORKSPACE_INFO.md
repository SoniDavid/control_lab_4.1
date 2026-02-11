# ROS2 Workspace Information

## Workspace Details

**Purpose**: Cross-simulator validation for MuJoCo → Gazebo Harmonic migration
**Target Gazebo Version**: Gazebo Sim 8.10 (Gazebo Harmonic)
**ROS2 Distribution**: Humble (recommended)
**Created**: 2026-02-11

## Package Overview

### gazebo_mujoco_bridge
A comprehensive ROS2 package for validating simulation results between MuJoCo and Gazebo Harmonic.

**Key Components**:
- Launch files for Gazebo Harmonic integration
- World files (SDF format) for simulation environments
- Sample URDF robot descriptions
- Configuration files for ROS-Gazebo bridge
- Python utilities for cross-simulator validation

## Quick Start

### 1. Automated Setup (Recommended)
```bash
cd ros2_ws
./setup_workspace.sh
source install/setup.bash
```

### 2. Manual Setup
```bash
# Install Gazebo Harmonic
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-harmonic

# Install ROS-Gazebo bridge
sudo apt install ros-humble-ros-gz

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Usage Examples

### Launch Gazebo Harmonic
```bash
ros2 launch gazebo_mujoco_bridge gazebo.launch.py
```

### Launch with custom world
```bash
ros2 launch gazebo_mujoco_bridge gazebo.launch.py world:=/path/to/your/world.sdf
```

### View available topics
```bash
ros2 topic list
```

### Monitor bridge connections
```bash
ros2 topic echo /clock
```

## Files Created

### Core Package Files
- `package.xml` - Package manifest with dependencies
- `CMakeLists.txt` - Build configuration
- `setup.py` - Python package configuration

### Launch Files
- `launch/gazebo.launch.py` - Main Gazebo Harmonic launcher

### World Files
- `worlds/empty.sdf` - Empty world with ground plane

### URDF Files
- `urdf/simple_robot.urdf` - Sample robot description

### Configuration
- `config/bridge_config.yaml` - ROS-Gazebo bridge mappings

### Documentation
- `README.md` - Package documentation
- `WORKSPACE_INFO.md` - This file

### Scripts
- `scripts/placeholder.py` - Template for custom scripts
- `setup_workspace.sh` - Automated workspace setup

## Dependencies

### Build Dependencies
- `ament_cmake`
- `ament_cmake_python`

### Runtime Dependencies
- `ros_gz_bridge` - ROS2-Gazebo communication
- `ros_gz_sim` - Gazebo simulator integration
- `ros_gz_image` - Image transport
- `gazebo_msgs` - Gazebo message types
- `robot_state_publisher` - Robot TF broadcasting
- `joint_state_publisher` - Joint state management
- `xacro` - URDF macros

## Next Steps

1. **Add Robot Models**: Place your robot URDF/SDF files in `urdf/` or `models/`
2. **Create Worlds**: Design simulation worlds in `worlds/`
3. **Implement Validation**: Add comparison scripts in `scripts/`
4. **Configure Bridges**: Update `config/bridge_config.yaml` for your topics
5. **Test Migration**: Run parallel simulations in MuJoCo and Gazebo

## Validation Strategy

This workspace supports cross-simulator validation through:

1. **State Recording**: Capture joint states, poses, and sensor data
2. **Trajectory Comparison**: Compare motion trajectories between simulators
3. **Physics Validation**: Analyze force/torque discrepancies
4. **Sensor Validation**: Compare sensor outputs (cameras, IMU, lidar)
5. **Control Validation**: Test controller performance across platforms

## Troubleshooting

### Gazebo doesn't start
- Verify installation: `gz sim --version`
- Check ROS2 sourcing: `echo $ROS_DISTRO`

### Build failures
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Missing dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## References

- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [ros_gz GitHub](https://github.com/gazebosim/ros_gz)
