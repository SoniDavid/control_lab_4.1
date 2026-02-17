# ROS2 Workspace for Gazebo Harmonic

This workspace is configured for cross-simulator validation between MuJoCo and Gazebo Harmonic (Gazebo Sim 8.10).

## Quick Start

### 1. Install Dependencies

```bash
# Install Gazebo Harmonic
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-harmonic

# Install ROS2-Gazebo integration
sudo apt install ros-humble-ros-gz

# Install workspace dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build the Workspace

```bash
# From the ros2_ws directory
colcon build --symlink-install
```

### 3. Source the Workspace

```bash
source install/setup.bash
```

### 4. Launch Gazebo

```bash
ros2 launch gazebo_mujoco_bridge gazebo.launch.py
```

## Workspace Structure

```
ros2_ws/
├── src/
│   └── gazebo_mujoco_bridge/    # Main package for cross-simulator validation
├── build/                        # Build artifacts (generated)
├── install/                      # Installation files (generated)
└── log/                          # Build logs (generated)
```

## Verifying Gazebo Harmonic Installation

```bash
# Check Gazebo version
gz sim --version

# Should output something like: Gazebo Sim, version 8.10.x
```

## ROS-Gazebo Bridge

The workspace includes bridges for communication between ROS2 and Gazebo:

- `/clock` - Simulation time synchronization
- Add more bridges as needed in launch files

## Troubleshooting

### Gazebo won't start
```bash
# Check if Gazebo Harmonic is installed correctly
gz sim --help

# Verify ROS-Gazebo bridge packages
ros2 pkg list | grep ros_gz
```

### Build errors
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Missing dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Additional Resources

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ros_gz Integration](https://github.com/gazebosim/ros_gz)
