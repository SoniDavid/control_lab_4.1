# Gazebo-MuJoCo Bridge

Cross-simulator validation package for the SO101 robotic arm. Records PID-controlled trajectories in MuJoCo and replays them in Gazebo Harmonic to compare physics engine behavior.

## Prerequisites

### System Requirements

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble
- Gazebo Harmonic (gz-sim 8.x)
- Python 3.10+
- MuJoCo (Python bindings)

### Install Gazebo Harmonic

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
     -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update && sudo apt install gz-harmonic
```

### Install ROS 2 Dependencies

```bash
sudo apt install ros-humble-ros-gz \
                 ros-humble-robot-state-publisher \
                 ros-humble-xacro \
                 ros-humble-sensor-msgs \
                 ros-humble-std-msgs
```

### Install Python Dependencies (for MuJoCo simulation)

```bash
pip install mujoco numpy scipy
```

## Package Structure

```
gazebo_mujoco_bridge/           # ROS 2 package (ros2_ws/src/)
├── launch/
│   ├── gazebo.launch.py        # Bare Gazebo world (not used for replay)
│   └── so101_replay.launch.py  # Full replay pipeline (use this one)
├── urdf/
│   ├── so101.urdf              # Generated URDF for Gazebo
│   └── so101.urdf.xacro        # XACRO source
├── worlds/
│   └── so101_world.sdf         # Gazebo world (ground plane, physics, lighting)
├── config/
│   ├── bridge_config.yaml      # Reference ROS-Gazebo bridge mappings
│   └── so101_controllers.yaml  # ros2_control config (unused, reference only)
├── scripts/
│   └── trajectory_replay.py    # ROS 2 replay node
├── meshes/                     # SO101 STL mesh files (13 files)
├── data/
│   └── mujoco_trajectory.csv   # Pre-exported trajectory (fallback)
├── CMakeLists.txt
└── package.xml

simulation_code/                # Standalone MuJoCo code (outside ros2_ws)
├── export_trajectory.py        # Generate MuJoCo trajectory CSV
├── mjcf_to_urdf.py             # Convert MJCF model to URDF/XACRO
├── setup_gazebo.sh             # One-time setup script
├── so101_control.py            # PID controller + perturbation model
├── so101_mujoco_pid_utils.py   # PID gains, motion helpers
├── so101_mujoco_utils2.py      # MuJoCo utilities (pose setting)
├── model/
│   ├── robot_from_urdf.xml     # SO101 MJCF model
│   ├── scene_urdf.xml          # Scene with robot
│   └── assets/                 # STL meshes for MuJoCo
└── trajectory_data/
    ├── mujoco_trajectory.csv   # Exported trajectory
    └── mujoco_trajectory_meta.json  # Run metadata (PID gains, config)
```

## Complete Workflow

### Step 1: One-Time Setup (URDF generation + mesh copy)

This converts the MuJoCo MJCF model to a Gazebo-compatible URDF and copies the mesh assets into the ROS 2 package.

```bash
cd simulation_code
bash setup_gazebo.sh
```

This runs `mjcf_to_urdf.py` and copies the resulting URDF + STL meshes into `ros2_ws/src/gazebo_mujoco_bridge/`.

### Step 2: Build the ROS 2 Workspace

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Step 3: Export the MuJoCo Trajectory

Run the PID-controlled simulation in MuJoCo and export the trajectory to CSV.

```bash
cd simulation_code
python3 export_trajectory.py
```

This produces:
- `trajectory_data/mujoco_trajectory.csv` — timestamped joint positions, desired positions, and torques
- `trajectory_data/mujoco_trajectory_meta.json` — PID gains, perturbation config, motion sequence

The motion sequence (8 seconds total at 500 Hz / 0.002s timestep):

| Phase            | Duration | Description                          |
|------------------|----------|--------------------------------------|
| move_to_zero     | 2.0s     | Interpolate from start to zero pose  |
| hold_zero        | 2.0s     | Hold zero with PID + perturbations   |
| return_to_start  | 2.0s     | Interpolate back to starting pose    |
| hold_start       | 2.0s     | Hold starting pose                   |

### Step 4: Replay in Gazebo

```bash
cd ros2_ws
source install/setup.bash

ros2 launch gazebo_mujoco_bridge so101_replay.launch.py \
    csv_in:=/absolute/path/to/simulation_code/trajectory_data/mujoco_trajectory.csv
```

For example:

```bash
ros2 launch gazebo_mujoco_bridge so101_replay.launch.py \
    csv_in:=$(realpath ../simulation_code/trajectory_data/mujoco_trajectory.csv)
```

#### Launch Arguments

| Argument           | Default   | Description                                      |
|--------------------|-----------|--------------------------------------------------|
| `csv_in`           | (bundled) | Path to `mujoco_trajectory.csv` (required)       |
| `csv_out`          | (auto)    | Output path for `gazebo_trajectory.csv`           |
| `gui`              | `true`    | Set `false` for headless (server-only) mode       |
| `publish_rate_hz`  | `500.0`   | Replay rate in Hz (matches MuJoCo 2ms timestep)  |

#### Headless Mode

For more stable operation (no GPU/display required):

```bash
ros2 launch gazebo_mujoco_bridge so101_replay.launch.py \
    csv_in:=$(realpath ../simulation_code/trajectory_data/mujoco_trajectory.csv) \
    gui:=false
```

### Step 5: Collect Results

After the replay finishes, the node writes `gazebo_trajectory.csv` (next to the input CSV unless `csv_out` is specified). This file contains:

```
ros_time, q_actual_shoulder_pan, qd_actual_shoulder_pan, q_actual_shoulder_lift, ...
```

Compare `mujoco_trajectory.csv` vs `gazebo_trajectory.csv` to analyze cross-simulator fidelity.

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  Gazebo Harmonic                 │
│                                                  │
│  JointPositionController (per joint, gz plugin)  │
│  JointStatePublisher     (gz plugin)             │
│          ▲                     │                 │
└──────────┼─────────────────────┼─────────────────┘
           │ ros_gz_bridge       │ ros_gz_bridge
           │                     ▼
  /model/so101/joint/       /so101/joint_states
    <name>/cmd_pos               │
      (Float64)                  ▼
           ▲            ┌──────────────────┐
           │            │ trajectory_replay │
           └────────────│    (ROS 2 node)  │
                        │  reads CSV →     │
                        │  publishes cmds  │
                        │  records states  │
                        │  → writes CSV    │
                        └──────────────────┘
```

The replay pipeline does **not** use `gz_ros2_control`. It uses Gazebo's native `JointPositionController` plugin (embedded in the URDF) and bridges topics via `ros_gz_bridge`.

### Startup Sequence

The launch file uses timed delays to ensure correct startup order:

1. **T=0s** — Set `GZ_SIM_RESOURCE_PATH`, start Gazebo + clock bridge + robot_state_publisher
2. **T=2s** — Spawn SO101 model from URDF
3. **T=3.5s** — Bridge joint state and joint command topics
4. **T=8s** — Start trajectory replay node (with additional 2s internal wait)

## Robot: SO101

6-DOF robotic arm with 5 arm joints + 1 gripper joint.

| Joint          | Type     | Axis | Limits (rad)       |
|----------------|----------|------|--------------------|
| shoulder_pan   | revolute | Z    | [-1.92, 1.92]      |
| shoulder_lift  | revolute | Z    | [-1.75, 1.75]      |
| elbow_flex     | revolute | Z    | [-1.69, 1.69]      |
| wrist_flex     | revolute | Z    | [-1.66, 1.66]      |
| wrist_roll     | revolute | Z    | [-2.74, 2.84]      |
| gripper        | revolute | Z    | [-0.175, 1.75]     |

### PID Gains (MuJoCo)

| Joint          | Kp    | Ki   | Kd   | Torque Limit (Nm) |
|----------------|-------|------|------|--------------------|
| shoulder_pan   | 41.5  | 0.51 | 1.22 | 8.0                |
| shoulder_lift  | 35.8  | 1.22 | 3.35 | 18.0               |
| elbow_flex     | 50.8  | 1.55 | 1.64 | 15.0               |
| wrist_flex     | 41.8  | 0.88 | 0.92 | 6.0                |
| wrist_roll     | 24.7  | 0.23 | 1.56 | 3.0                |

### Gazebo PID Gains (URDF JointPositionController)

All joints: `p_gain=998.22`, `d_gain=2.731`, `i_gain=0.0`

These are position-level PID gains for Gazebo's internal controller (different from MuJoCo's torque-level PID).

## Troubleshooting

### Gazebo can't find the world file

```
[Wrn] Fuel world download failed because Fetch failed
Unable to find or download file
```

This means the SDF path is wrong. Make sure you rebuilt after any changes:

```bash
colcon build --symlink-install && source install/setup.bash
```

### No robot visible in Gazebo

Make sure you're using `so101_replay.launch.py`, not `gazebo.launch.py`. The basic `gazebo.launch.py` only starts an empty world without spawning the robot.

### Empty gazebo_trajectory.csv / "No joint_states received"

The `ros_gz_bridge` for joint states may not be running. Check:

```bash
ros2 topic list | grep joint
ros2 topic echo /so101/joint_states
```

Ensure the world name in the bridge topic matches `so101_replay` (set in `so101_world.sdf`).

### Meshes not loading in Gazebo

`GZ_SIM_RESOURCE_PATH` must include the parent of the package share directory. The replay launch file sets this automatically. If running manually:

```bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix gazebo_mujoco_bridge)/share:$GZ_SIM_RESOURCE_PATH
```

### libEGL warnings

```
libEGL warning: egl: failed to create dri2 screen
```

This is a GPU/driver warning, not an error. The simulation still works. Use `gui:=false` if running headless.

## License

MIT
