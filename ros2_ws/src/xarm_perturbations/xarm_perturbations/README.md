# Cartesian PD Control and Perturbation Analysis for xArm Lite 6

This repository contains the implementation of a Cartesian Proportional-Derivative (PD) controller for the UFACTORY xArm Lite 6 manipulator. The project is built using **ROS 2** and **MoveIt Servo**, focusing on real-time task-space trajectory tracking under various perturbation conditions (Baseline, Sinusoidal, and Gaussian noise).

## 🚀 System Architecture

The control system consists of a custom ROS 2 package (`xarm_perturbations`) featuring:
1. **Trajectory Generator:** Generates a parameterized Lissajous figure-eight continuous curve in the Cartesian $XY$ plane with a mathematical soft-start ramp.
2. **Task-Space PD Controller:** Calculates the error using `tf2` (`link_base` $\rightarrow$ `link_eef`) and computes corrective velocities.
3. **Safety & Stability Modules:** Includes a static deadband ($0.002$ m), velocity saturation ($0.12$ m/s), and a first-order digital Low-Pass Filter (LPF) to mitigate derivative noise amplification.
4. **MoveIt Servo Interface:** Safely translates Cartesian `TwistStamped` commands into joint velocities for the physical robot.

## 🛠️ Dependencies

* **OS:** Ubuntu 22.04
* **Middleware:** ROS 2 Humble
* **Core Packages:** `moveit_servo`, `xarm_ros2`, `tf2_ros`
* **Python Libraries:** `numpy`, `pandas`, `matplotlib` (for evaluation scripts)

## ⚙️ Build Instructions

Clone this repository into your ROS 2 workspace `src` folder and build the package:

```bash
cd ~/ros2_ws/src
git clone <YOUR_GITHUB_REPO_URL>
cd ~/ros2_ws
colcon build --packages-select xarm_perturbations
source install/setup.bash
```

## 🏃‍♂️ Running the Experiments (Physical Robot)

Ensure the xArm Lite 6 is powered on, connected via Ethernet (192.168.1.123), and not in an emergency stop state.
### 1. Launch MoveIt Servo

This must remain active for all subsequent tests:
Bash

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123
```

(Note: Ensure the robot is in Mode 1 and State 0 before sending velocity commands).

### 2. Run the Controller (Data Logging Mode)

To run the nominal trajectory tracking and log the data to a CSV file:

```bash
ros2 run xarm_perturbations trajectory_and_controller --ros-args \
  -p radius:=0.15 \
  -p frequency:=0.035 \
  -p plane:=xy \
  -p hold_z:=false \
  -p yaw_deg:=90.0 \
  -p kp:="[2.665,2.665,2.665]" \
  -p kd:="[0.64,0.64,0.64]" \
  -p max_speed:=0.12 \
  -p deadband:=0.002 \
  -p save_csv:=true \
  -p csv_filename:=baseline_experiment.csv
```

### 3. Grid-Search Auto-Tuner (Optional)

To empirically find the optimal Kp​ and Kd​ gains using an automated cost function (RMSE + Saturation penalty), run the tuner node in a separate terminal while the controller is paused:
Bash

```bash
ros2 run xarm_perturbations grid_search_tuner
```

📊 Evaluation and Analysis

The repository includes a standalone Python script to parse the generated CSV files, compute the Root Mean Square Error (RMSE), and plot the tracking performance.
Bash

python3 analysis_evaluation.py

Experimental Results Summary

The fixed gains (Kp​=2.665, Kd​=0.64) were evaluated across three states. The results demonstrate that while the PD controller accurately tracks the baseline trajectory (Total RMSE: ~0.032m), the derivative term makes the system highly sensitive to stochastic, high-frequency Gaussian noise, resulting in the highest maximum absolute error (~0.115m).

    Baseline: Nominal tracking with no injected noise.

    Sinusoidal: Low-frequency deterministic perturbation.

    Gaussian: High-frequency stochastic white noise.