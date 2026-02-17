#!/usr/bin/env python3
"""
so101_replay.launch.py
======================
Full pipeline for replaying the SO101 MuJoCo PID trajectory in Gazebo Harmonic.
Does NOT require gz_ros2_control.  Uses Gazebo's native JointPositionController.

Architecture:
  Gazebo Harmonic  ──  JointPositionController (per joint, gz-sim plugin)
                   ──  JointStatePublisher     (gz-sim plugin)
                         │                         │
                   ros_gz_bridge            ros_gz_bridge
                         │                         │
  ROS2 cmd topics  ←─────┘        joint_states ───→  trajectory_replay node
  /model/so101/joint/<j>/cmd_pos     /so101/joint_states

Usage:
  # 1. One-time setup (generate URDF + copy meshes):
  #    cd <repo>/simulation_code && bash setup_gazebo.sh
  #    cd ../ros2_ws && colcon build --symlink-install && source install/setup.bash

  # 2. Export trajectory:
  #    cd simulation_code && python3 export_trajectory.py

  # 3. Run replay:
  ros2 launch gazebo_mujoco_bridge so101_replay.launch.py \\
      csv_in:=$(pwd)/simulation_code/trajectory_data/mujoco_trajectory.csv

Arguments:
  csv_in          – path to mujoco_trajectory.csv  (REQUIRED)
  csv_out         – path for output gazebo_trajectory.csv (optional)
  gui             – true/false (default: true)
  publish_rate_hz – replay speed in Hz (default: 500, matches MuJoCo 2ms step)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, Command, FindExecutable, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def generate_launch_description():
    pkg = get_package_share_directory("gazebo_mujoco_bridge")
    world_file = os.path.join(pkg, "worlds", "so101_world.sdf")
    urdf_file  = os.path.join(pkg, "urdf",   "so101.urdf")

    # Gazebo needs GZ_SIM_RESOURCE_PATH to resolve package:// mesh URIs
    # pkg is .../share/gazebo_mujoco_bridge, parent is .../share/
    gz_resource_path = os.path.dirname(pkg)
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    combined = f"{gz_resource_path}:{existing}" if existing else gz_resource_path

    set_gz_resource = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", combined
    )

    # ── Launch arguments ──────────────────────────────────────────────────
    default_csv = os.path.join(pkg, "data", "mujoco_trajectory.csv")
    csv_in_arg = DeclareLaunchArgument(
        "csv_in",
        default_value=default_csv,
        description="Path to mujoco_trajectory.csv (from export_trajectory.py)",
    )
    csv_out_arg = DeclareLaunchArgument(
        "csv_out",
        default_value="",
        description="Output path for gazebo_trajectory.csv (auto if empty)",
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Launch Gazebo GUI",
    )
    rate_arg = DeclareLaunchArgument(
        "publish_rate_hz", default_value="500.0",
        description="Replay publish rate in Hz (500 = 2ms, matches MuJoCo timestep)",
    )

    # ── robot_description from URDF ───────────────────────────────────────
    robot_description = ParameterValue(
        Command([FindExecutable(name="cat"), " ", urdf_file]),
        value_type=str,
    )

    # ── 1. robot_state_publisher (for TF/RViz, optional but useful) ───────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ── 2. Gazebo Harmonic ─────────────────────────────────────────────────
    # With GUI (default)
    gz_sim_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    # Headless (server-only, no GUI crash risk)
    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", world_file],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    # ── 3. Spawn robot (wait 2s for Gazebo to start) ──────────────────────
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name",  "so101",
                    "-file",  urdf_file,
                    "-x", "0.0", "-y", "0.0", "-z", "0.05",
                ],
                output="screen",
            )
        ],
    )

    # ── 4. ros_gz_bridge (wait 3s for robot to spawn) ─────────────────────
    # Bridge clock
    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_clock",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Bridge joint state: Gazebo → ROS2
    bridge_joint_states = TimerAction(
        period=3.5,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="bridge_joint_states",
            arguments=[
                (
                    "/world/so101_replay/model/so101/joint_state"
                    "@sensor_msgs/msg/JointState"
                    "[gz.msgs.Model"
                )
            ],
            remappings=[
                (
                    "/world/so101_replay/model/so101/joint_state",
                    "/so101/joint_states",
                )
            ],
            output="screen",
        )]
    )

    # Bridge per-joint cmd topics: ROS2 → Gazebo
    cmd_bridge_args = [
        f"/model/so101/joint/{jn}/cmd_pos"
        f"@std_msgs/msg/Float64"
        f"]gz.msgs.Double"
        for jn in JOINT_NAMES
    ]
    bridge_cmds = TimerAction(
        period=3.5,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="bridge_joint_cmds",
            arguments=cmd_bridge_args,
            output="screen",
        )]
    )

    # ── 5. Trajectory replay node (wait 8s total) ─────────────────────────
    trajectory_replay = TimerAction(
        period=8.0,
        actions=[Node(
            package="gazebo_mujoco_bridge",
            executable="trajectory_replay",
            output="screen",
            parameters=[
                {"csv_in":           LaunchConfiguration("csv_in")},
                {"csv_out":          LaunchConfiguration("csv_out")},
                {"publish_rate_hz":  LaunchConfiguration("publish_rate_hz")},
                {"wait_s":           2.0},   # extra wait inside the node
            ],
        )]
    )

    return LaunchDescription([
        set_gz_resource,
        csv_in_arg,
        csv_out_arg,
        gui_arg,
        rate_arg,
        robot_state_pub,
        gz_sim_gui,
        gz_sim_headless,
        bridge_clock,
        spawn_robot,
        bridge_joint_states,
        bridge_cmds,
        trajectory_replay,
    ])
