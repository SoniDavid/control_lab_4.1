#!/usr/bin/env python3
"""
Launch file for Gazebo Harmonic simulation.

This launch file starts Gazebo Harmonic with a specified world file
and sets up the ROS-Gazebo bridge for communication.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Gazebo Harmonic simulation."""

    # Get package directory
    pkg_gazebo_mujoco_bridge = get_package_share_directory('gazebo_mujoco_bridge')

    # Paths
    world_file_path = PathJoinSubstitution([
        pkg_gazebo_mujoco_bridge,
        'worlds',
        'empty.sdf'
    ])

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Path to world file'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Verbose output'
    )

    # Gazebo Harmonic server (using gz sim)
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -v 4'],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ROS-Gazebo bridge for clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        verbose_arg,
        gz_server,
        bridge_clock,
    ])
