#!/usr/bin/env python3
"""Launch lane_detector and lane_controller together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')

    perception_share = get_package_share_directory('lane_perception')
    control_share = get_package_share_directory('lane_control')

    perception_params = os.path.join(perception_share, 'config', 'lane_perception.yaml')
    control_params = os.path.join(control_share, 'config', 'lane_control.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from Gazebo',
        ),
        Node(
            package='lane_perception',
            executable='lane_detector',
            name='lane_detector',
            output='screen',
            parameters=[perception_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='lane_control',
            executable='lane_controller',
            name='lane_controller',
            output='screen',
            parameters=[control_params, {'use_sim_time': use_sim_time}],
        ),
    ])
