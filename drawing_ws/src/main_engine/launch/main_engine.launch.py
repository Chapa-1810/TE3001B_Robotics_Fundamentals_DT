#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xarm_as', executable='xarm_action_server', output='screen', emulate_tty=True,
        ),
        Node(
            package='figure_path_maker', executable='path_generator', output='screen', emulate_tty=True
        ),
        Node(
            package='main_engine', executable='main_engine', output='screen', emulate_tty=True
        ),
        Node(
            package='main_engine', executable='path_action_server', output='screen', emulate_tty=True
        )
    ])