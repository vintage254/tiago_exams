"""
Task 1 Launch File: SLAM Mapping

Launches the Nav2 stack in SLAM mode + the exploration node.

Prerequisites: The simulation must already be running:
  ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25

Usage:
  ros2 launch group25_tasks task1_mapping.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Launch Nav2 with SLAM enabled
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_2dnav'),
                'launch', 'tiago_nav_bringup.launch.py'
            )
        ),
        launch_arguments={
            'is_public_sim': 'false',
            'slam': 'True',
        }.items(),
    )

    # Launch exploration node
    exploration_node = Node(
        package='group25_tasks',
        executable='task1_exploration.py',
        name='task1_exploration',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ld = LaunchDescription()
    ld.add_action(nav_bringup)
    ld.add_action(exploration_node)
    return ld
