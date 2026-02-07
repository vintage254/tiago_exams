"""
Task 2 Launch File: ArUco Navigation

Launches Nav2 with a saved map + the ArUco navigator node.

Prerequisites: The simulation must already be running:
  ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25

Usage:
  ros2 launch group25_tasks task2_navigation.launch.py map_path:=/home/<user>/my_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.expanduser('~/my_map'),
        description='Path to the folder containing map.yaml and map.pgm'
    )

    # Launch Nav2 with the saved map (no SLAM)
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_2dnav'),
                'launch', 'tiago_nav_bringup.launch.py'
            )
        ),
        launch_arguments={
            'is_public_sim': 'false',
            'map_path': LaunchConfiguration('map_path'),
        }.items(),
    )

    # Launch ArUco navigator node
    aruco_nav_node = Node(
        package='group25_tasks',
        executable='task2_aruco_navigation.py',
        name='task2_aruco_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_path)
    ld.add_action(nav_bringup)
    ld.add_action(aruco_nav_node)
    return ld
