"""
Edge vehicle stack — separate ROS packages wired together:

  edge_sensor_source  → bus topics
  edge_processor      → /edge/*
  edge_viz (bus mode) → browser UI

  ros2 launch edge_bringup edge_vehicle.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('edge_bringup'), 'config', 'default.yaml')

    use_sim = LaunchConfiguration('use_sim')
    use_processor = LaunchConfiguration('use_processor')
    use_viz = LaunchConfiguration('use_viz')
    viz_port = LaunchConfiguration('viz_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('use_processor', default_value='true'),
        DeclareLaunchArgument('use_viz', default_value='true'),
        DeclareLaunchArgument('viz_port', default_value='8099'),

        Node(
            package='edge_sensor_source',
            executable='sensor_source_node',
            name='sensor_source',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(use_sim),
        ),
        Node(
            package='edge_processor',
            executable='processor_node',
            name='edge_processor',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(use_processor),
        ),
        Node(
            package='edge_viz',
            executable='viz_from_bus',
            name='edge_viz_bus',
            output='screen',
            arguments=['--host', '0.0.0.0', '--port', viz_port],
            condition=IfCondition(use_viz),
        ),
    ])
