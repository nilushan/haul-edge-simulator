"""Sim sensor source only (publishes bus topics)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('edge_bringup'), 'config', 'default.yaml')
    return LaunchDescription([
        Node(
            package='edge_sensor_source',
            executable='sensor_source_node',
            name='sensor_source',
            output='screen',
            parameters=[cfg],
        ),
    ])
