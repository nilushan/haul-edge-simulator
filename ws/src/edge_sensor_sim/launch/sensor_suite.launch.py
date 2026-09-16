import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('edge_sensor_sim'),
        'config',
        'default.yaml',
    )
    return LaunchDescription([
        Node(
            package='edge_sensor_sim',
            executable='sensor_suite_node',
            name='sensor_suite',
            output='screen',
            parameters=[params],
        ),
    ])
