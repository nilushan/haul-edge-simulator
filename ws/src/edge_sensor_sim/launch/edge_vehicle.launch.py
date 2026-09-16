"""
Realistic vehicle-edge stack:

  [source: sim StreamHub→ROS] → sensor bus topics
       → [processor_stub]
       → [viz_bus: ROS→Web UI]

Swap the source later for rosbag play or real drivers; processor + viz stay the same.

  ros2 launch edge_sensor_sim edge_vehicle.launch.py
  ros2 launch edge_sensor_sim edge_vehicle.launch.py use_sim:=false  # bus+proc+viz only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('edge_sensor_sim')
    params = os.path.join(pkg, 'config', 'default.yaml')

    use_sim = LaunchConfiguration('use_sim')
    use_processor = LaunchConfiguration('use_processor')
    use_viz = LaunchConfiguration('use_viz')
    viz_port = LaunchConfiguration('viz_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='Start synthetic sensor source on the bus'),
        DeclareLaunchArgument('use_processor', default_value='true',
                              description='Start example edge processor'),
        DeclareLaunchArgument('use_viz', default_value='true',
                              description='Start bus-subscribed web visualizer'),
        DeclareLaunchArgument('viz_port', default_value='8099'),

        Node(
            package='edge_sensor_sim',
            executable='sensor_suite_node',
            name='sensor_suite',
            output='screen',
            parameters=[params],
            condition=IfCondition(use_sim),
        ),
        Node(
            package='edge_sensor_sim',
            executable='processor_stub_node',
            name='edge_processor_stub',
            output='screen',
            parameters=[params],
            condition=IfCondition(use_processor),
        ),
        Node(
            package='edge_sensor_sim',
            executable='viz_bus_server',
            name='edge_viz_bus',
            output='screen',
            arguments=['--host', '0.0.0.0', '--port', viz_port],
            condition=IfCondition(use_viz),
        ),
    ])
