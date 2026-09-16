"""
Edge vehicle stack — libs under ws/src/libs, nodes under ws/src/nodes:

  edge_sensor_source  → bus topics
  edge_processor      → /edge/lidar/processed (optional stub)
  edge_rock_detect    → /edge/lidar/rocks + alerts
  edge_bund_detect    → /edge/lidar/bunds + alerts
  edge_vibe_detect    → vibration alerts
  edge_event_store    → SQLite persistence
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
    use_rock = LaunchConfiguration('use_rock_detect')
    use_bund = LaunchConfiguration('use_bund_detect')
    use_vibe = LaunchConfiguration('use_vibe_detect')
    use_store = LaunchConfiguration('use_event_store')
    use_viz = LaunchConfiguration('use_viz')
    viz_host = LaunchConfiguration('viz_host')
    viz_port = LaunchConfiguration('viz_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('use_processor', default_value='false'),
        DeclareLaunchArgument('use_rock_detect', default_value='true'),
        DeclareLaunchArgument('use_bund_detect', default_value='true'),
        DeclareLaunchArgument('use_vibe_detect', default_value='true'),
        DeclareLaunchArgument('use_event_store', default_value='true'),
        DeclareLaunchArgument('use_viz', default_value='true'),
        DeclareLaunchArgument('viz_host', default_value='0.0.0.0'),
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
            package='edge_rock_detect',
            executable='rock_detect_node',
            name='edge_rock_detect',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(use_rock),
        ),
        Node(
            package='edge_bund_detect',
            executable='bund_detect_node',
            name='edge_bund_detect',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(use_bund),
        ),
        Node(
            package='edge_vibe_detect',
            executable='vibe_detect_node',
            name='edge_vibe_detect',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(use_vibe),
        ),
        Node(
            package='edge_event_store',
            executable='event_store_node',
            name='edge_event_store',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(use_store),
        ),
        Node(
            package='edge_viz',
            executable='viz_from_bus',
            name='edge_viz_bus',
            output='screen',
            arguments=['--host', viz_host, '--port', viz_port],
            condition=IfCondition(use_viz),
        ),
    ])
