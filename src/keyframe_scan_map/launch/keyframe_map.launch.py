#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    cfg = PathJoinSubstitution([
        FindPackageShare('keyframe_scan_map'),
        'config',
        'keyframe_map.yaml',
    ])
    return LaunchDescription([
        Node(
            package='keyframe_scan_map',
            executable='keyframe_map_node',
            name='keyframe_map_node',
            output='screen',
            parameters=[cfg],
        ),
    ])
