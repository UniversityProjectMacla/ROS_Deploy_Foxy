#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[{
                'cloud_topic': '/livox/lidar',
                'odom_topic': '/lidar/odom',
                'delta_topic': '/lidar/relative_motion',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'registration_mode': 'scan_to_map',
                'voxel_leaf_size': 0.22,
                'crop_range_m': 40.0,
                'ndt_resolution': 0.85,
                'ndt_step_size': 0.1,
                'ndt_transformation_epsilon': 0.01,
                'ndt_max_iterations': 50,
                'max_fitness_score': 12.0,
                'min_points_per_cloud': 200,
                'publish_tf': False,
                'use_tf_initial_guess': True,
                'tf_initial_guess_timeout_sec': 0.1,
            }],
        ),
    ])
