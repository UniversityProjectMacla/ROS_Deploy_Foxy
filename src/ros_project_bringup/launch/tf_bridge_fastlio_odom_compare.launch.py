#!/usr/bin/env python3
"""TF bridge so FAST-LIO ``camera_init``/``body`` can be related to robot ``odom``/``base_link``.

Publishes:
  1) Always: static ``odom``→``camera_init`` (identity).
  2) Optional (``bridge_odom_to_map:=true``): static ``odom``→``map`` (identity).
  3) Optional (``bridge_body_to_base_link:=true``): static ``body``→``base_link`` (identity).

Foxy port notes:
  * ``static_transform_publisher`` uses **positional** arguments in Foxy.
  * ``SetUseSimTime`` is not available; ``use_sim_time`` is passed per node.
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    sim_raw = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    use_sim = sim_raw not in ('false', '0', 'no', 'off')
    sim_param = {'use_sim_time': use_sim}

    bridge_map = LaunchConfiguration('bridge_odom_to_map').perform(context).strip().lower() in (
        'true', '1', 'yes', 'on',
    )
    bridge_body_bl = LaunchConfiguration('bridge_body_to_base_link').perform(
        context
    ).strip().lower() in ('true', '1', 'yes', 'on')

    def _static_tf(name: str, parent: str, child: str) -> Node:
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            parameters=[sim_param],
            arguments=['0', '0', '0', '0', '0', '0', parent, child],
        )

    actions = [_static_tf('odom_to_camera_init_for_fastlio_record', 'odom', 'camera_init')]
    if bridge_map:
        actions.append(_static_tf('odom_to_map_for_fastlio_record', 'odom', 'map'))
    if bridge_body_bl:
        actions.append(_static_tf('body_to_base_link_for_fastlio_record', 'body', 'base_link'))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Must match `ros2 bag play ... --clock` and FAST-LIO.',
            ),
            DeclareLaunchArgument(
                'bridge_odom_to_map',
                default_value='false',
                description='If true, publish static odom->map (identity).',
            ),
            DeclareLaunchArgument(
                'bridge_body_to_base_link',
                default_value='false',
                description='If true, publish static body->base_link (identity).',
            ),
            OpaqueFunction(function=_setup),
        ]
    )
