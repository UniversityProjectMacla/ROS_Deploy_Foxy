#!/usr/bin/env python3
"""SLAM bringup (Foxy port, drivers removed).

Differences from the Humble (ROS_Deploy) edition:
  * No Livox driver and no Microstrain driver are started. All sensor data
    (LiDAR clouds, IMU) is expected on DDS — e.g. ``ros2 bag play <bag>``.
  * Foxy-compatible ``static_transform_publisher`` positional arguments
    (``x y z yaw pitch roll parent child``).
  * No ``SetUseSimTime`` (added in Galactic). ``use_sim_time`` is passed to
    every node as a parameter instead.

The SLAM algorithm stack itself (FAST-LIO + relay, NDT odometry, EKF,
keyframe map, pose graph) is the same as the original workspace.

**Run live on the robot (default):**
    ros2 launch ros_project_bringup launch_slam.launch.py
    # equivalent to use_lio:=true use_sim_time:=false use_rviz:=true

**Switch FAST-LIO ↔ NDT:**
    use_lio:=true  use_lidar_fusion:=false   # FAST-LIO (default)
    use_lio:=false use_lidar_fusion:=true    # NDT odometry

**Bag replay (sim time + /clock):**
    ros2 launch ros_project_bringup launch_slam.launch.py use_sim_time:=true
    ros2 bag play <bag> --clock

**Headless (no RViz):**
    ros2 launch ros_project_bringup launch_slam.launch.py use_rviz:=false
"""
from __future__ import annotations

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_slam_bringup_overlay_path(context) -> str:
    cfg_arg = LaunchConfiguration('bringup_config').perform(context).strip()
    if cfg_arg:
        p = os.path.abspath(os.path.expanduser(cfg_arg))
    else:
        envp = os.environ.get('ROS_PROJECT_SLAM_CONFIG', '').strip()
        p = os.path.abspath(os.path.expanduser(envp)) if envp else ''
    if not p:
        p = os.path.join(
            get_package_share_directory('ros_project_bringup'),
            'config',
            'slam_bringup.yaml',
        )
    if not os.path.isfile(p):
        raise FileNotFoundError(
            f'Bringup config not found: {p}. Set bringup_config:=/path or ROS_PROJECT_SLAM_CONFIG.'
        )
    return p


def _load_slam_bringup_config(overlay_path: str) -> dict:
    default_path = os.path.join(
        get_package_share_directory('ros_project_bringup'),
        'config',
        'slam_bringup.yaml',
    )
    with open(default_path, encoding='utf-8') as f:
        base_raw = yaml.safe_load(f) or {}
    if not isinstance(base_raw, dict):
        raise ValueError(f'{default_path} must be a mapping')
    if 'slam_bringup' in base_raw and isinstance(base_raw['slam_bringup'], dict):
        base = dict(base_raw['slam_bringup'])
    else:
        base = dict(base_raw)
    if os.path.normpath(overlay_path) == os.path.normpath(default_path):
        return base
    with open(overlay_path, encoding='utf-8') as f:
        ovr_raw = yaml.safe_load(f) or {}
    if not isinstance(ovr_raw, dict):
        ovr = {}
    elif 'slam_bringup' in ovr_raw and isinstance(ovr_raw['slam_bringup'], dict):
        ovr = ovr_raw['slam_bringup']
    else:
        ovr = ovr_raw
    if not isinstance(ovr, dict):
        ovr = {}
    return {**base, **dict(ovr)}


def _share_file(pkg: str, rel: str) -> str:
    rel = str(rel).strip().lstrip('/')
    return os.path.join(get_package_share_directory(pkg), rel)


def _static_tf_node(name: str, x: float, y: float, z: float,
                    roll_rad: float, pitch_rad: float, yaw_rad: float,
                    parent: str, child: str, sim_time_param: dict) -> Node:
    """Foxy static_transform_publisher: positional args (x y z yaw pitch roll parent child)."""
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        parameters=[sim_time_param],
        arguments=[
            str(x), str(y), str(z),
            str(yaw_rad), str(pitch_rad), str(roll_rad),
            parent, child,
        ],
    )


def _rviz_config_with_image_topic_override(
    rviz_cfg_path: str,
    *,
    image_topic: str,
) -> str:
    """Return base rviz path or a temp config whose first ``rviz_default_plugins/Image``
    display has ``Topic.Value`` set to ``image_topic``. No-op if the topic is empty,
    the file lacks a Visualization Manager, or there is no Image display."""
    t_raw = str(image_topic or '').strip()
    if not t_raw:
        return rviz_cfg_path
    with open(rviz_cfg_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    viz = cfg.get('Visualization Manager')
    if not isinstance(viz, dict):
        return rviz_cfg_path
    displays = viz.get('Displays')
    if not isinstance(displays, list):
        return rviz_cfg_path
    changed = False
    for d in displays:
        if (
            isinstance(d, dict)
            and d.get('Class') == 'rviz_default_plugins/Image'
        ):
            topic = d.get('Topic')
            if isinstance(topic, dict):
                topic['Value'] = t_raw
            else:
                d['Topic'] = {
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'Filter size': 10,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Value': t_raw,
                }
            changed = True
            break
    if not changed:
        return rviz_cfg_path
    fd, tmp_path = tempfile.mkstemp(prefix='perception_rviz_', suffix='.rviz')
    os.close(fd)
    with open(tmp_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return tmp_path


def _rviz_config_with_keyframe_dot_overrides(
    rviz_cfg_path: str,
    *,
    keyframe_pixels: str,
    keyframe_size_m: str,
) -> str:
    px_raw = str(keyframe_pixels or '').strip()
    m_raw = str(keyframe_size_m or '').strip()
    if not px_raw and not m_raw:
        return rviz_cfg_path
    with open(rviz_cfg_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    viz = cfg.get('Visualization Manager')
    if not isinstance(viz, dict):
        return rviz_cfg_path
    displays = viz.get('Displays')
    if not isinstance(displays, list):
        return rviz_cfg_path
    target = None
    for d in displays:
        if (
            isinstance(d, dict)
            and d.get('Name') == 'Keyframe map'
            and d.get('Class') == 'rviz_default_plugins/PointCloud2'
        ):
            target = d
            break
    if target is None:
        return rviz_cfg_path
    if px_raw:
        target['Size (Pixels)'] = max(1, int(float(px_raw)))
    if m_raw:
        target['Size (m)'] = max(0.001, float(m_raw))
    fd, tmp_path = tempfile.mkstemp(prefix='slam_rviz_', suffix='.rviz')
    os.close(fd)
    with open(tmp_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return tmp_path


def launch_setup(context, *args, **kwargs):
    U = _load_slam_bringup_config(_resolve_slam_bringup_overlay_path(context))

    _lio_arg = LaunchConfiguration('use_lio').perform(context).strip().lower()
    if _lio_arg not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['use_lio'] = _lio_arg in ('true', '1', 'yes', 'on')
    _lidar_arg = LaunchConfiguration('use_lidar_fusion').perform(context).strip().lower()
    if _lidar_arg not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['use_lidar_fusion'] = _lidar_arg in ('true', '1', 'yes', 'on')
    _rviz_arg = LaunchConfiguration('use_rviz').perform(context).strip().lower()
    if _rviz_arg not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['start_rviz'] = _rviz_arg in ('true', '1', 'yes', 'on')
    _vis_perc_arg = LaunchConfiguration('visualise_perception').perform(context).strip().lower()
    if _vis_perc_arg not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['visualise_perception'] = _vis_perc_arg in ('true', '1', 'yes', 'on')

    ust = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    use_sim_time = ust not in ('false', '0', 'no', 'off')
    sim_time_param = {'use_sim_time': use_sim_time}

    use_lidar = bool(U['use_lidar_fusion'])
    use_lio = bool(U['use_lio'])
    start_ndt = use_lidar and not use_lio
    ekf_use_lidar = use_lidar or use_lio
    fuse_z_raw = str(U['ekf_lidar_fuse_z_from_odom']).strip().lower()
    if fuse_z_raw in ('auto', ''):
        lidar_fuse_z_from_odom = use_lio
    else:
        lidar_fuse_z_from_odom = fuse_z_raw in ('true', '1', 'yes')

    lio_relay_publish_tf = bool(U.get('lio_relay_publish_tf', True))
    lio_relay_sync_tf_cloud_topic = str(
        U.get('lio_relay_sync_tf_cloud_topic', '') or ''
    ).strip()
    if (
        use_lio
        and lio_relay_publish_tf
        and not lio_relay_sync_tf_cloud_topic
        and bool(U.get('lio_relay_auto_sync_tf_cloud', True))
    ):
        lio_relay_sync_tf_cloud_topic = str(
            U.get('keyframe_cloud_topic', '/livox/lidar') or ''
        ).strip()
    if use_lio and lio_relay_publish_tf:
        ekf_publish_tf_effective = bool(U.get('ekf_publish_tf_when_lio', False))
    else:
        ekf_publish_tf_effective = bool(U['ekf_publish_tf'])

    start_rviz = bool(U['start_rviz'])
    rviz_kf_px = LaunchConfiguration('rviz_keyframe_map_size_pixels').perform(context).strip()
    rviz_kf_m = LaunchConfiguration('rviz_keyframe_map_size_m').perform(context).strip()
    start_keyframe_map = bool(U['start_keyframe_map'])
    start_pose_graph = bool(U['start_pose_graph'])
    pose_graph_pub_tf = bool(U['pose_graph_publish_map_odom_tf'])
    kf_apply_pg = bool(U['keyframe_apply_pose_graph_map'])
    if (
        start_keyframe_map
        and start_pose_graph
        and kf_apply_pg
        and pose_graph_pub_tf
    ):
        print(
            '[launch_slam] WARNING: keyframe_apply_pose_graph_map and '
            'pose_graph_publish_map_odom_tf are both true — risk of double correction.'
        )

    lx = float(U['livox_extrinsic_x'])
    ly = float(U['livox_extrinsic_y'])
    lz = float(U['livox_extrinsic_z'])
    lr = float(U['livox_extrinsic_roll_deg'])
    lp = float(U['livox_extrinsic_pitch_deg'])
    lyaw = float(U['livox_extrinsic_yaw_deg'])
    roll_rad = math.radians(lr)
    pitch_rad = math.radians(lp)
    yaw_rad = math.radians(lyaw)

    map_to_odom = _static_tf_node(
        'map_to_odom', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        'map', 'odom', sim_time_param,
    )
    base_to_livox = _static_tf_node(
        'base_link_to_livox',
        lx, ly, lz, roll_rad, pitch_rad, yaw_rad,
        'base_link',
        str(U.get('livox_cloud_frame_id', 'livox_frame') or 'livox_frame'),
        sim_time_param,
    )

    imu_px = float(U['imu_mount_x'])
    imu_py = float(U['imu_mount_y'])
    imu_pz = float(U['imu_mount_z'])
    imu_rr = float(U['imu_mount_roll_deg'])
    imu_rp = float(U['imu_mount_pitch_deg'])
    imu_ry = float(U['imu_mount_yaw_deg'])
    imu_parent = str(U['imu_mount_parent_frame']).strip()
    imu_child = str(U['imu_mount_child_frame']).strip()
    base_to_imu = _static_tf_node(
        'base_link_to_imu_link',
        imu_px, imu_py, imu_pz,
        math.radians(imu_rr), math.radians(imu_rp), math.radians(imu_ry),
        imu_parent, imu_child, sim_time_param,
    )

    livox_cloud_frame = str(U.get('livox_cloud_frame_id', 'livox_frame') or 'livox_frame').strip()
    livox_imu_child = str(U.get('livox_imu_child_frame', 'livox_imu') or 'livox_imu').strip()
    lxi_r = float(U.get('livox_imu_bridge_roll_deg', 0.0) or 0.0)
    lxi_p = float(U.get('livox_imu_bridge_pitch_deg', 0.0) or 0.0)
    lxi_y = float(U.get('livox_imu_bridge_yaw_deg', 0.0) or 0.0)
    livox_frame_to_imu_bridge = _static_tf_node(
        'livox_frame_to_livox_imu_bridge',
        0.0, 0.0, 0.0,
        math.radians(lxi_r), math.radians(lxi_p), math.radians(lxi_y),
        livox_cloud_frame, livox_imu_child, sim_time_param,
    )

    ekf_params = _share_file(U['ekf_params_pkg'], U['ekf_params_yaml'])
    keyframe_params = _share_file(U['keyframe_params_pkg'], U['keyframe_params_yaml'])

    ekf_lidar_odom = str(U['ekf_lidar_odom_topic']).strip()
    _delta_t = str(U.get('ekf_lidar_delta_topic', '') or '').strip()
    ekf_lidar_bridge = {
        'lidar_odom_topic': (ekf_lidar_odom if ekf_use_lidar else ''),
        'lidar_pose_topic': str(U['ekf_lidar_pose_topic']).strip(),
        'lidar_z_topic': str(U['ekf_lidar_z_topic']).strip(),
        'lidar_pose_var': float(U['ekf_lidar_pose_var']),
        'lidar_yaw_var': float(U['ekf_lidar_yaw_var']),
        'lidar_z_var': float(U['ekf_lidar_z_var']),
        'lidar_gate_nis': float(U['ekf_lidar_gate_nis']),
        'lidar_fuse_z_from_odom': lidar_fuse_z_from_odom,
        'lidar_require_frames': bool(U['ekf_lidar_require_frames']),
        'lidar_use_roll_pitch': bool(U['ekf_lidar_use_roll_pitch']),
        'lidar_delta_topic': (_delta_t if ekf_use_lidar else ''),
        'lidar_delta_vel_var': float(U.get('ekf_lidar_delta_vel_var', 0.22)),
        'lidar_delta_gate_nis': float(U.get('ekf_lidar_delta_gate_nis', 200.0)),
        'lidar_delta_nominal_dt_sec': float(U.get('ekf_lidar_delta_nominal_dt_sec', 0.1)),
    }

    imu_topic_effective = str(U['ekf_imu_topic']).strip()
    lidar_cloud = str(U['lidar_cloud_topic']).strip()
    # All sensors arrive via DDS / bag — sample the cloud topic stamp for TF if NDT/LIO is used.
    lidar_stamp_topic = lidar_cloud if (ekf_use_lidar and lidar_cloud) else ''

    ekf_core_overrides = {
        'imu_topic': imu_topic_effective,
        'publish_topic': str(U['ekf_publish_topic']).strip(),
        'pose_topic': str(U['ekf_pose_topic']).strip(),
        'path_topic': str(U['ekf_path_topic']).strip(),
        'odom_frame': str(U['ekf_odom_frame']).strip(),
        'base_link_frame': str(U['ekf_base_link_frame']).strip(),
        'publish_tf': ekf_publish_tf_effective,
        'nominal_dt': float(U['ekf_nominal_dt']),
        'use_stamp_dt': bool(U['ekf_use_stamp_dt']),
        'imu_source_id': 'livox',
        'imu_source_topic': str(U['ekf_imu_source_topic']).strip(),
        'lidar_cloud_stamp_topic': lidar_stamp_topic,
        'max_odom_tf_publish_rate_hz': float(U['ekf_max_odom_tf_publish_rate_hz']),
        'imu_stamp_offset_sec': float(U['ekf_imu_stamp_offset_sec']),
        'lidar_stamp_offset_sec': float(U['ekf_lidar_stamp_offset_sec']),
        'lidar_fusion_debug_log': bool(U.get('ekf_lidar_fusion_debug_log', False)),
        'lidar_fusion_debug_throttle_sec': float(
            U.get('ekf_lidar_fusion_debug_throttle_sec', 1.0)
        ),
        'lidar_soft_fuse_after_gate_reject': bool(
            U.get('ekf_lidar_soft_fuse_after_gate_reject', True)
        ),
        'lidar_fuse_xy_only': bool(U.get('ekf_lidar_fuse_xy_only', False)),
        'imu_gyro_z_bias_rad_s': float(U.get('ekf_imu_gyro_z_bias_rad_s', 0.0) or 0.0),
        'imu_auto_gyro_z_bias_enable': bool(
            U.get('ekf_imu_auto_gyro_z_bias_enable', False)
        ),
        'imu_auto_gyro_z_bias_tune_sec': float(
            U.get('ekf_imu_auto_gyro_z_bias_tune_sec', 4.0) or 4.0
        ),
        'imu_gyro_z_scale': float(U.get('ekf_imu_gyro_z_scale', 1.0) or 1.0),
    }
    if U.get('ekf_predict_use_linear_accel') is not None:
        ekf_core_overrides['predict_use_linear_accel'] = bool(
            U['ekf_predict_use_linear_accel']
        )

    lidar_odom_node = None
    if start_ndt:
        voxel = float(U['lidar_voxel_leaf_size'])
        crop = float(U['lidar_crop_range_m'])
        res = float(U['lidar_ndt_resolution'])
        max_it = int(U['lidar_ndt_max_iterations'])
        max_fit = float(U['lidar_max_fitness_score'])
        reg_mode = str(U['lidar_registration_mode']).strip()
        map_merge = float(U['lidar_map_merge_voxel_leaf_size'])
        map_max_pts = int(U['lidar_map_max_points'])
        lidar_step = float(U['lidar_ndt_step_size'])
        lidar_eps = float(U['lidar_ndt_transformation_epsilon'])
        lidar_min_pts = int(U['lidar_min_points_per_cloud'])
        lidar_pub_tf = bool(U['lidar_publish_tf'])
        lidar_use_tf_guess = bool(U['lidar_use_tf_initial_guess'])
        lidar_tf_timeout = float(U['lidar_tf_initial_guess_timeout_sec'])
        _smooth_lidar = bool(U.get('lidar_odom_smooth_enable', False))
        _pub_lidar = str(U['lidar_odom_topic']).strip()
        _raw_lidar = str(U.get('lidar_odom_raw_topic', '/lidar/odom_raw') or '/lidar/odom_raw').strip()
        _ndt_odom_out = _raw_lidar if _smooth_lidar else _pub_lidar
        lidar_params = {
            'cloud_topic': str(U['lidar_cloud_topic']).strip(),
            'odom_topic': _ndt_odom_out,
            'delta_topic': str(U['lidar_delta_topic']).strip(),
            'pose_correction_topic': str(U['lidar_pose_correction_topic']).strip(),
            'odom_frame': str(U['lidar_odom_frame']).strip(),
            'base_frame': str(U['lidar_base_frame']).strip(),
            'registration_mode': reg_mode,
            'voxel_leaf_size': voxel,
            'crop_range_m': crop,
            'ndt_resolution': res,
            'ndt_coarse_resolution': float(U.get('lidar_ndt_coarse_resolution', 0.0)),
            'ndt_step_size': lidar_step,
            'ndt_transformation_epsilon': lidar_eps,
            'ndt_voxel_min_points': int(U.get('lidar_ndt_voxel_min_points', 10)),
            'ndt_voxel_cov_eig_inflation_ratio': float(
                U.get('lidar_ndt_voxel_cov_eig_inflation_ratio', 0.05)
            ),
            'ndt_max_iterations': max_it,
            'max_fitness_score': max_fit,
            'min_points_per_cloud': lidar_min_pts,
            'publish_tf': lidar_pub_tf,
            'map_max_points': map_max_pts,
            'use_tf_initial_guess': lidar_use_tf_guess,
            'tf_initial_guess_timeout_sec': lidar_tf_timeout,
            'log_ndt_relative': bool(U.get('lidar_log_ndt_relative', False)),
            'log_registration_debug': bool(U.get('lidar_log_registration_debug', False)),
            'log_accumulated_pose': bool(U.get('lidar_log_accumulated_pose', False)),
        }
        if map_merge > 0.0:
            lidar_params['map_merge_voxel_leaf_size'] = map_merge
        lidar_params['scan_to_map_map_refresh_period'] = int(
            U.get('lidar_scan_to_map_map_refresh_period', 0)
        )
        lidar_params['scan_to_map_refresh_keep_scans'] = int(
            U.get('lidar_scan_to_map_refresh_keep_scans', 3)
        )
        lidar_params['scan_to_map_register_sensor_frame'] = bool(
            U.get('lidar_scan_to_map_register_sensor_frame', False)
        )
        lidar_params['ndt_fuse_prior_planar_yaw'] = bool(
            U.get('lidar_ndt_fuse_prior_planar_yaw', False)
        )
        lidar_params['ndt_prior_yaw_blend'] = float(U.get('lidar_ndt_prior_yaw_blend', 0.85))
        lidar_params['ndt_corridor_degeneracy_check'] = bool(
            U.get('lidar_ndt_corridor_degeneracy_check', True)
        )
        lidar_params['ndt_corridor_spin_yaw_min_rad'] = float(
            U.get('lidar_ndt_corridor_spin_yaw_min_rad', 0.5)
        )
        lidar_params['ndt_corridor_spin_max_corr_xy_m'] = float(
            U.get('lidar_ndt_corridor_spin_max_corr_xy_m', 0.1)
        )
        lidar_params['ndt_fallback_if_planar_correction_below_m'] = float(
            U.get('lidar_ndt_fallback_if_planar_correction_below_m', 0.0)
        )
        lidar_params['ndt_reject_opposite_ekf_step'] = bool(
            U.get('lidar_ndt_reject_opposite_ekf_step', False)
        )
        lidar_params['ndt_gate_until_prior_translation_m'] = float(
            U.get('lidar_ndt_gate_until_prior_translation_m', 0.0)
        )
        lidar_params['ndt_gate_force_after_sec'] = float(
            U.get('lidar_ndt_gate_force_after_sec', 0.0)
        )
        lidar_params['ndt_opposite_motion_min_ekf_step_m'] = float(
            U.get('lidar_ndt_opposite_motion_min_ekf_step_m', 0.05)
        )
        lidar_params['ndt_opposite_motion_min_ndt_step_m'] = float(
            U.get('lidar_ndt_opposite_motion_min_ndt_step_m', 0.05)
        )
        lidar_params['map_merge_keyframe_min_translation_m'] = float(
            U.get('lidar_map_merge_keyframe_min_translation_m', 0.0)
        )
        lidar_params['map_merge_keyframe_min_yaw_rad'] = math.radians(
            float(U.get('lidar_map_merge_keyframe_min_yaw_deg', 0.0))
        )
        lidar_params['sensor_extrinsic_rpy_xyz'] = [
            math.radians(lr),
            math.radians(lp),
            math.radians(lyaw),
            lx,
            ly,
            lz,
        ]
        lidar_odom_node = Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[lidar_params, sim_time_param],
        )

    ekf_node = Node(
        package='localisation_ekf',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_params, ekf_lidar_bridge, ekf_core_overrides, sim_time_param],
    )

    actions = []
    if not (start_pose_graph and pose_graph_pub_tf):
        actions.append(map_to_odom)
    # Bag replay usually lacks ``base_link``→``livox_frame`` / ``base_link``→``imu_link`` on /tf_static.
    if bool(U.get('publish_robot_static_tf_when_sensors_off', True)):
        actions.append(base_to_livox)
        actions.append(base_to_imu)
    if bool(U.get('publish_livox_imu_sensor_frame_tf', True)):
        actions.append(livox_frame_to_imu_bridge)
    actions.append(ekf_node)

    _smooth_lidar = bool(U.get('lidar_odom_smooth_enable', False))
    if lidar_odom_node is not None and _smooth_lidar:
        _pub_lidar = str(U['lidar_odom_topic']).strip()
        _raw_lidar = str(U.get('lidar_odom_raw_topic', '/lidar/odom_raw') or '/lidar/odom_raw').strip()
        _mode = str(U.get('lidar_odom_smooth_mode', 'full') or 'full').strip().lower()
        _ap = float(U.get('lidar_odom_smooth_alpha_pose', 0.18) or 0.18)
        _at = float(U.get('lidar_odom_smooth_alpha_twist_linear', 0.22) or 0.22)
        lidar_smooth_node = Node(
            package='ros_project_bringup',
            executable='lidar_odom_ema_smooth',
            name='lidar_odom_ema_smooth',
            output='screen',
            parameters=[
                {
                    'in_topic': _raw_lidar,
                    'out_topic': _pub_lidar,
                    'smooth_mode': _mode,
                    'alpha_pose': _ap,
                    'alpha_twist_linear': _at,
                },
                sim_time_param,
            ],
        )
        actions.append(lidar_smooth_node)

    if lidar_odom_node is not None:
        _ndt_delay = float(U.get('lidar_node_start_delay_sec', 0.0) or 0.0)
        if _ndt_delay > 0.0:
            actions.append(TimerAction(period=_ndt_delay, actions=[lidar_odom_node]))
        else:
            actions.append(lidar_odom_node)

    if use_lio:
        lio_sim = 'true' if use_sim_time else 'false'
        # Bag replay: merge fastlio_bag_replay_overlay so PointCloud2 lacking Livox tag/line uses lidar_type 0.
        # CRITICAL: only do this when actually replaying a bag (use_sim_time:=true). On a live MID-360 the
        # cloud includes `tag`/`line`, lidar_type 4 (MID360 handler) is correct, and forcing it to 0 sends
        # every point through ``default_handler`` with curvature=0, defeating per-point deskew. Combined
        # with an IMU mis-config it causes pose divergence -> VoxelGrid integer overflow -> "No Effective
        # Points!" -> map explosion on the live robot.
        _lio_bag_overlay = str(
            U.get('lio_bag_overlay_params_file', 'config/fastlio_bag_replay_overlay.yaml') or ''
        ).strip()
        if not use_sim_time:
            _lio_bag_overlay = ''
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('lio_bringup'),
                        'launch',
                        'lio_backend.launch.py',
                    ])
                ),
                launch_arguments=[
                    ('fastlio_params_file', TextSubstitution(text=str(U['fastlio_params_file']))),
                    ('lio_overlay_params_file', TextSubstitution(text=str(U['lio_overlay_params_file']))),
                    ('use_sim_time', lio_sim),
                    ('lio_bag_overlay_params_file', TextSubstitution(text=_lio_bag_overlay)),
                    ('lio_relay_publish_tf', 'true' if lio_relay_publish_tf else 'false'),
                    ('lio_relay_sync_tf_cloud_topic', TextSubstitution(text=lio_relay_sync_tf_cloud_topic)),
                ],
            )
        )
        if use_lio and bool(U.get('lio_auto_tf_bridge', True)):
            # ``odom -> camera_init`` keeps FAST-LIO's world tied to the robot ``odom`` frame; pairing
            # it with an identity ``body -> base_link`` connects the FAST-LIO branch to the EKF /
            # robot branch so consumers (RViz, message_filters) can resolve ``livox_frame`` against
            # ``base_link`` through either chain. Without the body bridge the two chains stay
            # disjoint and the keyframe / RViz TF lookups race against the EKF startup.
            _bridge_body_to_base = str(
                U.get('lio_auto_tf_bridge_body_to_base_link', True)
            ).strip().lower() in ('true', '1', 'yes', 'on')
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('ros_project_bringup'),
                            'launch',
                            'tf_bridge_fastlio_odom_compare.launch.py',
                        ])
                    ),
                    launch_arguments=[
                        ('use_sim_time', lio_sim),
                        ('bridge_odom_to_map', 'false'),
                        ('bridge_body_to_base_link', 'true' if _bridge_body_to_base else 'false'),
                    ],
                )
            )

    if start_keyframe_map:
        kf_loop = bool(U['keyframe_loop_closure_enable'])
        kf_deskew_imu = str(U['keyframe_deskew_imu_topic']).strip()
        kf_rot_raw = str(U.get('keyframe_deskew_imu_rotate_gyro_to_frame', '') or '').strip()
        if kf_rot_raw.lower() in ('disabled', 'none', 'false', 'off'):
            kf_gyro_to_frame = ''
        elif kf_rot_raw:
            kf_gyro_to_frame = kf_rot_raw
        else:
            kf_gyro_to_frame = ''
        keyframe_overrides = {
            'cloud_topic': str(U['keyframe_cloud_topic']).strip(),
            'map_cloud_topic': str(U['keyframe_map_cloud_topic']).strip(),
            'keyframe_path_topic': str(U['keyframe_path_topic']).strip(),
            'map_frame': str(U['keyframe_map_frame']).strip(),
            'robot_frame': str(U['keyframe_robot_frame']).strip(),
            'keyframe_min_dist_m': float(U['keyframe_min_dist_m']),
            'keyframe_min_yaw_deg': float(U['keyframe_min_yaw_deg']),
            'keyframe_min_time_sec': float(U['keyframe_min_time_sec']),
            'voxel_leaf_m': float(U['keyframe_voxel_leaf_m']),
            'max_map_points': int(U['keyframe_max_map_points']),
            'max_pts_per_scan': int(U['keyframe_max_pts_per_scan']),
            'publish_keyframe_path': bool(U['keyframe_publish_path']),
            'loop_closure_enable': kf_loop,
            'loop_min_index_gap': int(U.get('keyframe_loop_min_index_gap', 28)),
            'loop_proximity_xy_m': float(U.get('keyframe_loop_proximity_xy_m', 5.0)),
            'loop_proximity_yaw_deg': float(U.get('keyframe_loop_proximity_yaw_deg', 35.0)),
            'loop_store_voxel_leaf_m': float(U.get('keyframe_loop_store_voxel_leaf_m', 0.42)),
            'loop_max_stored_pts': int(U.get('keyframe_loop_max_stored_pts', 4500)),
            'loop_sample_points': int(U.get('keyframe_loop_sample_points', 450)),
            'loop_point_match_m': float(U.get('keyframe_loop_point_match_m', 0.38)),
            'loop_overlap_ratio': float(U.get('keyframe_loop_overlap_ratio', 0.32)),
            'loop_cooldown_sec': float(U.get('keyframe_loop_cooldown_sec', 6.0)),
            'apply_pose_graph_corrections': kf_apply_pg,
            'map_publish_min_interval_sec': float(U['keyframe_map_publish_min_interval_sec']),
            'tf_allow_latest_fallback': bool(U['keyframe_tf_allow_latest_fallback']),
            'tf_future_extrapolation_use_latest': bool(
                U['keyframe_tf_future_extrapolation_use_latest']
            ),
            'tf_lookup_timeout_sec': float(U['keyframe_tf_lookup_timeout_sec']),
            'tf_buffer_cache_sec': float(U['keyframe_tf_buffer_cache_sec']),
            'deskew_enable': bool(U['keyframe_deskew_enable']),
            'deskew_imu_topic': kf_deskew_imu,
            'deskew_max_imu_age_sec': float(U['keyframe_deskew_max_imu_age_sec']),
            'deskew_model': str(U['keyframe_deskew_model']).strip().lower(),
            'deskew_imu_sign': float(U['keyframe_deskew_imu_sign']),
            'deskew_imu_stamp_offset_sec': float(U['keyframe_deskew_imu_stamp_offset_sec']),
            'deskew_cloud_stamp_offset_sec': float(U['keyframe_deskew_cloud_stamp_offset_sec']),
            'deskew_imu_buffer_max_samples': int(U['keyframe_deskew_imu_buffer_max_samples']),
            'deskew_imu_interpolate': bool(U['keyframe_deskew_imu_interpolate']),
            'deskew_mean_gyro_fallback': bool(U['keyframe_deskew_mean_gyro_fallback']),
            'deskew_imu_rotate_gyro_to_frame': kf_gyro_to_frame,
            'deskew_imu_rotation_tf_timeout_sec': float(
                U.get('keyframe_deskew_imu_rotation_tf_timeout_sec', 0.08)
            ),
            'deskew_livox_imu_sensor_as_cloud_identity': bool(
                U.get('keyframe_deskew_livox_imu_sensor_as_cloud_identity', True)
            ),
            'rotation_adaptive_keyframes': bool(U['keyframe_rotation_adaptive_keyframes']),
            'rotation_gyro_z_thresh_rad_s': float(U['keyframe_rotation_gyro_thresh_rad_s']),
            'rotation_keyframe_scale': float(U['keyframe_rotation_keyframe_scale']),
            'reject_unstable_frame_enable': bool(
                U.get('keyframe_reject_unstable_frame_enable', False)
            ),
            'reject_unstable_frame_max_translation_m': float(
                U.get('keyframe_reject_unstable_frame_max_translation_m', 1.0)
            ),
            'reject_unstable_frame_max_yaw_deg': float(
                U.get('keyframe_reject_unstable_frame_max_yaw_deg', 45.0)
            ),
            'auto_level_enable': bool(U.get('keyframe_auto_level_enable', True)),
            'auto_level_min_keyframes': int(U.get('keyframe_auto_level_min_keyframes', 8)),
            'auto_level_min_points': int(U.get('keyframe_auto_level_min_points', 1500)),
            'auto_level_max_points': int(U.get('keyframe_auto_level_max_points', 20000)),
            'auto_level_ransac_iters': int(U.get('keyframe_auto_level_ransac_iters', 140)),
            'auto_level_plane_dist_thresh_m': float(
                U.get('keyframe_auto_level_plane_dist_thresh_m', 0.08)
            ),
            'auto_level_max_tilt_deg': float(U.get('keyframe_auto_level_max_tilt_deg', 35.0)),
            'use_lidar_odom_for_robot_pose': bool(
                U.get('keyframe_use_lidar_odom_for_robot_pose', False)
            ),
            'lidar_odom_topic': str(
                U.get('keyframe_lidar_odom_topic', '/lidar/odom') or '/lidar/odom'
            ).strip(),
            'lidar_odom_max_age_sec': float(U.get('keyframe_lidar_odom_max_age_sec', 0.15)),
            'lidar_odom_approximate_sync': bool(
                U.get('keyframe_lidar_odom_approximate_sync', True)
            ),
            'lidar_odom_approx_sync_slop_sec': float(
                U.get('keyframe_lidar_odom_approx_sync_slop_sec', 0.08)
            ),
            'prefilter_min_range_m': float(U.get('keyframe_prefilter_min_range_m', 0.5)),
            'prefilter_max_range_m': float(U.get('keyframe_prefilter_max_range_m', 20.0)),
            'prefilter_self_radius_m': float(U.get('keyframe_prefilter_self_radius_m', 0.5)),
            'prefilter_self_bbox_enable': bool(
                U.get('keyframe_prefilter_self_bbox_enable', True)
            ),
            'prefilter_self_bbox_min_x': float(U.get('keyframe_prefilter_self_bbox_min_x', -0.3)),
            'prefilter_self_bbox_max_x': float(U.get('keyframe_prefilter_self_bbox_max_x', 0.3)),
            'prefilter_self_bbox_min_y': float(U.get('keyframe_prefilter_self_bbox_min_y', -0.3)),
            'prefilter_self_bbox_max_y': float(U.get('keyframe_prefilter_self_bbox_max_y', 0.3)),
            'prefilter_self_bbox_min_z': float(U.get('keyframe_prefilter_self_bbox_min_z', -0.2)),
            'prefilter_self_bbox_max_z': float(U.get('keyframe_prefilter_self_bbox_max_z', 0.8)),
            'prefilter_intensity_enable': bool(
                U.get('keyframe_prefilter_intensity_enable', False)
            ),
            'prefilter_min_intensity': float(U.get('keyframe_prefilter_min_intensity', 10.0)),
            'prefilter_sor_enable': bool(U.get('keyframe_prefilter_sor_enable', False)),
            'prefilter_sor_mean_k': int(U.get('keyframe_prefilter_sor_mean_k', 20)),
            'prefilter_sor_stddev_mul': float(U.get('keyframe_prefilter_sor_stddev_mul', 1.0)),
            'prefilter_sor_max_points': int(U.get('keyframe_prefilter_sor_max_points', 4500)),
            'deskew_max_gyro_norm_rad_s': float(
                U.get('keyframe_deskew_max_gyro_norm_rad_s', 5.0)
            ),
        }
        keyframe_node = Node(
            package='keyframe_scan_map',
            executable='keyframe_map_node',
            name='keyframe_map_node',
            output='screen',
            parameters=[keyframe_params, keyframe_overrides, sim_time_param],
        )
        actions.append(keyframe_node)

    if start_keyframe_map and start_pose_graph:
        pose_graph_yaml = _share_file(U['pose_graph_params_pkg'], U['pose_graph_params_yaml'])
        pose_graph_overrides = {
            'publish_map_odom_tf': pose_graph_pub_tf,
            'odom_stamp_topic': str(U['ekf_publish_topic']).strip(),
            'weight_odom': float(U['pose_graph_weight_odom']),
            'weight_loop': float(U['pose_graph_weight_loop']),
            'max_graph_nodes': int(U['pose_graph_max_nodes']),
            'max_loop_edges': int(U['pose_graph_max_loop_edges']),
            'map_odom_tf_period_sec': float(U['pose_graph_map_odom_tf_period_sec']),
        }
        actions.append(
            Node(
                package='keyframe_scan_map',
                executable='pose_graph_node',
                name='pose_graph_node',
                output='screen',
                parameters=[pose_graph_yaml, pose_graph_overrides, sim_time_param],
            )
        )

    if start_rviz:
        rviz_cfg = _share_file(U['rviz_config_pkg'], U['rviz_config_yaml'])
        rviz_cfg = _rviz_config_with_keyframe_dot_overrides(
            rviz_cfg,
            keyframe_pixels=rviz_kf_px,
            keyframe_size_m=rviz_kf_m,
        )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            parameters=[sim_time_param],
        )
        actions.append(rviz_node)

    if bool(U.get('visualise_perception', False)):
        # Second rviz2 dedicated to a single perception debug image. Unique node name
        # so DDS / ros2 node list does not see two ``/rviz2`` collisions.
        perc_rviz_cfg = _share_file(
            U.get('perception_visualisation_rviz_pkg', 'ros_project_bringup'),
            U.get('perception_visualisation_rviz_yaml', 'rviz/perception_visualisation.rviz'),
        )
        perc_topic = str(
            U.get('perception_visualisation_topic', '/perception/visualisation')
            or '/perception/visualisation'
        ).strip()
        perc_rviz_cfg = _rviz_config_with_image_topic_override(
            perc_rviz_cfg, image_topic=perc_topic
        )
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_perception',
                arguments=['-d', perc_rviz_cfg],
                output='screen',
                parameters=[sim_time_param],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'If false (default): all nodes use wall-clock — appropriate for live '
                'IMU/LiDAR over DDS from the robot. Set true (and run `ros2 bag play '
                '... --clock`) for bag replay.'
            ),
        ),
        DeclareLaunchArgument(
            'bringup_config',
            default_value='',
            description=(
                'Path to a YAML (slam_bringup mapping) merged over defaults. If empty, '
                'use ROS_PROJECT_SLAM_CONFIG, else share/ros_project_bringup/config/slam_bringup.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'use_lio',
            default_value='true',
            description=(
                'Run FAST-LIO as the LiDAR-inertial front-end. Default true. '
                'Set use_lio:=false (and use_lidar_fusion:=true) to use NDT instead.'
            ),
        ),
        DeclareLaunchArgument(
            'use_lidar_fusion',
            default_value='',
            description='Override slam_bringup use_lidar_fusion (true|false). Empty = use YAML.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description=(
                'Start RViz with the SLAM display config. Default true. '
                'Set use_rviz:=false to run headless.'
            ),
        ),
        DeclareLaunchArgument(
            'visualise_perception',
            default_value='',
            description=(
                'When true, start a second rviz2 instance displaying the '
                'perception debug image (default topic /perception/visualisation, '
                'overridable in slam_bringup.yaml). Empty = use YAML value '
                '(visualise_perception, default false).'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_keyframe_map_size_pixels',
            default_value='',
            description='Optional RViz override for "Keyframe map" display Size (Pixels).',
        ),
        DeclareLaunchArgument(
            'rviz_keyframe_map_size_m',
            default_value='',
            description='Optional RViz override for "Keyframe map" display Size (m).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
