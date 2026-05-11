#!/usr/bin/env python3
"""
Keyframe scan map: optional Livox per-point ``timestamp`` + IMU **3D** deskew in the **LiDAR**
``header.frame_id`` (typically ``livox_frame``). External IMUs (e.g. Microstrain in ``imu_link``)
must have angular velocity rotated into that frame via ``deskew_imu_rotate_gyro_to_frame`` / TF.
then transform each cloud to `map`, keep scans that pass a distance/yaw/time keyframe rule,
merge and voxel-downsample, publish a single PointCloud2.

When ``use_lidar_odom_for_robot_pose`` is true, optional **approximate time sync** pairs each
cloud with ``/lidar/odom`` so pose exists before insertion (see ``lidar_odom_approximate_sync``).

Optional **simple loop closure** (`loop_closure_enable`) and optional **pose-graph map rebuild**
(`apply_pose_graph_corrections`): when `/pose_graph/corrected_keyframes` matches this node's
keyframe count, each stored map batch is transformed **T_new * inv(T_old)** per keyframe, poses
and `/keyframe_map` are rebuilt, and `/keyframe_map/keyframes` republished (Option 1+2). Use
`pose_graph_node` Option 3 (`publish_map_odom_tf`) only **without** map rebuild to avoid double
correction — see README.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

import message_filters
import numpy as np
import numpy.linalg as la
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Duration, Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32MultiArray, UInt32

# Livox / MID360 and most ROS 2 LiDAR drivers publish PointCloud2 with
# Best-Effort sensor-data QoS. A Reliable subscriber will silently fail to
# match a Best-Effort publisher, so default this subscriber to Best-Effort to
# match the live sensor. (rosbag2 replay still works because the player
# negotiates the bag's recorded QoS.) Override via the `cloud_sub_reliability`
# node parameter if you need Reliable for a specific deployment.
_LIDAR_SUB_QOS_RELIABLE = QoSProfile(
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)
_LIDAR_SUB_QOS_BEST_EFFORT = QoSProfile(
    depth=50,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)
_LIDAR_SUB_QOS = _LIDAR_SUB_QOS_BEST_EFFORT
# Match external IMU drivers (Microstrain, etc.): best-effort high rate.
_IMU_DESKEW_QOS = QoSProfile(
    depth=200,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import (
    Buffer,
    ExtrapolationException,
    TransformException,
    TransformListener,
)
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

from keyframe_scan_map.pose_graph_se2 import T_from_xyw, transform_points_se2


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def transform_matrix_from_stamped(t: TransformStamped) -> np.ndarray:
    tr = t.transform.translation
    q = t.transform.rotation
    m = quaternion_matrix([q.x, q.y, q.z, q.w])
    m[0, 3] = tr.x
    m[1, 3] = tr.y
    m[2, 3] = tr.z
    return m


def pose_to_matrix4_from_odometry(odom: Odometry) -> np.ndarray:
    """4x4: p_parent = T @ p_child (same convention as transform_matrix_from_stamped)."""
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    m = quaternion_matrix([q.x, q.y, q.z, q.w])
    m[0, 3] = float(p.x)
    m[1, 3] = float(p.y)
    m[2, 3] = float(p.z)
    return m


def transform_points_xyz(pts: np.ndarray, m: np.ndarray) -> np.ndarray:
    """pts (N,3), m 4x4 -> (N,3)"""
    if pts.size == 0:
        return pts
    n = pts.shape[0]
    h = np.ones((n, 1), dtype=np.float64)
    ph = np.hstack([pts.astype(np.float64), h])
    out = (m @ ph.T).T[:, :3]
    return out.astype(np.float32)


def voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
    if points.size == 0 or leaf <= 0:
        return points
    p = points.astype(np.float64)
    keys = np.floor(p / leaf).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inv)
    out = np.zeros((len(counts), 3), dtype=np.float32)
    for d in range(3):
        out[:, d] = (np.bincount(inv, weights=p[:, d]) / counts).astype(np.float32)
    return out


def voxel_downsample_xyz_i(
    points: np.ndarray, intensity: np.ndarray, leaf: float
) -> tuple[np.ndarray, np.ndarray]:
    """Voxel-centroid xyz + per-voxel **max** intensity (newest keyframe index wins in cell)."""
    if points.size == 0 or leaf <= 0:
        return points, intensity
    p = points.astype(np.float64)
    inte = np.asarray(intensity, dtype=np.float64).reshape(-1)
    if inte.shape[0] != p.shape[0]:
        return voxel_downsample(points, leaf), inte.astype(np.float32)
    keys = np.floor(p / leaf).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inv)
    ncell = len(counts)
    out = np.zeros((ncell, 3), dtype=np.float32)
    for d in range(3):
        out[:, d] = (np.bincount(inv, weights=p[:, d]) / counts).astype(np.float32)
    max_i = np.full(ncell, -1e30, dtype=np.float64)
    np.maximum.at(max_i, inv, inte)
    return out, max_i.astype(np.float32)


def statistical_outlier_mask(points: np.ndarray, mean_k: int, std_mul: float) -> np.ndarray:
    """Keep points with kNN mean distance <= mean + std_mul*std."""
    n = int(points.shape[0])
    if n < max(8, mean_k + 1) or mean_k < 2:
        return np.ones((n,), dtype=bool)
    p = points.astype(np.float64)
    diff = p[:, np.newaxis, :] - p[np.newaxis, :, :]
    d2 = np.sum(diff * diff, axis=2)
    np.fill_diagonal(d2, np.inf)
    k = min(max(2, int(mean_k)), n - 1)
    part = np.partition(d2, kth=k - 1, axis=1)[:, :k]
    md = np.sqrt(np.mean(part, axis=1))
    mu = float(np.mean(md))
    sd = float(np.std(md))
    th = mu + max(0.0, float(std_mul)) * sd
    return md <= th


def points_from_cloud2(msg: PointCloud2) -> Optional[np.ndarray]:
    """Livox (and others) often use float32 x,y,z with padded point_step; read_points returns a structured array."""
    try:
        pts = list(
            point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True
            )
        )
    except Exception:
        return None
    if len(pts) == 0:
        return None
    raw = np.array(pts)
    if raw.dtype.names and all(f in raw.dtype.names for f in ('x', 'y', 'z')):
        arr = np.column_stack(
            (
                raw['x'].astype(np.float64),
                raw['y'].astype(np.float64),
                raw['z'].astype(np.float64),
            )
        )
    elif raw.ndim == 2 and raw.shape[1] >= 3:
        arr = raw[:, :3].astype(np.float64, copy=False)
    else:
        arr = np.array(
            [[float(p[0]), float(p[1]), float(p[2])] for p in pts],
            dtype=np.float64,
        )
    arr = arr[np.isfinite(arr).all(axis=1)]
    if arr.shape[0] == 0:
        return None
    return arr.astype(np.float32)


def points_xyz_intensity_timestamp(
    msg: PointCloud2,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """Return xyz plus optional intensity and optional Livox timestamp fields."""
    names = [f.name for f in msg.fields]
    has_i = 'intensity' in names
    has_t = 'timestamp' in names
    fields: list[str] = ['x', 'y', 'z']
    if has_i:
        fields.append('intensity')
    if has_t:
        fields.append('timestamp')
    try:
        rows = list(point_cloud2.read_points(msg, field_names=tuple(fields), skip_nans=True))
    except Exception:
        xyz = points_from_cloud2(msg)
        return xyz, None, None
    if len(rows) == 0:
        return None, None, None
    arr = np.array(rows)
    if arr.dtype.names and all(n in arr.dtype.names for n in ('x', 'y', 'z')):
        xyz = np.column_stack(
            (
                arr['x'].astype(np.float64),
                arr['y'].astype(np.float64),
                arr['z'].astype(np.float64),
            )
        )
        inten = np.asarray(arr['intensity'], dtype=np.float64) if has_i else None
        ts = np.asarray(arr['timestamp'], dtype=np.float64) if has_t else None
    else:
        xyz = np.array([[float(r[0]), float(r[1]), float(r[2])] for r in rows], dtype=np.float64)
        col = 3
        inten = np.array([float(r[col]) for r in rows], dtype=np.float64) if has_i else None
        if has_i:
            col += 1
        ts = np.array([float(r[col]) for r in rows], dtype=np.float64) if has_t else None
    mask = np.isfinite(xyz).all(axis=1)
    if inten is not None:
        mask = mask & np.isfinite(inten)
    if ts is not None:
        mask = mask & np.isfinite(ts)
    xyz = xyz[mask]
    if xyz.shape[0] == 0:
        return None, None, None
    inten_out = inten[mask].astype(np.float32) if inten is not None else None
    ts_out = ts[mask] if ts is not None else None
    return xyz.astype(np.float32), inten_out, ts_out


def deskew_points_to_scan_end_varying(
    pts: np.ndarray,
    dt_sec: np.ndarray,
    omega: np.ndarray,
    *,
    sign: float = 1.0,
    model: str = 'rodrigues',
) -> np.ndarray:
    """Deskew with per-row angular velocity ``omega`` (N,3) and ``dt_sec`` (N,) to scan end."""
    mdl = (model or 'rodrigues').strip().lower()
    if pts.size == 0:
        return pts
    dt_sec = np.asarray(dt_sec, dtype=np.float64).reshape(-1)
    n = int(pts.shape[0])
    if dt_sec.shape[0] != n or omega.shape[0] != n:
        return pts
    w = np.asarray(omega, dtype=np.float64).reshape(n, 3) * float(sign)

    if mdl == 'yaw_only':
        wz = w[:, 2]
        ang = wz * dt_sec
        c = np.cos(ang)
        s = np.sin(ang)
        x = pts[:, 0].astype(np.float64)
        y = pts[:, 1].astype(np.float64)
        xd = c * x - s * y
        yd = s * x + c * y
        return np.column_stack([xd, yd, pts[:, 2]]).astype(np.float32)

    wnorm = np.linalg.norm(w, axis=1, keepdims=True)
    wnorm = np.maximum(wnorm, 1e-9)
    u = w / wnorm
    theta = wnorm * np.asarray(dt_sec, dtype=np.float64).reshape(-1, 1)
    p = pts.astype(np.float64)
    uxp = np.cross(u, p)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    udot = (p * u).sum(axis=1, keepdims=True)
    out = p * cos_t + uxp * sin_t + u * udot * (1.0 - cos_t)
    return out.astype(np.float32)


def deskew_points_to_scan_end(
    pts: np.ndarray,
    ts: np.ndarray,
    wx: float,
    wy: float,
    wz: float,
    *,
    sign: float = 1.0,
    model: str = 'rodrigues',
) -> np.ndarray:
    """Motion-compensate points in the **sensor** frame toward the end of the frame (Livox ``timestamp``).

    Livox IMU gyro is in ``livox_frame``; points are in the same frame. Uses constant angular
    velocity over the frame: **yaw_only** rotates x,y by ωz·dt; **rodrigues** applies the exact
    rotation about axis ω̂ for angle ‖ω‖·dt per point (handles tilting / off-axis spins).
    """
    mdl = (model or 'rodrigues').strip().lower()
    if pts.size == 0:
        return pts
    ts = np.asarray(ts, dtype=np.float64).reshape(-1)
    if ts.shape[0] != pts.shape[0]:
        return pts
    mx = float(np.max(ts))
    if not math.isfinite(mx):
        return pts
    # Livox driver stores offset_time as double (typically ns from frame start).
    scale = 1e-9 if mx > 1e4 else 1.0
    dt_sec = (mx - ts) * scale

    wz = float(sign) * float(wz)
    wx = float(sign) * float(wx)
    wy = float(sign) * float(wy)

    if mdl == 'yaw_only':
        if abs(wz) < 1e-9:
            return pts
        ang = wz * dt_sec
        c = np.cos(ang)
        s = np.sin(ang)
        x = pts[:, 0].astype(np.float64)
        y = pts[:, 1].astype(np.float64)
        xd = c * x - s * y
        yd = s * x + c * y
        return np.column_stack([xd, yd, pts[:, 2]]).astype(np.float32)

    omega = np.array([wx, wy, wz], dtype=np.float64)
    wnorm = float(np.linalg.norm(omega))
    if wnorm < 1e-9:
        return pts
    n = int(pts.shape[0])
    p = pts.astype(np.float64)
    u = omega / wnorm
    theta = wnorm * dt_sec
    u_b = np.broadcast_to(u, (n, 3))
    uxp = np.cross(u_b, p)
    cos_t = np.cos(theta)[:, np.newaxis]
    sin_t = np.sin(theta)[:, np.newaxis]
    udot = (p * u_b).sum(axis=1, keepdims=True)
    out = p * cos_t + uxp * sin_t + u_b * udot * (1.0 - cos_t)
    return out.astype(np.float32)


def livox_point_times_abs_ns(
    header_stamp,
    ts: np.ndarray,
    *,
    cloud_stamp_offset_sec: float,
) -> Optional[np.ndarray]:
    """Per-point absolute time (ns) aligned with ``header.stamp`` sweep end + offset."""
    if int(header_stamp.sec) == 0 and int(header_stamp.nanosec) == 0:
        return None
    t_end = Time.from_msg(header_stamp) + Duration(seconds=float(cloud_stamp_offset_sec))
    te = np.int64(t_end.nanoseconds)
    ts = np.asarray(ts, dtype=np.float64).reshape(-1)
    mx = float(np.max(ts))
    if not math.isfinite(mx):
        return None
    if mx > 1e4:
        delta_ns = (mx - ts).astype(np.int64)
    else:
        delta_ns = ((mx - ts) * 1e9).astype(np.int64)
    return te - delta_ns


def interp_gyro_batch(
    t_query_ns: np.ndarray, times_ns: np.ndarray, w: np.ndarray
) -> np.ndarray:
    """Linear interpolation of angular velocity (N,3) at query times (N,) from (M,) / (M,3)."""
    m = int(times_ns.shape[0])
    n = int(t_query_ns.shape[0])
    idx = np.searchsorted(times_ns, t_query_ns, side='right')
    idx = np.clip(idx, 1, m - 1)
    t0 = times_ns[idx - 1].astype(np.float64)
    t1 = times_ns[idx].astype(np.float64)
    denom = np.maximum((t1 - t0).astype(np.float64), 1.0)
    a = ((t_query_ns.astype(np.float64) - t0) / denom).reshape(-1, 1)
    return (1.0 - a) * w[idx - 1] + a * w[idx]


def numpy_xyz_to_pointcloud2(
    points: np.ndarray,
    header_frame: str,
    stamp,
    node: Node,
    *,
    intensity: Optional[np.ndarray] = None,
) -> PointCloud2:
    """If ``intensity`` is set (length N), adds ``intensity`` field for RViz rainbow (FAST-LIO style)."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = header_frame
    n = int(points.shape[0])
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    if n > 0:
        xyz = np.ascontiguousarray(points[:, :3], dtype=np.float32)
    else:
        xyz = np.zeros((0, 3), dtype=np.float32)
    if intensity is not None and n > 0:
        inte = np.asarray(intensity, dtype=np.float32).reshape(-1)
        if inte.shape[0] != n:
            inte = xyz[:, 2].copy()
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * n
        blk = np.hstack([xyz, inte.reshape(-1, 1)])
        msg.data = np.ascontiguousarray(blk, dtype=np.float32).tobytes()
    else:
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * n
        msg.data = xyz.tobytes() if n > 0 else b''
    return msg


def overlap_ratio(
    pts_a: np.ndarray,
    pts_b: np.ndarray,
    n_samples: int,
    match_m: float,
    rng: np.random.Generator,
    max_b_pts: int = 3500,
) -> float:
    """Fraction of subsampled points in a with a neighbor in b within match_m (both map frame)."""
    if pts_a.shape[0] < 20 or pts_b.shape[0] < 20:
        return 0.0
    n = min(n_samples, pts_a.shape[0])
    idx_a = rng.choice(pts_a.shape[0], size=n, replace=False)
    a_sub = pts_a[idx_a].astype(np.float64)
    if pts_b.shape[0] > max_b_pts:
        idx_b = rng.choice(pts_b.shape[0], size=max_b_pts, replace=False)
        b_sub = pts_b[idx_b].astype(np.float64)
    else:
        b_sub = pts_b.astype(np.float64)
    # (n, 1, 3) - (1, M, 3) -> (n, M) distances
    diff = a_sub[:, np.newaxis, :] - b_sub[np.newaxis, :, :]
    d2 = np.sum(diff * diff, axis=2)
    d_min = np.sqrt(np.min(d2, axis=1))
    return float(np.mean(d_min < match_m))


def _rotation_matrix_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues formula for a 3x3 rotation matrix."""
    ax = np.asarray(axis, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(ax))
    if n < 1e-12 or abs(angle) < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z = ax / n
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c
    return np.array(
        [
            [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
        ],
        dtype=np.float64,
    )


class KeyframeMapNode(Node):
    def __init__(self) -> None:
        super().__init__('keyframe_map_node')

        self.declare_parameter('cloud_topic', '/livox/lidar')
        self.declare_parameter('map_cloud_topic', '/keyframe_map')
        self.declare_parameter('keyframe_path_topic', '/keyframe_map/keyframes')
        # QoS reliability for the LiDAR cloud + odom subscriptions.
        # 'best_effort' (default) matches live Livox/MID360 drivers; rosbag2
        # replay still negotiates fine. Set to 'reliable' only for unusual
        # deployments that publish PointCloud2 with Reliable QoS.
        self.declare_parameter('cloud_sub_reliability', 'best_effort')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('keyframe_min_dist_m', 0.45)
        self.declare_parameter('keyframe_min_yaw_deg', 10.0)
        self.declare_parameter('keyframe_min_time_sec', 1.5)
        self.declare_parameter('voxel_leaf_m', 0.22)
        self.declare_parameter('max_map_points', 2_500_000)
        self.declare_parameter('max_pts_per_scan', 80_000)
        self.declare_parameter('publish_keyframe_path', True)

        self.declare_parameter('loop_closure_enable', False)
        self.declare_parameter('loop_min_index_gap', 28)
        self.declare_parameter('loop_proximity_xy_m', 5.0)
        self.declare_parameter('loop_proximity_yaw_deg', 35.0)
        self.declare_parameter('loop_store_voxel_leaf_m', 0.42)
        self.declare_parameter('loop_max_stored_pts', 4500)
        self.declare_parameter('loop_sample_points', 450)
        self.declare_parameter('loop_point_match_m', 0.38)
        self.declare_parameter('loop_overlap_ratio', 0.32)
        self.declare_parameter('loop_cooldown_sec', 6.0)
        self.declare_parameter(
            'loop_closure_pair_topic', '/keyframe_map/loop_closure_pair'
        )
        self.declare_parameter('apply_pose_graph_corrections', False)
        self.declare_parameter(
            'pose_graph_corrected_path_topic', '/pose_graph/corrected_keyframes'
        )
        self.declare_parameter('map_batch_store_voxel_m', 0.32)
        # Min wall time between full /keyframe_map publishes (map still merges every keyframe).
        self.declare_parameter('map_publish_min_interval_sec', 0.0)
        # If false, skip keyframe when TF at cloud stamp is missing (avoids livox vs pose mismatch).
        self.declare_parameter('tf_allow_latest_fallback', True)
        # Use /lidar/odom (or NDT/LIO topic) for T_odom<-base at cloud time; only TF for map<-odom
        # and base<-livox extrinsic. Avoids EKF lag / sparse odom TF at scan insertion.
        self.declare_parameter('use_lidar_odom_for_robot_pose', False)
        self.declare_parameter('lidar_odom_topic', '/lidar/odom')
        self.declare_parameter('lidar_odom_max_age_sec', 0.15)
        # Pair cloud + /lidar/odom by stamp (avoids processing cloud before matching odom arrives).
        self.declare_parameter('lidar_odom_approximate_sync', True)
        self.declare_parameter('lidar_odom_approx_sync_slop_sec', 0.08)
        # Prefilter in sensor frame to remove self-hits / distant clutter before map insertion.
        self.declare_parameter('prefilter_min_range_m', 0.5)
        self.declare_parameter('prefilter_max_range_m', 20.0)
        self.declare_parameter('prefilter_self_radius_m', 0.5)
        self.declare_parameter('prefilter_self_bbox_enable', True)
        self.declare_parameter('prefilter_self_bbox_min_x', -0.3)
        self.declare_parameter('prefilter_self_bbox_max_x', 0.3)
        self.declare_parameter('prefilter_self_bbox_min_y', -0.3)
        self.declare_parameter('prefilter_self_bbox_max_y', 0.3)
        self.declare_parameter('prefilter_self_bbox_min_z', -0.2)
        self.declare_parameter('prefilter_self_bbox_max_z', 0.8)
        self.declare_parameter('prefilter_intensity_enable', False)
        self.declare_parameter('prefilter_min_intensity', 10.0)
        self.declare_parameter('prefilter_sor_enable', False)
        self.declare_parameter('prefilter_sor_mean_k', 20)
        self.declare_parameter('prefilter_sor_stddev_mul', 1.0)
        self.declare_parameter('prefilter_sor_max_points', 4500)
        # Skip deskew update when gyro norm is implausibly large (protect against spikes).
        self.declare_parameter('deskew_max_gyro_norm_rad_s', 5.0)
        # If strict TF (above false), still use latest TF when stamp is slightly ahead of the buffer
        # (Livox cloud time vs EKF publish order — avoids dropped keyframes / map tearing).
        self.declare_parameter('tf_future_extrapolation_use_latest', True)
        self.declare_parameter('tf_lookup_timeout_sec', 0.55)
        self.declare_parameter('tf_buffer_cache_sec', 180.0)
        # Livox per-point ``timestamp`` + IMU yaw-rate deskew (reduces smear when rotating).
        self.declare_parameter('deskew_enable', True)
        self.declare_parameter('deskew_imu_topic', '/livox/imu')
        self.declare_parameter('deskew_max_imu_age_sec', 0.12)
        # rodrigues: full 3D gyro deskew (handheld / tilted spin); yaw_only: ωz on x,y only.
        self.declare_parameter('deskew_model', 'rodrigues')
        # If map shears the wrong way vs rotation, try -1.0 (sensor/driver convention).
        self.declare_parameter('deskew_imu_sign', 1.0)
        # Tighter keyframe spacing while ‖ω‖ is high (more overlap during fast motion).
        self.declare_parameter('rotation_adaptive_keyframes', True)
        self.declare_parameter('rotation_gyro_z_thresh_rad_s', 0.45)
        self.declare_parameter('rotation_keyframe_scale', 0.5)
        # Optional safety gate: drop clouds that imply impossible pose jumps vs last keyframe.
        self.declare_parameter('reject_unstable_frame_enable', False)
        self.declare_parameter('reject_unstable_frame_max_translation_m', 1.0)
        self.declare_parameter('reject_unstable_frame_max_yaw_deg', 45.0)
        # Auto-level map from dominant horizontal plane (floor or ceiling), applied once.
        self.declare_parameter('auto_level_enable', True)
        self.declare_parameter('auto_level_min_keyframes', 8)
        self.declare_parameter('auto_level_min_points', 1500)
        self.declare_parameter('auto_level_max_points', 20000)
        self.declare_parameter('auto_level_ransac_iters', 140)
        self.declare_parameter('auto_level_plane_dist_thresh_m', 0.08)
        self.declare_parameter('auto_level_max_tilt_deg', 35.0)
        # Clock sync: add to IMU / cloud stamps so external IMU + Livox LiDAR share one timeline.
        self.declare_parameter('deskew_imu_stamp_offset_sec', 0.0)
        self.declare_parameter('deskew_cloud_stamp_offset_sec', 0.0)
        self.declare_parameter('deskew_imu_buffer_max_samples', 512)
        self.declare_parameter('deskew_imu_interpolate', True)
        # If varying-gyro deskew fails, average IMU in buffer over each point's time span (rotation).
        self.declare_parameter('deskew_mean_gyro_fallback', True)
        # If set (e.g. livox_frame): rotate Imu.angular_velocity from header.frame_id into this
        # frame before deskew. Required when deskew uses Microstrain (imu_link) but points are in livox_frame.
        self.declare_parameter('deskew_imu_rotate_gyro_to_frame', '')
        self.declare_parameter('deskew_imu_rotation_tf_timeout_sec', 0.08)
        # Livox /livox/imu often uses header ``sensor`` (MID360). That is NOT the same frame as
        # Microstrain ``/imu/data`` ``sensor``; do not TF-rotate Livox gyro via the GX5 tree.
        # When true and deskew IMU is ``/livox/imu``, treat ``sensor`` as already in the rotate target basis.
        self.declare_parameter('deskew_livox_imu_sensor_as_cloud_identity', True)

        self._cloud_topic = self.get_parameter('cloud_topic').value
        self._map_topic = self.get_parameter('map_cloud_topic').value
        self._path_topic = self.get_parameter('keyframe_path_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._kf_dist = float(self.get_parameter('keyframe_min_dist_m').value)
        self._kf_yaw = math.radians(
            float(self.get_parameter('keyframe_min_yaw_deg').value)
        )
        self._kf_time = float(self.get_parameter('keyframe_min_time_sec').value)
        self._voxel = float(self.get_parameter('voxel_leaf_m').value)
        self._max_map = int(self.get_parameter('max_map_points').value)
        self._max_scan = int(self.get_parameter('max_pts_per_scan').value)
        pub_path_enable = bool(self.get_parameter('publish_keyframe_path').value)

        self._loop_enable = bool(self.get_parameter('loop_closure_enable').value)
        self._loop_min_gap = max(2, int(self.get_parameter('loop_min_index_gap').value))
        self._loop_xy = float(self.get_parameter('loop_proximity_xy_m').value)
        self._loop_yaw = math.radians(
            float(self.get_parameter('loop_proximity_yaw_deg').value)
        )
        self._loop_store_leaf = float(self.get_parameter('loop_store_voxel_leaf_m').value)
        self._loop_max_store = int(self.get_parameter('loop_max_stored_pts').value)
        self._loop_samples = int(self.get_parameter('loop_sample_points').value)
        self._loop_match_m = float(self.get_parameter('loop_point_match_m').value)
        self._loop_ov_thresh = float(self.get_parameter('loop_overlap_ratio').value)
        self._loop_cooldown_ns = int(
            float(self.get_parameter('loop_cooldown_sec').value) * 1e9
        )
        self._loop_pair_topic = str(
            self.get_parameter('loop_closure_pair_topic').value
        )
        self._apply_pg = bool(self.get_parameter('apply_pose_graph_corrections').value)
        self._pg_path_topic = str(
            self.get_parameter('pose_graph_corrected_path_topic').value
        )
        self._map_batch_leaf = float(self.get_parameter('map_batch_store_voxel_m').value)
        self._map_pub_min_ns = int(
            float(self.get_parameter('map_publish_min_interval_sec').value) * 1e9
        )
        self._tf_allow_latest_fallback = bool(
            self.get_parameter('tf_allow_latest_fallback').value
        )
        self._tf_future_extrap_latest = bool(
            self.get_parameter('tf_future_extrapolation_use_latest').value
        )
        self._tf_timeout = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self._tf_cache_sec = float(self.get_parameter('tf_buffer_cache_sec').value)
        self._use_lidar_odom_pose = bool(
            self.get_parameter('use_lidar_odom_for_robot_pose').value
        )
        self._lidar_odom_topic = str(
            self.get_parameter('lidar_odom_topic').value
        ).strip()
        self._lidar_odom_max_age_ns = int(
            max(1e-6, float(self.get_parameter('lidar_odom_max_age_sec').value)) * 1e9
        )
        self._lidar_odom_approx_sync = bool(
            self.get_parameter('lidar_odom_approximate_sync').value
        )
        self._lidar_odom_sync_slop = max(
            0.0, float(self.get_parameter('lidar_odom_approx_sync_slop_sec').value)
        )
        self._pf_min_r = max(0.0, float(self.get_parameter('prefilter_min_range_m').value))
        self._pf_max_r = max(self._pf_min_r, float(self.get_parameter('prefilter_max_range_m').value))
        self._pf_self_r = max(0.0, float(self.get_parameter('prefilter_self_radius_m').value))
        self._pf_bbox_en = bool(self.get_parameter('prefilter_self_bbox_enable').value)
        self._pf_bbox = (
            float(self.get_parameter('prefilter_self_bbox_min_x').value),
            float(self.get_parameter('prefilter_self_bbox_max_x').value),
            float(self.get_parameter('prefilter_self_bbox_min_y').value),
            float(self.get_parameter('prefilter_self_bbox_max_y').value),
            float(self.get_parameter('prefilter_self_bbox_min_z').value),
            float(self.get_parameter('prefilter_self_bbox_max_z').value),
        )
        self._pf_i_en = bool(self.get_parameter('prefilter_intensity_enable').value)
        self._pf_i_min = float(self.get_parameter('prefilter_min_intensity').value)
        self._pf_sor_en = bool(self.get_parameter('prefilter_sor_enable').value)
        self._pf_sor_k = int(self.get_parameter('prefilter_sor_mean_k').value)
        self._pf_sor_std = float(self.get_parameter('prefilter_sor_stddev_mul').value)
        self._pf_sor_max_pts = int(self.get_parameter('prefilter_sor_max_points').value)
        self._deskew_max_gyro_norm = float(
            self.get_parameter('deskew_max_gyro_norm_rad_s').value
        )
        self._warned_deskew_gyro_spike = False
        self._deskew_enable = bool(self.get_parameter('deskew_enable').value)
        self._deskew_imu_topic = str(self.get_parameter('deskew_imu_topic').value).strip()
        self._deskew_max_imu_age = float(
            self.get_parameter('deskew_max_imu_age_sec').value
        )
        _dm = str(self.get_parameter('deskew_model').value).strip().lower()
        self._deskew_model = _dm if _dm else 'rodrigues'
        self._deskew_sign = float(self.get_parameter('deskew_imu_sign').value)
        self._rot_adapt = bool(self.get_parameter('rotation_adaptive_keyframes').value)
        # Threshold on ‖ω‖ (rad/s), not only ωz — handheld rotation is rarely pure z.
        self._rot_gyro_thresh = float(
            self.get_parameter('rotation_gyro_z_thresh_rad_s').value
        )
        self._rot_scale = float(self.get_parameter('rotation_keyframe_scale').value)
        self._reject_unstable_frame_enable = bool(
            self.get_parameter('reject_unstable_frame_enable').value
        )
        self._reject_unstable_frame_max_translation_m = max(
            0.0,
            float(self.get_parameter('reject_unstable_frame_max_translation_m').value),
        )
        self._reject_unstable_frame_max_yaw_rad = math.radians(
            max(0.0, float(self.get_parameter('reject_unstable_frame_max_yaw_deg').value))
        )
        self._auto_level_enable = bool(self.get_parameter('auto_level_enable').value)
        self._auto_level_min_kf = max(
            1, int(self.get_parameter('auto_level_min_keyframes').value)
        )
        self._auto_level_min_pts = max(
            100, int(self.get_parameter('auto_level_min_points').value)
        )
        self._auto_level_max_pts = max(
            1000, int(self.get_parameter('auto_level_max_points').value)
        )
        self._auto_level_ransac_iters = max(
            10, int(self.get_parameter('auto_level_ransac_iters').value)
        )
        self._auto_level_dist = max(
            1e-3, float(self.get_parameter('auto_level_plane_dist_thresh_m').value)
        )
        self._auto_level_max_tilt_deg = float(
            self.get_parameter('auto_level_max_tilt_deg').value
        )
        self._deskew_imu_stamp_off = float(
            self.get_parameter('deskew_imu_stamp_offset_sec').value
        )
        self._deskew_cloud_stamp_off = float(
            self.get_parameter('deskew_cloud_stamp_offset_sec').value
        )
        _buf_max = max(32, int(self.get_parameter('deskew_imu_buffer_max_samples').value))
        self._deskew_interp = bool(self.get_parameter('deskew_imu_interpolate').value)
        self._deskew_mean_fallback = bool(
            self.get_parameter('deskew_mean_gyro_fallback').value
        )
        self._deskew_rotate_gyro_to = str(
            self.get_parameter('deskew_imu_rotate_gyro_to_frame').value
        ).strip()
        self._deskew_imu_rot_tf_timeout = float(
            self.get_parameter('deskew_imu_rotation_tf_timeout_sec').value
        )
        self._deskew_livox_sensor_cloud_identity = bool(
            self.get_parameter('deskew_livox_imu_sensor_as_cloud_identity').value
        )
        self._warned_imu_gyro_tf = False
        self._warned_cloud_frame_mismatch = False
        self._imu_buf: Deque[Tuple[int, float, float, float]] = deque(maxlen=_buf_max)
        self._last_map_pub_mono_ns: Optional[int] = None
        self._lidar_odom_buf: Deque[Odometry] = deque(maxlen=160)
        self._warned_lidar_odom_miss = False
        self._use_lidar_odom_sync = (
            self._use_lidar_odom_pose
            and bool(self._lidar_odom_topic)
            and self._lidar_odom_approx_sync
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=self._tf_cache_sec))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._last_imu_w = np.zeros(3, dtype=np.float64)
        self._last_imu_recv = None  # rclpy.time.Time (receive time; wall-clock fallback only)
        self._last_imu_msg_stamp = None  # IMU time (offset) for deskew gate vs cloud

        _qos_kind = str(
            self.get_parameter('cloud_sub_reliability').value
        ).strip().lower()
        if _qos_kind in ('reliable', 'rel', 'r'):
            cloud_sub_qos = _LIDAR_SUB_QOS_RELIABLE
        else:
            cloud_sub_qos = _LIDAR_SUB_QOS_BEST_EFFORT
        self.get_logger().info(
            f'keyframe_map: cloud subscription QoS = '
            f'{cloud_sub_qos.reliability.name} (param cloud_sub_reliability="{_qos_kind}")'
        )

        if self._use_lidar_odom_sync:
            sub_cloud = message_filters.Subscriber(
                self, PointCloud2, self._cloud_topic, qos_profile=cloud_sub_qos
            )
            sub_odom = message_filters.Subscriber(
                self, Odometry, self._lidar_odom_topic, qos_profile=cloud_sub_qos
            )
            self._ts_cloud_odom = message_filters.ApproximateTimeSynchronizer(
                [sub_cloud, sub_odom],
                queue_size=40,
                slop=self._lidar_odom_sync_slop,
            )
            self._ts_cloud_odom.registerCallback(self._on_cloud_odom_synced)
        else:
            self.create_subscription(
                PointCloud2,
                self._cloud_topic,
                self._on_cloud,
                cloud_sub_qos,
            )
            if self._use_lidar_odom_pose and self._lidar_odom_topic:
                self.create_subscription(
                    Odometry,
                    self._lidar_odom_topic,
                    self._on_lidar_odom,
                    cloud_sub_qos,
                )
        self._pub_map = self.create_publisher(PointCloud2, self._map_topic, 1)
        self._pub_path = (
            self.create_publisher(Path, self._path_topic, 10)
            if pub_path_enable
            else None
        )
        self._pub_loop_idx: Optional[rclpy.publisher.Publisher] = None
        self._pub_loop_score: Optional[rclpy.publisher.Publisher] = None
        self._pub_loop_pose: Optional[rclpy.publisher.Publisher] = None
        self._pub_loop_pair: Optional[rclpy.publisher.Publisher] = None
        if self._loop_enable:
            self._pub_loop_idx = self.create_publisher(
                UInt32, '/keyframe_map/loop_closure_match_index', 10
            )
            self._pub_loop_score = self.create_publisher(
                Float32, '/keyframe_map/loop_closure_overlap', 10
            )
            self._pub_loop_pose = self.create_publisher(
                PoseStamped, '/keyframe_map/loop_closure_anchor_pose', 10
            )
            self._pub_loop_pair = self.create_publisher(
                Int32MultiArray, self._loop_pair_topic, 10
            )

        self._map_pts: Optional[np.ndarray] = None
        # Per-point RViz intensity: keyframe index (newer = higher); cleared on pose-graph rebuild.
        self._map_intensity: Optional[np.ndarray] = None
        self._kf_poses: List[Tuple[float, float, float]] = []
        self._kf_scan_store: List[np.ndarray] = []
        self._kf_map_batches: List[np.ndarray] = []
        self._last_kf: Optional[Tuple[float, float, float, int]] = None
        self._kf_count = 0
        self._cloud_rx_count = 0
        self._rng = np.random.default_rng(seed=42)
        self._last_loop_pub_ns = 0
        self._auto_level_done = False
        # Persistent map-level correction (roll/pitch) applied to all future inserts/poses.
        self._level_R = np.eye(3, dtype=np.float64)
        self._last_pg_applied: Optional[np.ndarray] = None

        if self._apply_pg:
            self.create_subscription(Path, self._pg_path_topic, self._on_pose_graph_path, 10)

        need_imu = (self._deskew_enable or self._rot_adapt) and bool(self._deskew_imu_topic)
        if need_imu:
            self.create_subscription(
                Imu,
                self._deskew_imu_topic,
                self._on_imu,
                _IMU_DESKEW_QOS,
            )

        self.get_logger().info(
            f'keyframe_map: cloud={self._cloud_topic} -> {self._map_topic} '
            f'frame={self._map_frame} robot={self._robot_frame} '
            f'kf_dist={self._kf_dist}m kf_yaw={math.degrees(self._kf_yaw):.1f}deg '
            f'loop_closure={self._loop_enable} apply_pg_map={self._apply_pg} '
            f'map_pub_min_s={self._map_pub_min_ns * 1e-9:.2f} tf_latest_fallback={self._tf_allow_latest_fallback} '
            f'lidar_odom_pose={self._use_lidar_odom_pose}({self._lidar_odom_topic or "off"})'
            f'{" approx_sync=" + str(self._lidar_odom_sync_slop) + "s" if self._use_lidar_odom_sync else ""} '
            f'deskew={self._deskew_enable}({self._deskew_model}) interp={self._deskew_interp} '
            f'imu_off={self._deskew_imu_stamp_off:g}s cloud_off={self._deskew_cloud_stamp_off:g}s '
            f'imu_buf={_buf_max} rot_adapt={self._rot_adapt} '
            f'gyro_rot_to={self._deskew_rotate_gyro_to or "off"} '
            f'auto_level={self._auto_level_enable} '
            f'prefilter[r={self._pf_min_r:g}-{self._pf_max_r:g}m self_r={self._pf_self_r:g} '
            f'i={("on" if self._pf_i_en else "off")} sor={("on" if self._pf_sor_en else "off")}] '
            f'deskew_gyro_max={self._deskew_max_gyro_norm:g}rad/s '
            f'tf_cache={self._tf_cache_sec:.0f}s tf_future_fallback={self._tf_future_extrap_latest}'
        )

    def _cloud_stamp_adjusted(self, msg: PointCloud2) -> Time:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return Time()
        return Time.from_msg(msg.header.stamp) + Duration(
            seconds=float(self._deskew_cloud_stamp_off)
        )

    def _on_lidar_odom(self, msg: Odometry) -> None:
        self._lidar_odom_buf.append(msg)

    def _lidar_odom_nearest(self, stamp: Time) -> Optional[Odometry]:
        if not self._lidar_odom_buf:
            return None
        tgt = int(stamp.nanoseconds)
        best: Optional[Odometry] = None
        best_dt = self._lidar_odom_max_age_ns + 1
        for odom in self._lidar_odom_buf:
            t = int(Time.from_msg(odom.header.stamp).nanoseconds)
            dt = abs(t - tgt)
            if dt < best_dt:
                best_dt = dt
                best = odom
        if best is None or best_dt > self._lidar_odom_max_age_ns:
            return None
        return best

    def _compose_map_T_sensor(
        self,
        stamp: Time,
        src: str,
        odom_msg: Optional[Odometry] = None,
    ) -> Optional[np.ndarray]:
        if odom_msg is None:
            odom_msg = self._lidar_odom_nearest(stamp)
        if odom_msg is None:
            if not self._warned_lidar_odom_miss:
                self.get_logger().warn(
                    f'keyframe: no LiDAR odom near cloud stamp '
                    f'(topic={self._lidar_odom_topic!r}); check relay or lidar_odom_max_age_sec',
                    throttle_duration_sec=5.0,
                )
                self._warned_lidar_odom_miss = True
            return None
        odom_f = (odom_msg.header.frame_id or '').strip()
        base_f = (odom_msg.child_frame_id or '').strip() or self._robot_frame
        tf_mo = self._lookup_transform_stamped(self._map_frame, odom_f, stamp)
        if tf_mo is None:
            return None
        T_mo = transform_matrix_from_stamped(tf_mo)
        T_ob = pose_to_matrix4_from_odometry(odom_msg)
        tf_bs = self._lookup_transform_stamped(base_f, src, stamp)
        if tf_bs is None:
            return None
        T_bs = transform_matrix_from_stamped(tf_bs)
        return T_mo @ T_ob @ T_bs

    def _robot_xy_yaw_in_map_from_odom(
        self, stamp: Time, odom_msg: Odometry
    ) -> Optional[Tuple[float, float, float]]:
        odom_f = (odom_msg.header.frame_id or '').strip()
        tf_mo = self._lookup_transform_stamped(self._map_frame, odom_f, stamp)
        if tf_mo is None:
            return None
        T_mo = transform_matrix_from_stamped(tf_mo)
        T_ob = pose_to_matrix4_from_odometry(odom_msg)
        T_mb = T_mo @ T_ob
        yaw = math.atan2(float(T_mb[1, 0]), float(T_mb[0, 0]))
        return (float(T_mb[0, 3]), float(T_mb[1, 3]), float(yaw))

    def _interp_gyro_from_buffer(self, t_ns: np.ndarray) -> Optional[np.ndarray]:
        if not self._deskew_interp or len(self._imu_buf) < 2 or t_ns.size == 0:
            return None
        times = np.array([r[0] for r in self._imu_buf], dtype=np.int64)
        w = np.array([[r[1], r[2], r[3]] for r in self._imu_buf], dtype=np.float64)
        t_lo, t_hi = int(times[0]), int(times[-1])
        # Clamp query times to buffered span so fast spins still get interp (edges use endpoint ω).
        tq = np.clip(t_ns.astype(np.int64), t_lo, t_hi)
        return interp_gyro_batch(tq, times, w)

    def _gyro_mean_for_timespan(self, t_lo_ns: int, t_hi_ns: int) -> Optional[np.ndarray]:
        """Mean angular velocity over IMU samples in [t_lo, t_hi] (with margin) — stable under spin."""
        if len(self._imu_buf) < 1:
            return None
        lo = min(t_lo_ns, t_hi_ns) - int(80e6)
        hi = max(t_lo_ns, t_hi_ns) + int(80e6)
        acc = np.zeros(3, dtype=np.float64)
        n = 0
        for tns, wx, wy, wz in self._imu_buf:
            ti = int(tns)
            if lo <= ti <= hi:
                acc[0] += float(wx)
                acc[1] += float(wy)
                acc[2] += float(wz)
                n += 1
        if n < 1:
            return None
        return acc / float(n)

    def _angular_velocity_to_deskew_frame(
        self, msg: Imu, wx: float, wy: float, wz: float
    ) -> Tuple[float, float, float]:
        """Express angular velocity in ``deskew_imu_rotate_gyro_to_frame`` (LiDAR frame)."""
        tgt = self._deskew_rotate_gyro_to
        if not tgt:
            return wx, wy, wz
        imu_frame = (msg.header.frame_id or '').strip()
        if not imu_frame or imu_frame == tgt:
            return wx, wy, wz
        # Avoid using global TF frame ``sensor`` (Microstrain) to rotate Livox chip gyro.
        _imu_t = (self._deskew_imu_topic or '').strip()
        livox_imu_topic = _imu_t == '/livox/imu' or _imu_t.endswith('/livox/imu')
        if (
            self._deskew_livox_sensor_cloud_identity
            and livox_imu_topic
            and imu_frame == 'sensor'
            and tgt
        ):
            return wx, wy, wz
        vs = Vector3Stamped()
        vs.header = msg.header
        vs.vector.x = float(wx)
        vs.vector.y = float(wy)
        vs.vector.z = float(wz)
        tf_timeout = Duration(seconds=max(0.02, self._deskew_imu_rot_tf_timeout))
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                tgt, imu_frame, stamp, timeout=tf_timeout
            )
        except TransformException:
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    tgt, imu_frame, Time(), timeout=tf_timeout
                )
            except TransformException as exc:
                if not self._warned_imu_gyro_tf:
                    self.get_logger().warn(
                        f'deskew: cannot rotate ω from {imu_frame!r} to {tgt!r} ({exc}); '
                        f'using raw gyro (expect rotation shear). Check static TF chain.',
                    )
                    self._warned_imu_gyro_tf = True
                return wx, wy, wz
        out = do_transform_vector3(vs, tf_msg)
        return (float(out.vector.x), float(out.vector.y), float(out.vector.z))

    def _on_imu(self, msg: Imu) -> None:
        wx = float(msg.angular_velocity.x)
        wy = float(msg.angular_velocity.y)
        wz = float(msg.angular_velocity.z)
        wx, wy, wz = self._angular_velocity_to_deskew_frame(msg, wx, wy, wz)
        self._last_imu_w[0] = wx
        self._last_imu_w[1] = wy
        self._last_imu_w[2] = wz
        self._last_imu_recv = self.get_clock().now()
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            t = Time.from_msg(msg.header.stamp) + Duration(
                seconds=float(self._deskew_imu_stamp_off)
            )
            self._last_imu_msg_stamp = t
            self._imu_buf.append((int(t.nanoseconds), wx, wy, wz))

    def _deskew_imu_fresh_wall(self) -> bool:
        """Legacy gate: IMU arrived recently in wall time (cloud stamp missing or broken)."""
        if self._last_imu_recv is None:
            return False
        dt = (self.get_clock().now() - self._last_imu_recv).nanoseconds * 1e-9
        return dt <= self._deskew_max_imu_age

    def _deskew_imu_usable_for_cloud(self, cloud_time: Time) -> bool:
        """Gate deskew on IMU vs *cloud* time alignment, not processing latency.

        Using only wall-clock ``now()`` caused deskew to be skipped whenever the cloud
        callback ran slightly late → motion-smearing persisted while rotating even though
        gyro was valid at capture time.
        """
        if self._last_imu_msg_stamp is None:
            return self._deskew_imu_fresh_wall()
        if cloud_time.nanoseconds == 0:
            return self._deskew_imu_fresh_wall()
        dt = abs((cloud_time.nanoseconds - self._last_imu_msg_stamp.nanoseconds) * 1e-9)
        # One Livox frame ~100 ms; allow IMU stamp slightly before/after cloud stamp.
        slack = max(float(self._deskew_max_imu_age), 0.12) + 0.1
        return dt <= slack

    def _prefilter_points(
        self,
        xyz: np.ndarray,
        intensity: Optional[np.ndarray],
        ts: Optional[np.ndarray],
    ) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
        if xyz is None or xyz.size == 0:
            return xyz, intensity, ts
        x = xyz[:, 0]
        y = xyz[:, 1]
        z = xyz[:, 2]
        r = np.sqrt(x * x + y * y + z * z)
        mask = (r >= self._pf_min_r) & (r <= self._pf_max_r)
        if self._pf_self_r > 0.0:
            mask = mask & (r >= self._pf_self_r)
        if self._pf_bbox_en:
            xmin, xmax, ymin, ymax, zmin, zmax = self._pf_bbox
            in_box = (
                (x >= xmin) & (x <= xmax) &
                (y >= ymin) & (y <= ymax) &
                (z >= zmin) & (z <= zmax)
            )
            mask = mask & (~in_box)
        if intensity is not None and self._pf_i_en:
            mask = mask & (intensity >= self._pf_i_min)
        xyz = xyz[mask]
        if xyz.shape[0] == 0:
            return xyz, None, None
        intensity = intensity[mask] if intensity is not None else None
        ts = ts[mask] if ts is not None else None
        if self._pf_sor_en and xyz.shape[0] > max(8, self._pf_sor_k + 1):
            # Avoid O(N^2) distance matrix explosions on dense scans (process subset only).
            if self._pf_sor_max_pts > 0 and xyz.shape[0] > self._pf_sor_max_pts:
                pick = self._rng.choice(xyz.shape[0], size=self._pf_sor_max_pts, replace=False)
                x_sub = xyz[pick]
                keep_sub = statistical_outlier_mask(x_sub, self._pf_sor_k, self._pf_sor_std)
                keep = np.ones((xyz.shape[0],), dtype=bool)
                keep[pick] = keep_sub
            else:
                keep = statistical_outlier_mask(xyz, self._pf_sor_k, self._pf_sor_std)
            xyz = xyz[keep]
            intensity = intensity[keep] if intensity is not None else None
            ts = ts[keep] if ts is not None else None
        return xyz, intensity, ts

    def _deskew_gyro_ok(self) -> bool:
        if self._deskew_max_gyro_norm <= 0.0:
            return True
        g = float(np.linalg.norm(self._last_imu_w))
        ok = g <= self._deskew_max_gyro_norm
        if (not ok) and (not self._warned_deskew_gyro_spike):
            self.get_logger().warn(
                f'deskew: rejecting gyro spike ||w||={g:.2f} rad/s > {self._deskew_max_gyro_norm:.2f}; '
                'skip deskew for this scan',
                throttle_duration_sec=2.0,
            )
            self._warned_deskew_gyro_spike = True
        return ok

    def _keyframe_thresholds(self, cloud_time: Time) -> Tuple[float, float, float]:
        dist, yaw, t = self._kf_dist, self._kf_yaw, self._kf_time
        wnorm = float(np.linalg.norm(self._last_imu_w))
        if (
            self._rot_adapt
            and self._deskew_imu_usable_for_cloud(cloud_time)
            and wnorm > self._rot_gyro_thresh
        ):
            s = max(0.15, min(1.0, self._rot_scale))
            return dist * s, yaw * s, t * s
        return dist, yaw, t

    def _estimate_horizontal_plane_normal(self, pts: np.ndarray) -> Optional[np.ndarray]:
        if pts is None or pts.shape[0] < self._auto_level_min_pts:
            return None
        p = pts.astype(np.float64)
        if p.shape[0] > self._auto_level_max_pts:
            pick = self._rng.choice(p.shape[0], size=self._auto_level_max_pts, replace=False)
            p = p[pick]
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        cos_tilt = math.cos(math.radians(self._auto_level_max_tilt_deg))
        best_inliers = None
        best_n = None
        npts = p.shape[0]
        if npts < 3:
            return None
        for _ in range(self._auto_level_ransac_iters):
            i, j, k = self._rng.choice(npts, size=3, replace=False)
            a = p[j] - p[i]
            b = p[k] - p[i]
            n = np.cross(a, b)
            nn = float(np.linalg.norm(n))
            if nn < 1e-8:
                continue
            n = n / nn
            if abs(float(np.dot(n, up))) < cos_tilt:
                continue
            d = np.abs((p - p[i]) @ n)
            inliers = d <= self._auto_level_dist
            if best_inliers is None or int(np.count_nonzero(inliers)) > int(np.count_nonzero(best_inliers)):
                best_inliers = inliers
                best_n = n
        if best_inliers is None or best_n is None:
            return None
        pin = p[best_inliers]
        if pin.shape[0] < 20:
            return None
        c = np.mean(pin, axis=0)
        _, _, vh = np.linalg.svd(pin - c, full_matrices=False)
        n = vh[-1, :]
        if float(np.linalg.norm(n)) < 1e-8:
            return None
        n = n / float(np.linalg.norm(n))
        if n[2] < 0.0:
            n = -n
        if abs(float(np.dot(n, up))) < cos_tilt:
            return None
        return n.astype(np.float64)

    def _apply_auto_level_if_ready(self) -> None:
        if (not self._auto_level_enable) or self._auto_level_done:
            return
        if self._map_pts is None or self._map_pts.shape[0] < self._auto_level_min_pts:
            return
        if self._kf_count < self._auto_level_min_kf:
            return
        n = self._estimate_horizontal_plane_normal(self._map_pts)
        if n is None:
            return
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        dotv = float(np.clip(np.dot(n, up), -1.0, 1.0))
        ang = float(math.acos(dotv))
        if ang < math.radians(0.2):
            self._auto_level_done = True
            return
        axis = np.cross(n, up)
        R = _rotation_matrix_from_axis_angle(axis, ang)
        self._level_R = R @ self._level_R
        self._map_pts = (self._map_pts.astype(np.float64) @ R.T).astype(np.float32)
        if self._kf_scan_store:
            self._kf_scan_store = [
                (s.astype(np.float64) @ R.T).astype(np.float32) for s in self._kf_scan_store
            ]
        if self._kf_map_batches:
            self._kf_map_batches = [
                (s.astype(np.float64) @ R.T).astype(np.float32) for s in self._kf_map_batches
            ]
        if self._kf_poses:
            new_poses: List[Tuple[float, float, float]] = []
            for x, y, yaw in self._kf_poses:
                p = np.array([x, y, 0.0], dtype=np.float64)
                pr = R @ p
                h = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
                hr = R @ h
                yaw2 = math.atan2(float(hr[1]), float(hr[0]))
                new_poses.append((float(pr[0]), float(pr[1]), float(yaw2)))
            self._kf_poses = new_poses
            if self._last_kf is not None:
                lx, ly, lyaw, lstamp = self._last_kf
                p = np.array([lx, ly, 0.0], dtype=np.float64)
                pr = R @ p
                h = np.array([math.cos(lyaw), math.sin(lyaw), 0.0], dtype=np.float64)
                hr = R @ h
                self._last_kf = (
                    float(pr[0]),
                    float(pr[1]),
                    float(math.atan2(float(hr[1]), float(hr[0]))),
                    lstamp,
                )
        self._auto_level_done = True
        self.get_logger().info(
            f'auto_level: applied roll/pitch correction {math.degrees(ang):.2f}deg '
            f'from dominant horizontal plane normal=({n[0]:.3f},{n[1]:.3f},{n[2]:.3f})'
        )

    def _lookup_transform_stamped(
        self, target_frame: str, source_frame: str, stamp: Time
    ) -> Optional[TransformStamped]:
        """Return transform target_frame <- source_frame at ``stamp``, with optional fallbacks."""
        tf_timeout = Duration(seconds=max(0.05, self._tf_timeout))
        la = f'{source_frame} -> {target_frame}'
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=tf_timeout
            )
        except ExtrapolationException as e:
            em = str(e).lower()
            use_latest = self._tf_allow_latest_fallback or (
                self._tf_future_extrap_latest and 'future' in em
            )
            if use_latest:
                try:
                    return self._tf_buffer.lookup_transform(
                        target_frame, source_frame, Time(), timeout=tf_timeout
                    )
                except TransformException as e2:
                    self.get_logger().warn(
                        f'No TF {la} (after stamp/latest retry): {e2}',
                        throttle_duration_sec=5.0,
                    )
                    return None
            self.get_logger().warn(
                f'No TF {la} at cloud time: {e}',
                throttle_duration_sec=5.0,
            )
            return None
        except TransformException as e:
            if self._tf_allow_latest_fallback:
                try:
                    return self._tf_buffer.lookup_transform(
                        target_frame, source_frame, Time(), timeout=tf_timeout
                    )
                except TransformException as e2:
                    self.get_logger().warn(
                        f'No TF {la}: {e2}',
                        throttle_duration_sec=5.0,
                    )
                    return None
            self.get_logger().warn(
                f'No TF {la} at cloud time: {e}',
                throttle_duration_sec=5.0,
            )
            return None

    def _lookup_cloud_to_map(
        self,
        msg: PointCloud2,
        lidar_odom: Optional[Odometry] = None,
    ) -> Optional[np.ndarray]:
        src = (msg.header.frame_id or '').strip()
        t = self._cloud_stamp_adjusted(msg)
        if self._use_lidar_odom_pose:
            m = self._compose_map_T_sensor(t, src, lidar_odom)
            if m is None:
                return None
        else:
            tf = self._lookup_transform_stamped(self._map_frame, src, t)
            if tf is None:
                return None
            m = transform_matrix_from_stamped(tf)
        raw, intensity, ts = points_xyz_intensity_timestamp(msg)
        if raw is None:
            self.get_logger().warn(
                'Could not extract x,y,z from point cloud (check fields / NaNs)',
                throttle_duration_sec=10.0,
            )
            return None
        raw, intensity, ts = self._prefilter_points(raw, intensity, ts)
        if raw is None or raw.shape[0] == 0:
            return None
        if self._deskew_enable and ts is not None and self._deskew_gyro_ok():
            deskewed = False
            t_abs = livox_point_times_abs_ns(
                msg.header.stamp,
                ts,
                cloud_stamp_offset_sec=float(self._deskew_cloud_stamp_off),
            )
            mx = float(np.max(ts))
            scale = 1e-9 if mx > 1e4 else 1.0
            dt_sec = (mx - ts) * scale
            if t_abs is not None and self._deskew_interp and len(self._imu_buf) >= 2:
                w_arr = self._interp_gyro_from_buffer(t_abs)
                if w_arr is not None:
                    raw = deskew_points_to_scan_end_varying(
                        raw,
                        dt_sec,
                        w_arr,
                        sign=self._deskew_sign,
                        model=self._deskew_model,
                    )
                    deskewed = True
            if (
                not deskewed
                and self._deskew_mean_fallback
                and t_abs is not None
                and len(self._imu_buf) >= 2
            ):
                w_mean = self._gyro_mean_for_timespan(
                    int(np.min(t_abs)), int(np.max(t_abs))
                )
                if w_mean is not None:
                    raw = deskew_points_to_scan_end(
                        raw,
                        ts,
                        float(w_mean[0]),
                        float(w_mean[1]),
                        float(w_mean[2]),
                        sign=self._deskew_sign,
                        model=self._deskew_model,
                    )
                    deskewed = True
            if not deskewed and self._deskew_imu_usable_for_cloud(t):
                raw = deskew_points_to_scan_end(
                    raw,
                    ts,
                    float(self._last_imu_w[0]),
                    float(self._last_imu_w[1]),
                    float(self._last_imu_w[2]),
                    sign=self._deskew_sign,
                    model=self._deskew_model,
                )
        if raw.shape[0] > self._max_scan:
            idx = np.random.choice(raw.shape[0], self._max_scan, replace=False)
            raw = raw[idx]
            if intensity is not None:
                intensity = intensity[idx]
            if ts is not None:
                ts = ts[idx]
        pts_map = transform_points_xyz(raw, m)
        if self._auto_level_done:
            pts_map = (pts_map.astype(np.float64) @ self._level_R.T).astype(np.float32)
        return pts_map

    def _lookup_robot_pose_map(
        self,
        msg: PointCloud2,
        lidar_odom: Optional[Odometry] = None,
    ) -> Optional[Tuple[float, float, float]]:
        t = self._cloud_stamp_adjusted(msg)
        if self._use_lidar_odom_pose:
            odom_msg = lidar_odom or self._lidar_odom_nearest(t)
            if odom_msg is None:
                return None
            pose = self._robot_xy_yaw_in_map_from_odom(t, odom_msg)
            if pose is None:
                return None
            if self._auto_level_done:
                p = np.array([pose[0], pose[1], 0.0], dtype=np.float64)
                pr = self._level_R @ p
                h = np.array([math.cos(pose[2]), math.sin(pose[2]), 0.0], dtype=np.float64)
                hr = self._level_R @ h
                return (float(pr[0]), float(pr[1]), float(math.atan2(float(hr[1]), float(hr[0]))))
            return pose
        tf = self._lookup_transform_stamped(self._map_frame, self._robot_frame, t)
        if tf is None:
            return None
        tr = tf.transform.translation
        q = tf.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        if self._auto_level_done:
            p = np.array([float(tr.x), float(tr.y), 0.0], dtype=np.float64)
            pr = self._level_R @ p
            h = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
            hr = self._level_R @ h
            return (float(pr[0]), float(pr[1]), float(math.atan2(float(hr[1]), float(hr[0]))))
        return (float(tr.x), float(tr.y), float(yaw))

    def _is_keyframe(
        self, pose: Tuple[float, float, float], stamp_ns: int, cloud_time: Time
    ) -> bool:
        if self._last_kf is None:
            return True
        lx, ly, lyaw, lstamp_ns = self._last_kf
        x, y, yaw = pose
        kfd, kfy, kft = self._keyframe_thresholds(cloud_time)
        dist = math.hypot(x - lx, y - ly)
        dyaw = abs(wrap_angle(yaw - lyaw))
        dt = (stamp_ns - lstamp_ns) * 1e-9
        return dist >= kfd or dyaw >= kfy or dt >= kft

    def _publish_path(self) -> None:
        if not self._pub_path or not self._kf_poses:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._map_frame
        for x, y, yaw in self._kf_poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = sy
            ps.pose.orientation.w = cy
            path.poses.append(ps)
        self._pub_path.publish(path)

    def _store_keyframe_scan(self, pts_map: np.ndarray) -> None:
        leaf = max(self._loop_store_leaf, 1e-6)
        stored = voxel_downsample(pts_map, leaf)
        if stored.shape[0] > self._loop_max_store:
            pick = self._rng.choice(
                stored.shape[0], size=self._loop_max_store, replace=False
            )
            stored = stored[pick]
        self._kf_scan_store.append(stored.astype(np.float32))

    def _try_loop_closure(self, new_idx: int, current_pts_map: np.ndarray) -> None:
        if not self._loop_enable or self._pub_loop_idx is None:
            return
        if new_idx < self._loop_min_gap:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_loop_pub_ns < self._loop_cooldown_ns:
            return

        px, py, pyaw = self._kf_poses[new_idx]
        best_i = -1
        best_score = 0.0

        # Past keyframe index i with (new_idx - i) >= loop_min_index_gap (not recent chain).
        for i in range(0, new_idx - self._loop_min_gap + 1):
            if i >= len(self._kf_scan_store):
                break
            ox, oy, oyaw = self._kf_poses[i]
            if math.hypot(px - ox, py - oy) > self._loop_xy:
                continue
            if abs(wrap_angle(pyaw - oyaw)) > self._loop_yaw:
                continue
            old_pts = self._kf_scan_store[i]
            if old_pts.shape[0] < 20:
                continue
            score = overlap_ratio(
                current_pts_map,
                old_pts,
                self._loop_samples,
                self._loop_match_m,
                self._rng,
            )
            if score > best_score:
                best_score = score
                best_i = i

        if best_i < 0 or best_score < self._loop_ov_thresh:
            return

        self._last_loop_pub_ns = now_ns
        self._pub_loop_idx.publish(UInt32(data=best_i))
        self._pub_loop_score.publish(Float32(data=float(best_score)))

        ax, ay, ayaw = self._kf_poses[best_i]
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, ayaw)
        anchor = PoseStamped()
        anchor.header.stamp = self.get_clock().now().to_msg()
        anchor.header.frame_id = self._map_frame
        anchor.pose.position.x = ax
        anchor.pose.position.y = ay
        anchor.pose.position.z = 0.0
        anchor.pose.orientation.x = qx
        anchor.pose.orientation.y = qy
        anchor.pose.orientation.z = qz
        anchor.pose.orientation.w = qw
        self._pub_loop_pose.publish(anchor)

        if self._pub_loop_pair is not None:
            pair = Int32MultiArray()
            pair.data = [int(best_i), int(new_idx)]
            self._pub_loop_pair.publish(pair)

        self.get_logger().info(
            f'Loop closure: new_kf={new_idx} matches past_kf={best_i} overlap={best_score:.3f}'
        )

    def _poses_from_path_msg(self, path: Path) -> np.ndarray:
        rows: List[List[float]] = []
        for ps in path.poses:
            q = ps.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            rows.append(
                [float(ps.pose.position.x), float(ps.pose.position.y), float(yaw)]
            )
        return np.array(rows, dtype=np.float64)

    def _on_pose_graph_path(self, msg: Path) -> None:
        if not self._apply_pg:
            return
        if len(msg.poses) < 2:
            return
        n = len(self._kf_poses)
        if (
            n < 2
            or len(msg.poses) != n
            or len(self._kf_map_batches) != n
        ):
            self.get_logger().warn(
                f'pose_graph path len {len(msg.poses)} vs keyframes {n} '
                f'batches {len(self._kf_map_batches)} — skip map rebuild',
                throttle_duration_sec=5.0,
            )
            return
        P_old = np.array(
            [[float(x), float(y), float(yw)] for x, y, yw in self._kf_poses],
            dtype=np.float64,
        )
        P_new = self._poses_from_path_msg(msg)
        if (
            self._last_pg_applied is not None
            and self._last_pg_applied.shape == P_new.shape
            and np.allclose(P_new, self._last_pg_applied, atol=1e-4, rtol=0.0)
        ):
            return
        rebuilt: List[np.ndarray] = []
        rebuilt_i: List[np.ndarray] = []
        for i in range(n):
            T_old = T_from_xyw(P_old[i])
            T_new = T_from_xyw(P_new[i])
            T_rel = T_new @ la.inv(T_old)
            b = transform_points_se2(T_rel, self._kf_map_batches[i])
            rebuilt.append(b)
            rebuilt_i.append(np.full((b.shape[0],), float(i + 1), dtype=np.float32))
        merged = np.vstack(rebuilt)
        merged_i = np.concatenate(rebuilt_i) if rebuilt_i else np.zeros((0,), dtype=np.float32)
        self._map_pts, self._map_intensity = voxel_downsample_xyz_i(merged, merged_i, self._voxel)
        if self._map_pts.shape[0] > self._max_map:
            self._map_pts, self._map_intensity = voxel_downsample_xyz_i(
                self._map_pts, self._map_intensity, self._voxel * 1.5
            )
        self._kf_poses = [
            (float(P_new[i, 0]), float(P_new[i, 1]), float(P_new[i, 2]))
            for i in range(n)
        ]
        if self._loop_enable and len(self._kf_scan_store) == n:
            for i in range(n):
                T_old = T_from_xyw(P_old[i])
                T_new = T_from_xyw(P_new[i])
                T_rel = T_new @ la.inv(T_old)
                self._kf_scan_store[i] = transform_points_se2(
                    T_rel, self._kf_scan_store[i].astype(np.float32)
                )
        stamp = self.get_clock().now().to_msg()
        out = numpy_xyz_to_pointcloud2(
            self._map_pts,
            self._map_frame,
            stamp,
            self,
            intensity=self._map_intensity if self._map_intensity is not None else self._map_pts[:, 2].astype(np.float32),
        )
        self._pub_map.publish(out)
        self._publish_path()
        self._last_pg_applied = P_new.copy()
        self.get_logger().info(
            f'Pose graph applied to map: {n} keyframes, {self._map_pts.shape[0]} map pts'
        )

    def _on_cloud_odom_synced(self, cloud_msg: PointCloud2, odom_msg: Odometry) -> None:
        self._process_cloud(cloud_msg, odom_msg)

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._process_cloud(msg, None)

    def _process_cloud(
        self, msg: PointCloud2, lidar_odom: Optional[Odometry]
    ) -> None:
        self._cloud_rx_count += 1
        if self._cloud_rx_count == 1:
            self.get_logger().info(
                f'Receiving point clouds (frame_id={msg.header.frame_id!r}); '
                'building keyframe map when TF allows'
            )
        cf = (msg.header.frame_id or '').strip()
        if (
            self._deskew_rotate_gyro_to
            and cf
            and cf != self._deskew_rotate_gyro_to
            and not self._warned_cloud_frame_mismatch
        ):
            self.get_logger().warn(
                f'Point cloud frame_id {cf!r} differs from deskew_imu_rotate_gyro_to_frame '
                f'{self._deskew_rotate_gyro_to!r}. Set Livox ``frame_id`` / this param so they match.',
            )
            self._warned_cloud_frame_mismatch = True
        pose = self._lookup_robot_pose_map(msg, lidar_odom)
        if pose is None:
            return
        if self._reject_unstable_frame_enable and self._last_kf is not None:
            lx, ly, lyaw, _lstamp = self._last_kf
            dx = pose[0] - lx
            dy = pose[1] - ly
            dyaw = abs(wrap_angle(pose[2] - lyaw))
            if (
                math.hypot(dx, dy) > self._reject_unstable_frame_max_translation_m
                or dyaw > self._reject_unstable_frame_max_yaw_rad
            ):
                self.get_logger().warn(
                    'Rejecting unstable frame: '
                    f'dxy={math.hypot(dx, dy):.3f} m '
                    f'dyaw={math.degrees(dyaw):.1f} deg '
                    f'(limits {self._reject_unstable_frame_max_translation_m:.3f} m, '
                    f'{math.degrees(self._reject_unstable_frame_max_yaw_rad):.1f} deg)',
                    throttle_duration_sec=0.5,
                )
                return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            stamp_ns = self.get_clock().now().nanoseconds
            cloud_time = Time()
        else:
            cloud_time = self._cloud_stamp_adjusted(msg)
            stamp_ns = cloud_time.nanoseconds
        if not self._is_keyframe(pose, stamp_ns, cloud_time):
            return

        pts_map = self._lookup_cloud_to_map(msg, lidar_odom)
        if pts_map is None or pts_map.shape[0] == 0:
            return

        self._last_kf = (pose[0], pose[1], pose[2], stamp_ns)
        self._kf_poses.append((pose[0], pose[1], pose[2]))
        new_idx = len(self._kf_poses) - 1

        if self._apply_pg:
            leaf = max(self._map_batch_leaf, 1e-6)
            self._kf_map_batches.append(
                voxel_downsample(pts_map, leaf).astype(np.float32)
            )

        if self._loop_enable:
            self._store_keyframe_scan(pts_map)
            self._try_loop_closure(new_idx, pts_map)

        self._kf_count += 1

        kf_i = float(self._kf_count)
        new_int = np.full((pts_map.shape[0],), kf_i, dtype=np.float32)
        if self._map_pts is None:
            self._map_pts = pts_map
            self._map_intensity = new_int
        else:
            n_old = self._map_pts.shape[0]
            if self._map_intensity is None or int(self._map_intensity.shape[0]) != n_old:
                self._map_intensity = self._map_pts[:, 2].astype(np.float32)
            self._map_pts = np.vstack([self._map_pts, pts_map])
            self._map_intensity = np.concatenate([self._map_intensity, new_int])

        if self._map_pts.shape[0] > self._max_map:
            self.get_logger().warn(
                f'Map points {self._map_pts.shape[0]} > max {self._max_map}; '
                'voxel-downsampling harder',
                throttle_duration_sec=10.0,
            )
            self._map_pts, self._map_intensity = voxel_downsample_xyz_i(
                self._map_pts, self._map_intensity, self._voxel * 1.5
            )

        self._map_pts, self._map_intensity = voxel_downsample_xyz_i(
            self._map_pts, self._map_intensity, self._voxel
        )
        self._apply_auto_level_if_ready()

        now_mono = self.get_clock().now().nanoseconds
        do_pub = self._map_pub_min_ns <= 0 or (
            self._last_map_pub_mono_ns is None
            or (now_mono - self._last_map_pub_mono_ns) >= self._map_pub_min_ns
        )
        if do_pub:
            self._last_map_pub_mono_ns = now_mono
            stamp = self.get_clock().now().to_msg()
            inte = (
                self._map_intensity
                if self._map_intensity is not None
                and int(self._map_intensity.shape[0]) == int(self._map_pts.shape[0])
                else self._map_pts[:, 2].astype(np.float32)
            )
            out = numpy_xyz_to_pointcloud2(
                self._map_pts, self._map_frame, stamp, self, intensity=inte
            )
            self._pub_map.publish(out)
            self._publish_path()

        if self._kf_count % 20 == 0:
            self.get_logger().info(
                f'Keyframes: {self._kf_count}, map points: {self._map_pts.shape[0]}'
            )


def main() -> None:
    rclpy.init()
    node = KeyframeMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
