#!/usr/bin/env python3
"""
Trace planar translation: LiDAR odom (default /lidar/odom = LIO relay or smoothed NDT) vs EKF vs TF.

Run beside launch + bag (same ROS_DOMAIN_ID, ROS_USE_SIM_TIME=true).

How to read
-----------
- **step_lidar**: |Δxy| between consecutive **NDT** odometry messages (~10 Hz).
- **step_ekf**: |Δxy| of **EKF** state sampled at those same LiDAR times (snapshots).
- **ratio_window** (rolling): sum(step_ekf) / sum(step_lidar). Near **1** ⇒ EKF tracks NDT translation.
  Near **0** with nonzero step_lidar ⇒ translation dies **between NDT output and EKF/TF** (fusion / prediction).
- **step_tf**: |Δxy| from TF **odom→base_link** at each LiDAR message stamp (falls back to latest if lookup fails).

If **step_lidar ≈ 0** always ⇒ problem is **upstream of EKF** (NDT / topics / replay).

If **step_lidar >> 0** but **step_ekf ≈ 0** ⇒ **EKF** is not integrating LiDAR translation (or IMU-only prediction dominates).

If **step_ekf >> 0** but **step_tf ≈ 0** ⇒ **TF publish** / stamp / RViz frame issue.
"""
from __future__ import annotations

import collections
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def _xy(msg: Odometry) -> tuple[float, float]:
    p = msg.pose.pose.position
    return float(p.x), float(p.y)


def _hypot(dx: float, dy: float) -> float:
    return float(math.hypot(dx, dy))


class PipelineTranslationDebug(Node):
    def __init__(self) -> None:
        super().__init__('pipeline_translation_debug')

        self.declare_parameter('lidar_odom_topic', '/lidar/odom')
        self.declare_parameter('ekf_odom_topic', '/ekf/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('summary_period_sec', 2.0)
        self.declare_parameter('window_max_steps', 80)
        self.declare_parameter('print_every_lidar', False)
        self.declare_parameter('tf_timeout_sec', 0.25)

        lt = str(self.get_parameter('lidar_odom_topic').value).strip()
        et = str(self.get_parameter('ekf_odom_topic').value).strip()
        self._odom_f = str(self.get_parameter('odom_frame').value).strip()
        self._base_f = str(self.get_parameter('base_frame').value).strip()
        self._sum_period = max(0.5, float(self.get_parameter('summary_period_sec').value))
        self._win_max = max(10, int(self.get_parameter('window_max_steps').value))
        self._print_each = bool(self.get_parameter('print_every_lidar').value)
        self._tf_to = float(self.get_parameter('tf_timeout_sec').value)

        self._tfbuf = Buffer(cache_time=Duration(seconds=120.0))
        self._tflis = TransformListener(self._tfbuf, self, spin_thread=True)

        self._latest_ekf = None  # Odometry
        self._prev_lidar_xy: tuple[float, float] | None = None
        self._prev_ekf_snap: tuple[float, float] | None = None
        self._prev_tf_xy: tuple[float, float] | None = None

        self._steps_lidar: collections.deque[float] = collections.deque(maxlen=self._win_max)
        self._steps_ekf: collections.deque[float] = collections.deque(maxlen=self._win_max)
        self._steps_tf: collections.deque[float] = collections.deque(maxlen=self._win_max)

        self.create_subscription(Odometry, lt, self._on_lidar, 50)
        self.create_subscription(Odometry, et, self._on_ekf, 100)

        self._timer = self.create_timer(self._sum_period, self._on_summary)

        self.get_logger().info(
            f'pipeline_translation_debug: lidar={lt!r} ekf={et!r} '
            f'TF {self._odom_f!r}->{self._base_f!r} summary every {self._sum_period:g}s'
        )

    def _on_ekf(self, msg: Odometry) -> None:
        self._latest_ekf = msg

    def _tf_xy_for_stamp(self, stamp) -> tuple[float, float, str]:
        """TF odom→base at cloud/odom stamp; fallback to latest if buffer cannot extrapolate."""
        t = Time.from_msg(stamp)
        try:
            tr = self._tfbuf.lookup_transform(
                self._odom_f,
                self._base_f,
                t,
                timeout=Duration(seconds=self._tf_to),
            )
            trl = tr.transform.translation
            return float(trl.x), float(trl.y), ''
        except TransformException:
            try:
                tr = self._tfbuf.lookup_transform(
                    self._odom_f,
                    self._base_f,
                    Time(),
                    timeout=Duration(seconds=self._tf_to),
                )
                trl = tr.transform.translation
                return float(trl.x), float(trl.y), 'fallback_latest'
            except TransformException as e:
                return float('nan'), float('nan'), str(e).replace('\n', ' ')[:80]

    def _on_lidar(self, msg: Odometry) -> None:
        lx, ly = _xy(msg)
        stamp = msg.header.stamp

        ekf_xy = _xy(self._latest_ekf) if self._latest_ekf is not None else (float('nan'), float('nan'))

        tf_x, tf_y, tf_err = self._tf_xy_for_stamp(stamp)

        step_l = 0.0
        step_e = 0.0
        step_t = 0.0
        if self._prev_lidar_xy is not None:
            plx, ply = self._prev_lidar_xy
            step_l = _hypot(lx - plx, ly - ply)
            self._steps_lidar.append(step_l)

        if self._prev_ekf_snap is not None and self._latest_ekf is not None:
            pex, pey = self._prev_ekf_snap
            ex, ey = ekf_xy
            step_e = _hypot(ex - pex, ey - pey)
            self._steps_ekf.append(step_e)

        if self._prev_tf_xy is not None and not math.isnan(tf_x):
            ptx, pty = self._prev_tf_xy
            step_t = _hypot(tf_x - ptx, tf_y - pty)
            self._steps_tf.append(step_t)

        self._prev_lidar_xy = (lx, ly)
        self._prev_ekf_snap = ekf_xy
        if not math.isnan(tf_x):
            self._prev_tf_xy = (tf_x, tf_y)

        if self._print_each:
            self.get_logger().info(
                f'cloud@{stamp.sec}.{stamp.nanosec:09d} '
                f'lidar_xy=({lx:.4f},{ly:.4f}) step_l={step_l:.4f}m '
                f'ekf_xy=({ekf_xy[0]:.4f},{ekf_xy[1]:.4f}) step_e={step_e:.4f}m '
                f'tf_xy=({tf_x:.4f},{tf_y:.4f}) step_tf={step_t:.4f}m tf_err={tf_err!r}'
            )

    def _on_summary(self) -> None:
        sl = sum(self._steps_lidar)
        se = sum(self._steps_ekf)
        st = sum(self._steps_tf)
        n_l = len(self._steps_lidar)
        ratio = (se / sl) if sl > 1e-6 else float('nan')

        # Interpretation hint
        hint = ''
        if n_l == 0:
            hint = 'no lidar steps yet'
        elif sl < 1e-4:
            hint = 'NDT path ~no translation (check NDT / scan_to_map / bag)'
        elif ratio == ratio and ratio < 0.25:
            hint = 'EKF not tracking LiDAR xy (fusion / predict_use_linear_accel / rejects)'
        elif ratio == ratio and ratio > 0.6:
            hint = 'EKF tracks LiDAR ok — if RViz bad, check fixed frame / displays / keyframe map'
        else:
            hint = 'partial tracking — check soft fuse / variances'

        st_sl = (st / sl) if sl > 1e-6 else float('nan')

        self.get_logger().info(
            f'SUMMARY window(n≈{n_l}) sum|Δ|_lidar={sl:.3f}m sum|Δ|_ekf={se:.3f}m '
            f'ratio_ekf/lidar={ratio:.3f} sum|Δ|_tf={st:.3f}m ratio_tf/lidar={st_sl:.3f} '
            f'=> {hint}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PipelineTranslationDebug()
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
