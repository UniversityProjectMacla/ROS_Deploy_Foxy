#!/usr/bin/env python3
"""Exponential moving average filter for nav_msgs/Odometry (NDT -> EKF bridge).

NDT publishes to ``in_topic`` (typ. /lidar/odom_raw); this node publishes ``out_topic``
(typ. /lidar/odom) so EKF and tooling see a smoother pose.

Modes:
  xy   — smooth position.x,y only; z,q,twist from measurement
  xyz  — smooth x,y,z; orientation + twist from measurement
  full — smooth xyz + euler RPY + twist.xyz (helps reduce yaw jitter in EKF)
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

try:
    import tf_transformations as tft
except ImportError as e:
    raise SystemExit(
        'lidar_odom_ema_smooth needs tf_transformations (ros-foxy-tf-transformations).'
    ) from e


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class LidarOdomEmaSmooth(Node):
    def __init__(self) -> None:
        super().__init__('lidar_odom_ema_smooth')
        # use_sim_time is set by launch (sim_time_param); do not re-declare → ParameterAlreadyDeclaredException
        try:
            ust = bool(self.get_parameter('use_sim_time').get_parameter_value().bool_value)
        except ParameterNotDeclaredException:
            ust = False
        self.declare_parameter('in_topic', '/lidar/odom_raw')
        self.declare_parameter('out_topic', '/lidar/odom')
        self.declare_parameter('smooth_mode', 'full')  # xy | xyz | full
        self.declare_parameter('alpha_pose', 0.18)
        self.declare_parameter('alpha_twist_linear', 0.22)

        in_t = self.get_parameter('in_topic').get_parameter_value().string_value.strip()
        out_t = self.get_parameter('out_topic').get_parameter_value().string_value.strip()
        mode = self.get_parameter('smooth_mode').get_parameter_value().string_value.strip().lower()
        self._alpha_pose = float(self.get_parameter('alpha_pose').value)
        self._alpha_tw = float(self.get_parameter('alpha_twist_linear').value)

        self._alpha_pose = min(max(self._alpha_pose, 1e-6), 1.0)
        self._alpha_tw = min(max(self._alpha_tw, 1e-6), 1.0)
        self._mode = mode if mode in ('xy', 'xyz', 'full') else 'full'

        qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Odometry, out_t, qos)
        self.create_subscription(Odometry, in_t, self._cb, qos)

        self._prev_out: Odometry | None = None
        self.get_logger().info(
            f'EMA smooth {in_t!r} -> {out_t!r} mode={self._mode!r} '
            f'alpha_pose={self._alpha_pose:g} alpha_twist_linear={self._alpha_tw:g} '
            f'use_sim_time={ust}'
        )

    def _cb(self, msg: Odometry) -> None:
        if self._prev_out is None:
            out = Odometry()
            out.header = msg.header
            out.child_frame_id = msg.child_frame_id
            out.pose = msg.pose
            out.twist = msg.twist
            self._prev_out = out
            self._pub.publish(out)
            return

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        ap = self._alpha_pose
        at = self._alpha_tw

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        prev_p = self._prev_out.pose.pose.position
        prev_o = self._prev_out.pose.pose.orientation

        # Position EMA
        out.pose.pose.position.x = float(
            ap * float(p.x) + (1.0 - ap) * float(prev_p.x)
        )
        out.pose.pose.position.y = float(
            ap * float(p.y) + (1.0 - ap) * float(prev_p.y)
        )
        if self._mode in ('xyz', 'full'):
            out.pose.pose.position.z = float(
                ap * float(p.z) + (1.0 - ap) * float(prev_p.z)
            )
        else:
            out.pose.pose.position.z = float(p.z)

        # Orientation
        if self._mode == 'full':
            r_meas, pit_meas, y_meas = tft.euler_from_quaternion(
                [float(o.x), float(o.y), float(o.z), float(o.w)]
            )
            r_prev, pit_prev, y_prev = tft.euler_from_quaternion(
                [
                    float(prev_o.x),
                    float(prev_o.y),
                    float(prev_o.z),
                    float(prev_o.w),
                ]
            )
            yaw_sm = float(
                _wrap_pi(
                    float(y_prev)
                    + ap * _wrap_pi(float(y_meas) - float(y_prev))
                )
            )
            roll_sm = ap * float(r_meas) + (1.0 - ap) * float(r_prev)
            pitch_sm = ap * float(pit_meas) + (1.0 - ap) * float(pit_prev)
            qx, qy, qz, qw = tft.quaternion_from_euler(roll_sm, pitch_sm, yaw_sm)
            out.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        else:
            out.pose.pose.orientation = o

        # Twist linear EMA when full twist smoothing helps shape
        if self._mode == 'full':
            vl = msg.twist.twist.linear
            pv = self._prev_out.twist.twist.linear
            out.twist.twist.linear = Vector3(
                x=float(at * float(vl.x) + (1.0 - at) * float(pv.x)),
                y=float(at * float(vl.y) + (1.0 - at) * float(pv.y)),
                z=float(at * float(vl.z) + (1.0 - at) * float(pv.z)),
            )
            out.twist.twist.angular = msg.twist.twist.angular
        elif self._mode == 'xyz':
            out.twist.twist = msg.twist.twist
        else:
            out.twist.twist = msg.twist.twist

        out.pose.covariance = list(msg.pose.covariance)
        out.twist.covariance = list(msg.twist.covariance)

        self._prev_out = out
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    try:
        node = LidarOdomEmaSmooth()
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
