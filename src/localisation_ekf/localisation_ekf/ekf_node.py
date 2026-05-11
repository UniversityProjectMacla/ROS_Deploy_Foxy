#!/usr/bin/env python3
import math

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time

from geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
    TransformStamped,
    TwistStamped,
    Vector3Stamped,
)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64, String
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from localisation_ekf.ekf_filter import EKFPlanarIMU, wrap_angle

# Microstrain (and many IMU drivers) publish sensor_msgs/Imu with best-effort QoS.
# Default rclpy subscription is reliable — no IMU callbacks → EKF looks LiDAR-only.
_IMU_SUB_QOS = QoSProfile(
    depth=200,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)


class EKFNode(Node):
    def __init__(self):
        super().__init__("ekf_node")

        # --------------------
        # Parameters
        # --------------------
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("publish_topic", "/ekf/odom")
        self.declare_parameter("pose_topic", "/ekf/pose")
        self.declare_parameter("path_topic", "/ekf/path")

        self.declare_parameter("nominal_dt", 0.01)
        self.declare_parameter("use_stamp_dt", True)
        # False: yaw from gyro only; do not integrate body (ax,ay) into velocity (see EKFPlanarIMU).
        self.declare_parameter("predict_use_linear_accel", True)

        # TF / odometry parent frame (ROS convention: odom -> base_link)
        self.declare_parameter("odom_frame", "odom")
        # Legacy: was used as odometry parent; ignored if set (use odom_frame + map->odom TF)
        self.declare_parameter("world_frame", "")

        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        # Optional: stamp /ekf/odom + TF with ROS time (usually leave false so TF matches
        # sensor-timed PointCloud2; pose_graph can stamp map→odom from /ekf/odom instead).
        self.declare_parameter("publish_use_ros_time_in_headers", False)
        # Rotate IMU linear_accel + angular_velocity into base_link when header.frame_id differs
        self.declare_parameter("transform_imu_to_base_link", True)
        self.declare_parameter("imu_tf_lookup_timeout_sec", 0.05)
        # Latched std_msgs/String: which IMU hardware the stack was configured for (livox | microstrain)
        self.declare_parameter("imu_source_topic", "/ekf/imu_source")
        self.declare_parameter("imu_source_id", "livox")

        # Planar EKF noise (state order: px,py,z,yaw,vx,vy,bax,bay,bgz)
        self.declare_parameter(
            "process_noise_diag",
            [
                1e-4,
                1e-4,
                1e-6,
                1e-6,
                5e-3,
                5e-3,
                1e-8,
                1e-8,
                1e-10,
            ],
        )
        self.declare_parameter("initial_cov_diag", [0.5] * 9)

        # Uncertainty on unobserved roll/pitch (rad^2) in published odometry
        self.declare_parameter("flat_orientation_variance", 1e-4)

        # Optional LiDAR fusion inputs
        self.declare_parameter("lidar_odom_topic", "")
        self.declare_parameter("lidar_pose_topic", "")
        self.declare_parameter("lidar_z_topic", "")
        self.declare_parameter("lidar_pose_var", 0.05)
        # Often tighten yaw vs x,y,z so LiDAR corrects heading drift strongly
        self.declare_parameter("lidar_yaw_var", 0.02)
        self.declare_parameter("lidar_z_var", 0.05)
        self.declare_parameter("lidar_require_frames", True)
        # If false: fuse only x,y,yaw from LiDAR odom (z from /lidar/z or IMU hold)
        self.declare_parameter("lidar_fuse_z_from_odom", False)
        # If true (planar path only): fuse only x,y from LiDAR; yaw from IMU integration only.
        # Use when LiDAR/NDT heading is untrustworthy but position is good — fix IMU mount if yaw still wrong.
        self.declare_parameter("lidar_fuse_xy_only", False)
        self.declare_parameter("lidar_use_roll_pitch", False)
        self.declare_parameter("lidar_gate_nis", 16.0)
        # If set (e.g. /livox/lidar), stamp /ekf/odom + TF from this cloud header when not
        # fusing LiDAR odom — aligns TF with Livox time when IMU uses a different clock (Microstrain).
        self.declare_parameter("lidar_cloud_stamp_topic", "")
        # Limit /ekf/odom + TF rate during IMU-only coast (0 = unlimited). LiDAR/cloud publishes ignore this.
        self.declare_parameter("max_odom_tf_publish_rate_hz", 25.0)
        # Add to sensor header stamps so IMU + LiDAR share one timeline (fixed clock skew vs Livox).
        self.declare_parameter("imu_stamp_offset_sec", 0.0)
        self.declare_parameter("lidar_stamp_offset_sec", 0.0)
        # Subtracted from ωz (rad/s) after TF to base_link, before EKF predict. At rest, set ≈ mean(ωz)
        # on /imu/data (e.g. -0.012 if the GX5 reads -0.012 stationary) to kill pure bias integration drift.
        self.declare_parameter("imu_gyro_z_bias_rad_s", 0.0)
        # First tune_sec of IMU time: average ωz (after TF); then subtract that mean + manual bias. Use when
        # bag starts stationary (robot still); set manual to 0 to avoid double correction.
        self.declare_parameter("imu_auto_gyro_z_bias_enable", False)
        self.declare_parameter("imu_auto_gyro_z_bias_tune_sec", 4.0)
        # Applied after bias subtraction (try -1.0 if yaw runs opposite to truth / TF sign error).
        self.declare_parameter("imu_gyro_z_scale", 1.0)
        # Optional: fuse planar scan delta (TwistStamped linear.x/y = dx,dy in odom; dz unused)
        # into vx, vy between full LiDAR pose updates (gyro-only translation mode).
        self.declare_parameter("lidar_delta_topic", "")
        self.declare_parameter("lidar_delta_vel_var", 0.22)
        self.declare_parameter("lidar_delta_gate_nis", 200.0)
        self.declare_parameter("lidar_delta_nominal_dt_sec", 0.1)
        # Verbose LiDAR fusion line (throttled): NIS, innovations, applied flags.
        self.declare_parameter("lidar_fusion_debug_log", False)
        self.declare_parameter("lidar_fusion_debug_throttle_sec", 1.0)
        # After gated xy,yaw reject (NDT path), run one soft ungated update (large R). Default
        # true: avoids gyro-only coast when NIS fails on yaw but |Δxy| is small.
        self.declare_parameter("lidar_soft_fuse_after_gate_reject", True)

        imu_topic = self.get_parameter("imu_topic").value
        self.pub_topic = self.get_parameter("publish_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.path_topic = self.get_parameter("path_topic").value

        self.nominal_dt = float(self.get_parameter("nominal_dt").value)
        self.use_stamp_dt = bool(self.get_parameter("use_stamp_dt").value)

        odom_frame = str(self.get_parameter("odom_frame").value).strip() or "odom"
        world_legacy = str(self.get_parameter("world_frame").value).strip()
        if world_legacy and world_legacy != odom_frame:
            self.get_logger().warn(
                f"Parameter world_frame='{world_legacy}' is ignored; "
                f"publishing odometry/TF in '{odom_frame}'. "
                "Add a static map->odom transform if RViz fixed frame is map."
            )
        self.odom_frame = odom_frame

        self.base_link_frame = str(self.get_parameter("base_link_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self._publish_use_ros_time_in_headers = bool(
            self.get_parameter("publish_use_ros_time_in_headers").value
        )
        self.transform_imu = bool(
            self.get_parameter("transform_imu_to_base_link").value
        )
        self.imu_tf_timeout = float(
            self.get_parameter("imu_tf_lookup_timeout_sec").value
        )

        q_list = list(self.get_parameter("process_noise_diag").value)
        p0_list = list(self.get_parameter("initial_cov_diag").value)
        self.flat_orientation_variance = float(
            self.get_parameter("flat_orientation_variance").value
        )

        self.lidar_odom_topic = str(self.get_parameter("lidar_odom_topic").value)
        self.lidar_pose_topic = str(self.get_parameter("lidar_pose_topic").value)
        self.lidar_z_topic = str(self.get_parameter("lidar_z_topic").value)
        self.lidar_pose_var = float(self.get_parameter("lidar_pose_var").value)
        self.lidar_yaw_var = float(self.get_parameter("lidar_yaw_var").value)
        self.lidar_z_var = float(self.get_parameter("lidar_z_var").value)
        self.lidar_require_frames = bool(
            self.get_parameter("lidar_require_frames").value
        )
        self.lidar_fuse_z_from_odom = bool(
            self.get_parameter("lidar_fuse_z_from_odom").value
        )
        self.lidar_fuse_xy_only = bool(
            self.get_parameter("lidar_fuse_xy_only").value
        )
        self.lidar_use_roll_pitch = bool(
            self.get_parameter("lidar_use_roll_pitch").value
        )
        self.lidar_gate_nis = float(self.get_parameter("lidar_gate_nis").value)
        self._lidar_soft_fuse_after_reject = bool(
            self.get_parameter("lidar_soft_fuse_after_gate_reject").value
        )
        # Throttle state only; re-read lidar_fusion_debug_log each callback so
        # ``ros2 param set`` works without restart.
        self._lidar_fusion_debug_last_t = None

        self._imu_auto_bias_enable = bool(
            self.get_parameter("imu_auto_gyro_z_bias_enable").value
        )
        self._imu_auto_tune_sec = max(
            0.5, float(self.get_parameter("imu_auto_gyro_z_bias_tune_sec").value)
        )
        self._imu_auto_z_samples: list = []
        self._imu_auto_t0_msg = None
        self._imu_auto_mean = None  # frozen mean, rad/s
        self._imu_auto_warned_no_samples = False

        self._lidar_cloud_stamp = None
        self._lidar_cloud_stamp_topic = str(
            self.get_parameter("lidar_cloud_stamp_topic").value
        ).strip()
        self._max_odom_tf_hz = float(
            self.get_parameter("max_odom_tf_publish_rate_hz").value
        )
        self._last_imu_odom_pub_time = None
        self._imu_stamp_offset_sec = float(
            self.get_parameter("imu_stamp_offset_sec").value
        )
        self._lidar_stamp_offset_sec = float(
            self.get_parameter("lidar_stamp_offset_sec").value
        )

        self.lidar_delta_topic = str(
            self.get_parameter("lidar_delta_topic").value or ""
        ).strip()
        self.lidar_delta_vel_var = float(
            self.get_parameter("lidar_delta_vel_var").value
        )
        self.lidar_delta_gate_nis = float(
            self.get_parameter("lidar_delta_gate_nis").value
        )
        self._lidar_delta_nominal_dt = float(
            self.get_parameter("lidar_delta_nominal_dt_sec").value
        )
        self._lidar_delta_prev_stamp = None

        if self.lidar_use_roll_pitch:
            self.get_logger().warn(
                "lidar_use_roll_pitch is unsupported in planar EKF; forcing false"
            )
            self.lidar_use_roll_pitch = False
        if self.lidar_fuse_xy_only and self.lidar_fuse_z_from_odom:
            self.get_logger().warn(
                "lidar_fuse_xy_only ignored when lidar_fuse_z_from_odom is true (full pose fusion)"
            )
            self.lidar_fuse_xy_only = False

        # --------------------
        # EKF
        # --------------------
        self.ekf = EKFPlanarIMU(
            dt=self.nominal_dt,
            process_noise_diag=np.array(q_list, dtype=float),
            initial_cov_diag=np.array(p0_list, dtype=float),
        )
        self.ekf.nis_gate_default = self.lidar_gate_nis
        self.ekf.use_linear_accel = bool(
            self.get_parameter("predict_use_linear_accel").value
        )
        self.last_imu_stamp = None
        # ``odom``→``base_link`` TF uses **node clock time** (not odometry header time) so tf2 never sees
        # out-of-order transforms during bag replay (LiDAR/IMU stamps vs /clock). /ekf/odom headers unchanged.
        self._last_tf_pub_stamp_msg = None
        self._warned_lidar_odom_parent = False
        self._warned_lidar_odom_child = False
        self._warned_imu_tf = False
        # If LiDAR odom is NIS-rejected while EKF is near origin, snap once (gyro-only + tight R).
        self._lidar_bootstrap_done = False
        # Further snaps when LiDAR xy disagrees strongly with EKF (rejects / yaw–xy coupling).
        self._lidar_snap_remaining = 5

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --------------------
        # Publishers
        # --------------------
        self.pub_odom = self.create_publisher(Odometry, self.pub_topic, 10)
        self.pub_pose = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.pub_path = self.create_publisher(Path, self.path_topic, 10)

        imu_src_topic = str(self.get_parameter("imu_source_topic").value).strip()
        imu_src_id = str(self.get_parameter("imu_source_id").value).strip().lower()
        if imu_src_id not in ("livox", "microstrain"):
            imu_src_id = "unknown"
        self.pub_imu_source = None
        if imu_src_topic:
            qos_latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.pub_imu_source = self.create_publisher(String, imu_src_topic, qos_latched)
            m = String()
            m.data = imu_src_id
            self.pub_imu_source.publish(m)

        self.ekf_path = Path()
        self.ekf_path.header.frame_id = self.odom_frame

        # --------------------
        # TF Broadcaster
        # --------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # --------------------
        # Subscribers
        # --------------------
        # Cloud stamp first: publish odom→base_link at each cloud time before other callbacks in the
        # same executor tick where possible (helps tf2 vs NDT scan time).
        if self._lidar_cloud_stamp_topic:
            self.create_subscription(
                PointCloud2,
                self._lidar_cloud_stamp_topic,
                self._on_lidar_cloud_stamp,
                qos_profile_sensor_data,
            )

        self.create_subscription(Imu, imu_topic, self.imu_callback, _IMU_SUB_QOS)

        if self.lidar_odom_topic:
            self.create_subscription(
                Odometry, self.lidar_odom_topic, self.lidar_odom_callback, 20
            )
        if self.lidar_pose_topic:
            self.create_subscription(
                PoseStamped, self.lidar_pose_topic, self.lidar_pose_callback, 20
            )
        if self.lidar_z_topic:
            self.create_subscription(
                Float64, self.lidar_z_topic, self.lidar_z_callback, 20
            )

        if self.lidar_delta_topic:
            self.create_subscription(
                TwistStamped,
                self.lidar_delta_topic,
                self.lidar_delta_callback,
                20,
            )

        _accel = "accel+gyro" if self.ekf.use_linear_accel else "gyro-only translation"
        _gz_bias_log = float(self.get_parameter("imu_gyro_z_bias_rad_s").value)
        self.get_logger().info(
            f"EKF planar IMU node: TF {self.odom_frame} -> {self.base_link_frame}; "
            f"prediction={_accel}; IMU source={imu_src_id}"
            + (f" (latched on {imu_src_topic})" if imu_src_topic else "")
            + (
                f"; TF stamp sync from {self._lidar_cloud_stamp_topic}"
                if self._lidar_cloud_stamp_topic
                else ""
            )
            + (
                f"; LiDAR delta vel from {self.lidar_delta_topic!r} "
                f"(var={self.lidar_delta_vel_var:g})"
                if self.lidar_delta_topic
                else ""
            )
            + (
                f"; IMU odom/TF max {self._max_odom_tf_hz:g} Hz"
                if self._max_odom_tf_hz > 0.0
                else "; IMU odom/TF unlimited rate"
            )
            + (
                f"; stamp_off imu={self._imu_stamp_offset_sec:g}s lidar={self._lidar_stamp_offset_sec:g}s"
                if abs(self._imu_stamp_offset_sec) > 1e-12
                or abs(self._lidar_stamp_offset_sec) > 1e-12
                else ""
            )
            + (
                "; LiDAR fuse xy only (no yaw from /lidar/odom)"
                if self.lidar_fuse_xy_only
                else ""
            )
            + (
                f"; IMU ωz bias subtract {_gz_bias_log:.5f} rad/s"
                if abs(_gz_bias_log) > 1e-12
                else ""
            )
            + (
                f"; IMU auto ωz bias tune {self._imu_auto_tune_sec:g}s"
                if self._imu_auto_bias_enable
                else ""
            )
            + (
                f"; IMU ωz scale {float(self.get_parameter('imu_gyro_z_scale').value):g}"
                if abs(float(self.get_parameter("imu_gyro_z_scale").value) - 1.0) > 1e-9
                else ""
            )
        )

    @staticmethod
    def _stamp_add_sec(stamp_msg, sec: float):
        if abs(float(sec)) < 1e-15:
            return stamp_msg
        t = Time.from_msg(stamp_msg) + Duration(seconds=float(sec))
        return t.to_msg()

    def _imu_odom_publish_allowed(self) -> bool:
        if self._max_odom_tf_hz <= 0.0:
            return True
        now = self.get_clock().now()
        if self._last_imu_odom_pub_time is None:
            return True
        dt = (now - self._last_imu_odom_pub_time).nanoseconds * 1e-9
        return dt >= 1.0 / self._max_odom_tf_hz

    def _on_lidar_cloud_stamp(self, msg: PointCloud2) -> None:
        s = msg.header.stamp
        if int(s.sec) == 0 and int(s.nanosec) == 0:
            return
        eff_s = self._stamp_add_sec(s, self._lidar_stamp_offset_sec)
        self._lidar_cloud_stamp = eff_s
        # NDT can skip or lag vs Livox; RViz TF MessageFilter wants odom→base_link near each cloud time.
        self.publish_outputs(stamp_override=eff_s, include_path=False)

    # --------------------
    # Time handling
    # --------------------
    def compute_dt(self, stamp_msg) -> float:
        if (not self.use_stamp_dt) or (stamp_msg is None):
            return self.nominal_dt

        if self.last_imu_stamp is None:
            return self.nominal_dt

        t_now = Time.from_msg(stamp_msg)
        t_prev = Time.from_msg(self.last_imu_stamp)
        dt = (t_now - t_prev).nanoseconds * 1e-9

        if dt <= 0.0 or dt > 0.5:
            return self.nominal_dt

        return dt

    # --------------------
    # IMU callback (CORE)
    # --------------------
    def imu_callback(self, msg: Imu):
        eff_stamp = self._stamp_add_sec(msg.header.stamp, self._imu_stamp_offset_sec)
        dt = self.compute_dt(eff_stamp)
        self.last_imu_stamp = eff_stamp

        acc_meas = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=float,
        )
        gyro_meas = np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=float,
        )

        imu_frame = (msg.header.frame_id or "").strip()
        if (
            self.transform_imu
            and imu_frame
            and imu_frame != self.base_link_frame
        ):
            try:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.base_link_frame,
                        imu_frame,
                        Time.from_msg(eff_stamp),
                        timeout=Duration(seconds=self.imu_tf_timeout),
                    )
                except TransformException:
                    # Static base_link→imu_link may not exist at the IMU message time yet
                    # (startup ordering / clock); latest transform is correct for fixed extrinsic.
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.base_link_frame,
                        imu_frame,
                        Time(),
                        timeout=Duration(seconds=self.imu_tf_timeout),
                    )
                vs = Vector3Stamped()
                vs.header = msg.header
                vs.header.stamp = eff_stamp
                vs.vector = msg.linear_acceleration
                acc_t = do_transform_vector3(vs, tf_msg)
                vs.vector = msg.angular_velocity
                gyro_t = do_transform_vector3(vs, tf_msg)
                acc_meas = np.array(
                    [acc_t.vector.x, acc_t.vector.y, acc_t.vector.z], dtype=float
                )
                gyro_meas = np.array(
                    [gyro_t.vector.x, gyro_t.vector.y, gyro_t.vector.z], dtype=float
                )
            except TransformException as exc:
                if not self._warned_imu_tf:
                    self.get_logger().warn(
                        f"IMU in '{imu_frame}' but TF to '{self.base_link_frame}' failed "
                        f"({exc}); using raw IMU components until TF is available."
                    )
                    self._warned_imu_tf = True

        gyro_meas = np.array(gyro_meas, dtype=float, copy=True)
        gz_tf = float(gyro_meas[2])

        if self._imu_auto_bias_enable and self._imu_auto_mean is None:
            if self._imu_auto_t0_msg is None:
                self._imu_auto_t0_msg = Time.from_msg(eff_stamp)
            t_el = (Time.from_msg(eff_stamp) - self._imu_auto_t0_msg).nanoseconds * 1e-9
            if t_el <= self._imu_auto_tune_sec:
                self._imu_auto_z_samples.append(gz_tf)
            else:
                if self._imu_auto_z_samples:
                    self._imu_auto_mean = float(np.mean(self._imu_auto_z_samples))
                    self.get_logger().info(
                        f"IMU auto ωz bias: mean={self._imu_auto_mean:.6f} rad/s over "
                        f"{len(self._imu_auto_z_samples)} samples ({self._imu_auto_tune_sec:.2f}s window); "
                        "subtracting with manual offset"
                    )
                else:
                    if not self._imu_auto_warned_no_samples:
                        self.get_logger().warn(
                            "imu_auto_gyro_z_bias: tune window ended with no samples; ωz auto bias=0"
                        )
                        self._imu_auto_warned_no_samples = True
                    self._imu_auto_mean = 0.0

        manual = float(self.get_parameter("imu_gyro_z_bias_rad_s").value)
        auto_off = (
            float(self._imu_auto_mean)
            if self._imu_auto_mean is not None
            else 0.0
        )
        gscale = float(self.get_parameter("imu_gyro_z_scale").value)
        gyro_meas[2] = (gz_tf - manual - auto_off) * gscale

        self.ekf.predict(acc_meas, gyro_meas, dt)

        if not self._imu_odom_publish_allowed():
            return
        self._last_imu_odom_pub_time = self.get_clock().now()
        # Skip path on IMU: full Path at ~IMU Hz overloads RViz; path updates on LiDAR fusion only.
        self.publish_outputs(stamp_override=None, include_path=False)

    def lidar_delta_callback(self, msg: TwistStamped):
        """Fuse NDT planar step (dx, dy) / dt as a weak velocity measurement in odom frame."""
        eff = self._stamp_add_sec(msg.header.stamp, self._lidar_stamp_offset_sec)
        dx = float(msg.twist.linear.x)
        dy = float(msg.twist.linear.y)
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return

        if self._lidar_delta_prev_stamp is None:
            dt = float(self._lidar_delta_nominal_dt)
        else:
            t_now = Time.from_msg(eff)
            t_prev = Time.from_msg(self._lidar_delta_prev_stamp)
            dt = (t_now - t_prev).nanoseconds * 1e-9
        self._lidar_delta_prev_stamp = eff

        if dt <= 1e-6 or dt > 0.5:
            dt = float(self._lidar_delta_nominal_dt)
        dt = max(0.07, min(0.18, dt))

        vx_m = dx / dt
        vy_m = dy / dt
        applied = self.ekf.update_lidar_velocity_xy(
            vx_m,
            vy_m,
            var=self.lidar_delta_vel_var,
            gate_nis=self.lidar_delta_gate_nis,
        )
        if not applied:
            self.get_logger().warn(
                "LiDAR delta velocity EKF update rejected (NIS gate); "
                "increase lidar_delta_gate_nis or lidar_delta_vel_var.",
                throttle_duration_sec=20.0,
            )
        self.publish_outputs(stamp_override=eff, include_path=False)

    def lidar_odom_callback(self, msg: Odometry):
        meas_stamp = self._stamp_add_sec(msg.header.stamp, self._lidar_stamp_offset_sec)
        if self.lidar_require_frames:
            if (
                msg.header.frame_id != self.odom_frame
                and not self._warned_lidar_odom_parent
            ):
                self.get_logger().warn(
                    f"lidar odom frame_id is '{msg.header.frame_id}' "
                    f"(expected '{self.odom_frame}'); fix for consistent TF."
                )
                self._warned_lidar_odom_parent = True
            if (
                msg.child_frame_id != self.base_link_frame
                and not self._warned_lidar_odom_child
            ):
                self.get_logger().warn(
                    f"lidar odom child_frame_id is '{msg.child_frame_id}' "
                    f"(expected '{self.base_link_frame}')."
                )
                self._warned_lidar_odom_child = True
        self._fuse_lidar_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation,
            meas_stamp,
        )

    def lidar_pose_callback(self, msg: PoseStamped):
        meas_stamp = self._stamp_add_sec(msg.header.stamp, self._lidar_stamp_offset_sec)
        self._fuse_lidar_pose(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation,
            meas_stamp,
        )

    def lidar_z_callback(self, msg: Float64):
        self.ekf.update_lidar_z(
            z=float(msg.data), var=self.lidar_z_var, gate_nis=self.lidar_gate_nis
        )
        self.publish_outputs(stamp_override=None, include_path=False)

    def _lidar_bootstrap_from_xy_yaw(self, px: float, py: float, yaw: float) -> None:
        self.ekf.x[EKFPlanarIMU.I_PX] = px
        self.ekf.x[EKFPlanarIMU.I_PY] = py
        self.ekf.x[EKFPlanarIMU.I_YAW] = float(wrap_angle(yaw))
        for idx in (EKFPlanarIMU.I_PX, EKFPlanarIMU.I_PY, EKFPlanarIMU.I_YAW):
            self.ekf.P[idx, idx] = max(float(self.ekf.P[idx, idx]) * 6.0, 0.5)
        for idx in (EKFPlanarIMU.I_VX, EKFPlanarIMU.I_VY):
            self.ekf.P[idx, idx] = max(float(self.ekf.P[idx, idx]), 0.8)

    def _lidar_bootstrap_from_xy(self, px: float, py: float) -> None:
        """Snap position only; keep current yaw (xy-only LiDAR fusion mode)."""
        self.ekf.x[EKFPlanarIMU.I_PX] = px
        self.ekf.x[EKFPlanarIMU.I_PY] = py
        for idx in (EKFPlanarIMU.I_PX, EKFPlanarIMU.I_PY):
            self.ekf.P[idx, idx] = max(float(self.ekf.P[idx, idx]) * 6.0, 0.5)
        for idx in (EKFPlanarIMU.I_VX, EKFPlanarIMU.I_VY):
            self.ekf.P[idx, idx] = max(float(self.ekf.P[idx, idx]), 0.8)

    def _fuse_lidar_pose(self, px, py, z, orientation: Quaternion, stamp):
        # Planar correction: use yaw only; roll/pitch from LiDAR are ignored.
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _roll, _pitch, yaw = euler_from_quaternion(q)

        px_e0 = float(self.ekf.x[EKFPlanarIMU.I_PX])
        py_e0 = float(self.ekf.x[EKFPlanarIMU.I_PY])
        yaw_e0 = float(self.ekf.x[EKFPlanarIMU.I_YAW])
        innov_xy0 = float(np.hypot(float(px) - px_e0, float(py) - py_e0))
        innov_yaw0 = float(abs(wrap_angle(float(yaw) - yaw_e0)))
        nis_pre = None
        if not self.lidar_fuse_z_from_odom:
            if self.lidar_fuse_xy_only:
                nis_pre = float(
                    self.ekf.nis_lidar_xy(
                        float(px),
                        float(py),
                        self.lidar_pose_var,
                    )
                )
            else:
                nis_pre = float(
                    self.ekf.nis_lidar_xy_yaw(
                        float(px),
                        float(py),
                        float(yaw),
                        self.lidar_pose_var,
                        self.lidar_yaw_var,
                    )
                )

        applied_gated = False
        if self.lidar_fuse_z_from_odom:
            applied_gated = self.ekf.update_lidar_pose(
                px=float(px),
                py=float(py),
                z=float(z),
                roll=float(_roll),
                pitch=float(_pitch),
                yaw=float(yaw),
                var=self.lidar_pose_var,
                var_yaw=self.lidar_yaw_var,
                use_roll_pitch=self.lidar_use_roll_pitch,
                gate_nis=self.lidar_gate_nis,
            )
        elif self.lidar_fuse_xy_only:
            applied_gated = self.ekf.update_lidar_xy(
                float(px),
                float(py),
                self.lidar_pose_var,
                gate_nis=self.lidar_gate_nis,
            )
        else:
            applied_gated = self.ekf.update_lidar_xy_yaw(
                px=float(px),
                py=float(py),
                yaw=float(yaw),
                var=self.lidar_pose_var,
                var_yaw=self.lidar_yaw_var,
                gate_nis=self.lidar_gate_nis,
            )
        applied = applied_gated
        applied_soft = False

        if not applied:
            px_e = float(self.ekf.x[EKFPlanarIMU.I_PX])
            py_e = float(self.ekf.x[EKFPlanarIMU.I_PY])
            r_pred = float(np.hypot(px_e, py_e))
            r_meas = float(np.hypot(float(px), float(py)))
            innov_xy = float(np.hypot(float(px) - px_e, float(py) - py_e))
            snap = False
            if (not self._lidar_bootstrap_done) and r_pred < 0.03 and r_meas > 0.035:
                self.get_logger().info(
                    "LiDAR odom NIS rejected while EKF still near origin; "
                    "bootstrap px,py,yaw from LiDAR (Microstrain gyro-only coast)"
                )
                self._lidar_bootstrap_done = True
                snap = True
            elif self._lidar_snap_remaining > 0 and innov_xy > 0.07:
                self.get_logger().warn(
                    f"LiDAR odom NIS rejected with |Δxy|={innov_xy:.2f}m vs EKF; "
                    f"snapping pose to LiDAR ({self._lidar_snap_remaining} snaps left)",
                    throttle_duration_sec=3.0,
                )
                self._lidar_snap_remaining -= 1
                snap = True
            if snap:
                if self.lidar_fuse_xy_only:
                    self._lidar_bootstrap_from_xy(float(px), float(py))
                else:
                    self._lidar_bootstrap_from_xy_yaw(float(px), float(py), float(yaw))
                applied = True

        # Second line of defence: planar LiDAR after gated reject. Large R, no NIS.
        # Default always runs when lidar_soft_fuse_after_gate_reject (NDT + gyro-only translation).
        if (not applied) and (not self.lidar_fuse_z_from_odom):
            px_e2 = float(self.ekf.x[EKFPlanarIMU.I_PX])
            py_e2 = float(self.ekf.x[EKFPlanarIMU.I_PY])
            innov2 = float(np.hypot(float(px) - px_e2, float(py) - py_e2))
            do_soft = self._lidar_soft_fuse_after_reject or (innov2 > 0.03)
            if do_soft:
                if self.lidar_fuse_xy_only:
                    applied_soft = self.ekf.update_lidar_xy(
                        float(px),
                        float(py),
                        var=max(float(self.lidar_pose_var), 0.02) * 4.0,
                        gate_nis=None,
                    )
                else:
                    applied_soft = self.ekf.update_lidar_xy_yaw(
                        px=float(px),
                        py=float(py),
                        yaw=float(yaw),
                        var=max(float(self.lidar_pose_var), 0.02) * 4.0,
                        var_yaw=max(float(self.lidar_yaw_var), 0.01) * 4.0,
                        gate_nis=None,
                    )
                applied = applied or applied_soft

        if self.get_parameter("lidar_fusion_debug_log").value:
            now = self.get_clock().now().nanoseconds * 1e-9
            _thr = max(
                0.2, float(self.get_parameter("lidar_fusion_debug_throttle_sec").value)
            )
            if self._lidar_fusion_debug_last_t is None or (
                now - self._lidar_fusion_debug_last_t
            ) >= _thr:
                self._lidar_fusion_debug_last_t = now
                nis_s = f"{nis_pre:.2f}" if nis_pre is not None else "n/a"
                self.get_logger().info(
                    f"LiDAR fuse stamp={stamp.sec}.{stamp.nanosec:09d} "
                    f"nis~{nis_s} gate={self.lidar_gate_nis:g} "
                    f"mode={'xy_only' if self.lidar_fuse_xy_only else 'xy_yaw'} "
                    f"innov_xy={innov_xy0:.4f}m innov_yaw={math.degrees(innov_yaw0):.2f}deg "
                    f"gated={applied_gated} soft={applied_soft} applied={applied} "
                    f"meas=({float(px):.3f},{float(py):.3f}) ekf=({px_e0:.3f},{py_e0:.3f})"
                )

        if not applied:
            self.get_logger().warn(
                "LiDAR odom EKF update rejected (NIS gate). Pose will not track LiDAR; "
                "increase slam_bringup 'ekf_lidar_gate_nis' / lidar_pose_var or fix "
                "the LiDAR odometry source (NDT / LIO).",
                throttle_duration_sec=8.0,
            )
        # Do not overwrite last_imu_stamp: keep IMU dt chain; publish this
        # correction with the LiDAR message time for TF/odom sync.
        self.publish_outputs(stamp_override=stamp, include_path=True)

    # --------------------
    # Publish results
    # --------------------
    def publish_outputs(self, stamp_override=None, *, include_path: bool = True):
        pos, rpy = self.ekf.get_pose()
        px, py, pz = pos[0], pos[1], pos[2]
        roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
        x = self.ekf.get_state()
        vx, vy = float(x[EKFPlanarIMU.I_VX]), float(x[EKFPlanarIMU.I_VY])
        P = self.ekf.P
        v_roll = self.flat_orientation_variance

        if self._publish_use_ros_time_in_headers:
            stamp = self.get_clock().now().to_msg()
        elif stamp_override is not None:
            stamp = stamp_override
        elif self._lidar_cloud_stamp is not None:
            stamp = self._lidar_cloud_stamp
        elif self.last_imu_stamp is not None:
            stamp = self.last_imu_stamp
        else:
            stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame

        odom.pose.pose.position.x = float(px)
        odom.pose.pose.position.y = float(py)
        odom.pose.pose.position.z = float(pz)

        qx, qy, qz, qw = quaternion_from_euler(float(roll), float(pitch), float(yaw))
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)

        odom.pose.covariance[0] = float(P[0, 0])
        odom.pose.covariance[7] = float(P[1, 1])
        odom.pose.covariance[14] = float(P[2, 2])
        odom.pose.covariance[21] = float(v_roll)
        odom.pose.covariance[28] = float(v_roll)
        odom.pose.covariance[35] = float(P[3, 3])

        odom.twist.covariance[0] = float(P[4, 4])
        odom.twist.covariance[7] = float(P[5, 5])

        self.pub_odom.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pub_pose.publish(pose)

        if include_path:
            self.ekf_path.header.stamp = odom.header.stamp
            if self.ekf_path.poses:
                lp = self.ekf_path.poses[-1]
                if (
                    lp.header.stamp.sec == pose.header.stamp.sec
                    and lp.header.stamp.nanosec == pose.header.stamp.nanosec
                ):
                    self.ekf_path.poses[-1] = pose
                else:
                    self.ekf_path.poses.append(pose)
            else:
                self.ekf_path.poses.append(pose)

            if len(self.ekf_path.poses) > 400:
                self.ekf_path.poses.pop(0)

            self.pub_path.publish(self.ekf_path)

        if self.publish_tf:
            t = TransformStamped()
            # Must match odom.header.stamp (sensor / filter time). NDT looks up odom→base_link at
            # PointCloud2.header.stamp; stamping TF with wall/sim "now" leaves the cloud in the TF
            # future → lookup fails → NDT falls back to its stale pose while EKF still moves.
            stamp_tf = stamp
            if self._last_tf_pub_stamp_msg is not None:
                t_prev = Time.from_msg(self._last_tf_pub_stamp_msg)
                t_cur = Time.from_msg(stamp_tf)
                if t_cur <= t_prev:
                    stamp_tf = (t_prev + Duration(nanoseconds=1)).to_msg()
            t.header.stamp = stamp_tf
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_link_frame

            t.transform.translation.x = float(px)
            t.transform.translation.y = float(py)
            t.transform.translation.z = float(pz)
            t.transform.rotation = odom.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)
            self._last_tf_pub_stamp_msg = t.header.stamp


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
