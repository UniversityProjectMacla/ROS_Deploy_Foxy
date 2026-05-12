"""Local-side LAN PointCloud2 relay.

When the Livox driver runs on a remote robot and the SLAM stack
(``fastlio_mapping``, ``keyframe_map_node``, ``ekf_node`` cloud-stamp
sub) runs on a separate host across a bandwidth-limited link (domestic
Wi-Fi, mobile hotspot, slow LAN) where DDS multicast does not pass, the
publisher must send a separate **unicast copy** of every cloud to each
subscriber on the SLAM host. With three SLAM subscribers and a
~40 Mbps Livox MID-360 stream that is ~120 Mbps of duplicated traffic,
which routinely degrades ``/livox/lidar`` from 10 Hz to <1 Hz on
domestic Wi-Fi and starves FAST-LIO so far out of its IMU-integration
window that the keyframe pipeline never crosses its motion thresholds
and no map is produced.

This node fixes the multiplication: it subscribes **once** to the
remote cloud topic and republishes it on a local topic with identical
QoS (BEST_EFFORT, KEEP_LAST). When the launch wires ``lidar_cloud_topic``,
``keyframe_cloud_topic``, and FAST-LIO's ``common.lid_topic`` at the
relay's output, every SLAM consumer subscribes locally only, so the
over-the-wire copy count is fixed at 1 regardless of how many local
subscribers attach.

See ``docs/network_bandwidth.md``.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, PointCloud2


def _best_effort_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        depth=max(1, int(depth)),
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
    )


class CloudLanBridge(Node):
    """Single-host fanout point for a remote PointCloud2 (+ optional IMU)."""

    def __init__(self) -> None:
        super().__init__('cloud_lan_bridge')
        self.declare_parameter('in_topic', '/livox/lidar')
        self.declare_parameter('out_topic', '/livox/lidar/local')
        self.declare_parameter('depth', 5)
        self.declare_parameter('imu_in_topic', '')
        self.declare_parameter('imu_out_topic', '')
        self.declare_parameter('imu_depth', 50)

        in_t = str(self.get_parameter('in_topic').value).strip()
        out_t = str(self.get_parameter('out_topic').value).strip()
        depth = int(self.get_parameter('depth').value)
        imu_in = str(self.get_parameter('imu_in_topic').value).strip()
        imu_out = str(self.get_parameter('imu_out_topic').value).strip()
        imu_depth = int(self.get_parameter('imu_depth').value)

        if not in_t or not out_t:
            raise RuntimeError(
                'cloud_lan_bridge: in_topic and out_topic must both be set'
            )
        if in_t == out_t:
            raise RuntimeError(
                f'cloud_lan_bridge: in_topic and out_topic must differ ({in_t})'
            )

        cloud_qos = _best_effort_qos(depth)
        self._cloud_pub = self.create_publisher(PointCloud2, out_t, cloud_qos)
        self._cloud_sub = self.create_subscription(
            PointCloud2, in_t, self._on_cloud, cloud_qos
        )
        self._n_cloud = 0

        self._imu_pub = None
        if imu_in and imu_out and imu_in != imu_out:
            imu_qos = _best_effort_qos(imu_depth)
            self._imu_pub = self.create_publisher(Imu, imu_out, imu_qos)
            self.create_subscription(Imu, imu_in, self._on_imu, imu_qos)
            self._n_imu = 0

        self.get_logger().info(
            'cloud_lan_bridge: cloud {i} -> {o} (best_effort, depth={d})'.format(
                i=in_t, o=out_t, d=cloud_qos.depth
            )
            + (
                f'; imu {imu_in} -> {imu_out} (best_effort, depth={imu_depth})'
                if self._imu_pub is not None
                else ''
            )
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._cloud_pub.publish(msg)
        self._n_cloud += 1
        if self._n_cloud in (1, 25, 100) or self._n_cloud % 500 == 0:
            self.get_logger().info(
                f'cloud_lan_bridge: relayed {self._n_cloud} clouds'
            )

    def _on_imu(self, msg: Imu) -> None:
        if self._imu_pub is None:
            return
        self._imu_pub.publish(msg)
        self._n_imu += 1
        if self._n_imu in (1, 200) or self._n_imu % 2000 == 0:
            self.get_logger().info(
                f'cloud_lan_bridge: relayed {self._n_imu} imu samples'
            )


def main() -> None:
    rclpy.init()
    node = CloudLanBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
