# Cloud LAN bridge — workaround for bandwidth-limited remote Livox

## Symptom

- Livox driver runs on the **robot**, SLAM stack runs on a **separate PC**.
- The link between them is Wi-Fi (router or mobile hotspot) or another
  bandwidth-limited network.
- Without SLAM: `/livox/lidar` flows at ~10 Hz when you `ros2 topic echo
  /livox/lidar --qos-reliability best_effort` from the PC.
- With SLAM running on the PC: `/livox/lidar` collapses to <1 Hz on the PC
  side, even though IMU and any small topics (e.g. perception images) stay
  fine.
- No keyframes appear because FAST-LIO can't initialise at sub-1 Hz scan rate.

## Cause

Most consumer Wi-Fi networks (and **all** iPhone Personal Hotspots) block
DDS multicast between connected stations. When multicast is unavailable,
FastDDS (and Cyclone DDS) fall back to **unicast per subscriber** — the
publisher must send a separate copy of every message to each subscriber's
address. With three SLAM subscribers on the PC

- `fastlio_mapping`
- `keyframe_map_node`
- `ekf_node` (cloud-stamp sync)

a ~40 Mbps Livox MID-360 stream becomes ~120 Mbps over the wire. Domestic
Wi-Fi sustains 30–150 Mbps; mobile hotspot 30–80 Mbps; both are easily
saturated. The robot's publisher backs up, BEST_EFFORT drops kick in, and
your effective scan rate collapses.

IMU is unaffected because it's ~0.3 Mbps, well within budget even at 3×
fanout.

## Fix

Subscribe **once** on the PC and fan out locally. The number of copies on
the wire becomes 1 regardless of how many SLAM nodes consume the cloud.

This repo ships:

- `cloud_lan_bridge` — a small node in `ros_project_bringup` that
  subscribes to the wire-side topic (BEST_EFFORT) and republishes it on a
  local-only topic with identical QoS.
- A launch arg `cloud_lan_bridge:=true` (or YAML key `cloud_lan_bridge:
  true`) that starts the bridge **and** rewires every SLAM consumer
  (`lidar_cloud_topic`, `keyframe_cloud_topic`, FAST-LIO
  `common.lid_topic`, `lio_relay_sync_tf_cloud_topic` if explicit) to the
  relayed topic.

### Use

Either pass the launch arg:

```bash
ros2 launch ros_project_bringup launch_slam.launch.py cloud_lan_bridge:=true
```

…or enable it permanently in `slam_bringup.yaml`:

```yaml
slam_bringup:
  ros__parameters:
    cloud_lan_bridge: true
```

### Knobs

All under `slam_bringup` in `config/slam_bringup.yaml`:

| key | default | meaning |
|---|---|---|
| `cloud_lan_bridge` | `false` | master enable |
| `cloud_lan_bridge_in_topic` | `/livox/lidar` | wire-side input (what the remote driver publishes) |
| `cloud_lan_bridge_out_topic` | `/livox/lidar/local` | local-side output (what SLAM consumes) |
| `cloud_lan_bridge_depth` | `5` | KEEP_LAST depth on both sides |
| `cloud_lan_bridge_imu_enable` | `false` | also relay IMU; usually unnecessary, IMU is tiny |
| `cloud_lan_bridge_imu_in_topic` | `/livox/imu` | IMU wire-side input |
| `cloud_lan_bridge_imu_out_topic` | `/livox/imu/local` | IMU local-side output |
| `cloud_lan_bridge_imu_depth` | `50` | IMU KEEP_LAST depth |

The bridge only rewires SLAM consumers whose configured topic equals the
wire input — if you've customised `keyframe_cloud_topic` to something other
than the bridge input, the rewrite is a no-op for that consumer (it'll keep
reading whatever you set).

### Verify

After launching SLAM with `cloud_lan_bridge:=true`, on the PC:

```bash
# Should be ~10 Hz steady (only one subscriber on the wire now):
ros2 topic hz /livox/lidar --qos-reliability best_effort

# Should also be ~10 Hz (this is what FAST-LIO / keyframe_map / EKF consume):
ros2 topic hz /livox/lidar/local --qos-reliability best_effort

# Should show subscribers: only cloud_lan_bridge attached over the wire,
# fastlio_mapping / keyframe_map_node / ekf_node attached locally:
ros2 topic info /livox/lidar --verbose
ros2 topic info /livox/lidar/local --verbose
```

You should see FAST-LIO initialise (`IMU Initial Done`) and keyframes start
appearing as you move the robot (default thresholds `kf_dist=0.32 m`,
`kf_yaw=8°`).

## When to use this vs other fixes

| Situation | Best fix |
|---|---|
| Robot and PC can be Ethernet-connected | Ethernet (no code) |
| Robot and PC on a real Wi-Fi router with multicast | Configure router to pass multicast (no code), or use bridge as belt-and-braces |
| Robot and PC on consumer Wi-Fi / mobile hotspot (no multicast) | `cloud_lan_bridge:=true` |
| Single-host stall pattern (Livox driver + SLAM on same PC, freezes after a few seconds, recovers ~5 s after SLAM exits) | `disable_dds_shm:=true` — see `dds_transport.md`. **Different bug, different fix.** |

The two workarounds are independent and can be combined.

## Why this isn't the default

The bridge adds one extra in-process copy of every cloud on the SLAM
machine (a few MB/s of memcpy, negligible CPU on a real PC). On stacks
that already run multicast-clean networks or Ethernet point-to-point, the
relay is pure overhead with no benefit. So it's opt-in.
