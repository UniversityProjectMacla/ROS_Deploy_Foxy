# ROS_Deploy_Foxy

A **ROS 2 Foxy** fork of `ROS_Deploy` for **offline SLAM** on bag data.

Same SLAM algorithm stack as the Humble version (FAST-LIO, PCL NDT, planar
EKF, keyframe scan map, optional SE2 pose-graph), with **Livox and Microstrain
drivers removed**. All sensor data is consumed from DDS — typically
`ros2 bag play <bag>`.

> If you need to run on real Livox / Microstrain hardware, use the original
> Humble workspace (`ROS_Deploy`).

---

## 1. What's included

| Package | Role |
|---------|------|
| `FAST_LIO_ROS2` | `fastlio_mapping` — LiDAR-inertial odometry. Patched to remove the `livox_ros_driver2/CustomMsg` AVIA path (PointCloud2 only). |
| `lio_bringup` | FAST-LIO + `lio_odom_relay_node` (`/Odometry` → `/lidar/odom` as `odom`→`base_link`). |
| `lidar_odometry` | PCL NDT scan-to-map / scan-to-scan → `/lidar/odom`. |
| `localisation_ekf` | Python planar EKF: IMU prediction + LiDAR pose correction. |
| `keyframe_scan_map` | Keyframe-merged `/keyframe_map` (+ optional loop closure + `pose_graph_node` SE2 optimiser). |
| `ros_project_bringup` | `launch_slam.launch.py` + `slam_bringup.yaml` + RViz config (`rviz/slam.rviz`). |

## 2. What's removed compared to `ROS_Deploy`

- `livox_ros_driver2` package (vendored driver).
- All Microstrain driver references in launch and configs.
- `start_livox_driver` / `launch_sensors` / `use_microstrain_imu` runtime
  branches in `launch_slam.launch.py` (the launch never starts a driver).
- Microstrain-specific YAMLs: `ekf_python_gx5_microstrain.yaml`,
  `microstrain_params_overlay.yaml`, `livox_ndt_bag_axes_overlay.yaml`,
  `overlay_agile_mode.yaml`.

## 3. Foxy-specific changes (vs Humble)

| Area | Change |
|------|--------|
| `static_transform_publisher` | **Positional** arguments (`x y z yaw pitch roll parent child`). Humble's `--x` / `--frame-id` syntax does not work on Foxy. |
| `SetUseSimTime` launch action | Removed (Galactic+ only). `use_sim_time` is set as a parameter on every node instead. |
| FAST-LIO `CustomMsg` | Removed source paths, headers, and `livox_ros_driver2` build dependency. AVIA LiDAR is unsupported in this port. |
| Apt packages | Use `ros-foxy-*` (not `ros-humble-*`). |

## 4. Prerequisites

- **Ubuntu 20.04** (only OS officially supported by ROS 2 Foxy).
- **ROS 2 Foxy** (`ros-foxy-desktop` recommended).
- `colcon`, `rosdep`.
- System PCL (`libpcl-dev`) and Eigen.
- ROS apt packages: `ros-foxy-tf-transformations`, `ros-foxy-tf2-geometry-msgs`,
  `ros-foxy-sensor-msgs-py`, `ros-foxy-pcl-conversions`, `ros-foxy-pcl-ros`,
  `ros-foxy-rviz2`, `ros-foxy-rosbag2`, `ros-foxy-rosbag2-storage-default-plugins`.

> Foxy reached end of life in May 2023. Security patches are no longer
> released by OSRF; install only on isolated / offline machines if possible.

## 5. Build

```bash
cd ~/ROS_Deploy_Foxy
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Build subset while iterating:

```bash
colcon build --packages-select localisation_ekf lidar_odometry ros_project_bringup
```

## 6. Run — bag replay

### 6.1 NDT path (default if you set `use_lio: false` in YAML)

Terminal 1 — start the SLAM stack:

```bash
cd ~/ROS_Deploy_Foxy
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=false use_lidar_fusion:=true
```

Terminal 2 — play the bag (note `--clock`):

```bash
source /opt/ros/foxy/setup.bash
ros2 bag play ~/bags/session_01 --clock
```

### 6.2 FAST-LIO path (default in `slam_bringup.yaml`)

```bash
ros2 launch ros_project_bringup launch_slam.launch.py
# or explicitly:
ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=true use_lidar_fusion:=false
```

### 6.3 Quick sanity check

```bash
ros2 node list | grep -E 'ekf|lidar|keyframe|fastlio'
ros2 topic echo /lidar/odom --once
ros2 topic echo /ekf/odom --once
```

## 7. Where to tune

- **`src/ros_project_bringup/config/slam_bringup.yaml`** — main entry point.
  Stack toggles, NDT/EKF/keyframe/pose-graph knobs, topic names, static TF
  extrinsics.
- **`src/localisation_ekf/config/ekf_python.yaml`** — EKF process/measurement
  noise, initial covariance.
- **`src/FAST_LIO_ROS2/config/mid360.yaml`** + `src/lio_bringup/config/fastlio_mid360_overlay.yaml`
  — FAST-LIO covariances, extrinsics, publish flags.
- **`src/keyframe_scan_map/config/keyframe_map.yaml`** + `config/pose_graph.yaml`.
- **`src/ros_project_bringup/rviz/slam.rviz`** — RViz layout.

## 8. Frame chain

```
map  --[static identity]-->  odom  --[ekf_node]-->  base_link  --[static]-->  livox_frame
```

- `map`→`odom`: identity static publisher (or `pose_graph_node` if
  `pose_graph_publish_map_odom_tf: true`).
- `odom`→`base_link`: `ekf_node` (default); `lio_odom_relay_node` if
  `lio_relay_publish_tf: true` and `ekf_publish_tf_when_lio: false`.
- `base_link`→`livox_frame`: static publisher from `livox_extrinsic_*` in
  `slam_bringup.yaml`. **Calibrate** if your bag uses different extrinsics.
- `base_link`→`imu_link`: static from `imu_mount_*` (used when the bag
  does not provide it on `/tf_static`).

## 9. Known caveats

- **No live Livox / Microstrain drivers.** If you want them, port them
  yourself (Livox driver supports Foxy upstream) or use the Humble workspace.
- **AVIA LiDAR is unsupported** in this port (CustomMsg path stripped). Use
  any sensor that publishes `sensor_msgs/PointCloud2`.
- **Foxy is EOL.** New code rarely targets it; expect to vendor-patch
  third-party deps if they only ship for Humble+.

## 10. Original docs

The package-level READMEs (`src/lidar_odometry/README.md`,
`src/keyframe_scan_map/README.md`, `src/FAST_LIO_ROS2/README.md`,
`src/localisation_ekf/localisation_ekf/README.md`) are inherited from the
Humble workspace. Topic names, parameters, and tuning advice in those
documents still apply.
