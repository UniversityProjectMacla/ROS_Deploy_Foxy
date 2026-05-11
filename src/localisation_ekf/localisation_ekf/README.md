# EKF Node Quick Usage Guide

`ekf_node` runs a **planar IMU EKF** (`EKFPlanarIMU`) with:

- **Pose:** \(x, y, z, \theta\) (`theta` = yaw)
- **Velocities:** \(v_x, v_y\) (world frame, for IMU propagation)
- **Biases:** `bax`, `bay` (body horizontal accel, m/s²), `bgz` (yaw rate, rad/s)

`z` is updated by LiDAR (or held); it is **not** integrated from IMU. Roll and pitch are **not** estimated (output roll/pitch = 0).

Default parameters live in `config/ekf_python.yaml` (install/share `localisation_ekf/config/`).

Fusion modes:

- IMU-only
- IMU + LiDAR pose (`x, y, z, yaw`)
- IMU + LiDAR z-only
- IMU + LiDAR pose + z-only reinforcement

## State, process noise (Q), and measurement noise (R)

- **Q:** parameter `process_noise_diag` — length **9**, same order as the state above. Larger values → more model uncertainty / faster motion allowed; bias entries control how quickly biases can drift.
- **R (LiDAR correction):** IMU drives **prediction**; LiDAR drives **measurement updates**. With **`lidar_fuse_z_from_odom: true`**, the filter fuses `x, y, z, yaw`. With **`false`** (default for planar `/lidar/odom` where `z=0` is meaningless), it fuses **`x, y, yaw` only** so altitude is not pulled to zero—use **`lidar_z_topic`** or enable z fusion when your LiDAR pose has a real `z` (e.g. FAST-LIO). Use **`lidar_yaw_var`** (often smaller than `lidar_pose_var`) to weight **heading** strongly.
- **R:** `lidar_z_var` for z-only updates (`update_lidar_z`).
- **Initial covariance:** `initial_cov_diag` (length 9).
- **`lidar_require_frames`:** if true, warn once when `/lidar/odom` is not `odom` → `base_link`.
- After a LiDAR update, `/ekf/odom` and TF use the **LiDAR message stamp**; IMU `last_imu_stamp` is unchanged so **dt** between IMU samples stays correct.

## TF and frames (ROS convention)

- `nav_msgs/Odometry` uses `header.frame_id` = **`odom_frame`** (default `odom`), `child_frame_id` = **`base_link_frame`**.
- When `publish_tf` is true, the node broadcasts **`odom_frame` → `base_link_frame`**.
- Publish **`map` → `odom`** separately (static identity for bringup, or SLAM). Example chain:

  `map` → `odom` → `base_link` → `livox_frame`

The legacy parameter `world_frame` is **ignored**; use `odom_frame` and a separate `map`→`odom` transform.

## 1) IMU-only

```yaml
imu_topic: /livox/imu
odom_frame: odom
base_link_frame: base_link
publish_tf: true
lidar_odom_topic: ""
lidar_pose_topic: ""
lidar_z_topic: ""
```

Drift in `x/y/yaw` is expected without LiDAR corrections.

## 2) IMU + LiDAR full pose fusion

Set `lidar_odom_topic` or `lidar_pose_topic`. Tune `lidar_pose_var`, `lidar_gate_nis`.

`lidar_use_roll_pitch` is **not supported** in the planar filter (leave `false`).

## 3) IMU + LiDAR z-only

Set `lidar_z_topic` (`std_msgs/Float64`, `data = z`).

## 4) IMU + LiDAR pose + z-only reinforcement

Set pose source **and** `lidar_z_topic`; tune `lidar_pose_var` and `lidar_z_var`.

## Common outputs

- Odometry: `publish_topic` (default `/ekf/odom`)
- Pose: `pose_topic` (default `/ekf/pose`)
- Path: `path_topic` (default `/ekf/path`)
- TF: `odom_frame` → `base_link_frame` when `publish_tf: true`

## Notes

- Workspace defaults are **accuracy-first** (tight `lidar_pose_var` / `lidar_yaw_var`, higher `lidar_gate_nis`). **`launch_slam.launch.py`** can override those via **`ekf_lidar_pose_var`**, **`ekf_lidar_yaw_var`**, **`ekf_lidar_gate_nis`** without editing YAML. If estimates jump too much, **increase** variances or **lower** `lidar_gate_nis`.
- If the filter feels sluggish: decrease pose measurement variance or slightly increase velocity entries in `process_noise_diag`.
- Planar model assumes small roll/pitch; rely on LiDAR pose rate for correction in rough terrain.
