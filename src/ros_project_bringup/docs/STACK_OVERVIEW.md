# ROS stack overview (`ros_project_bringup` + fusion packages)

This document summarizes **functionality**, **programs**, **nodes**, **interactions**, **runtime process**, and **device setup**. Bringup is driven by `launch/launch_slam.launch.py`, which loads **`config/slam_bringup.yaml`** (optional overlay via `bringup_config` or `ROS_PROJECT_SLAM_CONFIG`).

---

## Why “NDT or LIO”?

They are **two alternative front-ends** that both produce **LiDAR-based odometry** for the same downstream consumers:

| Mode | Package / nodes | What it does |
|------|------------------|--------------|
| **NDT** (default when `use_lidar_fusion` and not `use_lio`) | `lidar_odometry` → **`lidar_odometry_node`** | PCL **NDT** **scan_to_map** on Livox clouds; motion prior from **`odom`→`base_link` TF** at cloud time (**`ekf_node`**) when enabled → **`/lidar/odom`**, **`/lidar/relative_motion`**, **`/lidar/pose_correction`**. |
| **LIO** (when `use_lio`) | `FAST_LIO_ROS2` + `lio_bringup` → **`fastlio_mapping`** + **`lio_odom_relay_node`** | FAST-LIO odometry relayed to the **same** **`/lidar/odom`** topic so the EKF and keyframe map see one interface. |

**Only one runs** in `launch_slam`: NDT is skipped if **`use_lio`** is true. The EKF always fuses **`/lidar/odom`** when LiDAR fusion is enabled—it does not care whether NDT or LIO produced it.

---

## Functionality

- **Front-end (fast):** IMU prediction + LiDAR odometry (NDT **or** LIO) → **planar Python EKF** → **`odom` → `base_link`** TF and **`/ekf/*`** topics.
- **Sensors:** Livox MID360 point cloud; IMU from **Livox** or optional **Lord Microstrain GX5-25**.
- **Back-end (optional, slower):** Keyframe merged map, loop detection, optional SE2 pose graph (SciPy), optional map rebuild or dynamic **`map` → `odom`**.
- **Status:** **`ekf_node`** publishes a **latched** **`std_msgs/String`** on **`/ekf/imu_source`** (configurable): **`data`** is **`livox`** or **`microstrain`**, matching `use_microstrain_imu` in bringup. Set **`ekf_imu_source_topic`** to `""` in **`ekf_python.yaml`** or bringup overrides to disable.

---

## Programs / packages

| Package | Role |
|---------|------|
| `livox_ros_driver2` | Livox driver (cloud + optional IMU). |
| `lidar_odometry` | NDT node → `/lidar/odom`. |
| `FAST_LIO_ROS2` + `lio_bringup` | Optional LIO → same `/lidar/odom` path. |
| `localisation_ekf` | `ekf_node` — fusion + TF + **`/ekf/imu_source`**. |
| `keyframe_scan_map` | `keyframe_map_node`, optional `pose_graph_node`. |
| `microstrain_inertial_driver` | Optional GX5-25 driver. |
| `ros_project_bringup` | `launch_slam.launch.py`, RViz, this doc (installed under share). |

---

## Nodes and interactions

- **Driver(s):** Livox launch; optional **Microstrain** node → `sensor_msgs/Imu`.
- **Static TF:** `map`→`odom` (identity unless pose-graph TF mode), `base_link`→`livox_frame`, optional `base_link`→`imu_link` (Microstrain mount).
- **LiDAR odom:** NDT **or** LIO → **`/lidar/odom`** (`odom`→`base_link` semantics in the message).
- **EKF:** Subscribes **IMU** + optional **`/lidar/odom`**; may **rotate IMU** vectors into `base_link` via TF; publishes **`/ekf/odom`**, pose, path, TF; **latched** **`/ekf/imu_source`** (`livox` / `microstrain`).
- **Keyframe / pose graph (optional):** Merged map, loop signals, corrected path / TF.

**Typical TF chain:** `map` → `odom` → `base_link` → `livox_frame` (+ `imu_link` if used).

---

## Runtime process

1. Start Livox (and Microstrain if enabled) + static TFs.
2. Start **NDT** or **LIO** relay → `/lidar/odom`.
3. **EKF** integrates IMU and corrects from LiDAR odom when enabled.
4. Optional keyframe map and pose graph on sparse keyframes.

---

## Device setup (what you configure)

- **Livox MID360:** `MID360_config.json` (or `livox_config_path`); extrinsic **`base_link` → `livox_frame`** in bringup (`livox_extrinsic_*`).
- **Microstrain GX5-25:** `microstrain_port`, `microstrain_baud`; install driver; **`imu_mount_*`** and alignment with `microstrain_frame_id` / `imu_mount_child_frame`; optional `ekf_params_yaml` for GX5-tuned EKF.
- **Frames:** Consistent `odom`, `base_link`, sensor frames so NDT/LIO and EKF agree.

---

## Where to edit

- **Bringup knobs:** `ros_project_bringup/config/slam_bringup.yaml` (merged with optional overlay).
- **EKF / NDT / keyframe YAML:** package `share/.../config/` files referenced from that dict.
