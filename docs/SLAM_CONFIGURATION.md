# SLAM configuration — knob vs rest

**Knobs** in each YAML are **only** the parameters named in **`docs/tuning.md`** (§1–§10). **`docs/tuning.md`** now lists **suggested values**, **what each parameter does**, and **what happens when you change it** (per knob). Everything else lives under **`REST`** / **`REST OF BRINGUP`** so launch and nodes still receive required topics, frames, and paths.

| File | Knob block | Canonical list |
|------|------------|----------------|
| `src/ros_project_bringup/config/slam_bringup.yaml` | **`KNOBS`** | `docs/tuning.md` §1, §2, §4, §5, §6, §7, §8, §9 (+ `microstrain_imu_origin` in §1) |
| `src/localisation_ekf/config/ekf_python_gx5_microstrain.yaml` | **`KNOBS`** | `docs/tuning.md` §3 (`process_noise_diag` only) |
| `src/lio_bringup/config/fastlio_mid360_overlay.yaml` | first keys under `preprocess` / `mapping` marked *knobs* | `docs/tuning.md` §10 |
| `src/keyframe_scan_map/config/keyframe_map.yaml` | **`KNOBS`** | `docs/tuning.md` §6–7 |
| `src/keyframe_scan_map/config/pose_graph.yaml` | **`KNOBS`** | `docs/tuning.md` §8 |

**Also edit when hardware changes (not in tuning.md tables):** `docs/imu_mount_note.md`, `docs/livox_mount_note.md`, and matching keys in **`slam_bringup.yaml` → `REST OF BRINGUP`** (`imu_mount_*`, `livox_extrinsic_*`, …).

**LiDAR JSON:** `src/livox_ros_driver2/config/MID360_config.json`

**Launch wiring:** `src/ros_project_bringup/launch/launch_slam.launch.py`
