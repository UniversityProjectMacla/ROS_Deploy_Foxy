# keyframe_scan_map

Simple **keyframe scan map**: Livox clouds transformed into **`map`**, merged only when the robot has moved enough (distance, yaw, or time) since the last keyframe. Publishes one **`sensor_msgs/PointCloud2`** plus an optional **`nav_msgs/Path`** of keyframe poses.

**Global map error** still tracks **odometry drift** (EKF + lidar odom). Optional **loop closure** only **detects** revisits; **pose_graph_node** can then optimize poses, and **keyframe_map_node** can optionally **rebuild** the merged cloud from corrected poses (**`apply_pose_graph_corrections`**) or you can use **dynamic `map`→`odom`** from the pose graph (**`publish_map_odom_tf`**) — **not both** (double correction).

## Topics

| | |
|--|--|
| Subscribes | `PointCloud2` (default `/livox/lidar`) |
| Publishes | `PointCloud2` (default `/keyframe_map`) |
| Publishes | `Path` (default `/keyframe_map/keyframes`) if `publish_keyframe_path: true` |
| Publishes (if **`loop_closure_enable: true`**) | See **Loop closure** |

## Loop closure (simple detection)

When **`loop_closure_enable`** is **true**, the node:

1. **Stores key poses** (already used for the path) and a **voxel-downsampled copy** of each keyframe scan in **`map`** (compact memory cap per keyframe).
2. On each **new** keyframe, compares the current scan to **past** keyframes that are:
   - at least **`loop_min_index_gap`** indices older (avoids matching the recent trajectory), and  
   - within **`loop_proximity_xy_m`** and **`loop_proximity_yaw_deg`** of the current pose (revisit / loop hypothesis).
3. **Match score**: subsample the current scan; fraction of samples with a neighbour in the past scan within **`loop_point_match_m`**. If the best candidate’s score ≥ **`loop_overlap_ratio`**, a loop is declared (subject to **`loop_cooldown_sec`** between publications).

**Outputs (detection; map correction is separate — see Pose graph):**

| Topic | Type | Meaning |
|-------|------|--------|
| `/keyframe_map/loop_closure_match_index` | `std_msgs/UInt32` | Index of the matched **past** keyframe in the keyframe list |
| `/keyframe_map/loop_closure_overlap` | `std_msgs/Float32` | Overlap score in \([0,1]\) |
| `/keyframe_map/loop_closure_anchor_pose` | `geometry_msgs/PoseStamped` | Matched keyframe pose in **`map`** (planar yaw) |
| `/keyframe_map/loop_closure_pair` | `std_msgs/Int32MultiArray` | **`[past_idx, new_idx]`** for **`pose_graph_node`** |

Bringup toggle:

```bash
ros2 launch ros_project_bringup launch_slam.launch.py keyframe_loop_closure_enable:=true
```

Tune thresholds in **`config/keyframe_map.yaml`**; tight **`loop_overlap_ratio`** reduces false positives, loose **`loop_proximity_xy_m`** increases candidates (more CPU).

## Parameters

See `config/keyframe_map.yaml` for keyframe spacing, voxel sizes, **`max_map_points`**, and all **`loop_*`** parameters.

## Launch

```bash
ros2 launch keyframe_scan_map keyframe_map.launch.py
```

Requires TF **`map` → … → cloud frame** (same chain as `launch_slam.launch.py`).

**QoS:** The node subscribes with **Reliable** depth 50 so it matches the Livox driver’s default `PointCloud2` publisher. A **Best Effort** subscription would not receive data from that publisher.

## What actually improves map accuracy (no change from before)

1. Fewer NDT skips, better odometry (FAST-LIO, etc.).
2. Real **`livox_extrinsic_*`** / TF calibration.
3. Denser keyframes for local overlap only.

Loop detection is for **logging, metrics, or future nodes** that consume `/keyframe_map/loop_closure_*`; it does not fix bent maps by itself.

## Pose graph (lightweight SE2, optional)

**`pose_graph_node`** (same package, separate executable) builds a **planar pose graph**. **Optimization** uses **`scipy.optimize.least_squares`** on stacked SE(2) residuals (first pose fixed); no g2o / custom bundle adjustment engine.

| Piece | Role |
|--------|------|
| **Nodes** | Each pose on `/keyframe_map/keyframes` (`map` frame). |
| **Odometry edges** | Between consecutive keyframes: relative SE2 measurement taken from the **initial** path (odometry chain). |
| **Loop edges** | Each `/keyframe_map/loop_closure_pair` message (`std_msgs/Int32MultiArray`, data `[past_idx, new_idx]`) adds a constraint that those poses should coincide (identity relative), weighted higher than odometry. |

**Output:** `nav_msgs/Path` **`/pose_graph/corrected_keyframes`** in **`map`**. First keyframe stays fixed (gauge). Install **`python3-scipy`** (`rosdep`); if SciPy is missing, the node **echoes** the raw keyframe path.

**Limits:** `max_graph_nodes` (default 180) — if the keyframe path is longer, the node **skips** optimization and republishes the raw path (throttled warning).

**After optimization (pick one correction path):**

| Mode | Parameters / launch | Effect |
|------|----------------------|--------|
| **1+2 — keyframe poses + map** | `keyframe_apply_pose_graph_map:=true` (`apply_pose_graph_corrections` in YAML) | Updates stored keyframe batches with per-frame SE2 deltas from corrected `Path`, republishes `/keyframe_map` and keyframes. |
| **3 — ROS TF** | `pose_graph_publish_map_odom_tf:=true` | Publishes **`map`→`odom`** (static publisher omitted in `launch_slam`). Uses `T_map_odom = T(P_corr_last) inv(T(P_raw_last))` after each solve. |

Do **not** enable **1+2** and **3** together unless you know you want stacked corrections.

**Bringup:**

```bash
ros2 launch ros_project_bringup launch_slam.launch.py \
  keyframe_loop_closure_enable:=true start_pose_graph:=true
```

Rebuild map from corrected path:

```bash
ros2 launch ros_project_bringup launch_slam.launch.py \
  keyframe_loop_closure_enable:=true start_pose_graph:=true \
  keyframe_apply_pose_graph_map:=true
```

TF-only drift fix at `map`–`odom`:

```bash
ros2 launch ros_project_bringup launch_slam.launch.py \
  keyframe_loop_closure_enable:=true start_pose_graph:=true \
  pose_graph_publish_map_odom_tf:=true
```

Standalone:

```bash
ros2 run keyframe_scan_map pose_graph_node --ros-args --params-file \
  $(ros2 pkg prefix keyframe_scan_map)/share/keyframe_scan_map/config/pose_graph.yaml
```

Tune **`config/pose_graph.yaml`**: **`weight_odom`**, **`weight_loop`**, **`max_graph_nodes`**, **`max_loop_edges`**, **`publish_map_odom_tf`**, **`map_odom_tf_period_sec`**, **`odom_frame`**.
