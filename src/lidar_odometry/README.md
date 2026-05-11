# lidar_odometry

PCL **Normal Distributions Transform (NDT)** odometry for ROS 2: **scan-to-map** (node + bringup default) or **scan-to-scan**.

## Node: `lidar_odometry_node`

- **Subscribes:** `sensor_msgs/PointCloud2` (default `/livox/lidar`)
- **Publishes:**
  - `nav_msgs/Odometry` (default `/lidar/odom`) — integrated pose in `odom_frame` → `base_frame` (for EKF / logging)
  - `geometry_msgs/TwistStamped` (default `/lidar/relative_motion`) — incremental motion **Δx**, **Δy**, **Δθ** per successful step:
    - `twist.linear.x` = Δx (m)
    - `twist.linear.y` = Δy (m)
    - `twist.angular.z` = Δθ (rad)
  - **`scan_to_map` only:** `geometry_msgs/PoseStamped` (default `/lidar/pose_correction`) — planar NDT **correction vs prediction** this step (`header.frame_id` = `odom_frame`).

`TwistStamped` is used as a compact carrier for the 2D delta (linear.z and angular x/y are zero).

## Modes (`registration_mode`)

### `scan_to_map` (default)

1. Same prefilter as **scan_to_scan** (NaN removal, crop, voxel).
2. **Map in `odom`:** transform each scan from the **sensor `frame_id`** to odom using a **planar motion prior** × **`sensor_extrinsic_rpy_xyz`** (base ← sensor). The prior is normally **`odom_frame` → `base_frame`** from **TF at the cloud stamp** (e.g. **`ekf_node`**), so IMU-coasted pose is used between LiDAR steps. If TF is missing, the node falls back to its last integrated NDT pose.
3. **NDT** aligns the predicted scan in odom to the **voxel map**; **`ndt.align` starts at identity** because the prior is already baked into the cloud transform. Apply **T_ndt** so corrected body pose is **T_ndt × prediction**.
4. Publish **`/lidar/pose_correction`** (planar NDT delta) plus **`/lidar/odom`** and **`/lidar/relative_motion`** (step delta).
5. Merge the **aligned** scan into the map (voxel merge). **`map_max_points`** triggers coarser voxels if the map grows too large.

### `scan_to_scan`

1. Remove NaN, crop to `crop_range_m`, **ApproximateVoxelGrid** downsample (`voxel_leaf_size`).
2. **Valid `header.stamp`** on the point cloud is required (zero stamp → skip) so TF/EKF stay time-consistent.
3. First cloud seeds the NDT **target**; publishes **identity** `/lidar/odom` and **Δ = 0** on `/lidar/relative_motion`.
4. From the second message on, **NDT** returns **T_ndt** (source → target). Planar motion uses **T_inc = T_ndt⁻¹** with **only**  
   `dx = T_inc(0,3)`, `dy = T_inc(1,3)`, `dθ = atan2(T_inc(1,0), T_inc(0,0))`; z / roll / pitch are **not** used for integration.
5. Global pose is re-projected to **planar** (z = 0, yaw-only quaternion) after each step.
6. Reject updates if NDT fails to converge or **fitness score** exceeds `max_fitness_score`.

**Extrinsics (both modes):** `sensor_extrinsic_rpy_xyz` = `[roll, pitch, yaw, x, y, z]` (rad / m) for **base_link ← cloud frame** (e.g. `livox_frame`). In **`launch_slam.launch.py`**, the same **`livox_extrinsic_*`** values used for the static TF are passed into this parameter when NDT runs.

**CPU:** large maps slow NDT; lower **`map_max_points`**, raise **`map_merge_voxel_leaf_size`**, or use **`scan_to_scan`** for lightweight runs.

## Parameters

| Parameter | Default | Notes |
|-----------|---------|--------|
| `registration_mode` | `scan_to_map` | `scan_to_scan` = consecutive scans only |
| `use_tf_initial_guess` | `true` | **scan_to_map:** TF `odom_frame`→`base_frame` at cloud time (`ekf_node`); `false` = use last NDT pose only |
| `tf_initial_guess_timeout_sec` | `0.1` | TF lookup timeout |
| `cloud_topic` | `/livox/lidar` | |
| `odom_topic` | `/lidar/odom` | Matches `ekf_node` `lidar_odom_topic` in bringup |
| `delta_topic` | `/lidar/relative_motion` | Δx, Δy, Δθ |
| `pose_correction_topic` | `/lidar/pose_correction` | **scan_to_map** only |
| `odom_frame` | `odom` | Parent frame of published odometry |
| `base_frame` | `base_link` | Child frame |
| `sensor_extrinsic_rpy_xyz` | six zeros | `[roll,pitch,yaw,x,y,z]` rad/m: **base ← sensor** |
| `map_merge_voxel_leaf_size` | −1 | **scan_to_map:** merge voxel (m); −1 ⇒ `voxel_leaf_size` |
| `map_max_points` | 400000 | **scan_to_map:** soft cap |
| `scan_to_map_map_refresh_period` | 0 | **scan_to_map:** every N **successful** alignments, discard accumulated map and rebuild the NDT target from the last **`scan_to_map_refresh_keep_scans`** aligned scans (voxel merged). **`global_pose_` unchanged**. Reduces map poisoning; **↑N** ⇒ less frequent refreshes (less jitter); **0** disables |
| `scan_to_map_refresh_keep_scans` | 3 | How many recent aligned scans to merge at each refresh (**1–8**). **>1** gives a wider target than a single scan and usually **reduces jitter** after refresh |
| `voxel_leaf_size` | 0.22 | m; smaller = denser, slower |
| `crop_range_m` | 40.0 | axis-aligned crop before NDT |
| `ndt_resolution` | 0.85 | NDT voxel resolution (m); tune with environment scale |
| `ndt_step_size` | 0.1 | Newton line search step |
| `ndt_transformation_epsilon` | 0.01 | convergence threshold |
| `ndt_max_iterations` | 50 | |
| `max_fitness_score` | 12.0 | reject bad matches (too low ⇒ skipped scans ⇒ IMU-only drift; see below) |
| `min_points_per_cloud` | 200 | |
| `publish_tf` | false | set true only if no other node publishes `odom`→`base_link` |

## Launch

```bash
ros2 launch lidar_odometry lidar_odometry.launch.py
```

Scan-to-map preset:

```bash
ros2 launch lidar_odometry lidar_odometry_scan_to_map.launch.py
```

## Dependencies

System PCL (e.g. `libpcl-dev`), ROS 2 `pcl_conversions`.

## EKF drift and `max_fitness_score`

When fitness exceeds **`max_fitness_score`**, this node **does not** publish **`/lidar/odom`** for that scan (**scan_to_scan** also replaces the internal target cloud on skip; **scan_to_map** leaves the map unchanged). Your **`ekf_node`** then has **no LiDAR measurement** on those steps and **integrates IMU only**, so **x / y drift** while **yaw** can still look reasonable from gyro. If the console shows repeated **`NDT fitness … > max … — skip`**, raise **`max_fitness_score`** in launch until skips are rare, then lower slightly if the corrected pose becomes jumpy. PCL’s fitness scale depends on environment, voxel size, and resolution.

## Limitations

- Planar-style output is derived from the full 3D transform (yaw from horizontal projection); large roll/pitch violate the model.
- **scan_to_scan:** consecutive-scan NDT drifts in long corridors; fuse with IMU / a global pose source (e.g. your EKF + FAST-LIO) for robustness.
- **scan_to_map:** map is **unbounded drift** in `odom` (same as odometry); not global SLAM. Wrong **`sensor_extrinsic_rpy_xyz`** breaks alignment.
