# IMU mount note (GX5-25 on 4-wheel robot)

See also: **`docs/livox_mount_note.md`** for Livox MID-360 position/orientation on the same `base_link`.

Values below match the physical description you provided: IMU centered horizontally on the wheel-center **`base_link`** origin, **6.5 cm** above that origin (mid-wheel height reference), and GX5 **sensor frame** rotated **90° clockwise** about vertical vs a nominal mount where **IMU +X = robot forward** (**REP-103 `base_link`**: **+X** forward, **+Y** left, **+Z** up).

**GX5 sensor frame (device manual):** +X along long edge **toward connector**, +Y 90° to the **right** of +X, +Z out the **bottom** of the unit.

---

## Parameters to set

### `ros_project_bringup/config/microstrain_params_overlay.yaml`

When **`launch_slam`** starts the Microstrain node locally, it merges this file (flat keys) over the vendor **`params.yml`**, then applies **`microstrain_*`** from **`slam_bringup.yaml`**. Keep **`mount_to_frame_id_transform`** aligned with **`imu_mount_*`** (same geometry as below). For a **driver-only** robot project, copy these keys into your own `params.yml` if you do not use this launch.

### `ros_project_bringup/config/slam_bringup.yaml` (under `slam_bringup:`)

| Key | Value | Unit |
|-----|-------|------|
| `imu_mount_x` | `0.0` | m |
| `imu_mount_y` | `0.0` | m |
| `imu_mount_z` | `0.065` | m |
| `imu_mount_roll_deg` | `0.0` | deg |
| `imu_mount_pitch_deg` | `0.0` | deg |
| `imu_mount_yaw_deg` | `-90.0` | deg |

Keep these aligned with the driver (defaults are already correct unless you rename frames):

| Key | Value | Meaning |
|-----|-------|--------|
| `microstrain_frame_id` | `imu_link` | `sensor_msgs/Imu` header `frame_id` |
| `imu_mount_child_frame` | `imu_link` | Child of static TF |
| `microstrain_mount_frame_id` | `base_link` | Logical mount parent |
| `imu_mount_parent_frame` | `base_link` | Parent of static TF |

`launch_slam.launch.py` sets `publish_mount_to_frame_id_transform: false` on the Microstrain node so **only** this static TF defines the mount (no duplicate TF from the driver).

---

## Where it is used

1. **`static_transform_publisher` (`base_link` → `imu_link`)**  
   Built from `imu_mount_*` in `launch_slam.launch.py`. Defines where the **GX5 sensor axes** sit relative to the robot body.

2. **`microstrain_inertial_driver`**  
   Uses `frame_id` / `mount_frame_id` for message semantics; fusion nodes use TF **`imu_link` → `base_link`** (via the inverse chain) when needed.

3. **`localisation_ekf` (`config/ekf_python_gx5_microstrain.yaml`)**  
   With `transform_imu_to_base_link: true`, IMU quantities are rotated into **`base_link`** using TF. Wrong `imu_mount_*` → wrong lateral/longitudinal fusion, bad gravity direction in base, poor coupling with lidar odometry.

4. **Anything consuming `/tf` + IMU** (e.g. RViz, some deskew / keyframe options when Microstrain is selected in `slam_bringup.yaml`).

---

## Robot / TX2i packages not in this repo

If **`ros_robot_bringup`** on the TX2i uses its **own** YAML for static TFs or Microstrain params, copy the **same numeric values** there so **robot bringup** and **laptop `launch_slam`** stay consistent when debugging.

---

## Verify

- RViz: show **`base_link`** and **`imu_link`**; check colored axes vs chassis (**+X** IMU toward connector along long edge, **+Z** out bottom of unit).  
- If vertical axis is wrong, re-check handedness vs manual (sign of yaw may flip to **`+90`**).

---

## Sign convention reminder

With **+Z up**, **90° clockwise** when looking down from above is **`yaw = -90°`** (degrees).
