# Livox MID-360 mount note (4-wheel robot, `base_link` = REP-103)

**`base_link` convention (same as rest of this repo):** **+X** = robot forward (toward front wheels), **+Y** = left, **+Z** = up. Origin = wheel-center / mid-height reference you use for the IMU note.

**Livox MID-360 point-cloud frame:** See [Livox Mid-360 User Manual](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf) § *Coordinates* — Cartesian **O–XYZ** is the sensor’s **point cloud / `livox_frame`** frame (origin **O** on the device per the dimensional figure in that section). Built-in Livox IMU offsets in that frame are documented there (e.g. chip position vs **O**); that is **inside the sensor**, separate from your robot mount below.

---

## Your measured robot mount (→ `slam_bringup.yaml`)

**Translation** of **`livox_frame`** origin w.r.t. **`base_link`** (meters):

| Axis | Your geometry | `livox_extrinsic_*` |
|------|----------------|---------------------|
| Forward (+X) | 12 cm ahead of origin | `livox_extrinsic_x: 0.12` |
| Left (+Y) | 7 mm **to the right** of origin (right = −Y) | `livox_extrinsic_y: -0.007` |
| Up (+Z) | 38 cm above origin | `livox_extrinsic_z: 0.38` |

**Rotation** you described for **Livox axes vs `base_link`**:

- Livox **+X** → robot **backward** → **−`base_link` X**
- Livox **+Y** → robot **right** → **−`base_link` Y**
- Livox **+Z** → **up** → **`+base_link` Z**

That is a **180° yaw** about vertical (right‑handed, **+Z** up): **`roll = 0`**, **`pitch = 0`**, **`yaw = 180°`** (same convention as `static_transform_publisher` in `launch_slam.launch.py`, degrees in YAML → radians in code).

```yaml
livox_extrinsic_roll_deg: 0.0
livox_extrinsic_pitch_deg: 0.0
livox_extrinsic_yaw_deg: 180.0
```

**Sanity:** In RViz, compare **`base_link`** and **`livox_frame`** axes to the physical unit.

---

## Where to set it

| File | Keys |
|------|------|
| `src/ros_project_bringup/config/slam_bringup.yaml` | `livox_extrinsic_x` … `livox_extrinsic_yaw_deg`, `livox_cloud_frame_id: livox_frame` |

`launch_slam.launch.py` feeds these into **`static_transform_publisher`** **`base_link` → `livox_frame`**.

**`MID360_config.json` → `extrinsic_parameter`:** Livox docs describe JSON extrinsics for device storage / tooling; for this ROS stack, **robot pose is normally handled by the static TF above**. Leave JSON at **0** unless you intentionally use Livox tooling that requires non‑zero device extrinsic (avoid double‑applying the same mount).

---

## What it affects

1. **All consumers of `PointCloud2` in `livox_frame`** — NDT / LIO / RViz assume cloud is in **`livox_frame`**; TF chains it to **`base_link`** / **`odom`**.
2. **`lidar_odometry_node`** (`lidar_base_frame: base_link`, TF initial guess) — wrong mount ⇒ bad registration / slip.
3. **FAST-LIO** (`extrinsic_T` / `extrinsic_R` in `lio_bringup/config/fastlio_mid360_overlay.yaml`) — **IMU–LiDAR within the fusion filter**, in addition to ROS TF; tune after ROS TF is sane (defaults include Livox internal IMU offset in **sensor** frame, not your robot lever arm).
4. **EKF / keyframe** when topics are deskewed or aligned using TF.

---

## TX2i / other bringups

If **`ros_robot_bringup`** uses its own static TF or Livox args, **mirror these numbers** so robot and laptop stacks match.

---

## Sign recap

- **Right of origin** along horizontal = **negative** **`livox_extrinsic_y`** under REP‑103 (**+Y** is left).
