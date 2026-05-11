# SLAM tuning — knobs only (`docs/tuning.md`)

Each **KNOBS** block in the YAMLs matches this document. **Suggested** values are starting points for your stack (jolty robot, NDT + EKF, Livox deskew + optional Microstrain); adjust with logs and bag replay at **rate 1.0**.

**Bag replay vs `use_sim_time`:** run `ros2 bag play --help` on your distro. **`--clock`** (when present) controls publishing simulation **`/clock`**, not “×100 real-time motion.” Playback speed is a **separate** flag (often **`--rate`** / **`--playback-rate`**). Saturated CPU at “high `--clock`” is usually from **wall-time load**, not faster simulated robot motion.

Legend for scalars: **↑** = increase parameter; **↓** = decrease.

---

## §1 — Architecture / IMU split (`slam_bringup.yaml` KNOBS)

| Parameter | Suggested value | What it does | If you change it |
|-----------|-----------------|--------------|-------------------|
| `use_microstrain_imu` | **`false`** without serial GX5; **`true`** on robot with GX5 | Selects Microstrain as EKF IMU source (via `microstrain_imu_topic`) and starts the driver when `launch_sensors:=true`. | **`true`:** EKF uses GX5; you **must** have the topic + TF mount correct. **`false`:** EKF uses `ekf_imu_topic` (REST), usually Livox. |
| `microstrain_imu_topic` | **`/imu/data_raw`** (recommended for `ekf_node`); **`/imu/data`** only if you intentionally use the driver-filtered stream | Topic name `ekf_node` subscribes to when `use_microstrain_imu` is true. | Wrong string → **no IMU**, prediction dies. Must match **exact** topic on robot/bag. |
| `microstrain_imu_origin` | **`robot`** when IMU is on DDS (default with `launch_sensors:=false`); **`local`** when this host runs the GX5 serial driver (`launch_sensors:=true`) | **`local`:** starts `microstrain_inertial_driver` when `launch_sensors` is true and publishes mount TF here. **`robot`:** no local driver — EKF subscribes to `microstrain_imu_topic`; mount TF must come over **DDS** from the robot (or bag). | Wrong combo → duplicate `/imu` publishers or missing IMU / missing `imu_link` TF. |
| `keyframe_deskew_imu_follow_ekf` | **`false`** for “GX5 EKF + Livox deskew” | If **`true`** with Microstrain on, launch **replaces** deskew IMU with the EKF IMU topic (usually GX5), which is **not** co-located with the Livox scan clock. | **`false`:** deskew stays on **`keyframe_deskew_imu_topic`** (REST, default `/livox/imu`). **`true`:** deskew follows EKF IMU → usually **wrong** for Livox cloud deskew. |

---

## §2 — EKF trust in `/lidar/odom` (`slam_bringup.yaml` KNOBS)

| Parameter | Suggested value | What it does | **↑** (increase) | **↓** (decrease) |
|-----------|-----------------|--------------|------------------|------------------|
| `ekf_lidar_pose_var` | **`0.022`** (try **`0.035–0.05`** if NDT slips) | Variance **R** on LiDAR **x, y** (and z if fused) — **larger** ⇒ trust LiDAR pose **less**. | Softer tracking of NDT position; **less** pull from bad scans. | **Tighter** track on NDT; **snappier** but follows NDT errors more. |
| `ekf_lidar_yaw_var` | **`0.012`** (try **`0.018–0.025`** if heading fights NDT) | Variance **R** on LiDAR **yaw**. | Same as above for **heading**. | Same: **trust LiDAR heading more**. |
| `ekf_lidar_gate_nis` | **`96.0`** (try **`48–64`** to reject outliers) | Innovation gate on LiDAR updates (NIS). **Higher** ⇒ allow larger disagreements before rejecting. | **More** outlier measurements still update the filter (risky if NDT is bad). | **Stricter** gate; **fewer** updates from suspicious LiDAR steps (can look “sticky” if too low). |

---

## §4 — EKF stamp alignment (`slam_bringup.yaml` KNOBS)

| Parameter | Suggested value | What it does | **↑** (more positive) | **↓** (more negative) |
|-----------|-----------------|--------------|------------------------|-------------------------|
| `ekf_imu_stamp_offset_sec` | **`0.0`** (tune **±0.002–0.020** s only with evidence) | Adds seconds to **every IMU** message stamp before prediction / TF sync. | Shifts IMU **later** vs LiDAR/odom timeline. | Shifts IMU **earlier**. |
| `ekf_lidar_stamp_offset_sec` | **`0.0`** (same tuning band) | Adds seconds to **LiDAR odom** (and related fusion) stamps. | Shifts LiDAR odom **later**. | Shifts LiDAR odom **earlier**. |

**What changing does:** fixes **constant** clock skew; wrong sign → wrong **dt** and misaligned fusion; tune only after NDT/deskew are sane.

---

## §5 — NDT robustness (`slam_bringup.yaml` KNOBS)

| Parameter | Suggested value | What it does | **↑** | **↓** |
|-----------|-----------------|--------------|-------|-------|
| `lidar_voxel_leaf_size` | **`0.22`** (try **`0.26–0.30`** if NDT is noisy) | Voxel downsample size (m) before NDT — **larger** ⇒ fewer points. | **Coarser** cloud, often **stabler** registration, less detail. | **Finer** detail, **heavier** CPU, can **overfit** noise on a jolty base. |
| `lidar_max_fitness_score` | **`12.0`** (try **`8–10`** to reject bad aligns) | Max NDT fitness to accept a match. | **Accept worse** fits (more updates, risk wrong pose). | **Reject** marginal alignments (fewer bad `/lidar/odom` steps; may skip more in hard scenes). |
| `lidar_ndt_max_iterations` | **`35`** (raise only if logs show non-convergence) | Max solver iterations per scan. | **Better chance** to converge on hard frames; **more CPU**. | **Faster** per cloud; may **fail** convergence more often. |
| `lidar_ndt_step_size` | **`0.1`** (try **`0.07`** if optimizer diverges) | Optimizer step size. | **Larger steps** — can be faster or **unstable**. | **Smaller steps** — usually **stabler**, slower. |
| `lidar_min_points_per_cloud` | **`200`** (try **`300`** if sparse scans hurt) | Minimum points after filter to run NDT. | **Stricter** — skip more marginal clouds. | **Looser** — accept sparser scans (can **degrade** pose). |
| `lidar_use_tf_initial_guess` | **`true`** for normal coupled EKF–NDT; **`false`** **briefly** to debug | Use **`odom`→`base_link` TF at cloud time** as motion prior for scan-to-map. | **`true`:** NDT follows EKF prediction between scans (good when both healthy). **`false`:** decouple debug — if skew **improves**, suspect **feedback** or stamp mismatch. | — |

---

## §6 — Keyframe deskew (`slam_bringup.yaml` KNOBS → `deskew_*` on node)

| Parameter | Suggested value | What it does | **↑** / change | **↓** / opposite |
|-----------|-----------------|--------------|----------------|-------------------|
| `keyframe_deskew_imu_stamp_offset_sec` | **`0.0`** (tune **± ms** if smear is systematic) | Constant time shift for **IMU** vs cloud when deskewing. | Corrects one direction of **shear**; wrong value **adds** smear. | Same, opposite direction. |
| `keyframe_deskew_cloud_stamp_offset_sec` | **`0.0`** (same) | Shift on **cloud** side for deskew window. | Same role on cloud timeline. | Same. |
| `keyframe_deskew_max_imu_age_sec` | **`0.28`** (try **`0.35`** if “IMU too old” / CPU load) | Max allowed age of IMU samples vs cloud stamp for deskew. | **More tolerant** of latency / buffering (deskew still runs). | **Stricter** sync; may **disable** deskew more often if IMU lags. |
| `keyframe_deskew_imu_sign` | **`1.0`** (only flip to **`-1.0`** if tests show mirrored shear) | Multiplies angular velocity used in deskew. | **`-1.0`** inverts rotation sense → can **fix** inverted smear or **destroy** map if wrong. | — |
| `keyframe_deskew_imu_rotate_gyro_to_frame` | **`livox_frame`** (same string as **`livox_cloud_frame_id`**) | TF2-rotates **`angular_velocity`** from **`Imu.header.frame_id`** into the **cloud frame** before Rodrigues deskew. | Empty string → **raw ω** (wrong basis if frame_id ≠ cloud frame) → **shear**. | Must match cloud **`frame_id`**. With Livox off this host, launch publishes **`livox_cloud_frame_id`→`livox_imu_child_frame`** (default **`livox_imu`**, not **`sensor`**) so Microstrain’s **`sensor`** frame_id is not hijacked. |
| `keyframe_deskew_enable` | **`true`**; set **`false`** for **one diagnostic bag** | Enables motion compensation per sweep. | **`false`:** see if skew is deskew-dominated (skew may shrink; map quality trades off). **`true`:** normal operation. | — |

---

## §7 — Keyframe density / map weight (`slam_bringup.yaml` KNOBS)

| Parameter | Suggested value | What it does | **↑** | **↓** |
|-----------|-----------------|--------------|-------|-------|
| `keyframe_min_dist_m` | **`0.22`** (try **`0.28–0.35`** if map is dense / drift-prone) | Minimum travel (m) before a new keyframe. | **Fewer** keyframes; less compounded error; **sparser** map. | **More** keyframes; denser map; **more** CPU. |
| `keyframe_min_yaw_deg` | **`5.0`** (try **`8–12`** if rotation smears) | Minimum yaw change (deg) for a keyframe. | Fewer rotation keyframes. | More rotation keyframes. |
| `keyframe_min_time_sec` | **`0.85`** (try **`1.0–1.2`** under CPU stress) | Minimum time between keyframes. | Lower keyframe rate. | Higher keyframe rate. |
| `keyframe_voxel_leaf_m` | **`0.2`** (try **`0.25–0.3`** if RViz lags) | Voxel size (m) for merged map cloud. | **Lighter** map, less detail. | **Heavier** / finer map. |
| `keyframe_max_map_points` | **`800000`** (try **`400000–600000`** if RAM/RViz struggles) | Cap on merged map points. | Lower cap → **less** memory; may crop old data harder depending on implementation. | Higher cap → **more** retained map. |
| `keyframe_max_pts_per_scan` | **`50000`** (try **`30000`** if CPU spikes) | Max points stored per keyframe scan. | Lower → **less** work per keyframe. | Higher → denser local scans. |

---

## §8 — Pose graph (`slam_bringup.yaml` KNOBS → `pose_graph.yaml` overrides)

| Parameter | Suggested value | What it does | **↑** | **↓** |
|-----------|-----------------|--------------|-------|-------|
| `pose_graph_weight_odom` | **`12.0`** (↓ if odom is clearly bad) | Relative weight of **odometry** edges in the graph. | **Trust integrated odometry more** in the graph. | **Trust odom less** — loops and structure dominate more. |
| `pose_graph_weight_loop` | **`55.0`** (↑ if loops are weak) | Relative weight of **loop-closure** edges. | **Stronger** loop corrections. | **Softer** loop pull (less bend on revisit). |
| `pose_graph_max_nodes` | **`180`** (↓ if solve is slow) | Max nodes kept in the graph. | Larger graph → **slower** solve, **more** history. | Smaller graph → **faster**, less history. |
| `pose_graph_max_loop_edges` | **`40`** (↓ with `max_nodes`) | Max loop edges. | More loop constraints possible. | Fewer; faster / simpler graph. |

---

## §9 — Stack mode (`slam_bringup.yaml` KNOBS)

| Parameter | Suggested value | What it does | If you change it |
|-----------|-----------------|--------------|-------------------|
| `use_lio` | **`false`** for NDT path; **`true`** to use FAST-LIO as `/lidar/odom` | Starts FAST-LIO + relay instead of `lidar_odometry_node` when true (see launch rules with `use_lidar_fusion`). | **`true`:** LIO odometry — often better on **rough 3D / vibration**; tune **`fastlio_mid360_overlay.yaml`**. **`false`:** NDT path; tune **`lidar_*`** KNOBS. |
| `use_lidar_fusion` | **`true`** with `use_lio:false`; **`false`** with `use_lio:true` | Starts NDT when true and not overridden by LIO-only bringup. | **`false` + `use_lio:true`:** **no NDT** (recommended LIO-only). **`true` + `use_lio:true`:** launch still prefers LIO path per `launch_slam` — follow README for your combo. |

---

## §3 — EKF process noise Q (`ekf_python_gx5_microstrain.yaml` KNOBS — `process_noise_diag`)

Order: **px, py, z, yaw, vx, vy, bax, bay, bgz**. Suggested **starting** vector (repo default):

`[2.5e-4, 2.5e-4, 1.0e-6, 5.0e-7, 3.5e-3, 3.5e-3, 1.0e-8, 1.0e-8, 5.0e-11]`

| Index | Name | What it does | **↑** | **↓** |
|-------|------|--------------|-------|-------|
| 0–1 | px, py | Process noise on horizontal position in **prediction** between LiDAR updates. | **More** random walk allowed — softer IMU-only coast, less stiff. | **Tighter** predicted path between scans. |
| 2 | z | Vertical process noise (usually tiny in planar fusion). | More z freedom in model (only matters if z is active in your fusion). | Tighter z in model. |
| 3 | yaw | Yaw process noise between scans. | Heading can drift more in prediction; less brittle. | Heading follows gyro integration more tightly between scans. |
| 4–5 | vx, vy | Velocity process noise. | Velocity state can change faster (**helps** absorb suspension if LiDAR R is tuned). | Smoother velocities; may lag real motion. |
| 6–8 | bax, bay, bgz | Accel / yaw-rate **bias** random walk. | Biases can move faster (only if logs show bias tracking issues). | **Do not** shrink to extremes without reason — biases can get stuck. |

---

## §10 — FAST-LIO overlay (`lio_bringup/config/fastlio_mid360_overlay.yaml` — use when `use_lio: true`)

| Parameter | Suggested value | What it does | **↑** | **↓** |
|-----------|-----------------|--------------|-------|-------|
| `preprocess.blind` | **`0.55`** (try **`0.6–0.7`** if near-body returns corrupt) | Drops LiDAR points closer than this (m). | Ignore more **near** clutter / body / multipath. | Use **more** near returns (can add noise). |
| `mapping.acc_cov` | **`0.15`** (try **`0.2–0.35`** if LIO is jittery) | IMU accel **process** noise in LIO. | **Trust accel measurements less** in propagation — often **smoother** LIO on bad IMU/vibration. | **Trust accel more** — can be tighter or **more jittery**. |
| `mapping.gyr_cov` | **`0.15`** (same band as accel for gyro) | IMU gyro process noise in LIO. | Same trade-off for **rotation** propagation. | Same. |
| `mapping.det_range` | **`55.0`** m (↓ in cluttered / indoor noise; ↑ in very open) | Max range (m) of points used in LIO map. | Use **farther** returns (more open world); more noise CPU. | **Shorter** window; can **stabilise** if far returns are bad. |
| `mapping.extrinsic_est_en` | **`true`** while calibrating; **`false`** when stable | Online estimate of **internal** Livox IMU–LiDAR extrinsic in FAST-LIO. | **`true`:** can **recover** bad initial extrinsic; can **wander** if ill-conditioned. **`false`:** **locks** extrinsic — needs good **`extrinsic_T` / `R`**. | — |

---

## Where the knobs live (file → block)

| File | Block name |
|------|------------|
| `src/ros_project_bringup/config/slam_bringup.yaml` | **`KNOBS`** |
| `src/localisation_ekf/config/ekf_python_gx5_microstrain.yaml` | **`KNOBS`** |
| `src/lio_bringup/config/fastlio_mid360_overlay.yaml` | Comments under **`preprocess`** / **`mapping`** (§10) |
| `src/keyframe_scan_map/config/keyframe_map.yaml` | **`KNOBS`** (defaults; `slam_bringup` overrides the same logical names with `keyframe_*` prefix where launch maps them) |
| `src/keyframe_scan_map/config/pose_graph.yaml` | **`KNOBS`** |

See also **`docs/SLAM_CONFIGURATION.md`**.

---

## Playback / RViz — what to watch and report back

Use **`Fixed Frame`** = **`map`** (or **`odom`** if you are only checking odometry). Note which you used.

**TF / frames**

- **`base_link`**, **`imu_link`**, **`livox_frame`**: axes look physically plausible vs the robot model?
- **Jumps / pops** in **`odom` → `base_link`** or **`map` → `odom`** (if used): how often, after what motion (straight, corner, bump)?
- **Yellow errors** in RViz about TF (extrapolation, future, missing): exact message text and which frames.

**Odometry / paths**

- **`/ekf/odom`** (Path or Odometry display): does the trajectory **track** the corridor / loop closure shape, or **cut corners / overshoot / drift**?
- **`/lidar/odom`** vs **`/ekf/odom`**: same question — is LiDAR odom **noisier** than EKF (expected if EKF is working) or **wildly** diverging?
- **Velocity feel**: does the arrow / motion look **jittery** on suspension vs smooth slow segments?

**LiDAR / map**

- **`/livox/lidar`** in **`livox_frame`** or **`base_link`**: **curved walls** (deskew / timing) vs straight; **double walls**; **streaks** in the direction of motion?
- **`/keyframe_map`** (merged cloud): **shear**, **banana** corridors, **layering** at different heights when you drive flat?
- **Loop closure** (if enabled): obvious wrong **snap** when a loop is found?

**Terminal / logs** (copy a few representative lines)

- NDT: **`did not converge`**, **`fitness … > max`**, **`skip`**, **`TF … unavailable`**
- EKF: repeated **warnings** about frames, stamps, or NaNs

**Bag / timing**

- Playback **`--rate 1`**. Note **run id** (1 / 2 / 3) and **approximate sim time** when failure starts.

---

## Tuning order for runs 1 → 3 (slow → agile)

**Start with run 1** (slowest / most gradual): dynamics are easier on NDT and EKF, so you can see **systematic** issues (wrong extrinsic, deskew sign, stamp skew) without aggressive motion masking them.

Then **run 2**, then **run 3**: each step stress-tests the same parameters. If you tune **only** on run 3 first, you risk **over-rejecting** LiDAR or **over-smoothing** so that run 1–2 look sluggish or wrong — possible but harder to reason about. After run 1 is acceptable, use run 3 as a **regression check** and only then tighten or loosen a few knobs (e.g. `lidar_max_fitness_score`, `ekf_lidar_gate_nis`).

---

*Tuning discipline: judge changes with **`ros2 bag play … --rate 1`** or on the live robot.*
