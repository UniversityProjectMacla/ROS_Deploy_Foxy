# Bag replay quick-start (Foxy)

Replace `~/bags/session_01` with whatever bag you want to play.

## Terminal 1 — SLAM stack

```bash
cd ~/ROS_Deploy_Foxy
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py
```

Switch to NDT instead of FAST-LIO:

```bash
ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=false use_lidar_fusion:=true
```

Switch off RViz:

```bash
ros2 launch ros_project_bringup launch_slam.launch.py start_rviz:=false
```
*(start_rviz is read from `slam_bringup.yaml`; for a one-off override use
`bringup_config:=/path/to/overlay.yaml` with `start_rviz: false`.)*

## Terminal 2 — play the bag

```bash
source /opt/ros/foxy/setup.bash
ros2 bag info ~/bags/session_01
ros2 bag play ~/bags/session_01 --clock
```

## Terminal 3 — verify

```bash
source /opt/ros/foxy/setup.bash
source ~/ROS_Deploy_Foxy/install/setup.bash
ros2 topic hz /livox/lidar
ros2 topic hz /ekf/odom
ros2 topic echo /lidar/odom --once
```

## Common arguments to `launch_slam.launch.py`

| Argument | Default | Effect |
|----------|---------|--------|
| `use_lio` | (yaml) | `true` = FAST-LIO + relay, `false` = NDT |
| `use_lidar_fusion` | (yaml) | `true` = NDT runs, `false` = no NDT |
| `use_sim_time` | `true` | Use `/clock` (bag replay); set `false` for wall clock |
| `bringup_config` | `''` | Path to a YAML overlay merged onto `slam_bringup.yaml` |
| `rviz_keyframe_map_size_pixels` | `''` | Override RViz "Keyframe map" Size (Pixels) |
| `rviz_keyframe_map_size_m` | `''` | Override RViz "Keyframe map" Size (m) |
