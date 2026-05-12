# Foxy FastDDS publisher-stall on `/livox/lidar`

## Symptom

- `ros2 topic echo /livox/lidar --qos-reliability best_effort` streams steadily
  with the Livox driver alone.
- After launching `ros2 launch ros_project_bringup launch_slam.launch.py`,
  `/livox/lidar` slows then **completely stops** within a few seconds.
- The cloud resumes ~5 s after the SLAM launch exits.
- `/livox/imu` is unaffected (~200 Hz throughout).

## Cause

FastDDS on Foxy uses shared memory (SHM) for local IPC by default. With a
high-payload publisher (Livox MID-360 cloud, ~600 KB at 10 Hz) and three SHM
matched subscribers from this stack — `fastlio_mapping`, `keyframe_map_node`,
`ekf_node` (cloud-stamp sync) — the publisher's SHM segment fills faster than
the subscribers can drain it while the SLAM stack is doing its first-second-of-
life expensive work (ikd-tree init, keyframe prefilter, EKF first-stamp lookup).
The publisher then **blocks waiting for SHM space**, the topic effectively
stalls, and the kernel reclaims after the FastDDS unmatched-subscriber GC fires
~5 s after the subscribers tear down.

`/livox/imu` is small enough that the segment never fills, so it stays alive.

## Fix

Force FastDDS to use UDP loopback only (no SHM). Kernel UDP buffers per
receiver and never back-pressures the publisher.

This repo ships a ready-to-use FastDDS profile at
`share/ros_project_bringup/config/fastdds_no_shm.xml` and a launch arg
`disable_dds_shm` that wires it in for every node the launch starts.

### Step 1 — in the shell that starts the Livox driver

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/fastdds_no_shm.xml
ros2 launch livox_ros_driver2 msg_MID360_launch.py   # or whichever launch you use
```

(The launch arg in step 2 only affects nodes started by `launch_slam`; the
Livox driver is launched separately, so it needs the env applied to **its**
shell too. Once you've set both ends to UDP-only the transport matches and the
stall goes away.)

### Step 2 — start SLAM with the workaround enabled

```bash
ros2 launch ros_project_bringup launch_slam.launch.py disable_dds_shm:=true
```

Or, equivalently, export the same env in the SLAM shell and omit the launch
arg:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/fastdds_no_shm.xml
ros2 launch ros_project_bringup launch_slam.launch.py
```

### Step 3 — verify

```bash
# Should now stay at 10 Hz steady while SLAM is running (in a fresh terminal,
# with the same export above):
ros2 topic echo /livox/lidar --qos-reliability best_effort --no-arr --truncate-length 80
```

## Making it permanent

If the fix works for you, add the export to your `~/.bashrc` (or `~/.zshrc`)
on `server00` so every shell picks it up and there is no risk of starting the
Livox driver in a shell without it:

```bash
# In ~/.bashrc, after the ROS source lines:
if command -v ros2 >/dev/null 2>&1; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix --share ros_project_bringup)/config/fastdds_no_shm.xml
fi
```

Once that is in place you can drop `disable_dds_shm:=true` from the launch
invocation — the env will already be set by the shell.

## Alternative: switch RMW to Cyclone DDS

Cyclone DDS uses a different transport that doesn't have this stall mode on
Foxy. If you have `ros-foxy-rmw-cyclonedds-cpp` installed (or are willing to
`sudo apt install` it), you can swap the RMW for both the Livox driver shell
and the SLAM shell:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

That is the more "official" fix; the SHM XML profile is the cheaper one if you
want to stay on FastDDS.

## Why not also disable SHM unconditionally inside the launch?

Because the Livox driver runs in a separate shell that this launch doesn't
control, and forcing only one side to UDP can still work but only if both
participants agree. Making it opt-in (`disable_dds_shm:=true` or the bashrc
export) keeps the launch behaviour explicit and avoids accidentally degrading
other deployments where SHM works fine.
