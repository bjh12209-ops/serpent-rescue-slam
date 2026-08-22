# D435 + WT901C485 + EKF + RTAB-Map

This package implements the following single-parent TF pipeline:

```text
D435 RGB + aligned depth -> rgbd_odometry -> /visual_odom
WT901C485 0x50 (head) --------------------> EKF -> odom -> base_link
WT901C485 0x51/0x52 ----------------------> browser body-pose view
                                                     |
RGB-D + IMU + odom TF --------------------------> RTAB-Map -> map -> odom
```

`rgbd_odometry` publishes `/visual_odom` expressed in the `odom` frame but
deliberately does not publish TF. `ekf_filter_node` is the only publisher of
`odom -> base_link`, and RTAB-Map publishes `map -> odom`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select wt901c485_driver my_robot_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Verify sensors only

Keep the robot stationary for the first two seconds:

```bash
ros2 launch my_robot_bringup sensor_bringup.launch.py
```

In another terminal:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /imu_50/data
ros2 topic hz /imu_51/data
ros2 topic hz /imu_52/data
ros2 topic echo /imu_50/data --once
```

## Calibrate the static transforms

The temporary sensor box uses the camera as the head/base reference. The
measured mounting transform is:

- `base_link -> camera_link`: identity
- `camera_link -> imu_50_link`: `(x, y, z) = (-0.05, 0.0, +0.02) m`
- 0x51/0x52 translations are unknown and are not published as fabricated TFs
- both rotations are zero because their REP-103 axes are mounted in the same
  direction

ROS REP-103 axes are x forward, y left, and z up. Verify the transform:

```bash
ros2 launch my_robot_bringup sensor_bringup.launch.py
ros2 run tf2_ros tf2_echo camera_link imu_50_link
```

The expected translation is approximately `(-0.05, 0.0, +0.02)` m with identity
rotation. Replace this temporary convention with URDF transforms measured from
the final snake-robot head `base_link` before tuning the final EKF.

## Validate visual odometry and EKF

```bash
ros2 launch my_robot_bringup sensor_fusion.launch.py
```

Check:

```bash
ros2 topic hz /visual_odom
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo camera_link imu_50_link
```

Move slowly at first. Verify that x moves forward, y left, and z up; roll,
pitch, and yaw must follow the right-hand rule.

Fused bringup is intentionally fail-operational: RGB-D visual odometry starts
even when the RS485 IMU is unplugged, and the EKF can continue from visual
odometry alone. When `/imu_50/data` is available it is fused normally. The
health monitor still marks a missing IMU as lost; this fallback preserves
mapping but does not claim inertial data exists.

## Unknown-environment online SLAM

The field robot does not require a prebuilt map. It starts an incremental
RTAB-Map database, builds the explored area online, and records that map while
the mission is running. Start a fresh database only when
`reset_database:=true` is intentional:

```bash
ros2 launch my_robot_bringup rtabmap_mapping.launch.py \
  reset_database:=true \
  database_path:=~/.ros/mission_001.db \
  rviz:=true \
  rtabmap_viz:=false
```

RViz displays the progressively accumulated 3-D map, current robot pose,
traveled path, TF tree, and RGB camera. The launch also publishes:

- `/slam/current_pose` (`geometry_msgs/PoseStamped`)
- `/slam/start_pose` (`geometry_msgs/PoseStamped`)
- `/slam/path` (`nav_msgs/Path`)
- `/slam/distance_traveled` (`std_msgs/Float64`, filtered XY meters)
- `/slam/distance_traveled_3d` (`std_msgs/Float64`, filtered XYZ meters)
- `/slam/distance_slam_corrected` (`std_msgs/Float64`, `/mapPath` XY meters)
- `/slam/distance_from_start` (`std_msgs/Float64`, meters)
- `/slam/diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- `/slam/rate_summary` (`std_msgs/String`)
- `/perception/person_detection` (`std_msgs/String`, normalized person-only JSON)

The camera can remain near 30 Hz while RTAB-Map performs the more expensive
graph and map update at 2 Hz. Visual odometry rate varies with CPU load and
scene texture; lowering the map rate prevents its queue from growing.

### Browser mission-control UI

After SLAM is running, open a second terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup mission_control.launch.py
```

Open `http://localhost:8080`. The gateway serves the UI and redirects it to the
read-only WebSocket telemetry endpoint automatically. It consumes the `/slam/*`
telemetry topics, `/slam/diagnostics`, `/map`, `/cloud_map`, `/info`, and
the D435 color image. Large map, cloud, path, and image streams are bounded or
throttled before they reach the browser. The PC default preserves RGB and up
to 60,000 valid cloud points; `max_cloud_points` can be reduced for an SBC.
Each complete cloud becomes a fresh 3 cm voxel snapshot, while an empty or
implausibly partial RTAB-Map product keeps the last valid map visible. Complete
snapshots are not unioned: `/cloud_map` is already accumulated and optimized,
so keeping older coordinates would create smeared duplicate walls.
`cloud_voxel_size` can be tuned independently.
The default `OVERVIEW` renders 2D and 3D together while keeping the camera and
person-only detection panels visible.
The cloud renderer uses a persistent CPU raster/depth buffer rather than
WebGL. A completed frame is swapped in atomically while the previous frame
remains visible, avoiding blank frames on integrated GPUs.
The camera itself uses an independent latest-frame-only `/camera.mjpg` stream
at up to 20 FPS, so cloud telemetry cannot create a camera backlog. The gateway
also publishes a 320 px, 2 Hz `/mission_control/yolo_input`; YOLO does not copy
the full 640x480 camera topic on every frame.

The gateway consumes `/imu_50/data`, `/imu_51/data`, and `/imu_52/data`
independently. Quaternion orientation is shown in the `SNAKE POSE` WebGL view;
0x51/0x52 spacing is explicitly visualization-only until measured. The optional
`/snake/segment_poses` input remains available for a later joint-aware estimator.

Use the robot-centered top-down view for a game-style explored map:

```bash
ros2 launch my_robot_bringup rtabmap_mapping.launch.py \
  database_path:=~/.ros/mission_001.db \
  reset_database:=false rviz:=true topdown:=true
```

Distance traveled is accumulated from `/visual_odom`, not the EKF
prediction. A 2.5 cm spatial deadband, low-pass filter, jump rejection, and
2.5 m/s physical speed gate prevent stationary pose jitter from becoming
distance. A rejected VO reset re-anchors the filter without adding the jump,
so one outlier cannot permanently freeze the displayed distance. The UI separately reports XY distance, XYZ distance, and the
loop-closure-corrected `/mapPath` length. The IMU improves orientation and the
EKF robot pose, but raw accelerometer double integration is intentionally not
used as a distance sensor because bias and gravity errors cause rapid drift.

### Camera-only mapping (no IMU and no EKF)

Use a separate database so camera-only results cannot overwrite the fused map:

```bash
ros2 launch my_robot_bringup rtabmap_camera_only_mapping.launch.py \
  database_path:=~/.ros/mission_camera_only.db \
  reset_database:=true \
  rviz:=true \
  rtabmap_viz:=false
```

In this mode, the WT901 driver and `ekf_filter_node` are not started.
`rgbd_odometry` is the only publisher of `odom -> base_link`. Move slowly and
avoid rapid rotation because no inertial rotation estimate is available when
visual features are blurred or leave the image. Camera-only RTAB-Map updates
the graph at 2 Hz on the Intel N97 while visual odometry remains near the camera
rate. The RViz and telemetry outputs are the same as in fused mode.

For later sessions, omit `reset_database` or set it to false to continue the
same database. Record raw data while mapping so the pipeline can be replayed:

```bash
ros2 bag record \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /imu_50/data /imu_51/data /imu_52/data /tf /tf_static
```

## Optional existing-map localization

This is a secondary experiment mode, not the default disaster deployment
workflow. Use it only when the mission intentionally reuses a known map.

```bash
ros2 launch my_robot_bringup rtabmap_localization.launch.py \
  database_path:=~/.ros/terrain.db \
  rtabmap_viz:=true
```
