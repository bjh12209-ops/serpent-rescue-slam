# WT901C485 ROS 2 driver

One dedicated I/O thread owns the half-duplex RS485 bus and polls all configured Modbus
addresses in sequence. No per-sensor serial threads are used, so requests cannot collide.

The current configuration polls `0x50` (head), `0x51` (middle), and `0x52`
(tail) sequentially on the same half-duplex bus with a 45 Hz target loop.
Each sensor publishes:

- `/imu_XX/data` (`sensor_msgs/msg/Imu`)
- `/imu_XX/mag` (`sensor_msgs/msg/MagneticField`)
- `/imu_XX/euler_deg` (`geometry_msgs/msg/Vector3Stamped`, x=roll, y=pitch, z=yaw)

Keep the robot stationary for two seconds after startup. The driver computes an independent
circular mean for each sensor's arbitrary 6-axis yaw and publishes yaw relative to that zero.

Build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select wt901c485_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch wt901c485_driver wt901c485.launch.py
```

When sensors are added or replaced, put only their verified decimal Modbus IDs
in `sensor_ids`. At 115200 baud, multiple sensors at high rate leave little
RS485 bandwidth margin; use the logged actual rates and timeout counts to choose
a safe per-sensor rate.
