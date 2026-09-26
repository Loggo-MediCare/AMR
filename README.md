# ROS 2 Jazzy AMR Status Monitoring Demo

ROS 2 Jazzy AMR health-monitoring prototype using a typed custom message, publisher/subscriber nodes, alarm callbacks, Docker, and rosbag2.

## Architecture

```text
Random AMR Sensor Simulation
        |
        v
amr_status_publisher
        |
        |  amr_interfaces/msg/AMRStatus
        v
    /amr_status
        |
        +----------------------+
        |                      |
        v                      v
amr_status_subscriber      rosbag2 record
        |                      |
        v                      v
 status_callback()         MCAP recording
        |                      |
        +<---- rosbag2 play ---+
        |
        +--> Battery < 22.0 V   -> WARNING
        +--> Motor Temp >= 80 C -> WARNING
        +--> motors_ready false -> WARNING
```

## Environment

- ROS 2 Jazzy
- Docker
- Python / `rclpy`
- Docker container: `ros2-jazzy-dev`
- Workspace: `/root/ros2_ws`

## Custom Message

The project uses:

```text
amr_interfaces/msg/AMRStatus
```

Definition:

```text
float32 battery_voltage
float32 motor_temperature
bool motors_ready
string debug_message
```

Verify:

```bash
ros2 interface show amr_interfaces/msg/AMRStatus
```

Expected:

```text
float32 battery_voltage
float32 motor_temperature
bool motors_ready
string debug_message
```

## Start Docker

From macOS:

```bash
docker start ros2-jazzy-dev
docker exec -it ros2-jazzy-dev bash
```

Inside Docker:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
```

## Build

Build the interface first:

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build --packages-select amr_interfaces
source install/setup.bash
```

Build the Python package:

```bash
colcon build \
  --packages-select my_py_pkg \
  --symlink-install

source install/setup.bash
```

## Run Publisher

Terminal A:

```bash
docker exec -it ros2-jazzy-dev bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 run my_py_pkg amr_status_publisher
```

Example:

```text
Battery=24.3 V | Motor Temp=55.2 C | Motors Ready=True
Battery=21.8 V | Motor Temp=69.2 C | Motors Ready=False
Battery=24.4 V | Motor Temp=89.2 C | Motors Ready=False
```

## Run Subscriber

Terminal B:

```bash
docker exec -it ros2-jazzy-dev bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 run my_py_pkg amr_status_subscriber
```

Example:

```text
Battery=24.3 V | Motor Temp=55.2 C | Motors Ready=True | Status=OK

Battery=21.8 V | Motor Temp=69.2 C | Motors Ready=False | Status=CHECK AMR
WARNING: BATTERY LOW! 21.8 V
AMR MOTORS NOT READY

Battery=24.4 V | Motor Temp=89.2 C | Motors Ready=False | Status=CHECK AMR
WARNING: MOTOR OVERHEAT! 89.2 C
AMR MOTORS NOT READY
```

## Subscriber Callback

ROS 2 automatically calls `status_callback()` whenever a typed `AMRStatus` message arrives.

```python
def status_callback(self, msg):
    battery_voltage = msg.battery_voltage
    motor_temperature = msg.motor_temperature
    motors_ready = msg.motors_ready
    debug_message = msg.debug_message
```

No string parsing or regular expressions are required.

## Inspect the Topic

```bash
ros2 topic info /amr_status
```

Expected:

```text
Type: amr_interfaces/msg/AMRStatus
Publisher count: 1
Subscription count: 1
```

View typed messages:

```bash
ros2 topic echo /amr_status
```

Example:

```yaml
battery_voltage: 24.3
motor_temperature: 85.7
motors_ready: false
debug_message: CHECK AMR
---
```

## rosbag2 Record

Create a persistent recording directory:

```bash
mkdir -p /workspace/bags
cd /workspace/bags
```

Record:

```bash
ros2 bag record \
  --topics /amr_status \
  -o amr_status_typed_test
```

Stop recording with `Ctrl+C`.

Inspect:

```bash
ros2 bag info /workspace/bags/amr_status_typed_test
```

The bag should report:

```text
Topic: /amr_status
Type: amr_interfaces/msg/AMRStatus
```

## rosbag2 Replay

Stop the live publisher first:

```bash
pkill -f amr_status_publisher
```

Confirm:

```bash
pgrep -af amr_status_publisher
```

Then replay:

```bash
ros2 bag play /workspace/bags/amr_status_typed_test
```

Run the subscriber in another terminal:

```bash
ros2 run my_py_pkg amr_status_subscriber
```

The subscriber processes replayed telemetry exactly like live telemetry, including battery and motor-temperature warnings.

## Project Structure

```text
AMR/
├── README.md
├── .gitignore
├── amr_status_ros2_setup.sh
├── bags/
│   └── amr_status_typed_test/
│       ├── amr_status_typed_test_0.mcap
│       └── metadata.yaml
├── stl_references/
│   ├── README.md
│   ├── stl_360_preview.html
│   ├── make_stl_preview.py
│   ├── import_stls_to_freecad.py
│   ├── rotate_view_freecad.py
│   ├── bumblebot/
│   │   ├── Base_Plate.stl
│   │   └── Bumblebot_3d_Models.zip
│   └── skycam_camera_mount/
│       ├── Skycam-camera-front.stl
│       ├── Skycam-camera-back.stl
│       ├── Skycam-camera-pan.stl
│       ├── Skycam-camera-tilt.stl
│       └── Skycam-pan-tilt-top.stl
├── firmware/
│   └── arduino/
│       └── wheel_encoder/
│           └── wheel_encoder.ino
└── ros2_ws/
    └── src/
        ├── amr_description/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── launch/
        │   │   └── display.launch.py
        │   └── urdf/
        │       └── amr.urdf
        ├── amr_interfaces/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   └── msg/
        │       └── AMRStatus.msg
        └── my_py_pkg/
            ├── package.xml
            ├── setup.py
            └── my_py_pkg/
                ├── amr_status_publisher.py
                ├── amr_status_subscriber.py
                ├── encoder_serial_bridge.py
                ├── wheel_odometry_node.py
                ├── simulated_encoder_ticks.py
                └── offline_encoder_odometry_test.py
```

## STL References and 360 Preview

`stl_references/` holds candidate mechanical parts pulled from O'Reilly reference material, kept separate from the ROS 2 workspace so they don't get mixed into `colcon build`:

- `bumblebot/` — BumbleBot base plate (`Base_Plate.stl`), from *Build Autonomous Mobile Robot from Scratch using ROS*, Ch. 7. Candidate for a `base_link` mesh in a future URDF.
- `skycam_camera_mount/` — five-part pan/tilt camera mount, from *3D Printing Projects*, Ch. 8. Reference geometry for a future cuVSLAM camera bracket, not a final Jetson mount.

Open `stl_references/stl_360_preview.html` directly in a browser (no server needed) to spin all 6 parts 360° and sanity-check geometry before committing to a URDF mesh scale. It's a self-contained file — the triangle data is embedded inline, so it works offline.

See `stl_references/README.md` for the STL-to-URDF mesh snippet and scale-unit warning (STL is usually authored in millimeters; URDF expects meters).

## Wheel Odometry Pipeline

Milestone 3 replaces the placeholder odom TF with real differential-drive wheel odometry plumbing while keeping the mock node available for regression testing.

```text
Wheel encoder
   ↓
Arduino interrupt counting
   ↓
Serial USB
   ↓
encoder_serial_bridge
   ↓
/left_wheel_ticks + /right_wheel_ticks
   ↓
wheel_odometry_node
   ↓
differential-drive kinematics
   ↓
/odom + odom -> base_footprint TF
   ↓
future Nav2/localization
```

The TF architecture is intentionally:

```text
odom
└── base_footprint
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        ├── camera_link
        └── imu_link
```

The wheel odometry node publishes `odom -> base_footprint`. The URDF and `robot_state_publisher` keep the static `base_footprint -> base_link` transform. Do not change this back to `odom -> base_link`.

### Arduino Encoder Firmware

Firmware lives at:

```text
firmware/arduino/wheel_encoder/wheel_encoder.ino
```

Assumptions, all placeholders until hardware is selected:

- Encoder type: quadrature encoder with channel A and channel B.
- Left encoder pins: A=`2`, B=`4`.
- Right encoder pins: A=`3`, B=`5`.
- Baud rate: `115200`.
- Packet format:

```text
L:<signed_left_ticks>,R:<signed_right_ticks>
```

Example:

```text
L:12345,R:12312
```

The Arduino sketch uses interrupts on encoder channel A and direction inference from the A/B state. It maintains signed cumulative counters:

```cpp
volatile long left_ticks;
volatile long right_ticks;
```

Arduino `long` is commonly signed 32-bit on AVR boards, so long-running counters can roll over. The ROS 2 odometry logic handles signed 32-bit rollover when computing tick deltas.

### ROS 2 Nodes

Serial bridge:

```bash
ros2 run my_py_pkg encoder_serial_bridge \
  --ros-args \
  -p serial_device:=/dev/ttyACM0 \
  -p baud_rate:=115200
```

The bridge publishes:

```text
/left_wheel_ticks   std_msgs/msg/Int64
/right_wheel_ticks  std_msgs/msg/Int64
```

If pyserial is missing or `/dev/ttyACM0` is disconnected, the bridge logs a clear error and publishes nothing. It does not invent encoder data.

Wheel odometry:

```bash
ros2 run my_py_pkg wheel_odometry_node
```

Parameters, all placeholders until calibrated:

```text
wheel_radius           0.05 m
wheel_track            0.24 m
ticks_per_revolution   600
```

The node publishes:

```text
/odom                  nav_msgs/msg/Odometry
odom -> base_footprint TF
```

Only one node should publish `odom -> base_footprint` at a time. Do not run `mock_odom_publisher` and `wheel_odometry_node` together.

The `/odom` message includes explicit PLACEHOLDER covariance values for future
sensor fusion. They are not calibrated yet:

```text
pose_covariance_x      0.05
pose_covariance_y      0.05
pose_covariance_yaw    0.10
twist_covariance_linear_x   0.10
twist_covariance_angular_z  0.20
```

Unmeasured planar dimensions (`z`, `roll`, `pitch`, lateral/vertical twist)
are assigned high placeholder covariance (`999.0`) to make the uncertainty
semantics explicit. These values must be tuned from real encoder noise, floor
slip, and calibration tests before `robot_localization` consumes this odometry.

### Odometry Equations

The implementation intentionally uses the basic differential-drive approximation from the source material:

```text
ticks_per_meter = ticks_per_revolution / (2 * pi * wheel_radius)

delta_left  = (new_left_ticks  - old_left_ticks)  / ticks_per_meter
delta_right = (new_right_ticks - old_right_ticks) / ticks_per_meter

delta_distance = (delta_right + delta_left) / 2
delta_theta    = (delta_right - delta_left) / wheel_track

x     += delta_distance * cos(theta)
y     += delta_distance * sin(theta)
theta += delta_theta
```

Linear and angular velocities are computed as:

```text
linear_velocity  = delta_distance / dt
angular_velocity = delta_theta / dt
```

### Calibration

Calibration has not been performed yet.

Straight-line calibration:

1. Drive the robot physically 1 meter.
2. Compare measured distance against encoder-calculated distance.
3. Adjust `wheel_radius`, `ticks_per_revolution`, or equivalent `ticks_per_meter`.

Rotation calibration:

1. Rotate the robot physically 360 degrees (`2*pi` radians).
2. Compare measured rotation against encoder-calculated `theta`.
3. Adjust `wheel_track`.

### Overflow and Robustness

Handled cases:

- Signed 32-bit counter rollover: tick deltas are computed with rollover-aware math.
- Arduino `long` range: documented in firmware and handled in host-side delta calculation.
- Sudden unreasonable tick jumps: ignored using `max_tick_jump`; odometry state is preserved.
- First reading: initializes previous tick counters without moving the robot.
- `dt <= 0`: ignored, odometry state is preserved.
- Malformed serial packets: ignored by the serial bridge without resetting odometry.
- Disconnected Arduino: logged clearly; no fake encoder data is generated.

### Simulated Encoder Input

For offline development without Arduino hardware:

```bash
ros2 run my_py_pkg simulated_encoder_ticks
```

This node is explicitly simulated test input. It publishes deterministic cumulative tick sequences for stopped, forward, reverse, rotate-in-place, and curved motion cases.

Pure offline math test:

```bash
ros2 run my_py_pkg offline_encoder_odometry_test
```

### Future IMU Fusion

Do not configure `robot_localization` yet. Future architecture:

```text
wheel odometry ─┐
                ├→ robot_localization EKF → fused odometry
IMU ────────────┘
```

## Current Milestone

```text
Typed AMRStatus.msg        ✅
Typed Publisher            ✅
Typed Subscriber           ✅
status_callback() alarms   ✅
rosbag2 record             ✅
rosbag2 replay             ✅
ROS 2 Jazzy + Docker       ✅
```

## Next Steps

```text
Real STM32 / sensor telemetry
        ↓
ROS 2 AMRStatus
        ↓
Health monitoring
        ↓
rosbag2 logging
        ↓
Jetson / dashboard / diagnostics
```
