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
└── ros2_ws/
    └── src/
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
                └── amr_status_subscriber.py
```

## STL References and 360 Preview

`stl_references/` holds candidate mechanical parts pulled from O'Reilly reference material, kept separate from the ROS 2 workspace so they don't get mixed into `colcon build`:

- `bumblebot/` — BumbleBot base plate (`Base_Plate.stl`), from *Build Autonomous Mobile Robot from Scratch using ROS*, Ch. 7. Candidate for a `base_link` mesh in a future URDF.
- `skycam_camera_mount/` — five-part pan/tilt camera mount, from *3D Printing Projects*, Ch. 8. Reference geometry for a future cuVSLAM camera bracket, not a final Jetson mount.

Open `stl_references/stl_360_preview.html` directly in a browser (no server needed) to spin all 6 parts 360° and sanity-check geometry before committing to a URDF mesh scale. It's a self-contained file — the triangle data is embedded inline, so it works offline.

See `stl_references/README.md` for the STL-to-URDF mesh snippet and scale-unit warning (STL is usually authored in millimeters; URDF expects meters).

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
