# ROS 2 Jazzy AMR Status Monitoring Demo

A simple ROS 2 Jazzy publisher/subscriber example for AMR health monitoring.

The demo simulates:
- Battery voltage
- Motor temperature
- Low-battery warning
- Motor over-temperature warning

The publisher sends random AMR status data on `/amr_status`.
The subscriber receives the message through `status_callback()` and checks alarm conditions.

## Architecture

```text
AMR Status Publisher
        |
        |  /amr_status
        v
AMR Status Subscriber
        |
        +--> Battery < 22.0 V   -> WARNING
        +--> Motor Temp >= 80 C -> WARNING
```

## Environment

- ROS 2 Jazzy
- Docker
- Python / `rclpy`
- Docker container: `ros2-jazzy-dev`
- ROS workspace: `/root/ros2_ws`

## Enter the ROS 2 Docker Container

From the Mac terminal:

```bash
docker start ros2-jazzy-dev
docker exec -it ros2-jazzy-dev bash
```

Inside Docker:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
```

## Build the Package

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select my_py_pkg \
  --symlink-install

source install/setup.bash
```

Expected:

```text
Summary: 1 package finished
```

## Run the AMR Status Publisher

Open Terminal A:

```bash
docker exec -it ros2-jazzy-dev bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 run my_py_pkg amr_status_publisher
```

Example output:

```text
[INFO] [amr_status_publisher]: AMR random status publisher started
[INFO] [amr_status_publisher]: Publishing: Battery: 24.3 V | Motor Temp: 47.7 C
[INFO] [amr_status_publisher]: Publishing: Battery: 22.5 V | Motor Temp: 85.7 C
```

## Run the AMR Status Subscriber

Open Terminal B:

```bash
docker exec -it ros2-jazzy-dev bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 run my_py_pkg amr_status_subscriber
```

Example output:

```text
[INFO] [amr_status_subscriber]: AMR status monitor started
[INFO] [amr_status_subscriber]: Received: Battery: 22.5 V | Motor Temp: 85.7 C
[WARN] [amr_status_subscriber]: WARNING: MOTOR OVERHEAT! 85.7 C

[INFO] [amr_status_subscriber]: Received: Battery: 21.2 V | Motor Temp: 51.0 C
[WARN] [amr_status_subscriber]: WARNING: BATTERY LOW! 21.2 V
```

## Subscriber Callback

ROS 2 automatically calls the callback whenever a new `/amr_status` message arrives:

```python
def status_callback(self, msg):
    self.get_logger().info(f'Received: {msg.data}')
```

The callback checks:
- `motor_temperature >= 80.0`
- `battery_voltage < 22.0`

## Inspect the ROS 2 Graph

Open Terminal C:

```bash
docker exec -it ros2-jazzy-dev bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
```

List nodes:

```bash
ros2 node list
```

Expected:

```text
/amr_status_publisher
/amr_status_subscriber
```

List topics:

```bash
ros2 topic list
```

Show topic info:

```bash
ros2 topic info /amr_status
```

Expected:

```text
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

Watch messages:

```bash
ros2 topic echo /amr_status
```

Check frequency:

```bash
ros2 topic hz /amr_status
```

Expected rate is approximately 1 Hz.

## Manually Publish a Test Message

Normal:

```bash
ros2 topic pub --once /amr_status std_msgs/msg/String \
"{data: 'Battery: 24.6 V | Motor Temp: 38.2 C'}"
```

Motor over-temperature:

```bash
ros2 topic pub --once /amr_status std_msgs/msg/String \
"{data: 'Battery: 24.6 V | Motor Temp: 85.0 C'}"
```

Low battery:

```bash
ros2 topic pub --once /amr_status std_msgs/msg/String \
"{data: 'Battery: 21.5 V | Motor Temp: 45.0 C'}"
```

Both alarms:

```bash
ros2 topic pub --once /amr_status std_msgs/msg/String \
"{data: 'Battery: 21.5 V | Motor Temp: 88.0 C'}"
```

## Project Structure

```text
ros2_ws/
└── src/
    └── my_py_pkg/
        ├── package.xml
        ├── setup.py
        └── my_py_pkg/
            ├── __init__.py
            ├── amr_status_publisher.py
            └── amr_status_subscriber.py
```

## ROS 2 Data Flow

```text
Random Battery + Motor Temperature
              |
              v
   amr_status_publisher
              |
              | std_msgs/msg/String
              v
         /amr_status
              |
              v
   amr_status_subscriber
              |
       status_callback()
              |
       +------+------+
       |             |
       v             v
 Battery Alarm   Motor Alarm
```

## Next Improvement

The current prototype uses:

```text
std_msgs/msg/String
```

A better production-style design is a custom message:

```text
AMRStatus.msg
```

For example:

```text
float32 battery_voltage
float32 motor_temperature
bool motors_ready
string debug_message
```

This avoids string parsing and is better for dashboards, rosbag2, Jetson, and embedded controllers.

## Stop a Node

Press:

```text
Ctrl+C
```
