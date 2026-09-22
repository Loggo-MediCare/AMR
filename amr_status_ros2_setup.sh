#!/usr/bin/env bash
set -eo pipefail

# AMR ROS 2 Jazzy pub/sub setup
# Run INSIDE the ros2-jazzy-dev container.
# Expected workspace: /root/ros2_ws
# Expected package:   my_py_pkg

WS="${HOME}/ros2_ws"
PKG="${WS}/src/my_py_pkg"
PYMOD="${PKG}/my_py_pkg"

echo "== 1. Source ROS 2 Jazzy =="
source /opt/ros/jazzy/setup.bash

echo "== 2. Check package =="
if [[ ! -d "$PKG" ]]; then
  echo "ERROR: $PKG does not exist."
  echo "Create my_py_pkg first, then re-run this script."
  exit 1
fi

echo "== 3. Create AMR status publisher =="
cat > "${PYMOD}/amr_status_publisher.py" <<'PY'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AMRStatusPublisher(Node):
    def __init__(self):
        super().__init__('amr_status_publisher')

        self.publisher_ = self.create_publisher(
            String,
            '/amr_status',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('AMR status publisher started')

    def publish_status(self):
        battery_voltage = 24.6
        motor_temperature = 38.2

        msg = String()
        msg.data = (
            f'Battery: {battery_voltage:.1f} V | '
            f'Motor Temp: {motor_temperature:.1f} C'
        )

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = AMRStatusPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY

echo "== 4. Create AMR status subscriber =="
cat > "${PYMOD}/amr_status_subscriber.py" <<'PY'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AMRStatusSubscriber(Node):
    def __init__(self):
        super().__init__('amr_status_subscriber')

        self.subscription = self.create_subscription(
            String,
            '/amr_status',
            self.status_callback,
            10
        )

        self.get_logger().info('AMR status subscriber started')

    def status_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = AMRStatusSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY

chmod +x \
  "${PYMOD}/amr_status_publisher.py" \
  "${PYMOD}/amr_status_subscriber.py"

echo "== 5. Add std_msgs dependency to package.xml if missing =="
python3 - <<'PY'
from pathlib import Path

p = Path.home() / "ros2_ws/src/my_py_pkg/package.xml"
text = p.read_text()

if "<depend>std_msgs</depend>" not in text:
    if "<depend>rclpy</depend>" in text:
        text = text.replace(
            "<depend>rclpy</depend>",
            "<depend>rclpy</depend>\n  <depend>std_msgs</depend>"
        )
    else:
        text = text.replace(
            "</package>",
            "  <depend>rclpy</depend>\n"
            "  <depend>std_msgs</depend>\n"
            "</package>"
        )
    p.write_text(text)
    print("Added std_msgs dependency.")
else:
    print("std_msgs dependency already present.")
PY

echo "== 6. Add console_scripts entry points to setup.py if missing =="
python3 - <<'PY'
from pathlib import Path
import re

p = Path.home() / "ros2_ws/src/my_py_pkg/setup.py"
text = p.read_text()

entries = [
    "amr_status_publisher = my_py_pkg.amr_status_publisher:main",
    "amr_status_subscriber = my_py_pkg.amr_status_subscriber:main",
]

missing = [entry for entry in entries if entry not in text]

if missing:
    m = re.search(r"('console_scripts'\s*:\s*\[)(.*?)(\])", text, flags=re.S)
    if not m:
        raise SystemExit("Could not find 'console_scripts' list in setup.py")

    body = m.group(2)
    if body.strip() and not body.rstrip().endswith(","):
        body = body.rstrip() + ",\n"

    additions = "".join(f"            '{entry}',\n" for entry in missing)
    new_body = body + additions

    text = text[:m.start(2)] + new_body + text[m.end(2):]
    p.write_text(text)
    print("Added:", ", ".join(missing))
else:
    print("Entry points already present.")
PY

echo "== 7. Build my_py_pkg =="
cd "$WS"
colcon build --packages-select my_py_pkg --symlink-install

echo "== 8. Source built workspace =="
source "${WS}/install/setup.bash"

echo
echo "=========================================="
echo "READY"
echo "=========================================="
echo
echo "Terminal A:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source /root/ros2_ws/install/setup.bash"
echo "  ros2 run my_py_pkg amr_status_publisher"
echo
echo "Terminal B:"
echo "  docker exec -it ros2-jazzy-dev bash"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source /root/ros2_ws/install/setup.bash"
echo "  ros2 run my_py_pkg amr_status_subscriber"
echo
echo "Terminal C (inspect ROS graph):"
echo "  docker exec -it ros2-jazzy-dev bash"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source /root/ros2_ws/install/setup.bash"
echo "  ros2 node list"
echo "  ros2 topic list"
echo "  ros2 topic info /amr_status"
echo "  ros2 topic echo /amr_status"
echo "  ros2 topic hz /amr_status"
