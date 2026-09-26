import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


try:
    import serial
except ImportError:
    serial = None


PACKET_PATTERN = re.compile(r'^\s*L:(?P<left>-?\d+),R:(?P<right>-?\d+)\s*$')


class EncoderSerialBridge(Node):
    """
    Reads Arduino encoder packets and publishes cumulative wheel tick counts.

    Expected serial format:
        L:<signed_left_ticks>,R:<signed_right_ticks>

    This node never invents encoder data. If the serial device or pyserial is
    unavailable, it logs the problem and publishes nothing.
    """

    def __init__(self):
        super().__init__('encoder_serial_bridge')

        self.declare_parameter('serial_device', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('read_rate_hz', 50.0)

        self.serial_device = (
            self.get_parameter('serial_device').get_parameter_value().string_value
        )
        self.baud_rate = (
            self.get_parameter('baud_rate').get_parameter_value().integer_value
        )
        read_rate_hz = (
            self.get_parameter('read_rate_hz').get_parameter_value().double_value
        )

        self.left_publisher = self.create_publisher(
            Int64,
            '/left_wheel_ticks',
            10,
        )
        self.right_publisher = self.create_publisher(
            Int64,
            '/right_wheel_ticks',
            10,
        )

        self.serial_port = None
        self.malformed_count = 0

        if serial is None:
            self.get_logger().error(
                'pyserial is not installed; cannot open Arduino serial device. '
                'Install python3-serial or pyserial before using real hardware.'
            )
        else:
            self.open_serial_port()

        period = 1.0 / max(read_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.read_serial)

    def open_serial_port(self):
        try:
            self.serial_port = serial.Serial(
                self.serial_device,
                self.baud_rate,
                timeout=0.0,
            )
            self.get_logger().info(
                f'Opened encoder serial device {self.serial_device} '
                f'at {self.baud_rate} baud'
            )
        except Exception as exc:
            self.serial_port = None
            self.get_logger().error(
                f'Failed to open encoder serial device {self.serial_device}: {exc}'
            )

    def read_serial(self):
        if self.serial_port is None:
            return

        while self.serial_port.in_waiting:
            try:
                raw_line = self.serial_port.readline()
            except Exception as exc:
                self.get_logger().error(f'Error reading encoder serial data: {exc}')
                return

            try:
                line = raw_line.decode('ascii', errors='replace').strip()
            except Exception as exc:
                self.get_logger().warn(f'Failed to decode encoder line: {exc}')
                continue

            match = PACKET_PATTERN.match(line)
            if not match:
                self.malformed_count += 1
                if self.malformed_count <= 5 or self.malformed_count % 50 == 0:
                    self.get_logger().warn(
                        f'Ignoring malformed encoder packet #{self.malformed_count}: {line!r}'
                    )
                continue

            left_msg = Int64()
            right_msg = Int64()
            left_msg.data = int(match.group('left'))
            right_msg.data = int(match.group('right'))
            self.left_publisher.publish(left_msg)
            self.right_publisher.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port is not None:
            node.serial_port.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
