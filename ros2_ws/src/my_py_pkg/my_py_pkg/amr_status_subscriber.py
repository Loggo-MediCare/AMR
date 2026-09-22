import re

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

        self.get_logger().info('AMR status monitor started')

    def status_callback(self, msg):

        self.get_logger().info(f'Received: {msg.data}')

        match = re.search(
            r'Battery:\s*([\d.]+)\s*V\s*\|\s*Motor Temp:\s*([\d.]+)\s*C',
            msg.data
        )

        if not match:
            self.get_logger().error(
                f'Invalid AMR status message: {msg.data}'
            )
            return

        battery_voltage = float(match.group(1))
        motor_temperature = float(match.group(2))

        if motor_temperature >= 80.0:
            self.get_logger().warn(
                f'WARNING: MOTOR OVERHEAT! {motor_temperature:.1f} C'
            )

        if battery_voltage < 22.0:
            self.get_logger().warn(
                f'WARNING: BATTERY LOW! {battery_voltage:.1f} V'
            )


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
