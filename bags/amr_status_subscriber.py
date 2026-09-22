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

        self.get_logger().info(
            'AMR status monitor started'
        )

    def status_callback(self, msg):

        self.get_logger().info(
            f'Received: {msg.data}'
        )

        try:
            parts = msg.data.split('|')

            battery_voltage = float(
                parts[0]
                .split(':')[1]
                .replace('V', '')
                .strip()
            )

            motor_temperature = float(
                parts[1]
                .split(':')[1]
                .replace('C', '')
                .strip()
            )

            # Motor over-temperature alarm
            if motor_temperature >= 80.0:
                self.get_logger().warn(
                    f'🔥 WARNING: Motor temperature too high! '
                    f'{motor_temperature:.1f} C'
                )

            # Battery low alarm
            if battery_voltage < 22.0:
                self.get_logger().warn(
                    f'🔋 WARNING: Battery voltage low! '
                    f'{battery_voltage:.1f} V'
                )

        except (ValueError, IndexError):
            self.get_logger().error(
                f'Invalid AMR status message: {msg.data}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = AMRStatusSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
