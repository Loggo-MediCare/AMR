import rclpy
from rclpy.node import Node
from amr_interfaces.msg import AMRStatus


class AMRStatusSubscriber(Node):

    def __init__(self):
        super().__init__('amr_status_subscriber')

        self.subscription = self.create_subscription(
            AMRStatus,
            '/amr_status',
            self.status_callback,
            10
        )

        self.get_logger().info(
            'AMR typed status monitor started'
        )

    def status_callback(self, msg):

        self.get_logger().info(
            f'Battery={msg.battery_voltage:.1f} V | '
            f'Motor Temp={msg.motor_temperature:.1f} C | '
            f'Motors Ready={msg.motors_ready} | '
            f'Status={msg.debug_message}'
        )

        if msg.motor_temperature >= 80.0:
            self.get_logger().warn(
                f'WARNING: MOTOR OVERHEAT! '
                f'{msg.motor_temperature:.1f} C'
            )

        if msg.battery_voltage < 22.0:
            self.get_logger().warn(
                f'WARNING: BATTERY LOW! '
                f'{msg.battery_voltage:.1f} V'
            )

        if not msg.motors_ready:
            self.get_logger().warn(
                'AMR MOTORS NOT READY'
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

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
