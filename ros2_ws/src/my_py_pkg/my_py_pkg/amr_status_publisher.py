import random

import rclpy
from rclpy.node import Node
from amr_interfaces.msg import AMRStatus


class AMRStatusPublisher(Node):

    def __init__(self):
        super().__init__('amr_status_publisher')

        self.publisher_ = self.create_publisher(
            AMRStatus,
            '/amr_status',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            'AMR typed status publisher started'
        )

    def publish_status(self):

        msg = AMRStatus()

        msg.battery_voltage = random.uniform(21.0, 25.5)
        msg.motor_temperature = random.uniform(30.0, 95.0)

        msg.motors_ready = (
            msg.battery_voltage >= 22.0
            and msg.motor_temperature < 80.0
        )

        msg.debug_message = (
            'OK' if msg.motors_ready else 'CHECK AMR'
        )

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Battery={msg.battery_voltage:.1f} V | '
            f'Motor Temp={msg.motor_temperature:.1f} C | '
            f'Motors Ready={msg.motors_ready}'
        )


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
