import random
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

        self.get_logger().info('AMR random status publisher started')

    def publish_status(self):

        # Simulate real AMR sensor values
        battery_voltage = random.uniform(21.0, 25.5)
        motor_temperature = random.uniform(30.0, 95.0)

        msg = String()

        msg.data = (
            f'Battery: {battery_voltage:.1f} V | '
            f'Motor Temp: {motor_temperature:.1f} C'
        )

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Publishing: {msg.data}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = AMRStatusPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
