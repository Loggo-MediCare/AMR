import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class SimulatedEncoderTicks(Node):
    """
    Safe simulated encoder source for offline development only.

    This node is NOT Arduino hardware input. It publishes deterministic
    cumulative tick sequences to exercise wheel_odometry_node without a robot.
    """

    def __init__(self):
        super().__init__('simulated_encoder_ticks')

        self.left_publisher = self.create_publisher(Int64, '/left_wheel_ticks', 10)
        self.right_publisher = self.create_publisher(Int64, '/right_wheel_ticks', 10)

        self.sequence = self.build_sequence()
        self.index = 0
        self.timer = self.create_timer(0.2, self.publish_next)
        self.get_logger().info(
            'Simulated encoder tick source started (offline test only)'
        )

    @staticmethod
    def build_sequence():
        samples = []
        left = 0
        right = 0

        def append_many(count, left_step, right_step):
            nonlocal left, right
            for _ in range(count):
                left += left_step
                right += right_step
                samples.append((left, right))

        append_many(5, 0, 0)       # stopped
        append_many(20, 30, 30)    # straight forward
        append_many(12, -25, -25)  # straight reverse
        append_many(12, -20, 20)   # rotate left in place
        append_many(12, 20, -20)   # rotate right in place
        append_many(18, 15, 30)    # curved motion
        return samples

    def publish_next(self):
        if self.index >= len(self.sequence):
            self.index = len(self.sequence) - 1

        left, right = self.sequence[self.index]
        self.index += 1

        left_msg = Int64()
        right_msg = Int64()
        left_msg.data = left
        right_msg.data = right
        self.left_publisher.publish(left_msg)
        self.right_publisher.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedEncoderTicks()

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
