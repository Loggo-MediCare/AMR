import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MockOdomPublisher(Node):
    """
    Milestone 2 placeholder: broadcasts the odom -> base_footprint transform.

    This is NOT real odometry. There is no wheel encoder or Gazebo feed
    yet, so the robot is published as stationary at the origin of odom.
    Replace with a real odometry source (wheel encoders / Gazebo diff
    drive plugin) in a later milestone.
    """

    def __init__(self):
        super().__init__('mock_odom_publisher')

        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_odom_tf)

        self.get_logger().info(
            'Mock odom publisher started (odom -> base_footprint, PLACEHOLDER identity transform)'
        )

    def publish_odom_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        # PLACEHOLDER: robot stays at the odom origin until a real
        # odometry source (encoders / Gazebo) replaces this node.
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = MockOdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
