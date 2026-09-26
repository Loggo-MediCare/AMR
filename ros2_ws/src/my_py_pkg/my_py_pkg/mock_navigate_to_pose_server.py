import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.node import Node


class MockNavigateToPoseServer(Node):

    def __init__(self):
        super().__init__('mock_navigate_to_pose_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback,
        )
        self.get_logger().info(
            'Mock NavigateToPose action server started on navigate_to_pose'
        )

    def execute_callback(self, goal_handle):
        pose = goal_handle.request.pose.pose
        position = pose.position
        orientation = pose.orientation

        self.get_logger().info(
            'Accepted goal: '
            f'x={position.x:.3f}, y={position.y:.3f}, '
            f'orientation=(x={orientation.x:.3f}, '
            f'y={orientation.y:.3f}, z={orientation.z:.3f}, '
            f'w={orientation.w:.3f})'
        )

        time.sleep(2.0)
        goal_handle.succeed()
        self.get_logger().info('Mock navigation succeeded.')

        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = MockNavigateToPoseServer()

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
