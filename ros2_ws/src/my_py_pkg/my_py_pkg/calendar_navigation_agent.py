import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def fetch_next_calendar_event():
    # Later this can be replaced with Google Calendar API OAuth2 access.
    return {
        'event_id': 'evt_20260926_01',
        'task': 'Patrol Station A',
        'coords': {'x': 2.5, 'y': -1.0, 'z': 0.0, 'w': 1.0},
    }


class CalendarNavigationAgent(Node):

    def __init__(self):
        super().__init__('calendar_navigation_agent')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        self.is_navigating = False
        self.current_event_id = None

        self.timer = self.create_timer(
            10.0,
            self.check_schedule_and_navigate,
        )
        self.get_logger().info('Calendar Navigation Agent started.')

    def check_schedule_and_navigate(self):
        if self.is_navigating:
            self.get_logger().info(
                'Robot is navigating; skipping this schedule check.'
            )
            return

        event = fetch_next_calendar_event()
        if event and event['event_id'] != self.current_event_id:
            self.current_event_id = event['event_id']
            self.get_logger().info(f"New schedule triggered: {event['task']}")
            self.send_goal(event['coords'])

    def send_goal(self, coords):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server is not ready.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(coords['x'])
        goal_msg.pose.pose.position.y = float(coords['y'])
        goal_msg.pose.pose.orientation.z = float(coords['z'])
        goal_msg.pose.pose.orientation.w = float(coords['w'])

        self.is_navigating = True
        self.get_logger().info(
            f"Sending Nav2 goal ({coords['x']}, {coords['y']})..."
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            self.is_navigating = False
            return

        self.get_logger().info('Nav2 accepted the goal; navigating...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation completed successfully.')
        else:
            self.get_logger().warn(
                f'Navigation stopped or failed; status code: {status}'
            )
        self.is_navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = CalendarNavigationAgent()

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
