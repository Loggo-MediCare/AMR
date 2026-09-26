import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


class ImuMessageValidator(Node):
    """
    Validates the Milestone 4 raw IMU stream before future EKF integration.

    Checks:
      - /imu/data_raw uses sensor_msgs/msg/Imu
      - header.frame_id is the expected imu_link
      - message timestamps are populated
      - timestamp progression and observed host-side message age are reasonable
      - covariance fields are present and orientation semantics are explicit
      - TF contains base_link -> imu_link

    The observed-age diagnostic is not a clock synchronization test. It is only
    meaningful when the incoming IMU timestamp is already expressed in the same
    compatible ROS time domain as this validator clock. For the current
    simulation, both nodes use the ROS clock. Future hardware device timestamps
    must first be mapped or synchronized into the ROS/Jetson time domain before
    interpreting this diagnostic.
    """

    def __init__(self):
        super().__init__('imu_message_validator')

        self.declare_parameter('topic_name', '/imu/data_raw')
        self.declare_parameter('expected_frame_id', 'imu_link')
        self.declare_parameter('tf_parent_frame', 'base_link')
        self.declare_parameter('tf_child_frame', 'imu_link')
        self.declare_parameter('max_observed_age_sec', 1.0)

        self.topic_name = self.get_parameter(
            'topic_name'
        ).get_parameter_value().string_value
        self.expected_frame_id = self.get_parameter(
            'expected_frame_id'
        ).get_parameter_value().string_value
        self.tf_parent_frame = self.get_parameter(
            'tf_parent_frame'
        ).get_parameter_value().string_value
        self.tf_child_frame = self.get_parameter(
            'tf_child_frame'
        ).get_parameter_value().string_value
        self.max_observed_age_sec = self.get_parameter(
            'max_observed_age_sec'
        ).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.valid_count = 0
        self.invalid_count = 0
        self.missing_tf_warned = False
        self.previous_timestamp = None
        self.create_subscription(Imu, self.topic_name, self.validate_message, 10)

        self.get_logger().info(
            f'Validating {self.topic_name}: expected frame_id='
            f'{self.expected_frame_id}, required TF '
            f'{self.tf_parent_frame} -> {self.tf_child_frame}, '
            f'max_observed_age_sec={self.max_observed_age_sec:.3f} '
            '(PLACEHOLDER diagnostic threshold)'
        )

    @staticmethod
    def timestamp_is_populated(msg):
        return msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0

    @staticmethod
    def covariance_has_nine_values(covariance):
        return len(covariance) == 9

    def validate_message(self, msg):
        message_valid = True

        if msg.header.frame_id != self.expected_frame_id:
            self.invalid_count += 1
            self.get_logger().error(
                'Invalid IMU frame_id: '
                f'expected {self.expected_frame_id}, got {msg.header.frame_id}'
            )
            return

        if not self.timestamp_is_populated(msg):
            message_valid = False
            self.get_logger().error('Invalid IMU timestamp: header.stamp is zero')

        msg_time = Time.from_msg(msg.header.stamp)
        now = self.get_clock().now()
        observed_age_sec = (now - msg_time).nanoseconds / 1e9

        if self.previous_timestamp is not None and msg_time <= self.previous_timestamp:
            self.get_logger().warn(
                'IMU timestamp did not progress monotonically. '
                'This is diagnostic only; message is not rejected.'
            )

        if observed_age_sec < 0.0:
            self.get_logger().warn(
                'IMU observed host-side message age is negative '
                f'({observed_age_sec * 1000.0:.3f} ms). This may indicate clock '
                'domain mismatch or simulation time ordering. Message is not rejected.'
            )
        elif observed_age_sec > self.max_observed_age_sec:
            self.get_logger().warn(
                'IMU observed host-side message age exceeds PLACEHOLDER threshold: '
                f'{observed_age_sec:.3f} s > {self.max_observed_age_sec:.3f} s. '
                'This is not a synchronization-error measurement and the message '
                'is not rejected.'
            )

        if not self.covariance_has_nine_values(msg.orientation_covariance):
            message_valid = False
            self.get_logger().error('orientation_covariance does not have 9 values')
        if not self.covariance_has_nine_values(msg.angular_velocity_covariance):
            message_valid = False
            self.get_logger().error(
                'angular_velocity_covariance does not have 9 values'
            )
        if not self.covariance_has_nine_values(msg.linear_acceleration_covariance):
            message_valid = False
            self.get_logger().error(
                'linear_acceleration_covariance does not have 9 values'
            )

        try:
            self.tf_buffer.lookup_transform(
                self.tf_parent_frame,
                self.tf_child_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            tf_available = True
        except TransformException as exc:
            tf_available = False
            if not self.missing_tf_warned:
                self.get_logger().warn(
                    'Required TF is not available yet: '
                    f'{self.tf_parent_frame} -> {self.tf_child_frame}: {exc}'
                )
                self.missing_tf_warned = True

        if not tf_available:
            message_valid = False

        if not message_valid:
            self.invalid_count += 1
            return

        self.valid_count += 1
        self.previous_timestamp = msg_time
        if msg.orientation_covariance[0] == -1.0:
            orientation_status = 'orientation unavailable (raw IMU convention)'
        else:
            orientation_status = 'orientation covariance present'

        if self.valid_count == 1 or self.valid_count % 10 == 0:
            self.get_logger().info(
                'Valid IMU sample '
                f'#{self.valid_count}: frame_id={msg.header.frame_id}, '
                f'angular_velocity.z={msg.angular_velocity.z:.3f}, '
                f'linear_acceleration.x={msg.linear_acceleration.x:.3f}, '
                f'observed host-side age={observed_age_sec * 1000.0:.3f} ms, '
                f'{orientation_status}, TF '
                f'{self.tf_parent_frame}->{self.tf_child_frame} available'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImuMessageValidator()

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
