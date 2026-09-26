import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def diagonal_covariance(x, y, z):
    covariance = [0.0] * 9
    covariance[0] = x
    covariance[4] = y
    covariance[8] = z
    return covariance


class SimulatedImuPublisher(Node):
    """
    Deterministic simulated raw IMU source for offline validation only.

    This node is not a hardware driver. It publishes sensor_msgs/Imu messages
    in the imu_link frame so timestamp, covariance, topic, and frame validation
    can be proven before a physical IMU is selected.

    Orientation is intentionally marked unavailable by default using the
    sensor_msgs/Imu convention orientation_covariance[0] = -1. The simulated
    source only represents raw gyro and accelerometer measurements.
    """

    def __init__(self):
        super().__init__('simulated_imu_publisher')

        self.declare_parameter('topic_name', '/imu/data_raw')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('samples_per_case', 5)
        self.declare_parameter('stationary_accel_z', 9.80665)
        self.declare_parameter('yaw_rate', 0.5)
        self.declare_parameter('forward_accel_x', 0.8)
        self.declare_parameter('angular_velocity_covariance', 0.01)
        self.declare_parameter('linear_acceleration_covariance', 0.10)

        self.topic_name = self.get_parameter(
            'topic_name'
        ).get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id'
        ).get_parameter_value().string_value
        publish_rate_hz = self.get_parameter(
            'publish_rate_hz'
        ).get_parameter_value().double_value
        self.samples_per_case = max(
            1,
            self.get_parameter('samples_per_case')
            .get_parameter_value()
            .integer_value,
        )
        stationary_accel_z = self.get_parameter(
            'stationary_accel_z'
        ).get_parameter_value().double_value
        yaw_rate = self.get_parameter(
            'yaw_rate'
        ).get_parameter_value().double_value
        forward_accel_x = self.get_parameter(
            'forward_accel_x'
        ).get_parameter_value().double_value
        angular_covariance = self.get_parameter(
            'angular_velocity_covariance'
        ).get_parameter_value().double_value
        linear_covariance = self.get_parameter(
            'linear_acceleration_covariance'
        ).get_parameter_value().double_value

        self.angular_velocity_covariance = diagonal_covariance(
            angular_covariance,
            angular_covariance,
            angular_covariance,
        )
        self.linear_acceleration_covariance = diagonal_covariance(
            linear_covariance,
            linear_covariance,
            linear_covariance,
        )

        # Standard sensor_msgs/Imu convention: -1 in element 0 means this raw
        # IMU stream does not provide a valid orientation estimate.
        self.orientation_covariance = [-1.0] + [0.0] * 8

        self.cases = [
            {
                'name': 'stationary',
                'angular_velocity': (0.0, 0.0, 0.0),
                'linear_acceleration': (0.0, 0.0, stationary_accel_z),
            },
            {
                'name': 'positive_yaw_rotation',
                'angular_velocity': (0.0, 0.0, yaw_rate),
                'linear_acceleration': (0.0, 0.0, stationary_accel_z),
            },
            {
                'name': 'negative_yaw_rotation',
                'angular_velocity': (0.0, 0.0, -yaw_rate),
                'linear_acceleration': (0.0, 0.0, stationary_accel_z),
            },
            {
                'name': 'forward_acceleration',
                'angular_velocity': (0.0, 0.0, 0.0),
                'linear_acceleration': (forward_accel_x, 0.0, stationary_accel_z),
            },
        ]
        self.sample_index = 0
        self.last_case_name = None

        self.publisher = self.create_publisher(Imu, self.topic_name, 10)
        self.timer = self.create_timer(
            1.0 / max(publish_rate_hz, 1.0),
            self.publish_sample,
        )

        self.get_logger().info(
            'Simulated raw IMU publisher started on '
            f'{self.topic_name} with frame_id={self.frame_id}. '
            'Orientation is intentionally unavailable '
            '(orientation_covariance[0] = -1). Covariances are PLACEHOLDER '
            'values for validation only, not calibrated sensor noise.'
        )

    def publish_sample(self):
        case_index = (self.sample_index // self.samples_per_case) % len(self.cases)
        case = self.cases[case_index]
        self.sample_index += 1

        if case['name'] != self.last_case_name:
            self.last_case_name = case['name']
            self.get_logger().info(f'Publishing simulated IMU case: {case["name"]}')

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # This raw simulated IMU source does not provide a legitimate
        # orientation estimate. Keep the quaternion fields as "unused" zeros
        # and require consumers to check orientation_covariance[0] == -1.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        msg.orientation_covariance = self.orientation_covariance

        av_x, av_y, av_z = case['angular_velocity']
        msg.angular_velocity.x = av_x
        msg.angular_velocity.y = av_y
        msg.angular_velocity.z = av_z
        msg.angular_velocity_covariance = self.angular_velocity_covariance

        la_x, la_y, la_z = case['linear_acceleration']
        msg.linear_acceleration.x = la_x
        msg.linear_acceleration.y = la_y
        msg.linear_acceleration.z = la_z
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedImuPublisher()

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
