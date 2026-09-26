import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int64
from tf2_ros import TransformBroadcaster

from my_py_pkg.wheel_odometry_math import DifferentialDriveOdometry


def yaw_to_quaternion(theta):
    half = theta / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def diagonal_covariance(x, y, z, roll, pitch, yaw):
    covariance = [0.0] * 36
    covariance[0] = x
    covariance[7] = y
    covariance[14] = z
    covariance[21] = roll
    covariance[28] = pitch
    covariance[35] = yaw
    return covariance


class WheelOdometryNode(Node):
    """
    Real wheel odometry pipeline stage.

    Subscribes to cumulative encoder tick counts and publishes:
      - nav_msgs/Odometry on /odom
      - dynamic TF odom -> base_footprint

    PLACEHOLDER physical parameters must be calibrated against the real robot.
    PLACEHOLDER covariance values are populated for future EKF integration and
    must be tuned from real encoder/slip calibration before robot_localization
    consumes this topic.
    """

    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_track', 0.24)
        self.declare_parameter('ticks_per_revolution', 600)
        self.declare_parameter('max_tick_jump', 100000)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('pose_covariance_x', 0.05)
        self.declare_parameter('pose_covariance_y', 0.05)
        self.declare_parameter('pose_covariance_yaw', 0.10)
        self.declare_parameter('twist_covariance_linear_x', 0.10)
        self.declare_parameter('twist_covariance_angular_z', 0.20)

        self.wheel_radius = (
            self.get_parameter('wheel_radius').get_parameter_value().double_value
        )
        self.wheel_track = (
            self.get_parameter('wheel_track').get_parameter_value().double_value
        )
        self.ticks_per_revolution = (
            self.get_parameter('ticks_per_revolution')
            .get_parameter_value()
            .integer_value
        )
        self.max_tick_jump = (
            self.get_parameter('max_tick_jump').get_parameter_value().integer_value
        )
        publish_rate_hz = (
            self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        )
        pose_covariance_x = (
            self.get_parameter('pose_covariance_x').get_parameter_value().double_value
        )
        pose_covariance_y = (
            self.get_parameter('pose_covariance_y').get_parameter_value().double_value
        )
        pose_covariance_yaw = (
            self.get_parameter('pose_covariance_yaw').get_parameter_value().double_value
        )
        twist_covariance_linear_x = (
            self.get_parameter('twist_covariance_linear_x')
            .get_parameter_value()
            .double_value
        )
        twist_covariance_angular_z = (
            self.get_parameter('twist_covariance_angular_z')
            .get_parameter_value()
            .double_value
        )

        # PLACEHOLDER covariance values for future robot_localization/EKF use.
        # Z/roll/pitch are set high because this planar wheel odometry source
        # does not measure them. Tune all values after real encoder/slip tests.
        self.pose_covariance = diagonal_covariance(
            pose_covariance_x,
            pose_covariance_y,
            999.0,
            999.0,
            999.0,
            pose_covariance_yaw,
        )
        self.twist_covariance = diagonal_covariance(
            twist_covariance_linear_x,
            999.0,
            999.0,
            999.0,
            999.0,
            twist_covariance_angular_z,
        )

        self.odometry = DifferentialDriveOdometry(
            wheel_radius=self.wheel_radius,
            wheel_track=self.wheel_track,
            ticks_per_revolution=self.ticks_per_revolution,
            max_tick_jump=self.max_tick_jump,
        )

        self.latest_left_ticks = None
        self.latest_right_ticks = None
        self.warned_waiting_for_ticks = False

        self.create_subscription(
            Int64,
            '/left_wheel_ticks',
            self.left_tick_callback,
            10,
        )
        self.create_subscription(
            Int64,
            '/right_wheel_ticks',
            self.right_tick_callback,
            10,
        )

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            1.0 / max(publish_rate_hz, 1.0),
            self.publish_odometry,
        )

        self.get_logger().info(
            'Wheel odometry node started with PLACEHOLDER parameters: '
            f'wheel_radius={self.wheel_radius:.3f} m, '
            f'wheel_track={self.wheel_track:.3f} m, '
            f'ticks_per_revolution={self.ticks_per_revolution}, '
            f'ticks_per_meter={self.odometry.ticks_per_meter:.3f}. '
            'Odometry covariance values are PLACEHOLDER values for future EKF tuning.'
        )

    def left_tick_callback(self, msg):
        self.latest_left_ticks = int(msg.data)

    def right_tick_callback(self, msg):
        self.latest_right_ticks = int(msg.data)

    def publish_odometry(self):
        if self.latest_left_ticks is None or self.latest_right_ticks is None:
            if not self.warned_waiting_for_ticks:
                self.get_logger().warn(
                    'Waiting for both /left_wheel_ticks and /right_wheel_ticks'
                )
                self.warned_waiting_for_ticks = True
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        state, status = self.odometry.update(
            self.latest_left_ticks,
            self.latest_right_ticks,
            now_sec,
        )

        if status == 'initialized':
            self.get_logger().info('Initialized wheel odometry from first tick pair')
        elif status == 'ignored_non_positive_dt':
            self.get_logger().warn('Ignored odometry update with dt <= 0')
            return
        elif status == 'ignored_unreasonable_tick_jump':
            self.get_logger().warn(
                'Ignored unreasonable encoder tick jump; odometry state preserved'
            )
            return

        qx, qy, qz, qw = yaw_to_quaternion(state.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = state.x
        odom_msg.pose.pose.position.y = state.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.covariance = self.pose_covariance
        odom_msg.twist.twist.linear.x = state.linear_velocity
        odom_msg.twist.twist.angular.z = state.angular_velocity
        odom_msg.twist.covariance = self.twist_covariance
        self.odom_publisher.publish(odom_msg)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = state.x
        transform.transform.translation.y = state.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()

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
