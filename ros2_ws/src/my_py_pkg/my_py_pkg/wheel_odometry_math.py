import math
from dataclasses import dataclass


INT32_MODULUS = 2 ** 32
INT32_HALF_RANGE = 2 ** 31


def signed_tick_delta(new_ticks, old_ticks):
    """Return signed delta while tolerating signed 32-bit counter rollover."""
    raw_delta = (int(new_ticks) - int(old_ticks)) % INT32_MODULUS
    if raw_delta >= INT32_HALF_RANGE:
        raw_delta -= INT32_MODULUS
    return raw_delta


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class OdometryState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    delta_left_m: float = 0.0
    delta_right_m: float = 0.0
    delta_distance_m: float = 0.0
    delta_theta_rad: float = 0.0


class DifferentialDriveOdometry:
    """
    Differential-drive odometry using the basic O'Reilly-style approximation:

        x += delta_distance * cos(theta)
        y += delta_distance * sin(theta)
        theta += delta_theta

    This intentionally does not use midpoint/RK integration yet, so the math
    stays directly traceable to the source material for this milestone.
    """

    def __init__(
        self,
        wheel_radius,
        wheel_track,
        ticks_per_revolution,
        max_tick_jump,
    ):
        if wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be > 0')
        if wheel_track <= 0.0:
            raise ValueError('wheel_track must be > 0')
        if ticks_per_revolution <= 0:
            raise ValueError('ticks_per_revolution must be > 0')

        self.wheel_radius = float(wheel_radius)
        self.wheel_track = float(wheel_track)
        self.ticks_per_revolution = int(ticks_per_revolution)
        self.max_tick_jump = int(max_tick_jump)
        self.ticks_per_meter = (
            self.ticks_per_revolution / (2.0 * math.pi * self.wheel_radius)
        )

        self.state = OdometryState()
        self.previous_left_ticks = None
        self.previous_right_ticks = None
        self.previous_time_sec = None

    def update(self, left_ticks, right_ticks, now_sec):
        now_sec = float(now_sec)
        if self.previous_left_ticks is None:
            self.previous_left_ticks = int(left_ticks)
            self.previous_right_ticks = int(right_ticks)
            self.previous_time_sec = now_sec
            return self.state, 'initialized'

        dt = now_sec - self.previous_time_sec
        if dt <= 0.0:
            return self.state, 'ignored_non_positive_dt'

        delta_left_ticks = signed_tick_delta(
            int(left_ticks),
            self.previous_left_ticks,
        )
        delta_right_ticks = signed_tick_delta(
            int(right_ticks),
            self.previous_right_ticks,
        )

        if (
            abs(delta_left_ticks) > self.max_tick_jump
            or abs(delta_right_ticks) > self.max_tick_jump
        ):
            return self.state, 'ignored_unreasonable_tick_jump'

        delta_left = delta_left_ticks / self.ticks_per_meter
        delta_right = delta_right_ticks / self.ticks_per_meter
        delta_distance = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_track

        theta_before = self.state.theta
        self.state.x += delta_distance * math.cos(theta_before)
        self.state.y += delta_distance * math.sin(theta_before)
        self.state.theta = normalize_angle(theta_before + delta_theta)
        self.state.linear_velocity = delta_distance / dt
        self.state.angular_velocity = delta_theta / dt
        self.state.delta_left_m = delta_left
        self.state.delta_right_m = delta_right
        self.state.delta_distance_m = delta_distance
        self.state.delta_theta_rad = delta_theta

        self.previous_left_ticks = int(left_ticks)
        self.previous_right_ticks = int(right_ticks)
        self.previous_time_sec = now_sec

        return self.state, 'updated'
