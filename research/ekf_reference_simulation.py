import math
import random


def mat_transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def mat_mul(left, right):
    rows = len(left)
    cols = len(right[0])
    inner = len(right)
    return [
        [
            sum(left[row][index] * right[index][col] for index in range(inner))
            for col in range(cols)
        ]
        for row in range(rows)
    ]


def mat_add(left, right):
    return [
        [left[row][col] + right[row][col] for col in range(len(left[0]))]
        for row in range(len(left))
    ]


def mat_sub(left, right):
    return [
        [left[row][col] - right[row][col] for col in range(len(left[0]))]
        for row in range(len(left))
    ]


def mat_vec_mul(matrix, vector):
    return [
        sum(matrix[row][col] * vector[col] for col in range(len(vector)))
        for row in range(len(matrix))
    ]


def identity(size):
    return [
        [1.0 if row == col else 0.0 for col in range(size)]
        for row in range(size)
    ]


def inverse_2x2(matrix):
    a = matrix[0][0]
    b = matrix[0][1]
    c = matrix[1][0]
    d = matrix[1][1]
    determinant = a * d - b * c
    if abs(determinant) < 1e-15:
        raise ValueError('2x2 matrix is singular')
    inv_det = 1.0 / determinant
    return [
        [d * inv_det, -b * inv_det],
        [-c * inv_det, a * inv_det],
    ]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rmse(errors):
    if not errors:
        return 0.0
    return math.sqrt(sum(error * error for error in errors) / len(errors))


def covariance_diag(matrix):
    return [matrix[index][index] for index in range(len(matrix))]


def matrix_is_symmetric(matrix, tolerance=1e-10):
    for row in range(len(matrix)):
        for col in range(len(matrix)):
            if abs(matrix[row][col] - matrix[col][row]) > tolerance:
                return False
    return True


def matrix_has_valid_variances(matrix, tolerance=1e-12):
    return all(matrix[index][index] >= -tolerance for index in range(len(matrix)))


def matrix_has_finite_values(matrix):
    return all(math.isfinite(value) for row in matrix for value in row)


def measurement_model(state, sensor_offset_x=0.0, sensor_offset_y=0.0):
    x, y, yaw = state
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return [
        x + sensor_offset_x * cos_yaw - sensor_offset_y * sin_yaw,
        y + sensor_offset_x * sin_yaw + sensor_offset_y * cos_yaw,
    ]


def measurement_jacobian(state, sensor_offset_x=0.0, sensor_offset_y=0.0):
    yaw = state[2]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return [
        [
            1.0,
            0.0,
            -sensor_offset_x * sin_yaw - sensor_offset_y * cos_yaw,
        ],
        [
            0.0,
            1.0,
            sensor_offset_x * cos_yaw - sensor_offset_y * sin_yaw,
        ],
    ]


def wheel_to_motion_jacobian(wheel_track):
    return [
        [0.5, 0.5],
        [-1.0 / wheel_track, 1.0 / wheel_track],
    ]


def motion_to_state_jacobian(yaw):
    return [
        [math.cos(yaw), 0.0],
        [math.sin(yaw), 0.0],
        [0.0, 1.0],
    ]


def q_wheel_to_state(q_wheel, yaw, wheel_track):
    wheel_to_motion = wheel_to_motion_jacobian(wheel_track)
    q_motion = mat_mul(mat_mul(wheel_to_motion, q_wheel), mat_transpose(wheel_to_motion))
    g_motion = motion_to_state_jacobian(yaw)
    return mat_mul(mat_mul(g_motion, q_motion), mat_transpose(g_motion))


def q_motion_to_state(q_motion, yaw):
    g_motion = motion_to_state_jacobian(yaw)
    return mat_mul(mat_mul(g_motion, q_motion), mat_transpose(g_motion))


class DifferentialDriveEKF:
    """
    Offline EKF reference for Chapter-8-style wheeled mobile robot localization.

    State:
      x = [position_x, position_y, yaw]

    Measurement:
      synthetic external position sensor, optionally offset from the robot
      center of rotation.

    This is a research/reference simulation only. It is not robot_localization,
    does not publish ROS topics, and does not represent the current raw IMU
    measurement model.
    """

    def __init__(
        self,
        wheel_track,
        q_mode,
        q_parameters,
        r_measurement,
        sensor_offset_x=0.0,
        sensor_offset_y=0.0,
        initial_state=None,
        initial_covariance=None,
        use_joseph_update=True,
    ):
        self.wheel_track = wheel_track
        self.q_mode = q_mode
        self.q_parameters = q_parameters
        self.r_measurement = r_measurement
        self.sensor_offset_x = sensor_offset_x
        self.sensor_offset_y = sensor_offset_y
        self.use_joseph_update = use_joseph_update
        self.state = list(initial_state or [0.0, 0.0, 0.0])
        self.covariance = initial_covariance or [
            [1e-6, 0.0, 0.0],
            [0.0, 1e-6, 0.0],
            [0.0, 0.0, 1e-6],
        ]
        self.last_a = identity(3)
        self.last_q_state = [[0.0, 0.0, 0.0] for _ in range(3)]
        self.last_c = measurement_jacobian(
            self.state,
            self.sensor_offset_x,
            self.sensor_offset_y,
        )

    def process_noise_state(self, delta_left, delta_right, yaw, dt):
        if self.q_mode == 'wheel_constant':
            sigma_left = self.q_parameters['sigma_left']
            sigma_right = self.q_parameters['sigma_right']
            q_wheel = [
                [sigma_left * sigma_left, 0.0],
                [0.0, sigma_right * sigma_right],
            ]
            return q_wheel_to_state(q_wheel, yaw, self.wheel_track)

        if self.q_mode == 'motion_constant':
            sigma_delta_d = self.q_parameters['sigma_delta_d']
            sigma_delta_yaw = self.q_parameters['sigma_delta_yaw']
            q_motion = [
                [sigma_delta_d * sigma_delta_d, 0.0],
                [0.0, sigma_delta_yaw * sigma_delta_yaw],
            ]
            return q_motion_to_state(q_motion, yaw)

        if self.q_mode == 'wheel_travel_dependent':
            scale = self.q_parameters['delta_noise_scale']
            q_wheel = [
                [scale * abs(delta_left), 0.0],
                [0.0, scale * abs(delta_right)],
            ]
            return q_wheel_to_state(q_wheel, yaw, self.wheel_track)

        if self.q_mode == 'wheel_speed_dependent':
            scale = self.q_parameters['speed_noise_scale']
            radius = self.q_parameters['wheel_radius']
            omega_left = delta_left / (radius * dt)
            omega_right = delta_right / (radius * dt)
            q_wheel = [
                [scale * abs(omega_left), 0.0],
                [0.0, scale * abs(omega_right)],
            ]
            return q_wheel_to_state(q_wheel, yaw, self.wheel_track)

        raise ValueError(f'unknown q_mode: {self.q_mode}')

    def predict(self, delta_left, delta_right, dt):
        x, y, yaw = self.state
        delta_distance = (delta_right + delta_left) / 2.0
        delta_yaw = (delta_right - delta_left) / self.wheel_track

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        predicted_state = [
            x + delta_distance * cos_yaw,
            y + delta_distance * sin_yaw,
            wrap_angle(yaw + delta_yaw),
        ]

        self.last_a = [
            [1.0, 0.0, -delta_distance * sin_yaw],
            [0.0, 1.0, delta_distance * cos_yaw],
            [0.0, 0.0, 1.0],
        ]
        self.last_q_state = self.process_noise_state(
            delta_left,
            delta_right,
            yaw,
            dt,
        )

        a_p = mat_mul(self.last_a, self.covariance)
        a_p_at = mat_mul(a_p, mat_transpose(self.last_a))

        self.state = predicted_state
        self.covariance = mat_add(a_p_at, self.last_q_state)
        return self.state

    def correct(self, measurement_xy):
        c_matrix = measurement_jacobian(
            self.state,
            self.sensor_offset_x,
            self.sensor_offset_y,
        )
        self.last_c = c_matrix
        predicted_measurement = measurement_model(
            self.state,
            self.sensor_offset_x,
            self.sensor_offset_y,
        )
        innovation = [
            measurement_xy[0] - predicted_measurement[0],
            measurement_xy[1] - predicted_measurement[1],
        ]

        c_p = mat_mul(c_matrix, self.covariance)
        s_matrix = mat_add(
            mat_mul(c_p, mat_transpose(c_matrix)),
            self.r_measurement,
        )
        kalman_gain = mat_mul(
            mat_mul(self.covariance, mat_transpose(c_matrix)),
            inverse_2x2(s_matrix),
        )
        correction = mat_vec_mul(kalman_gain, innovation)
        self.state = [
            self.state[0] + correction[0],
            self.state[1] + correction[1],
            wrap_angle(self.state[2] + correction[2]),
        ]

        i_minus_kc = mat_sub(identity(3), mat_mul(kalman_gain, c_matrix))
        if self.use_joseph_update:
            # Joseph form preserves symmetry and positive semi-definiteness
            # better under floating-point arithmetic than P = (I-KH)P.
            left_term = mat_mul(
                mat_mul(i_minus_kc, self.covariance),
                mat_transpose(i_minus_kc),
            )
            right_term = mat_mul(
                mat_mul(kalman_gain, self.r_measurement),
                mat_transpose(kalman_gain),
            )
            self.covariance = mat_add(left_term, right_term)
        else:
            self.covariance = mat_mul(i_minus_kc, self.covariance)
        return self.state


def command_profile(time_sec, speed_scale=1.0):
    linear_velocity = speed_scale * (0.45 + 0.08 * math.sin(0.7 * time_sec))
    angular_velocity = speed_scale * 0.45 * math.sin(0.55 * time_sec)
    return linear_velocity, angular_velocity


def wheel_increments_from_motion(linear_velocity, angular_velocity, dt, wheel_track):
    delta_distance = linear_velocity * dt
    delta_yaw = angular_velocity * dt
    delta_left = delta_distance - (wheel_track * delta_yaw / 2.0)
    delta_right = delta_distance + (wheel_track * delta_yaw / 2.0)
    return delta_left, delta_right


def integrate_motion(state, delta_left, delta_right, wheel_track):
    x, y, yaw = state
    delta_distance = (delta_right + delta_left) / 2.0
    delta_yaw = (delta_right - delta_left) / wheel_track
    return [
        x + delta_distance * math.cos(yaw),
        y + delta_distance * math.sin(yaw),
        wrap_angle(yaw + delta_yaw),
    ]


def position_error(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


def generate_measurement(true_state, sensor_offset_x, sensor_offset_y, r_measurement):
    ideal = measurement_model(true_state, sensor_offset_x, sensor_offset_y)
    std_x = math.sqrt(r_measurement[0][0])
    std_y = math.sqrt(r_measurement[1][1])
    return [
        ideal[0] + random.gauss(0.0, std_x),
        ideal[1] + random.gauss(0.0, std_y),
    ]


def covariance_sanity(covariance_snapshots):
    return {
        'symmetric': all(matrix_is_symmetric(item) for item in covariance_snapshots),
        'non_negative_diagonal': all(
            matrix_has_valid_variances(item) for item in covariance_snapshots
        ),
        'finite': all(matrix_has_finite_values(item) for item in covariance_snapshots),
    }


def run_simulation(
    *,
    seed=42,
    q_mode='wheel_constant',
    q_parameters=None,
    r_measurement=None,
    sensor_offset_x=0.0,
    sensor_offset_y=0.0,
    speed_scale=1.0,
    use_joseph_update=True,
):
    random.seed(seed)

    duration_sec = 10.0
    dt = 0.02
    steps = int(duration_sec / dt)
    wheel_track = 0.24
    q_parameters = q_parameters or {
        'sigma_left': 0.003,
        'sigma_right': 0.003,
    }
    r_measurement = r_measurement or [
        [0.04 * 0.04, 0.0],
        [0.0, 0.04 * 0.04],
    ]
    true_wheel_noise_std = 0.003

    true_state = [0.0, 0.0, 0.0]
    odom_state = [0.0, 0.0, 0.0]
    ekf = DifferentialDriveEKF(
        wheel_track=wheel_track,
        q_mode=q_mode,
        q_parameters=q_parameters,
        r_measurement=r_measurement,
        sensor_offset_x=sensor_offset_x,
        sensor_offset_y=sensor_offset_y,
        use_joseph_update=use_joseph_update,
    )

    true_states = []
    odom_states = []
    ekf_states = []
    covariance_snapshots = [ekf.covariance]
    prediction_only_snapshots = []

    for step in range(steps):
        time_sec = step * dt
        linear_velocity, angular_velocity = command_profile(time_sec, speed_scale)
        true_left, true_right = wheel_increments_from_motion(
            linear_velocity,
            angular_velocity,
            dt,
            wheel_track,
        )

        true_state = integrate_motion(true_state, true_left, true_right, wheel_track)

        noisy_left = true_left + random.gauss(0.0, true_wheel_noise_std)
        noisy_right = true_right + random.gauss(0.0, true_wheel_noise_std)
        odom_state = integrate_motion(odom_state, noisy_left, noisy_right, wheel_track)

        ekf.predict(noisy_left, noisy_right, dt)
        prediction_only_snapshots.append([row[:] for row in ekf.covariance])

        measurement = generate_measurement(
            true_state,
            sensor_offset_x,
            sensor_offset_y,
            r_measurement,
        )
        ekf.correct(measurement)

        if step in (0, steps // 2, steps - 1):
            covariance_snapshots.append([row[:] for row in ekf.covariance])

        true_states.append(tuple(true_state))
        odom_states.append(tuple(odom_state))
        ekf_states.append(tuple(ekf.state))

    odom_position_errors = [
        position_error(odom, truth)
        for odom, truth in zip(odom_states, true_states)
    ]
    ekf_position_errors = [
        position_error(fused, truth)
        for fused, truth in zip(ekf_states, true_states)
    ]
    odom_yaw_errors = [
        wrap_angle(odom[2] - truth[2])
        for odom, truth in zip(odom_states, true_states)
    ]
    ekf_yaw_errors = [
        wrap_angle(fused[2] - truth[2])
        for fused, truth in zip(ekf_states, true_states)
    ]

    return {
        'ekf': ekf,
        'dt': dt,
        'duration_sec': duration_sec,
        'wheel_track': wheel_track,
        'q_mode': q_mode,
        'q_parameters': q_parameters,
        'r_measurement': r_measurement,
        'sensor_offset': (sensor_offset_x, sensor_offset_y),
        'odom_position_rmse': rmse(odom_position_errors),
        'ekf_position_rmse': rmse(ekf_position_errors),
        'odom_yaw_rmse': rmse(odom_yaw_errors),
        'ekf_yaw_rmse': rmse(ekf_yaw_errors),
        'final_true': true_states[-1],
        'final_odom': odom_states[-1],
        'final_ekf': ekf_states[-1],
        'initial_covariance_diag': covariance_diag(covariance_snapshots[0]),
        'mid_covariance_diag': covariance_diag(covariance_snapshots[2]),
        'final_covariance_diag': covariance_diag(covariance_snapshots[-1]),
        'last_prediction_covariance_diag': covariance_diag(prediction_only_snapshots[-1]),
        'sanity': covariance_sanity(covariance_snapshots + prediction_only_snapshots),
    }


def format_matrix(matrix):
    return '\n'.join(
        '  [' + ', '.join(f'{value: .6f}' for value in row) + ']'
        for row in matrix
    )


def print_result(label, result):
    print(label)
    print(f"  sensor offset: {result['sensor_offset']}")
    print(f"  Q mode: {result['q_mode']} {result['q_parameters']}")
    print(f"  R: {result['r_measurement']}")
    print(f"  Pure odometry position RMSE: {result['odom_position_rmse']:.4f} m")
    print(f"  EKF fused position RMSE:    {result['ekf_position_rmse']:.4f} m")
    print(f"  Pure odometry yaw RMSE:     {result['odom_yaw_rmse']:.4f} rad")
    print(f"  EKF fused yaw RMSE:         {result['ekf_yaw_rmse']:.4f} rad")
    print(f"  initial covariance diag:    {result['initial_covariance_diag']}")
    print(f"  mid-run covariance diag:    {result['mid_covariance_diag']}")
    print(f"  final covariance diag:      {result['final_covariance_diag']}")
    print(f"  last prediction-only diag:  {result['last_prediction_covariance_diag']}")


def main():
    centered = run_simulation()
    offset = run_simulation(sensor_offset_x=0.20, sensor_offset_y=0.05)
    motion_q = run_simulation(
        q_mode='motion_constant',
        q_parameters={'sigma_delta_d': 0.003, 'sigma_delta_yaw': 0.025},
    )
    travel_low = run_simulation(
        q_mode='wheel_travel_dependent',
        q_parameters={'delta_noise_scale': 1e-5},
        speed_scale=0.7,
    )
    travel_high = run_simulation(
        q_mode='wheel_travel_dependent',
        q_parameters={'delta_noise_scale': 1e-5},
        speed_scale=1.7,
    )

    equal_r = centered
    unequal_r = run_simulation(
        r_measurement=[[0.03 * 0.03, 0.0], [0.0, 0.09 * 0.09]],
    )
    correlated_r = run_simulation(
        r_measurement=[[0.04 * 0.04, 0.0010], [0.0010, 0.05 * 0.05]],
    )

    print('Offline EKF reference simulation')
    print('State: [x, y, yaw]')
    print('Measurement: SYNTHETIC EXTERNAL POSITION SENSOR')
    print('This is a book-reference lab, not production robot_localization config.')
    print()

    print('Centered measurement model C(k):')
    print(format_matrix(centered['ekf'].last_c))
    print()
    print('Offset measurement model C(k), sx=0.20 m, sy=0.05 m:')
    print(format_matrix(offset['ekf'].last_c))
    print('  non-zero yaw column shows yaw is observable through offset geometry')
    print()

    print_result('Centered sensor result', centered)
    print()
    print_result('Non-zero-offset sensor result', offset)
    print()
    print_result('Q Model A: wheel-space noise', centered)
    print()
    print_result('Q Model B: motion-space noise', motion_q)
    print()
    print('Speed/motion-dependent Q comparison')
    print(
        '  low speed final covariance diag:  '
        f"{travel_low['final_covariance_diag']}"
    )
    print(
        '  high speed final covariance diag: '
        f"{travel_high['final_covariance_diag']}"
    )
    print('  larger wheel motion increases covariance growth with this model')
    print()
    print('Measurement-noise R comparison')
    print(
        '  equal diagonal R:      '
        f"position RMSE={equal_r['ekf_position_rmse']:.4f}, "
        f"yaw RMSE={equal_r['ekf_yaw_rmse']:.4f}, "
        f"final diag={equal_r['final_covariance_diag']}"
    )
    print(
        '  unequal diagonal R:    '
        f"position RMSE={unequal_r['ekf_position_rmse']:.4f}, "
        f"yaw RMSE={unequal_r['ekf_yaw_rmse']:.4f}, "
        f"final diag={unequal_r['final_covariance_diag']}"
    )
    print(
        '  correlated full R:     '
        f"position RMSE={correlated_r['ekf_position_rmse']:.4f}, "
        f"yaw RMSE={correlated_r['ekf_yaw_rmse']:.4f}, "
        f"final diag={correlated_r['final_covariance_diag']}"
    )
    print()
    print('Covariance sanity checks')
    print(f"  symmetric: {centered['sanity']['symmetric']}")
    print(f"  non-negative diagonal: {centered['sanity']['non_negative_diagonal']}")
    print(f"  finite: {centered['sanity']['finite']}")
    print('  correction uses Joseph stabilized covariance update')


if __name__ == '__main__':
    main()
