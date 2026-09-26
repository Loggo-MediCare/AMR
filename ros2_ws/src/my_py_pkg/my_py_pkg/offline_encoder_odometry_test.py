from my_py_pkg.wheel_odometry_math import DifferentialDriveOdometry


def run_case(name, samples, expectation):
    odom = DifferentialDriveOdometry(
        wheel_radius=0.05,          # PLACEHOLDER
        wheel_track=0.24,           # PLACEHOLDER
        ticks_per_revolution=600,   # PLACEHOLDER
        max_tick_jump=100000,
    )
    time_sec = 0.0
    for left, right in samples:
        odom.update(left, right, time_sec)
        time_sec += 0.1

    state = odom.state
    print(
        f'{name}: x={state.x:.4f}, y={state.y:.4f}, '
        f'theta={state.theta:.4f} rad -- expected {expectation}'
    )


def main():
    print('Offline simulated encoder odometry tests')
    print('All physical constants are PLACEHOLDER values.')

    run_case(
        'stopped',
        [(0, 0), (0, 0), (0, 0)],
        'x/y/theta unchanged',
    )
    run_case(
        'straight_forward',
        [(0, 0), (120, 120), (240, 240), (360, 360)],
        'x increases, theta nearly unchanged',
    )
    run_case(
        'straight_reverse',
        [(0, 0), (-120, -120), (-240, -240)],
        'x decreases, theta nearly unchanged',
    )
    run_case(
        'rotate_left_in_place',
        [(0, 0), (-120, 120), (-240, 240)],
        'position nearly unchanged, theta increases',
    )
    run_case(
        'rotate_right_in_place',
        [(0, 0), (120, -120), (240, -240)],
        'position nearly unchanged, theta decreases',
    )
    run_case(
        'curved_motion',
        [(0, 0), (100, 200), (200, 400), (300, 600)],
        'x changes and theta changes',
    )


if __name__ == '__main__':
    main()
