import math


def build_model(sample_period_sec):
    """Build the discrete yaw double-integrator model.

    State:
      x = [yaw, yaw_rate]

    Units:
      yaw: rad
      yaw_rate: rad/s
      yaw_acceleration input: rad/s^2
      sample_period_sec: seconds/sample

    This is an educational timing model, not complete AMR dynamics.
    """
    t = float(sample_period_sec)
    return (
        ((1.0, t), (0.0, 1.0)),
        (0.5 * t * t, t),
    )


def mat_vec_mul(matrix, vector):
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1],
    )


def mat_mul(left, right):
    return (
        (
            left[0][0] * right[0][0] + left[0][1] * right[1][0],
            left[0][0] * right[0][1] + left[0][1] * right[1][1],
        ),
        (
            left[1][0] * right[0][0] + left[1][1] * right[1][0],
            left[1][0] * right[0][1] + left[1][1] * right[1][1],
        ),
    )


def mat_pow(matrix, exponent):
    result = ((1.0, 0.0), (0.0, 1.0))
    base = matrix
    power = int(exponent)
    while power > 0:
        if power % 2 == 1:
            result = mat_mul(result, base)
        base = mat_mul(base, base)
        power //= 2
    return result


def vec_add(left, right):
    return (left[0] + right[0], left[1] + right[1])


def vec_sub(left, right):
    return (left[0] - right[0], left[1] - right[1])


def vec_scale(vector, scalar):
    return (vector[0] * scalar, vector[1] * scalar)


def generate_yaw_acceleration(time_sec):
    """Deterministic yaw-acceleration profile in rad/s^2."""
    if 1.0 <= time_sec < 2.0:
        return 0.5
    if 3.0 <= time_sec < 4.0:
        return -0.5
    return 0.0


def generate_inputs(sample_period_sec, duration_sec):
    sample_count = int(round(duration_sec / sample_period_sec)) + 1
    return [
        generate_yaw_acceleration(index * sample_period_sec)
        for index in range(sample_count)
    ]


def step_state(model_a, model_b, state, yaw_acceleration):
    return vec_add(mat_vec_mul(model_a, state), vec_scale(model_b, yaw_acceleration))


def simulate_true_state(model_a, model_b, yaw_acceleration):
    states = [(0.0, 0.0)]
    for input_value in yaw_acceleration[:-1]:
        states.append(step_state(model_a, model_b, states[-1], input_value))
    return states


def quantize_delay(delay_sec, sample_period_sec):
    delay_samples = int(round(delay_sec / sample_period_sec))
    effective_delay_sec = delay_samples * sample_period_sec
    return delay_samples, effective_delay_sec


def get_known_input_history(inputs, current_index, delay_samples, horizon_samples=None):
    """Return causal input history.

    Online at sample k, known inputs from the delayed state are:
      inputs[k-delay_samples : k]

    For h < d, use only the first h known samples:
      inputs[k-delay_samples : k-delay_samples+h]
    """
    if horizon_samples is None:
        horizon_samples = delay_samples
    if horizon_samples > delay_samples:
        raise ValueError('causal history cannot use horizon_samples > delay_samples')

    start = current_index - delay_samples
    end = start + horizon_samples
    return inputs[start:end]


def get_oracle_input_history(inputs, current_index, delay_samples, horizon_samples):
    """Return offline-only input history that may include future inputs.

    If horizon_samples > delay_samples, this includes inputs after current_index
    and is therefore a NON-CAUSAL ORACLE REFERENCE, not deployable online logic.
    """
    start = current_index - delay_samples
    end = start + horizon_samples
    return inputs[start:end]


def predict_recursive(model_a, model_b, delayed_state, input_history):
    prediction = delayed_state
    for input_value in input_history:
        prediction = step_state(model_a, model_b, prediction, input_value)
    return prediction


def predict_closed_form(model_a, model_b, delayed_state, input_history):
    horizon = len(input_history)
    prediction = mat_vec_mul(mat_pow(model_a, horizon), delayed_state)
    for index, input_value in enumerate(input_history):
        power = horizon - 1 - index
        propagated_input = mat_vec_mul(mat_pow(model_a, power), model_b)
        prediction = vec_add(prediction, vec_scale(propagated_input, input_value))
    return prediction


def rmse(values):
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def spatial_error(velocity_mps, timestamp_error_sec):
    return velocity_mps * timestamp_error_sec


def run_experiment(
    sample_rate_hz=200.0,
    delay_sec=0.040,
    horizon_sec=0.040,
    duration_sec=5.0,
    oracle=False,
):
    sample_period_sec = 1.0 / sample_rate_hz
    model_a, model_b = build_model(sample_period_sec)
    inputs = generate_inputs(sample_period_sec, duration_sec)
    states = simulate_true_state(model_a, model_b, inputs)

    delay_samples, effective_delay_sec = quantize_delay(delay_sec, sample_period_sec)
    horizon_samples = int(round(horizon_sec / sample_period_sec))
    effective_horizon_sec = horizon_samples * sample_period_sec

    if not oracle and horizon_samples > delay_samples:
        raise ValueError('h > d requires NON-CAUSAL ORACLE mode')

    max_horizon = max(delay_samples, horizon_samples)
    start_index = delay_samples
    end_index = len(states) - max(0, horizon_samples - delay_samples)

    yaw_model_errors = []
    yaw_current_state_errors = []
    yaw_rate_model_errors = []
    yaw_rate_current_state_errors = []
    recursive_closed_form_diffs = []
    records = []

    for current_index in range(start_index, end_index):
        delayed_index = current_index - delay_samples
        target_index = delayed_index + horizon_samples
        delayed_state = states[delayed_index]

        if oracle:
            input_history = get_oracle_input_history(
                inputs,
                current_index,
                delay_samples,
                horizon_samples,
            )
        else:
            input_history = get_known_input_history(
                inputs,
                current_index,
                delay_samples,
                horizon_samples,
            )

        if len(input_history) != horizon_samples:
            continue

        recursive_prediction = predict_recursive(
            model_a,
            model_b,
            delayed_state,
            input_history,
        )
        closed_form_prediction = predict_closed_form(
            model_a,
            model_b,
            delayed_state,
            input_history,
        )

        model_error = vec_sub(recursive_prediction, states[target_index])
        current_state_error = vec_sub(recursive_prediction, states[current_index])
        closed_form_diff = vec_sub(recursive_prediction, closed_form_prediction)

        yaw_model_errors.append(model_error[0])
        yaw_rate_model_errors.append(model_error[1])
        yaw_current_state_errors.append(current_state_error[0])
        yaw_rate_current_state_errors.append(current_state_error[1])
        recursive_closed_form_diffs.append(
            math.hypot(closed_form_diff[0], closed_form_diff[1])
        )
        records.append(
            {
                'current_index': current_index,
                'delayed_index': delayed_index,
                'target_index': target_index,
                'delayed_state': delayed_state,
                'predicted_state': recursive_prediction,
                'closed_form_state': closed_form_prediction,
                'true_current_state': states[current_index],
                'true_target_state': states[target_index],
                'input_history_start': delayed_index,
                'input_history_end': delayed_index + horizon_samples - 1,
                'input_history_length': horizon_samples,
                'model_error': model_error,
                'current_state_error': current_state_error,
                'closed_form_diff': closed_form_diff,
            }
        )

    return {
        'sample_rate_hz': sample_rate_hz,
        'sample_period_sec': sample_period_sec,
        'requested_delay_sec': delay_sec,
        'delay_samples': delay_samples,
        'effective_delay_sec': effective_delay_sec,
        'requested_horizon_sec': horizon_sec,
        'horizon_samples': horizon_samples,
        'effective_horizon_sec': effective_horizon_sec,
        'oracle': oracle,
        'yaw_model_rmse': rmse(yaw_model_errors),
        'yaw_current_state_rmse': rmse(yaw_current_state_errors),
        'yaw_rate_model_rmse': rmse(yaw_rate_model_errors),
        'yaw_rate_current_state_rmse': rmse(yaw_rate_current_state_errors),
        'max_recursive_closed_form_diff': max(recursive_closed_form_diffs or [0.0]),
        'records': records,
    }


def print_index_walkthrough(result, preferred_index=100):
    record = next(
        (
            item
            for item in result['records']
            if item['current_index'] == preferred_index
        ),
        result['records'][0],
    )
    print('Index walk-through:')
    print(f"  current index k:       {record['current_index']}")
    print(f"  delayed index k-d:     {record['delayed_index']}")
    print(f"  input history start:   {record['input_history_start']}")
    print(f"  input history end:     {record['input_history_end']}")
    print(f"  history length:        {record['input_history_length']}")
    print(f"  prediction horizon h:  {result['horizon_samples']}")
    print(f"  target index k-d+h:    {record['target_index']}")
    print(f"  delayed state:         {record['delayed_state']}")
    print(f"  predicted state:       {record['predicted_state']}")
    print(f"  true target state:     {record['true_target_state']}")
    print(f"  true current state:    {record['true_current_state']}")
    print(f"  model error:           {record['model_error']}")
    print(f"  current-state error:   {record['current_state_error']}")
    print(f"  recursive-closed diff: {record['closed_form_diff']}")


def print_result(label, result):
    mode = 'NON-CAUSAL ORACLE REFERENCE' if result['oracle'] else 'causal'
    print(f'{label} ({mode})')
    print(f"  Sample period: {result['sample_period_sec'] * 1000.0:.3f} ms")
    print(f"  Requested delay: {result['requested_delay_sec'] * 1000.0:.3f} ms")
    print(f"  Delay samples: {result['delay_samples']}")
    print(f"  Effective delay: {result['effective_delay_sec'] * 1000.0:.3f} ms")
    print(f"  Requested horizon: {result['requested_horizon_sec'] * 1000.0:.3f} ms")
    print(f"  Horizon samples: {result['horizon_samples']}")
    print(f"  Effective horizon: {result['effective_horizon_sec'] * 1000.0:.3f} ms")
    print(f"  yaw model RMSE: {result['yaw_model_rmse']:.12e} rad")
    print(
        '  yaw current-state RMSE: '
        f"{result['yaw_current_state_rmse']:.12e} rad"
    )
    print(
        '  yaw_rate model RMSE: '
        f"{result['yaw_rate_model_rmse']:.12e} rad/s"
    )
    print(
        '  yaw_rate current-state RMSE: '
        f"{result['yaw_rate_current_state_rmse']:.12e} rad/s"
    )
    print(
        '  max recursive-vs-closed-form diff: '
        f"{result['max_recursive_closed_form_diff']:.12e}"
    )


def main():
    delay_sec = 0.040
    sample_rate_hz = 200.0

    print('IMU delay predictor reference simulation')
    print('State x = [yaw(rad), yaw_rate(rad/s)], input = yaw_acceleration(rad/s^2)')
    print()

    causal_short = run_experiment(
        sample_rate_hz=sample_rate_hz,
        delay_sec=delay_sec,
        horizon_sec=0.020,
    )
    causal_matched = run_experiment(
        sample_rate_hz=sample_rate_hz,
        delay_sec=delay_sec,
        horizon_sec=0.040,
    )
    oracle_long = run_experiment(
        sample_rate_hz=sample_rate_hz,
        delay_sec=delay_sec,
        horizon_sec=0.060,
        oracle=True,
    )

    print_result('Predictor horizon 20 ms (h < d)', causal_short)
    print()
    print_result('Predictor horizon 40 ms (h = d)', causal_matched)
    print()
    print_result('Predictor horizon 60 ms (h > d)', oracle_long)
    print()
    print_index_walkthrough(causal_matched, preferred_index=100)
    print()
    print('Timing-to-spatial-error example:')
    print('  AMR speed: 0.5 m/s')
    print('  timestamp error: 40.0 ms')
    print(f'  spatial error: {spatial_error(0.5, 0.040):.3f} m')


if __name__ == '__main__':
    main()
