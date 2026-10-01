"""Dynamic safety-field reference model (OFFLINE, dependency-light).

NOT production ROS code.
NOT certified safety code.
Does not modify, replace, or interface with any ROS package.

Purpose
-------
Model how robot speed, system latency, braking capability, and an explicit
uncertainty margin affect a minimum stopping distance, and classify an
obstacle distance into one of three conceptual safety states:

    NORMAL  -- obstacle is outside the slowdown boundary
    SLOW    -- obstacle is inside the slowdown boundary but outside STOP
    STOP    -- obstacle is at or inside the minimum stopping distance

ENGINEERING REFERENCE MODEL -- NOT A CERTIFIED SAFETY FORMULA.

This prototype is not safety-certified. The N20 drivetrain used on this AMR
has no mechanical fail-safe brake. All numeric inputs used here (speed,
latency, braking deceleration, uncertainty margin, slowdown multiplier) are
explicit parameters supplied by the caller -- this module never invents or
assumes a final project calibration value.
"""

from __future__ import annotations

NORMAL = "NORMAL"
SLOW = "SLOW"
STOP = "STOP"


def calculate_reaction_distance(robot_speed_mps: float, total_latency_s: float) -> float:
    """reaction_distance = abs(robot_speed_mps) * total_latency_s

    total_latency_s may conceptually bundle sensor latency, compute latency,
    control latency, and actuator/brake response latency. This function does
    not decompose it -- callers supply the already-summed total.
    """
    speed = abs(robot_speed_mps)
    if total_latency_s < 0:
        raise ValueError("total_latency_s must be >= 0")
    return speed * total_latency_s


def calculate_braking_distance(robot_speed_mps: float, braking_deceleration_mps2: float) -> float:
    """braking_distance = robot_speed_mps^2 / (2 * braking_deceleration_mps2)"""
    speed = abs(robot_speed_mps)
    if braking_deceleration_mps2 <= 0:
        raise ValueError("braking_deceleration_mps2 must be > 0")
    return (speed ** 2) / (2.0 * braking_deceleration_mps2)


def calculate_minimum_stop_distance(
    robot_speed_mps: float,
    total_latency_s: float,
    braking_deceleration_mps2: float,
    uncertainty_margin_m: float,
) -> float:
    """minimum_stop_distance = reaction_distance + braking_distance + uncertainty_margin_m"""
    if uncertainty_margin_m < 0:
        raise ValueError("uncertainty_margin_m must be >= 0")
    reaction = calculate_reaction_distance(robot_speed_mps, total_latency_s)
    braking = calculate_braking_distance(robot_speed_mps, braking_deceleration_mps2)
    return reaction + braking + uncertainty_margin_m


def classify_safety_state(
    obstacle_distance_m: float,
    robot_speed_mps: float,
    total_latency_s: float,
    braking_deceleration_mps2: float,
    uncertainty_margin_m: float,
    slowdown_multiplier: float,
) -> str:
    """Classify obstacle_distance_m into NORMAL / SLOW / STOP.

    STOP:   obstacle_distance_m <= minimum_stop_distance
    SLOW:   minimum_stop_distance < obstacle_distance_m <= minimum_stop_distance * slowdown_multiplier
    NORMAL: obstacle_distance_m > minimum_stop_distance * slowdown_multiplier

    slowdown_multiplier is an explicit required parameter, not a hidden
    constant, so callers must decide and justify their own slowdown margin.
    """
    if obstacle_distance_m < 0:
        raise ValueError("obstacle_distance_m must be >= 0")
    if slowdown_multiplier < 1:
        raise ValueError("slowdown_multiplier must be >= 1")

    stop_distance = calculate_minimum_stop_distance(
        robot_speed_mps, total_latency_s, braking_deceleration_mps2, uncertainty_margin_m
    )
    slow_boundary = stop_distance * slowdown_multiplier

    if obstacle_distance_m <= stop_distance:
        return STOP
    if obstacle_distance_m <= slow_boundary:
        return SLOW
    return NORMAL


def _run_self_tests() -> None:
    """Lightweight assertion-based self-tests. Not a full test suite."""

    # Zero speed: no reaction/braking distance, only the uncertainty margin.
    assert calculate_reaction_distance(0.0, 0.5) == 0.0
    assert calculate_braking_distance(0.0, 1.0) == 0.0
    assert calculate_minimum_stop_distance(0.0, 0.5, 1.0, 0.2) == 0.2

    # Obstacle inside STOP region.
    state = classify_safety_state(
        obstacle_distance_m=0.1,
        robot_speed_mps=0.5,
        total_latency_s=0.1,
        braking_deceleration_mps2=1.0,
        uncertainty_margin_m=0.1,
        slowdown_multiplier=2.0,
    )
    assert state == STOP, state

    # Obstacle inside SLOW region (between stop_distance and slow_boundary).
    stop_distance = calculate_minimum_stop_distance(0.5, 0.1, 1.0, 0.1)
    state = classify_safety_state(
        obstacle_distance_m=stop_distance * 1.5,
        robot_speed_mps=0.5,
        total_latency_s=0.1,
        braking_deceleration_mps2=1.0,
        uncertainty_margin_m=0.1,
        slowdown_multiplier=2.0,
    )
    assert state == SLOW, state

    # Obstacle in NORMAL region (well beyond the slow boundary).
    state = classify_safety_state(
        obstacle_distance_m=stop_distance * 10.0,
        robot_speed_mps=0.5,
        total_latency_s=0.1,
        braking_deceleration_mps2=1.0,
        uncertainty_margin_m=0.1,
        slowdown_multiplier=2.0,
    )
    assert state == NORMAL, state

    # Faster speed increases stopping distance.
    slow_speed_stop = calculate_minimum_stop_distance(0.2, 0.1, 1.0, 0.1)
    fast_speed_stop = calculate_minimum_stop_distance(1.0, 0.1, 1.0, 0.1)
    assert fast_speed_stop > slow_speed_stop

    # Longer latency increases stopping distance.
    short_latency_stop = calculate_minimum_stop_distance(0.5, 0.05, 1.0, 0.1)
    long_latency_stop = calculate_minimum_stop_distance(0.5, 0.5, 1.0, 0.1)
    assert long_latency_stop > short_latency_stop

    # Weaker braking (smaller deceleration) increases stopping distance.
    strong_braking_stop = calculate_minimum_stop_distance(0.5, 0.1, 5.0, 0.1)
    weak_braking_stop = calculate_minimum_stop_distance(0.5, 0.1, 0.5, 0.1)
    assert weak_braking_stop > strong_braking_stop

    # Invalid braking deceleration raises an error.
    try:
        calculate_braking_distance(0.5, 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for braking_deceleration_mps2 <= 0")

    try:
        calculate_braking_distance(0.5, -1.0)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for negative braking_deceleration_mps2")

    print("All self-tests passed.")


def _run_demo() -> None:
    print("DEMO ONLY -- NOT AMR CALIBRATION VALUES")
    print()

    # DEMO ONLY -- NOT AMR CALIBRATION VALUES
    demo_robot_speed_mps = 0.5
    demo_total_latency_s = 0.15
    demo_braking_deceleration_mps2 = 1.5
    demo_uncertainty_margin_m = 0.15
    demo_slowdown_multiplier = 2.0
    demo_obstacle_distance_m = 1.0

    reaction = calculate_reaction_distance(demo_robot_speed_mps, demo_total_latency_s)
    braking = calculate_braking_distance(demo_robot_speed_mps, demo_braking_deceleration_mps2)
    stop_distance = calculate_minimum_stop_distance(
        demo_robot_speed_mps,
        demo_total_latency_s,
        demo_braking_deceleration_mps2,
        demo_uncertainty_margin_m,
    )
    slow_boundary = stop_distance * demo_slowdown_multiplier
    state = classify_safety_state(
        demo_obstacle_distance_m,
        demo_robot_speed_mps,
        demo_total_latency_s,
        demo_braking_deceleration_mps2,
        demo_uncertainty_margin_m,
        demo_slowdown_multiplier,
    )

    print(f"robot speed (m/s):            {demo_robot_speed_mps}")
    print(f"reaction distance (m):        {reaction:.4f}")
    print(f"braking distance (m):         {braking:.4f}")
    print(f"minimum stopping distance (m):{stop_distance:.4f}")
    print(f"slowdown boundary (m):        {slow_boundary:.4f}")
    print(f"obstacle distance (m):        {demo_obstacle_distance_m}")
    print(f"resulting state:              {state}")


if __name__ == "__main__":
    _run_self_tests()
    print()
    _run_demo()
