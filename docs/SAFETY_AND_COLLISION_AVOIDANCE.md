# AMR Safety and Collision Avoidance Architecture

This document describes the intended safety architecture for the AMR
prototype. It builds on `docs/HARDWARE_BOM.md` (AMR Hardware Baseline v1)
and does not change any value defined there.

**This prototype is not safety-certified.** The N20 drivetrain has no
mechanical fail-safe brake. Nothing in this document should be read as a
certification claim.

## 1. Architectural Layers

### PHYSICAL SAFETY

- physical E-stop
- H-bridge ENABLE/STBY/SLEEP hardware disable path
- optional bumper/contact switch
- optional near-field ToF/ultrasonic sensing
- motor power isolation where appropriate

### LOW-LEVEL SAFETY

- MCU command watchdog
- stale-command detection
- motor output OFF on MCU boot/reset
- motor output OFF on communication loss
- motor output OFF on controller failure
- encoder plausibility checks
- sensor health monitoring

### ROBOTICS SOFTWARE

- desired cmd_vel
- Safety Supervisor
- safe cmd_vel
- MCU motor controller

### PERCEPTION

- stereo/RGB-D obstacle detection
- optional near-field sensors
- future sensor fusion

## 2. Conceptual Control Chain

```text
Nav2 / Teleop
    ↓
desired cmd_vel
    ↓
Safety Supervisor
    ↓
safe cmd_vel
    ↓
MCU
    ↓
Dual H-Bridge
    ↓
Left / Right motors
```

**ROS 2 is NOT the sole safety mechanism.** The MCU / low-level hardware
must independently stop the motors if commands become stale or the host
disappears, regardless of what the ROS 2 graph is doing.

## 3. Safety Supervisor State Machine

### NORMAL

- obstacle outside safety field
- valid command
- required sensors healthy
- normal velocity allowed

### SLOW

- obstacle enters warning / slowdown region
- reduce commanded velocity smoothly

### STOP

- obstacle enters critical stopping region
- command linear and angular velocity toward zero

### FAULT

- stale cmd_vel
- communication loss
- invalid required sensor data
- safety input active
- controller fault
- motor output must enter safe state

## 4. Three Distinct Stop Mechanisms

For this prototype, keep these concepts distinct — they are not
interchangeable and fail independently:

| Mechanism | Layer | What it stops |
|---|---|---|
| Motor command stop | Software (cmd_vel = 0) | Commanded motion only; relies on software being healthy |
| H-bridge hardware disable | Low-level hardware (ENABLE/STBY/SLEEP) | Motor output at the driver, independent of software state |
| Physical E-stop / power isolation | Physical | Removes power regardless of MCU or software state |

Do not assume a motor command stop is sufficient on its own — it depends on
the software issuing it still being correct and running.

## 5. Sensor Health and Plausibility Checks

Required classes of checks, with examples:

- stale timestamp
- impossible encoder jump
- missing camera frames
- missing IMU data
- stale cmd_vel
- MCU heartbeat loss

Spoofed or corrupted sensor/communication data can create false braking or
missed-obstacle conditions, so plausibility checking is a defensive layer,
not an optional nicety.

## 6. Cybersecurity

Cybersecurity hardening (authentication, integrity checking of
sensor/control links, protection against spoofed messages) is **FUTURE**
work for this prototype, not part of the current baseline.

## 7. Future Compliance Targets

This prototype is **not certified** against any safety standard today. For
reference, the industrial safety standards and framework that a future
production version of this design would need to be evaluated against
include:

- **ISO 10218** (industrial robot safety requirements)
- **ANSI/RIA R15.06** (industrial robot safety, US equivalent)
- **CE certification** (where applicable to the deployment market)
- A named design philosophy of **Awareness, Fault tolerance, and Explicit
  communication** as the three guiding principles for AMR safety design

Listing these is not a compliance claim — it records what an independent
risk assessment and eventual certification path would need to address, so
the gap is tracked rather than forgotten.

## 8. Source Grounding

The following general engineering concepts (drawn from O'Reilly reference
material on mobile-robot safety) inform this document:

- passive safety such as bumpers
- active avoidance / slowdown
- emergency stop on unrecoverable danger, communication loss, or control failure
- fail-safe stopping concepts
- stopping requires finite detection/control/braking time
- minimum safety distance is therefore required
- sensor fusion for collision avoidance
- safety-field size should depend on robot speed / operating condition
- intrinsic, extrinsic, and temporal sensor calibration
- false-positive versus missed-collision tradeoff
- sensor/communication spoofing as a future security concern

These are general engineering concepts, not evidence of certification. This
prototype does not have industrial safety certification or certified
braking hardware, and nothing above should be read as such a claim.

## 9. Engineering Reference Model

The reaction/braking/minimum-stop-distance formulas used by
`research/dynamic_safety_field_reference.py` are summarized here for
reference:

```text
reaction_distance =
    abs(robot_speed_mps) * total_latency_s

braking_distance =
    robot_speed_mps^2 / (2 * braking_deceleration_mps2)

minimum_stop_distance =
    reaction_distance
    + braking_distance
    + uncertainty_margin_m
```

**ENGINEERING REFERENCE MODEL — NOT A CERTIFIED SAFETY FORMULA.**

`total_latency_s` may conceptually bundle sensor latency, compute latency,
control latency, and actuator/brake response latency — all values are
explicit inputs supplied by the caller. No final calibration values are
assumed here; see `docs/CALIBRATION_PLAN.md` for what remains to be
measured.
