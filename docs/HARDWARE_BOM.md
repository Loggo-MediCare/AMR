# AMR Hardware Bill of Materials

Classification legend:

- **MEASURED** = physically measured on our prototype
- **SELECTED** = architecture/family selected
- **CANDIDATE** = likely prototype choice, not final
- **REQUIRED** = must-have requirement
- **UNKNOWN** = must confirm before purchase/integration
- **OPTIONAL** = future or non-core component

## 1. System Architecture

```text
Jetson Orin Nano Super
        │
       ROS 2
        │
      cmd_vel
        ↓
MCU / low-level controller
   ├───────────────┐
   ↓               ↑
PWM + DIR       Encoder A/B
   ↓               │
Dual H-Bridge      │
   ↓               │
N20 geared motors ┘
        ↓
70 mm drive wheels

IMU
 ↓
ROS 2 sensor_msgs/Imu
 ↓
future robot_localization EKF

Stereo / RGB-D camera
 ↓
visual odometry / perception
 ↓
ROS 2 / future Holoscan

Wheel odometry + IMU + VO
 ↓
local state estimation
 ↓
odom -> base_footprint
```

## 2. Chassis and Mechanical Geometry

| Item | Current specification | Status | Notes |
|---|---:|---|---|
| Drive topology | 2WD differential drive | SELECTED | Two active wheels |
| Drive wheel quantity | 2 | MEASURED | Left + right |
| Drive wheel diameter | 70 mm | MEASURED | Physical measurement |
| Drive wheel radius | 35 mm / 0.035 m | MEASURED | Used by wheel odometry |
| Wheel track | 130 mm / 0.130 m | MEASURED | Center-to-center between driven wheels |
| Wheel circumference | ~219.9 mm / 0.2199 m | DERIVED | pi × 0.070 m |
| Passive caster | Ø12 mm steel ball | MEASURED | Not part of odometry equations |

Keep these independent:

- wheel radius
- wheel track
- encoder counts/revolution

## 3. Drive Motors

Selected family:

N20 / GA12-N20 geared brushed DC motor

Important:

N20 is a motor/gearbox family.
Not every N20 includes an encoder.

Our AMR must use an encoder-equipped version.

Current candidate:

| Parameter | Candidate value | Status |
|---|---:|---|
| Motor family | GA12-N20 | SELECTED |
| Nominal voltage | 6 V | CANDIDATE |
| Operating range | 3–12 V | CANDIDATE |
| Gear ratio | 298:1 | CANDIDATE |
| No-load output speed @ 6 V | 50 RPM | CANDIDATE |
| Loaded output speed @ 6 V | 40 RPM | CANDIDATE |
| Rated current @ 6 V / 298:1 | ~120 mA | CANDIDATE |
| Stall current @ 6 V / 298:1 | ~300 mA | CANDIDATE |
| Output shaft | 3 mm | CANDIDATE |

These values are not final until the actual purchased motor is confirmed.

## 4. Encoder Requirement

Required:

N20 geared DC motor
+
magnetic incremental encoder
+
A/B quadrature channels

| Parameter | Requirement | Status |
|---|---|---|
| Encoder type | Magnetic incremental | REQUIRED |
| Channels | A + B | REQUIRED |
| Direction sensing | Quadrature | REQUIRED |
| Encoder PPR / CPR | TBD | UNKNOWN |
| PPR definition | Must confirm | UNKNOWN |
| Encoder shaft location | Motor shaft or output shaft | UNKNOWN |
| Decoded ticks/wheel revolution | TBD | UNKNOWN |
| Forward/reverse sign | Physical calibration | UNKNOWN |

Do NOT assume:

- 14 poles
- 8344 counts/output revolution

Those are reference-example values only.

## 5. Encoder Counting Convention

Current known firmware architecture:

Channel A:
CHANGE interrupt

Channel B:
direction state

If confirmed by repo audit, this corresponds to x2 counting when PPR means
channel-A cycles/revolution.

Do not call it x4 unless both A and B transitions are actually counted.

Future required parameter:

ticks_per_wheel_revolution

Calibration:

Turn wheel forward exactly 10 revolutions.

N =
abs(end_ticks - start_ticks) / 10

Repeat in reverse and compare.

## 6. Wheel Odometry Calculations

wheel circumference:

C = pi × 0.070
  ≈ 0.2199 m

Once true decoded ticks/wheel revolution N is known:

meters_per_tick =
    0.2199 / N

ticks_per_meter =
    N / 0.2199

Do not insert final numerical values until N is confirmed.

## 7. Prototype Speed Estimate

At 50 RPM:

speed ≈
0.2199 × 50 / 60
≈ 0.183 m/s

At 40 RPM:

speed ≈
0.2199 × 40 / 60
≈ 0.147 m/s

Label as THEORETICAL IDEAL SPEEDS.

Real speed depends on:

- robot mass
- battery voltage
- PWM duty cycle
- floor friction
- wheel slip
- motor mismatch
- drivetrain loss

## 8. Motor Driver

Do not drive motors directly from MCU GPIO or Jetson GPIO.

Architecture:

MCU PWM + DIR
      ↓
Dual H-Bridge
      ↓
Left / Right N20

Requirements:

| Requirement | Status |
|---|---|
| Two independent motor channels | REQUIRED |
| Bidirectional H-bridge | REQUIRED |
| PWM speed control | REQUIRED |
| Compatible with 6 V motor rail | REQUIRED |
| Logic compatible with selected MCU | REQUIRED |
| Inductive-load protection | REQUIRED |
| Back-EMF recirculation path | REQUIRED |
| Current margin above stall current | REQUIRED |
| ~>=1 A/channel preferred | CANDIDATE |

Size driver using stall current, not only rated running current.

## 9. Power Architecture

Recommended:

Battery / DC supply
       |
       +--> Motor rail
       |      ↓
       |   Dual H-Bridge
       |      ↓
       |   N20 motors
       |
       +--> DC/DC regulation
              ↓
            MCU
            Jetson
            sensors

Requirements:

- common reference ground where required
- motors not powered from Jetson/MCU GPIO rail
- supply handles startup/stall transients
- avoid motor-induced brownouts
- local decoupling near motor driver and MCU

## 10. Motor EMI / Noise

Concerns:

- brushed motor commutation noise
- encoder signal corruption
- IMU disturbance
- serial communication errors
- supply dips

Consider:

- short motor power wiring
- separate encoder and motor power wiring
- twisted pair where useful
- local decoupling
- optional motor-terminal suppression capacitor
- grounding quality
- oscilloscope validation later

## 11. Low-Level Controller / MCU

Role:

- encoder acquisition
- PWM output
- direction output
- low-level timing
- fault monitoring
- serial/CAN communication

Jetson should NOT perform individual encoder-edge timing.

MCU model:
UNKNOWN / not finalized

Required capabilities:

- enough GPIO for left/right A/B
- hardware interrupts or quadrature support
- sufficient edge-rate capability
- PWM outputs
- serial / USB / CAN
- timestamp support

## 12. Encoder Interrupt-Rate Sizing

If:

E = decoded encoder edges per motor revolution
M = motor RPM

then:

edge_rate_per_motor_hz =
    M / 60 × E

If encoder is motor-shaft mounted:

motor_RPM =
    wheel_RPM × gear_ratio

Worst-case two-wheel rate:

total_ISR_rate_hz =
    2 × edge_rate_per_motor_hz

Do not finalize until actual encoder specs are known.

## 13. IMU

| Capability | Status |
|---|---|
| 3-axis gyroscope | REQUIRED |
| 3-axis accelerometer | REQUIRED |
| Stable sample/update rate | REQUIRED |
| Known coordinate frame | REQUIRED |
| Timestamp support | REQUIRED |
| Linux/ROS integration | REQUIRED |
| Magnetometer | OPTIONAL |

Do not treat magnetometer yaw as ground truth.

Intended fusion:

wheel odometry velocity
+
IMU angular velocity
      ↓
future robot_localization EKF

## 14. Sensor Timing Requirement

Encoder and IMU are asynchronous.

Preserve measurement acquisition timestamp.

Do not blindly replace it with host receive timestamp.

Architecture:

sensor acquisition
      ↓
timestamp
      ↓
transport
      ↓
ROS message
      ↓
state estimation

## 15. Jetson Compute Platform

Target:

Jetson Orin Nano Super

| Parameter | Current status |
|---|---|
| Platform | Jetson Orin Nano Super — SELECTED |
| Power mode | MAXN 25 W — known target |
| Actual JetPack version | UNKNOWN |
| Actual QSPI/UEFI firmware | UNKNOWN |
| Current boot media | UNKNOWN |
| 128 GB+ NVMe installed | UNKNOWN |
| Ready for selected JetPack install | UNKNOWN |

Do not invent actual board state.

## 16. Vision Sensor

Target architecture:

Stereo or RGB-D camera
      ↓
visual frontend
      ↓
visual odometry / perception
      ↓
Jetson GPU

Camera model:
UNKNOWN

Evaluate later:

- stereo/RGB-D capability
- Linux driver
- ROS 2 support
- timestamp quality
- calibration access
- Jetson compatibility
- frame rate
- resolution
- hardware synchronization if stereo

## 17. LiDAR / ToF

LiDAR is NOT mandatory for current final architecture.

Current direction:

stereo/RGB-D
+
IMU
+
wheel encoders

Optional LiDAR/ToF may be used later for:

- obstacle sensing
- costmap input
- comparison
- validation

## 18. Stall / Fault Diagnostics

Future concept:

high PWM command
+
encoder speed approximately zero
      ↓
possible stall / jam

If current sensing exists:

high current
+
zero encoder velocity
      ↓
strong stall evidence

Fault cases:

- motor disconnected
- encoder disconnected
- one wheel stalled
- left/right velocity mismatch
- implausible encoder jump
- direction mismatch

## 19. Optional Current Sensing

Status:
OPTIONAL

Useful for:

- stall detection
- motor characterization
- battery/power diagnostics
- left/right drivetrain comparison

Do not select a sensor model yet.

## 20. LabVIEW Role

LabVIEW is not part of the embedded motor-control loop.

Use it for:

- instrumentation
- test
- calibration
- hardware-in-the-loop
- validation

Potential monitored signals:

- encoder ticks
- RPM
- PWM duty cycle
- motor current
- IMU
- wheel velocity
- left/right mismatch
- fault flags

## 21. ROS 2 Role

ROS 2 owns higher-level integration:

- cmd_vel
- wheel odometry
- sensor messages
- TF
- diagnostics
- future EKF
- navigation
- device orchestration

Do not use ROS 2 Python timing for safety-critical encoder edge acquisition.

## 22. Future Holoscan / MONAI Extension

Not required for drivetrain.

Future architecture:

Stereo / medical sensor
      ↓
Holoscan
      ↓
GPU preprocessing / inference
      ↓
ROS 2 application layer

MONAI belongs to future medical-AI development/training,
not motor control.

## 23. Purchase Checklist — N20 Motor

- [ ] N20 geared DC motor
- [ ] magnetic encoder included
- [ ] A/B quadrature outputs
- [ ] nominal voltage
- [ ] gear ratio
- [ ] encoder PPR / CPR / pole count
- [ ] exact PPR/CPR definition
- [ ] encoder on motor shaft or output shaft
- [ ] no-load RPM
- [ ] loaded/rated RPM
- [ ] rated current
- [ ] stall current
- [ ] output shaft diameter/shape
- [ ] two identical motors available

## 24. Current Status Summary

### MEASURED

Drive topology: 2WD differential + caster
Wheel diameter: 70 mm
Wheel radius: 35 mm
Wheel track: 130 mm
Caster: Ø12 mm steel ball

### SELECTED

Jetson Orin Nano Super
N20 geared DC motor family
Wheel encoder architecture
ROS 2 architecture

### CANDIDATE

Motor voltage: 6 V
Gear ratio: 298:1
No-load output RPM: 50 RPM
Loaded output RPM: 40 RPM
Dual H-bridge: ~>=1 A/channel preferred

### REQUIRED

Encoder-equipped N20
A/B quadrature encoder
3-axis gyro
3-axis accelerometer
dual motor driver
PWM + bidirectional control
encoder timestamping

### UNKNOWN

Actual encoder PPR/CPR
Actual ticks/wheel revolution
Actual encoder shaft location
Actual maximum encoder edge rate
Actual MCU model
Actual IMU model
Actual camera model
Physical Jetson firmware/QSPI state
Physical Jetson boot media
NVMe installation state

### FUTURE CALIBRATION

ticks/wheel revolution
effective wheel radius
effective wheel track
forward/reverse encoder sign
left/right motor mismatch
PWM vs RPM curve
straight-line distance error
rotation/yaw error

## 25. Design Principle

Keep system layers separated:

PHYSICAL
motor / encoder / wheel / IMU / camera
        ↓
LOW-LEVEL CONTROL
MCU / PWM / encoder counting / fault detection
        ↓
ROBOTICS MIDDLEWARE
ROS 2 / TF / odometry / diagnostics / EKF
        ↓
PERCEPTION / GPU
Jetson / stereo / future Holoscan
        ↓
HIGH-LEVEL
Nav2 / AI / future medical applications

The first AMR prototype is intentionally low-cost.

Its purpose is to learn and validate:

sensing
→ deterministic control
→ state estimation
→ diagnostics
→ robotics middleware
→ GPU intelligence

## 26. Motor Driver Safe-State Requirements

An H-bridge enables bidirectional current through a DC motor, but invalid
switching combinations can cause destructive shoot-through or direct supply
shorts.

The AMR motor-control path therefore requires an explicit safe-state design.

| Requirement | Status | Notes |
|---|---|---|
| Dual H-bridge for left/right motor | REQUIRED | Independent bidirectional control |
| Shoot-through protection | REQUIRED | Prevent destructive switch combinations |
| Defined motor-off state at MCU boot | REQUIRED | Motors must not move during startup |
| Defined motor-off state during MCU reset | REQUIRED | No uncontrolled motion |
| Defined state if control process crashes | REQUIRED | Default to motor stop |
| Defined state on communication timeout | REQUIRED | Stop if commands become stale |
| Brake/coast behavior documented | REQUIRED | Know exact driver behavior |
| PWM and DIR truth table documented | REQUIRED | Avoid ambiguous direction control |
| Back-EMF recirculation protection | REQUIRED | Confirm in driver datasheet |
| Driver current rating > motor stall current | REQUIRED | Do not size only from rated current |

Safe-state principle:

```text
MCU boot/reset
      ↓
GPIO state unknown / initializing
      ↓
MOTOR OUTPUT MUST REMAIN OFF
```

Do not assume GPIO default levels are safe.

The final MCU + H-bridge design must explicitly enforce a disabled motor state
during:

- boot
- reset
- communication failure
- software failure

If the selected motor driver has:

- ENABLE
- STBY
- SLEEP
- MOTOR ENABLE

or equivalent pins, evaluate using them as the hardware-safe disable path.

Do NOT select L293D simply because a reference book used it.

The actual H-bridge model remains TBD unless another BOM section explicitly
selects one.

## 27. Manual Stop, Deadman, and Command Timeout

The AMR must not continue moving indefinitely because the previous motion
command remains active.

Required behaviors:

- explicit STOP
- low-level command watchdog
- local motor-disable capability
- teleoperation deadman behavior
- automatic stop if commands become stale

Normal case:

```text
valid cmd_vel
      ↓
refresh command watchdog
      ↓
motion remains enabled
```

Failure case:

```text
no new valid motion command
for timeout interval
        ↓
target_left = 0
target_right = 0
        ↓
motor stop
```

Do NOT invent a final timeout.

Use:

```text
command_timeout_ms = TBD
```

| Safety behavior | Status |
|---|---|
| Explicit ROS STOP command | REQUIRED |
| Low-level command watchdog | REQUIRED |
| Local physical motor-disable method | REQUIRED |
| Teleop deadman behavior | REQUIRED |
| Stop on stale ROS command | REQUIRED |
| Stop on host disconnect | REQUIRED |
| Stop on communication loss | REQUIRED |
| Final timeout value | FUTURE CALIBRATION |

Important:

ROS 2 is not solely responsible for stopping the motors.
The MCU / low-level controller must independently stop motor output when valid
commands stop arriving.

## 28. Motor and Encoder Direction Convention

Canonical convention:

```text
positive wheel command
        ↓
physical forward wheel rotation
        ↓
positive encoder delta
```

Apply this rule independently to LEFT and RIGHT wheels.

| Item | Left | Right | Status |
|---|---|---|---|
| Positive command means forward | REQUIRED | REQUIRED | REQUIRED |
| Forward motion gives positive encoder delta | REQUIRED | REQUIRED | REQUIRED |
| Physical motor polarity | TBD | TBD | FUTURE CALIBRATION |
| Encoder sign inversion needed | TBD | TBD | FUTURE CALIBRATION |
| Software sign correction | TBD | TBD | FUTURE CALIBRATION |

Keep these concepts separate:

- motor electrical polarity
- encoder electrical polarity
- software sign convention

Differential-drive motors are often physically mirrored.
Therefore identical electrical wiring does NOT guarantee identical forward
direction or encoder sign.

Future calibration:

1. Safely raise/support the robot.
2. Command LEFT wheel only with a small positive command.
3. Verify physical forward direction.
4. Record encoder sign.
5. Repeat for RIGHT wheel.
6. Repeat both wheels with negative commands.
7. Determine whether wiring or software sign inversion is needed.
8. Record final convention in this BOM.

Do NOT modify firmware or control code in this documentation task.

## 29. Left / Right Motor Characterization

Two nominally identical geared DC motors will not necessarily produce identical
RPM for the same PWM.

Possible causes:

- motor manufacturing tolerance
- brush friction
- gearbox friction
- wheel friction
- drivetrain alignment
- robot load
- supply voltage
- temperature

Therefore:

```text
same PWM
!=
guaranteed same wheel speed
```

Future characterization:

| PWM command | Left RPM | Right RPM | Left current | Right current |
|---|---|---|---|---|
| 10% | TBD | TBD | TBD | TBD |
| 20% | TBD | TBD | TBD | TBD |
| 30% | TBD | TBD | TBD | TBD |
| 40% | TBD | TBD | TBD | TBD |
| 50% | TBD | TBD | TBD | TBD |
| 60% | TBD | TBD | TBD | TBD |
| 70% | TBD | TBD | TBD | TBD |
| 80% | TBD | TBD | TBD | TBD |
| 90% | TBD | TBD | TBD | TBD |
| 100% | TBD | TBD | TBD | TBD |

Recommended future test conditions:

- unloaded wheel test
- robot-on-floor test
- forward
- reverse
- battery voltage recorded
- left/right RPM compared
- motor current compared if current sensing is available

Future closed-loop concept:

```text
target_left_velocity
        ↓
      PID-L
        ↓
       PWM
        ↓
   left motor
        ↑
 left encoder

target_right_velocity
        ↓
      PID-R
        ↓
       PWM
        ↓
   right motor
        ↑
 right encoder
```

Status: FUTURE CALIBRATION

## 30. PWM Deadband and Minimum Effective PWM

Brushed DC motors normally have static friction and gearbox friction.

Therefore low PWM duty cycles may produce:

- no motion
- motor buzzing
- intermittent movement
- unstable very-low-speed behavior

Do NOT assume:

```text
PWM 10% = 10% of rated RPM
```

Required future measurements:

```text
minimum_start_pwm_left
minimum_start_pwm_right

minimum_sustain_pwm_left
minimum_sustain_pwm_right
```

Definitions:

- **minimum_start_pwm** — lowest PWM that reliably starts a stationary wheel
- **minimum_sustain_pwm** — lowest PWM that keeps an already-moving wheel rotating reliably

| Parameter | Left | Right | Status |
|---|---:|---:|---|
| Minimum start PWM | TBD | TBD | FUTURE CALIBRATION |
| Minimum sustain PWM | TBD | TBD | FUTURE CALIBRATION |
| Velocity deadband | TBD | TBD | FUTURE CALIBRATION |
| Maximum allowed PWM | TBD | TBD | FUTURE CALIBRATION |

Conceptual control behavior:

```text
if abs(target_velocity) < velocity_deadband:
    command = STOP
else:
    apply closed-loop control
```

A future controller may use a minimum-effective-PWM floor, but this must be
measured on the real drivetrain rather than guessed.

Do not implement controller logic in this task.

## 31. Power Brownout, Stall, and Bring-Up Test

Motor startup and stall conditions can create large current transients.

These transients can cause:

- Jetson reboot
- MCU reset
- serial errors
- encoder corruption
- IMU corruption
- H-bridge thermal stress
- battery voltage sag

Therefore power validation must be part of AMR bring-up.

### Bench bring-up

- [ ] Verify motor rail voltage with motors stopped
- [ ] Verify logic rail voltage with motors stopped
- [ ] Verify MCU boots with motors disconnected
- [ ] Verify H-bridge outputs remain OFF during MCU boot
- [ ] Verify one motor forward
- [ ] Verify one motor reverse
- [ ] Verify both motors forward
- [ ] Verify both motors reverse
- [ ] Verify rapid stop
- [ ] Verify command timeout stop
- [ ] Verify encoder signs
- [ ] Verify no unexpected motor motion on reset

### Dynamic power test

- [ ] both motors start simultaneously
- [ ] both motors reverse simultaneously
- [ ] forward -> stop -> reverse
- [ ] one-wheel high-load condition
- [ ] two-wheel high-load condition
- [ ] Jetson does not reboot
- [ ] MCU does not reset
- [ ] encoder data remains valid
- [ ] IMU remains valid
- [ ] serial/USB communication remains valid

Measurements to capture later:

```text
Vmotor_idle
Vmotor_min_startup
Vmotor_min_heavy_load

Vlogic_idle
Vlogic_min_startup

Jetson rail minimum
MCU rail minimum
```

If instrumentation is available, also capture:

- motor current
- supply current
- startup current
- stall current
- battery voltage sag
- DC/DC output sag

A motor stall test must be brief and controlled.

Do not hold a small DC gearmotor in stall for a long period.

The exact safe stall-test duration depends on the actual motor and driver datasheets.

Status: FUTURE VALIDATION

## 32. Velocity and Acceleration Limits

The AMR should not jump directly from zero command to unrestricted full motor output.

Future command pipeline:

```text
ROS 2 cmd_vel
    ->
velocity limit
    ->
acceleration / slew-rate limit
    ->
left/right wheel velocity targets
    ->
closed-loop motor controller
```

Future parameters:

```text
max_linear_velocity_mps = TBD
max_angular_velocity_rps = TBD

max_linear_acceleration_mps2 = TBD
max_angular_acceleration_rps2 = TBD
```

Do NOT invent numerical values.

Reasons for limiting velocity and acceleration:

- reduce wheel slip
- reduce startup current
- reduce mechanical shock
- improve odometry consistency
- improve teleoperation safety
- reduce power transients
- improve controllability

Status: FUTURE CALIBRATION

## 33. Hardware Coverage Matrix

The purpose of this section is to explicitly record which common autonomous
vehicle hardware categories are:

REQUIRED
SELECTED
OPTIONAL
NOT PLANNED
UNKNOWN

Do NOT assume that every self-driving-car sensor is required for this AMR.

Use:

| Hardware category | AMR implementation | Status | Role |
|---|---|---|---|
| Wheel encoder | N20 A/B quadrature encoder | REQUIRED | Wheel rotation / odometry |
| On-board compute | Jetson Orin Nano Super | SELECTED | ROS 2 / GPU / perception |
| Low-level controller | MCU / Arduino-class controller | SELECTED ARCHITECTURE | PWM / encoder acquisition / watchdog |
| Camera | Stereo or RGB-D | REQUIRED TARGET | Visual perception / VO |
| IMU | 3-axis gyro + accelerometer | REQUIRED | Angular velocity / state estimation |
| LiDAR | TBD | OPTIONAL | Costmap / ranging / comparison |
| Ultrasonic / ToF | TBD | OPTIONAL | Near-field obstacle / docking |
| GPS | None | NOT PLANNED | Indoor AMR does not currently require global GNSS |
| Radar | None | NOT PLANNED | Not required for current prototype |
| Motor driver | Dual H-bridge | REQUIRED | Drive left/right N20 motors |
| Battery | TBD | REQUIRED / UNKNOWN | Main mobile power |
| Voltage regulation | DC/DC converters | REQUIRED / UNKNOWN | Separate motor/logic rails |

Add note:

The absence of GPS, radar, or LiDAR is intentional for the current prototype
and must not automatically be treated as a missing BOM item.

Current target sensing architecture remains:

wheel encoders
+
IMU
+
stereo/RGB-D

Optional ranging sensors may be added later.

## 34. Battery and Power Budget

Create a dedicated power subsystem section.

Do NOT copy battery capacity from reference educational robots.

Our actual battery has not yet been selected.

Use:

| Parameter | Value | Status |
|---|---:|---|
| Battery chemistry | TBD | UNKNOWN |
| Nominal battery voltage | TBD | UNKNOWN |
| Battery capacity | TBD Ah | UNKNOWN |
| Continuous discharge current | TBD | UNKNOWN |
| Peak discharge current | TBD | UNKNOWN |
| Motor rail | ~6 V candidate | CANDIDATE |
| Jetson input rail | TBD from actual platform power design | UNKNOWN |
| MCU logic rail | TBD | UNKNOWN |
| Sensor rail | TBD | UNKNOWN |
| Main power switch | Required | REQUIRED |
| Fuse / over-current protection | Required | REQUIRED |
| Motor DC/DC regulator | TBD | REQUIRED IF NEEDED |
| Logic DC/DC regulator | TBD | REQUIRED |

Add future power-budget equation:

total_continuous_power =
Jetson
+ MCU
+ sensors
+ motor average load
+ conversion losses

peak_power must additionally account for:

motor startup current
motor stall transients
compute load transients

Do not calculate final battery capacity until actual Jetson, camera, motors,
motor driver, MCU, and sensor power measurements are known.

## 35. Sensor Interface and Mounting Matrix

Create:

| Sensor | Interface | Timing requirement | Mounting requirement | Status |
|---|---|---|---|---|
| Left encoder | MCU GPIO / interrupt | deterministic | motor/wheel drivetrain | REQUIRED |
| Right encoder | MCU GPIO / interrupt | deterministic | motor/wheel drivetrain | REQUIRED |
| IMU | I2C/SPI/other TBD | timestamp required | rigidly mounted to chassis | REQUIRED |
| Stereo/RGB-D camera | USB/MIPI/other TBD | timestamp important | rigid mount / calibrated pose | REQUIRED TARGET |
| LiDAR | TBD | timestamp important | rigid TF-known mount | OPTIONAL |
| Ultrasonic/ToF | TBD | moderate | near-field coverage | OPTIONAL |
| GPS | N/A | N/A | N/A | NOT PLANNED |
| Radar | N/A | N/A | N/A | NOT PLANNED |

Add requirement:

Every sensor used for localization/perception should eventually have a known
relationship to base_link through the robot TF tree.

Do not invent physical mounting dimensions.

## 36. Optional Near-Field and Docking Sensors

Document optional low-cost sensors that may complement vision.

Possible future sensors:

- ultrasonic
- short-range ToF
- bumper/contact switch
- docking alignment sensor

Potential uses:

- docking
- wall proximity
- blind-zone detection
- low-speed collision prevention

Status:

OPTIONAL

Do not make these sensors part of wheel odometry or EKF by default.

Do not redesign the AMR around these sensors without an explicit future
requirement.

Overall architecture (confirmed intact after this edit):

```text
PHYSICAL
motor / encoder / wheel / IMU / camera
        ↓
LOW-LEVEL CONTROL
MCU / PWM / encoder counting / watchdog
        ↓
ROBOTICS
ROS 2 / TF / odometry / diagnostics / future EKF
        ↓
GPU / PERCEPTION
Jetson / stereo-RGBD / future Holoscan
        ↓
HIGH LEVEL
Nav2 / AI / future medical applications
```
