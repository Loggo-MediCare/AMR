# AMR Calibration Plan

This document tracks the calibration work required before the values in
`docs/HARDWARE_BOM.md` (AMR Hardware Baseline v1) can move from
CANDIDATE/UNKNOWN/FUTURE CALIBRATION to confirmed, measured values. It does
not change any value defined there.

## 1. Drivetrain Calibration

Measured chassis values (already confirmed, from the hardware baseline):

| Parameter | Value |
|---|---|
| Wheel diameter | 70 mm |
| Wheel radius | 0.035 m |
| Wheel track | 0.130 m |
| Caster | Ø12 mm steel ball |

Still unknown:

- encoder PPR/CPR
- decoded ticks per wheel revolution
- encoder shaft location
- gear ratio confirmation (298:1 is currently CANDIDATE)

Future calibration procedure:

1. Turn the wheel exactly 10 revolutions.
2. Measure the encoder delta over those 10 revolutions.
3. Calculate `ticks_per_wheel_revolution`.
4. Repeat forward and reverse.
5. Repeat for left and right wheels independently.
6. Compare left/right asymmetry.

Wheel-track calibration should additionally be validated using real
robot turning tests later (e.g. commanding a known rotation and comparing
against odometry), not assumed from the measured 0.130 m alone.

## 2. Camera Intrinsic Calibration

Future values:

- `fx`
- `fy`
- `cx`
- `cy`
- distortion coefficients

Camera model remains TBD (see Section 16 of `HARDWARE_BOM.md`).

## 3. Stereo / RGB-D Extrinsic Calibration

Document once a camera is selected:

- camera-to-camera transform (if stereo)
- baseline
- rotation
- translation

## 4. Camera-to-IMU Extrinsic Calibration

Document:

- rotation
- translation
- rigid mounting requirement

## 5. IMU Calibration

Future calibration targets:

- gyro bias
- accelerometer bias
- scale
- axis orientation
- stationary validation

Do NOT assume magnetometer heading is reliable near motors/metal — treat
magnetometer yaw as untrustworthy by default on this chassis (consistent
with Section 13 of `HARDWARE_BOM.md`).

### Candidate Calibration Motion Sequence

For IMUs in the BNO055-style fusion-sensor class, a documented approach is
to expose the sensor to a slow motion sequence covering the full range of
orientations, for example:

1. Lay the sensor flat and hold stationary.
2. Perform a slow figure-eight sweep.
3. Tilt up and down through its range.
4. Roll side to side.

This exposes the sensor to enough orientation diversity to estimate offset
and scale. The exact sequence and duration depend on the actual IMU
selected (still TBD) and its datasheet/SDK calibration requirements — this
is a candidate procedure to adapt, not a final calibration script.

## 6. Temporal Calibration

Preserve this principle throughout the system:

```text
measurement acquisition timestamp
!=
host receive timestamp
```

Document alignment requirements among:

- encoders
- IMU
- camera
- optional LiDAR/ToF

Prefer hardware timestamps where supported. Software synchronization may be
used for approximate matching, but must not silently rewrite measurement
time as if it were the true acquisition time.

Candidate software mechanism: ROS 2's **`message_filters`** package (e.g. an
approximate-time synchronizer) is the standard way to align messages from
sensors publishing at different, not-perfectly-synchronized rates. This is
a candidate tool to use, not a guarantee that software sync alone is
sufficient — hardware timestamping is still preferred where available.

## 7. Validation Matrix

| Sensor / Parameter | Method | Status | Acceptance criterion |
|---|---|---|---|
| Wheel ticks/revolution | 10-revolution rotation test, forward + reverse, left + right | FUTURE CALIBRATION | TBD |
| Wheel track (effective) | Known-rotation turning test vs. odometry | FUTURE CALIBRATION | TBD |
| Encoder forward/reverse sign | Manual direction + sign check per wheel | FUTURE CALIBRATION | TBD |
| Camera intrinsics (fx, fy, cx, cy, distortion) | Standard camera calibration procedure (camera TBD) | UNKNOWN | TBD |
| Stereo/RGB-D extrinsics | Stereo calibration procedure (camera TBD) | UNKNOWN | TBD |
| Camera-to-IMU extrinsics | Rigid-mount extrinsic calibration | UNKNOWN | TBD |
| IMU gyro/accel bias and scale | Stationary + motion validation | FUTURE CALIBRATION | TBD |
| Temporal alignment (encoder/IMU/camera) | Hardware timestamp comparison | FUTURE CALIBRATION | TBD |

Numeric tolerances are intentionally left as TBD until the corresponding
hardware is selected and measured — do not substitute assumed values here.
