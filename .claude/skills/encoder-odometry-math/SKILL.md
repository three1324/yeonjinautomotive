<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: encoder-odometry-math
description: 'Compute odometry from wheel encoders. Use when implementing quadrature decoding, differential drive kinematics, or pose integration.'
---

# Computing Odometry from Quadrature Encoders

## Quadrature Encoding Fundamentals

A quadrature encoder outputs two square-wave signals (channel A and channel B) offset by 90°. The phase relationship between A and B determines direction. Counting modes:

- **1x counting**: Count rising edges of channel A only → `counts_per_rev = PPR` (pulses per revolution)
- **2x counting**: Count both edges of channel A → `counts_per_rev = 2 × PPR`
- **4x counting**: Count both edges of BOTH channels → `counts_per_rev = 4 × PPR`

Always use 4x counting for maximum resolution. With a 64 PPR encoder and a 48:1 gear ratio:

```
counts_per_rev = 4 × 64 × 48 = 12,288 counts/revolution
```

## Distance Per Count

Given wheel diameter $d_w$ (in meters):

$$
\text{distance\_per\_count} = \frac{\pi \cdot d_w}{\text{counts\_per\_rev}}
$$

Example: 90mm wheel, 12,288 counts/rev:

$$
\text{distance\_per\_count} = \frac{\pi \times 0.090}{12288} = 2.30 \times 10^{-5} \text{ m/count} \approx 0.023 \text{ mm/count}
$$

## Wheel Velocities

Read encoder counts at a fixed interval $\Delta t$ (typically 10-20ms). Compute velocity for each wheel:

$$
v_{\text{wheel}} = \frac{\Delta \text{counts}}{\Delta t} \times \text{distance\_per\_count}
$$

Where $\Delta \text{counts} = \text{current\_counts} - \text{previous\_counts}$.

Handle encoder overflow: if using signed 32-bit integers, the count wraps at $\pm 2^{31}$. With 12,288 counts/rev and a wheel doing 5 rev/s, you get 61,440 counts/s. Time to overflow: $2^{31} / 61440 \approx 34,952$ seconds (~9.7 hours). For long-running robots, compute deltas (not absolute positions) and reset periodically, or use 64-bit counters.

## Differential Drive Forward Kinematics

For a two-wheeled differential drive robot with wheel separation $L$ (distance between wheel contact points, center-to-center):

$$
v = \frac{v_R + v_L}{2}
$$

$$
\omega = \frac{v_R - v_L}{L}
$$

Where:
- $v$ = linear velocity of the robot center (m/s)
- $\omega$ = angular velocity about the robot center (rad/s)
- $v_R, v_L$ = right and left wheel velocities (m/s)
- $L$ = wheel separation (m)

## Pose Integration: Euler Method

Given current pose $(x, y, \theta)$ and computed $(v, \omega)$:

$$
\theta_{t+1} = \theta_t + \omega \cdot \Delta t
$$

$$
x_{t+1} = x_t + v \cdot \cos(\theta_t) \cdot \Delta t
$$

$$
y_{t+1} = y_t + v \cdot \sin(\theta_t) \cdot \Delta t
$$

This is a first-order approximation. It works well when $\omega \cdot \Delta t$ is small (high update rate, gentle turns).

## Pose Integration: Runge-Kutta (Second-Order)

For better accuracy during turns, use the midpoint method. Compute the heading at the midpoint of the arc:

$$
\theta_{\text{mid}} = \theta_t + \frac{\omega \cdot \Delta t}{2}
$$

$$
x_{t+1} = x_t + v \cdot \cos(\theta_{\text{mid}}) \cdot \Delta t
$$

$$
y_{t+1} = y_t + v \cdot \sin(\theta_{\text{mid}}) \cdot \Delta t
$$

$$
\theta_{t+1} = \theta_t + \omega \cdot \Delta t
$$

This significantly reduces integration error during arcs and S-curves.

## Exact Arc Integration

When $\omega \neq 0$, the robot traces a circular arc of radius $R = v / \omega$:

$$
x_{t+1} = x_t + R \cdot [\sin(\theta_t + \omega \Delta t) - \sin(\theta_t)]
$$

$$
y_{t+1} = y_t - R \cdot [\cos(\theta_t + \omega \Delta t) - \cos(\theta_t)]
$$

$$
\theta_{t+1} = \theta_t + \omega \cdot \Delta t
$$

When $\omega \approx 0$ (straight line), fall back to Euler to avoid division by zero.

## Covariance Estimation

The `nav_msgs/Odometry` message requires a 6x6 covariance matrix for both pose and twist. For a differential drive encoder-only odometry:

```python
# Pose covariance (x, y, z, roll, pitch, yaw) - only x, y, yaw are relevant
pose_cov = [
    0.01,  0.0,  0.0,  0.0,  0.0,  0.0,   # x variance
    0.0,   0.01, 0.0,  0.0,  0.0,  0.0,    # y variance
    0.0,   0.0,  1e6,  0.0,  0.0,  0.0,    # z (set large = unknown)
    0.0,   0.0,  0.0,  1e6,  0.0,  0.0,    # roll (unknown)
    0.0,   0.0,  0.0,  0.0,  1e6,  0.0,    # pitch (unknown)
    0.0,   0.0,  0.0,  0.0,  0.0,  0.03,   # yaw variance
]
```

Covariance should increase with speed and angular rate—encoders are less accurate when slipping at high speeds or during sharp turns. A simple model:

$$
\sigma_x^2 = k_1 \cdot |v| \cdot \Delta t + k_2
$$

$$
\sigma_\theta^2 = k_3 \cdot |\omega| \cdot \Delta t + k_4
$$

Where $k_1$ through $k_4$ are tuning constants determined experimentally.

## Publishing Odometry in ROS 2

The odometry node must publish:
1. `nav_msgs/Odometry` on `/odom` — pose + twist with covariances
2. TF transform from `odom` → `base_link` (or `base_footprint`)

Both must be published at the same rate (typically 50-100 Hz) and with the same timestamp.

## Common Problems and Solutions

| Problem | Symptom | Solution |
|---------|---------|----------|
| Wrong wheel separation | Robot drifts sideways or arcs during straight-line command | Measure $L$ physically. Then calibrate: drive a known straight line and adjust $L$ until odometry matches. |
| Wrong wheel diameter | Distance traveled doesn't match command | Drive a known distance (e.g., 2m), measure actual vs. reported, apply correction factor. |
| Encoder noise / missed counts | Jittery velocity, odometry jumps | Check wiring (use shielded cable), add hardware debouncing, verify encoder pull-ups. |
| Wheel slip on turns | Odometry heading diverges from reality | Reduce turn speed, fuse with IMU gyroscope for heading (EKF). |
| Asymmetric wheels | Robot curves when commanded straight | Calibrate each wheel diameter independently ($d_L \neq d_R$). Apply separate distance_per_count. |
| Integer overflow | Sudden large jump in position | Compute $\Delta$ counts as `(int32_t)(current - previous)` — signed subtraction handles wrap correctly. |

## Calibration Procedure

1. **Wheel diameter**: Manually push the robot exactly 2.000 m in a straight line. Record encoder counts. Compute actual diameter: $d = \frac{2.000 \times \text{counts\_per\_rev}}{\pi \times \Delta\text{counts}}$.
2. **Wheel separation**: Command the robot to rotate exactly 360° in place (use IMU or visual reference). Compute $L = \frac{(v_R - v_L) \times t}{2\pi}$, or equivalently $L = \frac{\Delta d_R - \Delta d_L}{2\pi}$ where $\Delta d$ is the arc length traveled.
3. **Repeat 5 times, average the results.** Mechanical compliance (tire squish) and surface friction affect these values.
