<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: wheel-odometry-model
description: 'Configure wheel odometry model. Use when computing differential drive odometry, calibrating wheel parameters, or setting covariance.'
---

# Wheel Odometry for Differential Drive

## Differential Drive Kinematics

For a two-wheeled differential-drive robot with wheel separation $L$ (distance between left and right wheel centers):

$$v = \frac{v_{\text{right}} + v_{\text{left}}}{2}$$

$$\omega = \frac{v_{\text{right}} - v_{\text{left}}}{L}$$

Where $v$ is linear velocity, $\omega$ is angular velocity, and $v_{\text{left}}$/$v_{\text{right}}$ are individual wheel velocities.

---

## Encoder Ticks to Distance

Each wheel velocity is computed from encoder ticks:

$$d = \frac{\text{ticks}}{\text{ticks\_per\_rev}} \times \pi \times D$$

Where $D$ is wheel diameter in meters. The velocity is then:

$$v_{\text{wheel}} = \frac{d}{\Delta t}$$

### Critical Measurements

- **`wheel_diameter`**: Measure under load (the wheel compresses slightly). A 1% error compounds over distance.
- **`wheel_separation`**: Measure center-to-center between tire contact patches, not between wheel hubs. A 2% error causes the robot to think it turned more/less than reality.
- **`ticks_per_rev`**: Total quadrature encoder counts per revolution (e.g., 64 CPR encoder × 4 quadrature = 256 ticks after gearing).

---

## Pose Integration

At each timestep, integrate velocities into pose:

$$\Delta\theta = \omega \cdot \Delta t$$

$$\Delta x = v \cdot \Delta t \cdot \cos(\theta + \Delta\theta / 2)$$

$$\Delta y = v \cdot \Delta t \cdot \sin(\theta + \Delta\theta / 2)$$

The midpoint method (using $\theta + \Delta\theta/2$) is more accurate than Euler integration for curved paths.

---

## The nav_msgs/Odometry Message

```
Header header
string child_frame_id              # "base_link" or "base_footprint"
PoseWithCovariance pose            # x, y, yaw (integrated position)
TwistWithCovariance twist          # vx, vyaw (instantaneous velocities)
```

The `pose` is in the `odom` frame. The `twist` is in the `child_frame_id` (body) frame.

---

## Covariance Matrices

### Twist Covariance (6×6, row-major)

Set non-zero values for the dimensions you compute. Higher values = less certain:

```python
twist_covariance = [0.0] * 36
twist_covariance[0]  = 0.01   # vx variance (m/s)² — increase if wheels slip
twist_covariance[35] = 0.03   # vyaw variance (rad/s)² — increase if wheel_separation uncertain
```

### Pose Covariance (6×6, row-major)

Grows over time as integration error accumulates. A common approach scales covariance with distance traveled:

```python
pose_covariance = [0.0] * 36
pose_covariance[0]  = 0.1   # x variance
pose_covariance[7]  = 0.1   # y variance
pose_covariance[35] = 0.05  # yaw variance
```

---

## Python Node Skeleton

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class DiffDriveOdometry(Node):
    def __init__(self):
        super().__init__('diff_drive_odometry')

        # Parameters
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('wheel_diameter', 0.1)
        self.declare_parameter('ticks_per_rev', 1440)
        self.declare_parameter('publish_tf', False)  # False if using robot_localization

        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_diameter').value / 2.0
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.pub_tf = self.get_parameter('publish_tf').value

        self.odom_pub = self.create_publisher(Odometry, 'odom/unfiltered', 10)
        if self.pub_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_time = None

        # Subscribe to encoder data (adapt to your actual message type)
        # self.create_subscription(EncoderMsg, 'encoders', self.encoder_cb, 10)
        self.create_timer(0.02, self.update)  # 50 Hz placeholder

    def compute_from_ticks(self, left_ticks, right_ticks, dt):
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return 0.0, 0.0

        dl = (left_ticks - self.prev_left_ticks) / self.ticks_per_rev * 2 * math.pi * self.wheel_rad
        dr = (right_ticks - self.prev_right_ticks) / self.ticks_per_rev * 2 * math.pi * self.wheel_rad

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        v = (dr + dl) / (2.0 * dt) if dt > 0 else 0.0
        omega = (dr - dl) / (self.wheel_sep * dt) if dt > 0 else 0.0
        return v, omega

    def integrate(self, v, omega, dt):
        dtheta = omega * dt
        dx = v * dt * math.cos(self.theta + dtheta / 2.0)
        dy = v * dt * math.sin(self.theta + dtheta / 2.0)
        self.x += dx
        self.y += dy
        self.theta += dtheta

    def publish_odometry(self, v, omega, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = quaternion_from_yaw(self.theta)
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.05

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.03

        self.odom_pub.publish(msg)

        if self.pub_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = quaternion_from_yaw(self.theta)
            self.tf_broadcaster.sendTransform(t)

    def update(self):
        # Replace with actual encoder reading logic
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Integration with robot_localization

Publish wheel odometry to a topic like `odom/unfiltered`. In the EKF config:

```yaml
odom0: "odom/unfiltered"
odom0_config: [false, false, false,
               false, false, false,
               true,  false, false,     # vx
               false, false, true,      # vyaw
               false, false, false]
```

Set `publish_tf: false` in the odometry node — let robot_localization publish the `odom → base_link` transform.

---

## Common Issues

| Problem | Cause | Fix |
|---------|-------|-----|
| Robot drifts in arcs instead of straight | `wheel_separation` wrong | Measure carefully under load, tune empirically |
| Distance traveled is consistently off | `wheel_diameter` wrong or encoder resolution incorrect | Measure diameter under weight, verify `ticks_per_rev` |
| Odometry jumps when wheels slip | Encoder counts still accumulate during slip | Increase twist covariance, let EKF weight IMU more |
| Odometry publishing rate too low | Timer or encoder read rate bottleneck | Target 20–50 Hz minimum |
