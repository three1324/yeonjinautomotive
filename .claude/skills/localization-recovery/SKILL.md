<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: localization-recovery
description: 'Handle localization failures and recovery. Use when AMCL diverges, global relocalization is needed, or handling kidnapped robot.'
---

# Localization Recovery — Detecting and Fixing Localization Failures

## Symptoms of Localization Failure

| Observable | What's Happening |
|-----------|-----------------|
| Robot position in RViz jumps erratically | AMCL particles split between competing hypotheses |
| Costmap doesn't match physical walls | map→odom transform is wrong (AMCL converged to wrong location) |
| Nav2 reports "TF timeout" or "Transform not available" | AMCL or EKF stopped publishing transforms |
| Robot drives into walls despite clear path | Localization offset causes planned path to misalign with reality |
| AMCL covariance continuously increases | Particles are spreading — no good scan match found |

---

## Root Causes

1. **Kidnapped robot**: Robot was physically moved while powered off or localization was paused
2. **Odometry spike**: Wheel encoder glitch or IMU spike injected a huge motion estimate, flinging particles away
3. **Symmetric environment**: Two locations look identical to the laser (parallel corridors, symmetric rooms)
4. **Map change**: Furniture moved, doors opened/closed, environment no longer matches the map
5. **Initial pose wrong**: Robot started with an incorrect initial pose estimate

---

## Recovery Strategy 1: Global Re-Localization

Spawn particles uniformly across the entire map. AMCL will reconverge as the robot moves and collects scan data.

### Via Service Call

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

### Via Behavior Tree

```xml
<ReinitializeGlobalLocalization service_name="/reinitialize_global_localization"
                                 server_timeout="5000"/>
```

**Warning**: Global re-localization requires many particles spread across a large map. The filter needs 10–30 seconds of robot motion with diverse scan data to reconverge. During this time, navigation should be paused or limited to safe exploratory behavior.

### Temporary Particle Boost

Increase particle count during recovery:

```bash
ros2 param set /amcl min_particles 5000
ros2 param set /amcl max_particles 10000
# After re-localization converges:
ros2 param set /amcl min_particles 500
ros2 param set /amcl max_particles 2000
```

---

## Recovery Strategy 2: Manual Initial Pose

Set the robot's pose explicitly when you know approximately where it is.

### Via RViz

Click "2D Pose Estimate" in RViz, then click and drag on the map at the robot's approximate location and heading.

### Via Topic

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 1.5, y: 2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.07]
  }
}" --once
```

The covariance determines how widely the initial particles are spread. Larger covariance = wider spread = more robust but slower to converge.

---

## Recovery Strategy 3: Navigate to a Landmark

If the robot is somewhat localized but drifting, navigate it to a known distinctive location (e.g., a corner, a doorway, a narrow passage) where the scan is highly unique. AMCL naturally corrects when it encounters a scan that strongly disambiguates the position.

This is the most reliable passive approach — unique geometry forces particles to converge.

---

## Automated Recovery: Covariance Monitoring

Monitor AMCL's covariance and trigger recovery automatically.

### The /amcl_pose Topic

AMCL publishes `geometry_msgs/PoseWithCovarianceStamped` on `/amcl_pose`. The covariance matrix encodes localization uncertainty. Large diagonal values (especially positions 0, 7, 35 for x, y, yaw) indicate poor localization.

### Monitor Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        self.declare_parameter('covariance_threshold', 0.5)
        self.threshold = self.get_parameter('covariance_threshold').value

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.reinit_client = self.create_client(
            Empty, '/reinitialize_global_localization')

        self.recovery_triggered = False

    def pose_cb(self, msg):
        cov = msg.pose.covariance
        x_var = cov[0]
        y_var = cov[7]
        yaw_var = cov[35]

        max_var = max(x_var, y_var, yaw_var)
        if max_var > self.threshold and not self.recovery_triggered:
            self.get_logger().warn(
                f'Localization degraded (max_var={max_var:.3f}), triggering global re-localization')
            self.reinit_client.call_async(Empty.Request())
            self.recovery_triggered = True
        elif max_var < self.threshold * 0.5:
            self.recovery_triggered = False  # Reset once converged
```

---

## Behavior Tree Integration

### Condition: Check Localization Quality

```xml
<Condition ID="IsLocalizationGood"
           topic="/amcl_pose"
           covariance_threshold="0.5"/>
```

### Recovery Subtree

```xml
<Fallback>
  <Condition ID="IsLocalizationGood"/>
  <Sequence>
    <Action ID="ReinitializeGlobalLocalization"/>
    <Action ID="Spin" spin_dist="6.28"/>  <!-- Full rotation to collect diverse scans -->
    <Action ID="Wait" wait_duration="5"/>
    <Condition ID="IsLocalizationGood"/>
  </Sequence>
</Fallback>
```

The robot spins in place to gather scan data from all directions, accelerating particle convergence.

---

## Preventing Localization Loss

| Prevention | How |
|-----------|-----|
| Set reasonable alpha values | Overly tight motion model (low alpha) causes particles to miss the true pose |
| Adequate particle count | `min_particles: 500` minimum for indoor environments |
| Enable recovery particles | Set `recovery_alpha_fast: 0.1` and `recovery_alpha_slow: 0.001` |
| Avoid purely symmetric environments | Add visual landmarks or distinct furniture near patrol routes |
| Monitor continuously | Run the covariance monitor as a lifecycle component |
| Save and restore pose | On graceful shutdown, save current pose; on startup, load it as `initial_pose` |

---

## Pose Persistence Across Reboots

Save the last known pose when the robot shuts down:

```python
# On shutdown callback:
with open('/tmp/last_pose.yaml', 'w') as f:
    yaml.dump({'x': pose.x, 'y': pose.y, 'yaw': yaw}, f)
```

On startup, load it into AMCL's `initial_pose`:

```yaml
amcl:
  ros__parameters:
    set_initial_pose: true
    initial_pose:
      x: 1.5   # Loaded from file
      y: 2.0
      yaw: 0.3
```

This avoids the cold-start global localization problem entirely, assuming the robot wasn't moved while off.
