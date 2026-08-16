<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: collision-monitor
description: 'Configure Nav2 collision monitor. Use when adding safety polygons, setting stop/slowdown zones, or VL53L0X range sensor integration.'
---

# Collision Monitor

## Purpose

The collision monitor (`nav2_collision_monitor::CollisionMonitor`) is the **final safety layer** before velocity commands reach the motors. It directly reads sensor data (laser scans, point clouds, range sensors) and modifies or stops `cmd_vel` based on proximity to obstacles. It operates independently of costmaps.

**Critical rule**: The collision monitor must ALWAYS be the last node in the velocity pipeline, after the velocity smoother. Nothing should modify `cmd_vel` after the collision monitor.

```
Controller → Velocity Smoother → Collision Monitor → Motor Driver
```

## How It Differs from Costmap-Based Avoidance

Costmap obstacle avoidance (via the controller) plans around obstacles using processed grid data. The collision monitor is a **reactive safety system** that works on raw sensor readings with minimal processing delay. It catches:

- Obstacles that appeared after the last costmap update
- Fast-moving obstacles (people, pets)
- Sensor data that hasn't propagated through the costmap pipeline
- Edge cases where the controller's plan would clip an obstacle

## Configuration

```yaml
collision_monitor:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic: "collision_monitor_state"
    transform_tolerance: 0.2
    source_timeout: 1.0
    base_shift_correction: true
    stop_pub_timeout: 2.0
    polygons: ["PolygonStop", "PolygonSlow", "PolygonApproach"]
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
      min_height: 0.05
      max_height: 0.5
      enabled: true
    PolygonStop:
      type: "circle"
      radius: 0.22
      action_type: "stop"
      max_points: 3
      visualize: true
      polygon_pub_topic: "polygon_stop"
    PolygonSlow:
      type: "polygon"
      points: "[[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]]"
      action_type: "slowdown"
      max_points: 3
      slowdown_ratio: 0.3
      visualize: true
      polygon_pub_topic: "polygon_slowdown"
    PolygonApproach:
      type: "polygon"
      points: "[[0.8, 0.4], [0.8, -0.4], [-0.2, -0.4], [-0.2, 0.4]]"
      action_type: "approach"
      max_points: 3
      time_before_collision: 2.0
      simulation_time_step: 0.1
      visualize: true
      polygon_pub_topic: "polygon_approach"
```

## Polygon Types

### Circle
```yaml
PolygonStop:
  type: "circle"
  radius: 0.22    # meters from base_frame_id origin
```
Simple circular zone. Use for the innermost stop zone—matches the robot's physical radius.

### Polygon
```yaml
PolygonSlow:
  type: "polygon"
  points: "[[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]]"
```
Arbitrary convex polygon defined by vertices in `base_frame_id`. Points are `[x, y]` pairs. X is forward, Y is left. Vertices must form a convex hull.

### Footprint-based
```yaml
PolygonDynamic:
  type: "polygon"
  footprint_topic: "/local_costmap/published_footprint"
  action_type: "stop"
```
Uses the robot's published footprint, which can change dynamically (e.g., with a gripper).

## Action Types

### stop
Immediately sets `cmd_vel` to zero when `max_points` or more observation points fall within the polygon.

### slowdown
Reduces velocity by `slowdown_ratio` (0.0–1.0):
```yaml
action_type: "slowdown"
slowdown_ratio: 0.3   # Velocity reduced to 30% of commanded
```

### limit
Limits maximum velocity to configured values when triggered:
```yaml
action_type: "limit"
linear_limit: 0.2    # m/s max
angular_limit: 0.5   # rad/s max
```

### approach
Dynamically limits velocity based on distance to collision. Uses forward simulation:
```yaml
action_type: "approach"
time_before_collision: 2.0      # Seconds of lookahead
simulation_time_step: 0.1      # Resolution of forward simulation
```

The robot slows proportionally as it approaches an obstacle, reaching zero velocity at the obstacle boundary. This gives the smoothest deceleration behavior.

## Observation Sources

### LaserScan
```yaml
scan:
  type: "scan"
  topic: "/scan"
  min_height: 0.05    # Ignore ground reflections
  max_height: 0.5     # Ignore readings above robot height
  enabled: true
```

### PointCloud2
```yaml
pointcloud:
  type: "pointcloud"
  topic: "/depth_camera/points"
  min_height: 0.05
  max_height: 1.0
  enabled: true
```

### Range (sonar/IR)
```yaml
sonar:
  type: "range"
  topic: "/sonar/range"
  enabled: true
```

### source_timeout
```yaml
source_timeout: 1.0   # Seconds
```
If no data is received from a source within this timeout, the collision monitor triggers a **stop** as a safety precaution. Set this appropriately for your sensor rates. If your LIDAR publishes at 10 Hz, `source_timeout: 0.5` would be risky—use 1.0–2.0.

## Layered Safety Zones

Best practice is to define multiple concentric zones:

```
[Approach zone - outer] → Gentle slowdown based on time-to-collision
  [Slowdown zone - middle] → Reduce to 30% speed
    [Stop zone - inner] → Full stop, matches robot footprint
```

Polygons are evaluated in the order listed in the `polygons` parameter. The **most restrictive** action wins. If a point is in both the slowdown and stop zones, the stop action takes precedence.

## Key Parameters

| Parameter | Description |
|-----------|-------------|
| `cmd_vel_in_topic` | Input velocity topic (from velocity smoother) |
| `cmd_vel_out_topic` | Output velocity topic (to motor driver) |
| `state_topic` | Publishes current monitor state for diagnostics |
| `base_shift_correction` | Compensates for robot motion between sensor reading and processing |
| `stop_pub_timeout` | How long to publish zero velocity after an obstacle is detected (prevents coasting) |
| `max_points` | Minimum number of sensor points in a zone to trigger the action (noise filter) |
| `transform_tolerance` | TF lookup tolerance in seconds |

## Debugging

Enable visualization to see polygons in RViz:
```yaml
PolygonStop:
  visualize: true
  polygon_pub_topic: "polygon_stop"
```

This publishes `geometry_msgs/PolygonStamped` that you can display in RViz. The `state_topic` publishes which polygons are currently triggered.

## Common Issues

- **Robot stops for no visible reason**: `max_points: 1` is too sensitive. Increase to 3+ to filter noise.
- **Robot doesn't stop in time**: The stop zone is too small or `source_timeout` is allowing stale data.
- **Collision monitor overriding navigation**: If the approach zone is too large, the robot may constantly slow down during normal navigation. Shrink the outer zone or use `limit` instead of `approach`.
