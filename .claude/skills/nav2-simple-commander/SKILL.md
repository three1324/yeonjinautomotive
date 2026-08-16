<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: nav2-simple-commander
description: 'Use Nav2 SimpleCommander Python API. Use when sending navigation goals programmatically, checking nav state, or canceling goals.'
---

# Nav2 Simple Commander (Python API)

## Overview

`nav2_simple_commander` provides `BasicNavigator`, a Python class that wraps Nav2 action servers and services into a simple imperative API. It handles action client setup, goal sending, feedback monitoring, and result checking.

```bash
# Package: nav2_simple_commander (installed with Nav2)
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
```

## Core API

### Initialization

```python
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

rclpy.init()
nav = BasicNavigator()

# Wait for Nav2 lifecycle managers to report active
nav.waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')
```

`waitUntilNav2Active()` blocks until the `bt_navigator` and `amcl` nodes are in the active lifecycle state. Pass `localizer=''` if not using AMCL (e.g., using robot_localization with static map).

### Setting Initial Pose

```python
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = nav.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.orientation.w = 1.0

nav.setInitialPose(initial_pose)
```

This publishes to `/initialpose` for AMCL. Wait a moment after setting for AMCL to converge.

### Single Goal Navigation

```python
goal = PoseStamped()
goal.header.frame_id = 'map'
goal.header.stamp = nav.get_clock().now().to_msg()
goal.pose.position.x = 2.0
goal.pose.position.y = 1.0
goal.pose.orientation.w = 1.0

nav.goToPose(goal)

while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    if feedback:
        # feedback.distance_remaining, feedback.navigation_time,
        # feedback.estimated_time_remaining, feedback.number_of_recoveries
        if feedback.navigation_time > Duration(seconds=120):
            nav.cancelTask()

result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    print('Goal reached!')
elif result == TaskResult.CANCELED:
    print('Goal canceled!')
elif result == TaskResult.FAILED:
    print('Goal failed!')
```

### Navigate Through Poses

Passes through intermediate poses without stopping:

```python
poses = [pose1, pose2, pose3]  # List of PoseStamped
nav.goThroughPoses(poses)
```

### Follow Waypoints

Stops at each waypoint and optionally executes a task:

```python
waypoints = [wp1, wp2, wp3]  # List of PoseStamped
nav.followWaypoints(waypoints)
```

### Cancel Navigation

```python
nav.cancelTask()  # Cancel the current goal
```

## Path Computation (No Navigation)

Compute a path without actually navigating:

```python
# Single path
path = nav.getPath(start_pose, goal_pose, planner_id='GridBased', use_start=True)
# path is a nav_msgs/Path

# Multi-pose path
path = nav.getPathThroughPoses(start_pose, [goal1, goal2, goal3])

# Smooth a path
smoothed = nav.smoothPath(path, smoother_id='', max_duration=2.0)
```

Use `use_start=False` to let the planner use the robot's current position as start.

## Costmap Operations

```python
# Get costmaps
global_costmap = nav.getGlobalCostmap()
local_costmap = nav.getLocalCostmap()

# Clear costmaps (useful after false obstacle detections)
nav.clearAllCostmaps()
nav.clearLocalCostmap()
nav.clearGlobalCostmap()
```

## Complete Patrol Example

```python
#!/usr/bin/env python3
"""Autonomous house patrol using Nav2 Simple Commander."""

import rclpy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import time


def create_pose(nav, x, y, yaw):
    """Create a PoseStamped in map frame."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    # Define patrol waypoints: (x, y, yaw)
    patrol_points = [
        (1.5, 0.0, 0.0),       # Hallway
        (3.0, 1.0, 1.57),      # Kitchen
        (3.0, -1.0, -1.57),    # Living room
        (0.0, 0.0, 3.14),      # Home
    ]

    patrol_count = 0
    max_patrols = 10

    while rclpy.ok() and patrol_count < max_patrols:
        patrol_count += 1
        nav.get_logger().info(f'Starting patrol {patrol_count}/{max_patrols}')

        waypoints = [create_pose(nav, x, y, yaw) for x, y, yaw in patrol_points]
        nav.followWaypoints(waypoints)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                wp_idx = feedback.current_waypoint
                nav.get_logger().info(
                    f'Waypoint {wp_idx + 1}/{len(waypoints)}'
                )

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            nav.get_logger().info(f'Patrol {patrol_count} complete')
        elif result == TaskResult.FAILED:
            nav.get_logger().warn(f'Patrol {patrol_count} failed, clearing costmaps')
            nav.clearAllCostmaps()
            time.sleep(5.0)
        elif result == TaskResult.CANCELED:
            nav.get_logger().info('Patrol canceled')
            break

        # Pause between patrols
        time.sleep(30.0)

    nav.get_logger().info('Patrol routine finished')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Useful Methods Reference

| Method | Returns | Description |
|--------|---------|-------------|
| `goToPose(pose)` | None | Navigate to a single pose |
| `goThroughPoses(poses)` | None | Navigate through poses without stopping |
| `followWaypoints(poses)` | None | Navigate to each pose, stopping at each |
| `isTaskComplete()` | bool | Check if current task is done |
| `getFeedback()` | Feedback | Get current action feedback |
| `getResult()` | TaskResult | Get final result after completion |
| `cancelTask()` | None | Cancel current navigation |
| `getPath(start, goal)` | Path | Compute path without navigating |
| `smoothPath(path)` | Path | Smooth a computed path |
| `clearAllCostmaps()` | None | Clear both costmaps |
| `setInitialPose(pose)` | None | Set AMCL initial pose |
| `waitUntilNav2Active()` | None | Block until Nav2 is ready |
| `changeMap(map_path)` | None | Load a new map |
| `lifecycleStartup()` | None | Manually start Nav2 lifecycle |
| `lifecycleShutdown()` | None | Manually shut down Nav2 |

## Tips

- Always call `waitUntilNav2Active()` before sending goals
- Check `isTaskComplete()` in a loop—don't just block on the action result
- Use `getPath()` to verify a path exists before committing to navigation
- `clearAllCostmaps()` is essential after the robot is manually moved or after false obstacle detections
- `goThroughPoses()` is smoother than `followWaypoints()` for corridor traversal
