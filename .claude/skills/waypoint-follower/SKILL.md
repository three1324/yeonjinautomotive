<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: waypoint-follower
description: 'Use Nav2 waypoint follower. Use when sending multiple sequential goals, adding task executors at waypoints, or patrol routes.'
---

# Waypoint Follower

## Purpose

The waypoint follower (`nav2_waypoint_follower::WaypointFollower`) is a Nav2 action server that accepts a list of poses and navigates to each sequentially. It internally calls the `navigate_to_pose` action for each waypoint. Use it for patrol routes, multi-room navigation, and inspection tasks.

## Configuration

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: true
    action_server_result_timeout: 900.0
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200  # milliseconds
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `loop_rate` | `20` | Hz for the action server's main loop |
| `stop_on_failure` | `true` | If true, abort remaining waypoints when one fails. If false, skip failed waypoint and continue. |
| `action_server_result_timeout` | `900.0` | Seconds to wait for `navigate_to_pose` result before timing out. **Critical for long patrols.** |
| `waypoint_task_executor_plugin` | `""` | Plugin executed at each waypoint arrival |

## Task Executor Plugins

Task executors run when the robot arrives at each waypoint. Built-in plugins:

### WaitAtWaypoint
Pauses for a configurable duration:
```yaml
wait_at_waypoint:
  plugin: "nav2_waypoint_follower::WaitAtWaypoint"
  enabled: true
  waypoint_pause_duration: 2000  # milliseconds
```

### PhotoAtWaypoint
Takes a photo using a camera topic:
```yaml
photo_at_waypoint:
  plugin: "nav2_waypoint_follower::PhotoAtWaypoint"
  enabled: true
  image_topic: "/camera/color/image_raw"
  save_dir: "/tmp/waypoint_photos"
  image_format: "png"
```

### InputAtWaypoint
Waits for external input (e.g., operator confirmation) before continuing:
```yaml
input_at_waypoint:
  plugin: "nav2_waypoint_follower::InputAtWaypoint"
  enabled: true
  timeout: 10.0  # 0 = wait forever
```

### Custom Task Executor

Create a class inheriting from `nav2_core::WaypointTaskExecutor`:

```cpp
#include "nav2_core/waypoint_task_executor.hpp"

class MyTaskExecutor : public nav2_core::WaypointTaskExecutor
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                  const std::string & plugin_name) override;
  bool processAtWaypoint(const geometry_msgs::msg::PoseStamped & curr_pose,
                        const int & curr_waypoint_index) override;
};
```

Register with `pluginlib` and configure in YAML.

## Sending Waypoints from Python

### Using BasicNavigator

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy

rclpy.init()
nav = BasicNavigator()
nav.waitUntilNav2Active()

waypoints = []
for (x, y, yaw) in [(1.0, 0.0, 0.0), (2.0, 1.0, 1.57), (0.0, 0.0, 3.14)]:
    wp = PoseStamped()
    wp.header.frame_id = 'map'
    wp.header.stamp = nav.get_clock().now().to_msg()
    wp.pose.position.x = x
    wp.pose.position.y = y
    wp.pose.orientation.z = math.sin(yaw / 2)
    wp.pose.orientation.w = math.cos(yaw / 2)
    waypoints.append(wp)

nav.followWaypoints(waypoints)

while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    if feedback:
        print(f'Waypoint {feedback.current_waypoint}/{len(waypoints)}')

result = nav.getResult()
print(f'Navigation result: {result}')
```

### Using the Action Client Directly

```python
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient

client = ActionClient(node, FollowWaypoints, 'follow_waypoints')
client.wait_for_server()

goal = FollowWaypoints.Goal()
goal.poses = waypoints  # List of PoseStamped
future = client.send_goal_async(goal)
```

## Patrol Pattern with Looping

The waypoint follower does NOT natively loop. Implement looping in your code:

```python
patrol_waypoints = [wp1, wp2, wp3, wp4]

while rclpy.ok():
    nav.followWaypoints(patrol_waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # Handle feedback
    result = nav.getResult()
    if result == TaskResult.FAILED:
        nav.get_logger().warn('Patrol leg failed, retrying...')
        nav.clearAllCostmaps()
        time.sleep(2.0)
    # Loop restarts
```

## Integration with Behavior Trees

The BT node `FollowWaypoints` calls the waypoint follower action:

```xml
<FollowWaypoints goals="{goals}" server_name="follow_waypoints"
                 server_timeout="10" error_code_id="{follow_wp_error}"/>
```

For patrol patterns in BTs, use `NavigateThroughPoses` instead, which sends all poses to the controller at once for smoother traversal:

```xml
<NavigateThroughPoses goals="{goals}" server_name="navigate_through_poses"
                      server_timeout="10" error_code_id="{nav_error}"/>
```

The difference: `FollowWaypoints` stops at each waypoint. `NavigateThroughPoses` passes through waypoints smoothly without stopping.

## Common Issues

### action_server_result_timeout Too Short
**Symptom**: Waypoint following aborts mid-patrol with no navigation error.
**Cause**: The default timeout may be too short for patrols that take several minutes between waypoints (e.g., navigating between floors, waiting at waypoints).
**Fix**: Set `action_server_result_timeout` to cover your longest possible leg. For a house patrol, 900.0 (15 minutes) is reasonable.

### stop_on_failure Behavior
With `stop_on_failure: true`: If waypoint 3 of 10 fails (e.g., blocked path), ALL remaining waypoints are aborted.
With `stop_on_failure: false`: The failed waypoint is skipped, and the robot proceeds to waypoint 4. The final result still reports which waypoints were missed.

For patrol robots, `stop_on_failure: false` is usually preferred—you want the robot to continue the patrol even if one room is temporarily inaccessible.

### Waypoint Ordering
Waypoints are executed strictly in order. The planner doesn't optimize the route. If you need TSP-style optimization, compute the order before sending to the waypoint follower.
