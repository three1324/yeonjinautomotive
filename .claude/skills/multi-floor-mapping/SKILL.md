<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: multi-floor-mapping
description: 'Handle multi-floor mapping. Use when managing separate maps per floor, floor transitions, or multi-level navigation.'
---

# Multi-Floor Navigation

## The Challenge

Each floor of a building requires its own 2D occupancy grid map. The navigation stack must switch maps, re-localize the robot, and update costmaps when the robot changes floors. There is no standard ROS 2 package for this—it requires custom orchestration.

## Coordinate System Considerations

Each floor's map has its own origin. The `map` frame exists on a single 2D plane, so when switching floors:
- The `map → odom` transform becomes invalid.
- AMCL's particle distribution is meaningless for the new floor.
- The global costmap contains stale data from the previous floor.

You must explicitly handle the transition: load the new map, reinitialize AMCL, and clear the costmaps.

## Approach 1: Single Map Server with LoadMap Service

The simplest approach. One map server instance, and you call `/map_server/load_map` when the floor changes.

### Map Server Config

```yaml
map_server:
  ros__parameters:
    yaml_filename: /home/robot/maps/floor1.yaml   # Default floor at startup.
    topic_name: map
    frame_id: map
```

### Floor Transition Sequence

1. Robot arrives at elevator/stairway transition point.
2. Detect floor change (sensor, button, or action completion).
3. Call `/map_server/load_map` with the new floor's YAML.
4. Publish new `/initialpose` for AMCL on the new floor.
5. Clear costmaps via `/global_costmap/clear_entirely_global_costmap`.
6. Resume navigation.

## Approach 2: Multiple Map Servers

Run one map server per floor, each on a namespaced topic. A multiplexer node relays the active floor's map to `/map`.

```python
# Launch:
Node(package='nav2_map_server', executable='map_server', name='map_server_floor1',
     parameters=[{'yaml_filename': '/home/robot/maps/floor1.yaml'}],
     remappings=[('map', '/floor1/map')]),
Node(package='nav2_map_server', executable='map_server', name='map_server_floor2',
     parameters=[{'yaml_filename': '/home/robot/maps/floor2.yaml'}],
     remappings=[('map', '/floor2/map')]),
```

Advantage: maps are preloaded, so switching is faster. Disadvantage: more memory usage, more complexity.

## Floor-Specific Navigation Goals

Maintain a per-floor goal database:

```yaml
floors:
  floor1:
    map_yaml: /home/robot/maps/floor1.yaml
    initial_pose: {x: 1.0, y: 2.0, yaw: 0.0}
    goals:
      kitchen: {x: 5.2, y: 3.1, yaw: 1.57}
      living_room: {x: 8.0, y: 6.5, yaw: 0.0}
      elevator_entrance: {x: 12.0, y: 1.5, yaw: 3.14}
  floor2:
    map_yaml: /home/robot/maps/floor2.yaml
    initial_pose: {x: 12.0, y: 1.5, yaw: 0.0}
    goals:
      bedroom: {x: 3.0, y: 7.0, yaw: 0.0}
      bathroom: {x: 6.5, y: 2.0, yaw: 1.57}
      elevator_entrance: {x: 12.0, y: 1.5, yaw: 3.14}
```

## Floor Change Detection

| Method | Pros | Cons |
|--------|------|------|
| Button/UI trigger | Reliable, deterministic | Requires human interaction or external signal |
| Barometric pressure | Passive, no infrastructure needed | Noisy, slow, affected by HVAC |
| Visual landmark | Automatic | Requires camera + trained detector |
| Elevator integration | Deterministic | Requires elevator API access |
| WiFi fingerprint change | Infrastructure-based | Unreliable, slow |

For a home patrol robot, a deterministic trigger (e.g., complete the "ride elevator" action → switch floor) is most reliable.

## Floor Transition Manager (Python)

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import yaml


class FloorTransitionManager(Node):
    def __init__(self):
        super().__init__('floor_transition_manager')

        # Load floor config
        self.declare_parameter('floor_config', '/home/robot/config/floors.yaml')
        config_path = self.get_parameter('floor_config').value
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.current_floor = 'floor1'

        # Clients
        self.load_map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Subscribe to floor change commands
        self.create_subscription(String, '/floor_change', self.on_floor_change, 10)

        self.get_logger().info(f'Floor manager ready. Current floor: {self.current_floor}')

    def on_floor_change(self, msg: String):
        target_floor = msg.data
        if target_floor == self.current_floor:
            self.get_logger().info(f'Already on {target_floor}')
            return
        if target_floor not in self.config['floors']:
            self.get_logger().error(f'Unknown floor: {target_floor}')
            return

        self.get_logger().info(f'Switching from {self.current_floor} to {target_floor}')
        floor_cfg = self.config['floors'][target_floor]

        # Step 1: Load new map
        self._load_map(floor_cfg['map_yaml'])

        # Step 2: Clear costmaps
        self._clear_costmaps()

        # Step 3: Set initial pose on new floor
        self._set_initial_pose(floor_cfg['initial_pose'])

        self.current_floor = target_floor
        self.get_logger().info(f'Floor transition to {target_floor} complete')

    def _load_map(self, map_yaml: str):
        if not self.load_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('load_map service not available')
            return
        req = LoadMap.Request()
        req.map_url = map_yaml
        future = self.load_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            self.get_logger().info(f'Map loaded: {map_yaml}')
        else:
            self.get_logger().error('Failed to load map')

    def _clear_costmaps(self):
        for client in [self.clear_global_costmap, self.clear_local_costmap]:
            if client.wait_for_service(timeout_sec=2.0):
                req = ClearEntireCostmap.Request()
                client.call_async(req)

    def _set_initial_pose(self, pose_cfg: dict):
        import math
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose_cfg['x']
        msg.pose.pose.position.y = pose_cfg['y']
        yaw = pose_cfg['yaw']
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Covariance: moderate uncertainty for AMCL to converge.
        msg.pose.covariance[0] = 0.25   # x variance
        msg.pose.covariance[7] = 0.25   # y variance
        msg.pose.covariance[35] = 0.07  # yaw variance
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'Initial pose set: x={pose_cfg["x"]}, y={pose_cfg["y"]}')


def main():
    rclpy.init()
    node = FloorTransitionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Timing Considerations

After calling `load_map`, wait briefly before publishing `initialpose`. The map publication is asynchronous—AMCL needs to receive and process the new map before it can use the initial pose. A 0.5-1.0 second delay (or better, a confirmation callback) ensures correct sequencing.

## AMCL Re-Initialization Details

When AMCL receives a new map and a new initial pose:
1. It rebuilds its internal likelihood field from the new OccupancyGrid.
2. It distributes particles around the initial pose using the provided covariance.
3. Over the next few scan cycles, particles converge to the correct pose.

Set the initial pose covariance large enough that AMCL can converge even if the pose estimate is slightly off (e.g., elevator drift). Typical values: 0.25 m² for x/y, 0.07 rad² for yaw.

## Testing Multi-Floor

1. Create maps for each floor separately using SLAM.
2. Define `initial_pose` for each floor at a known landmark (e.g., elevator exit).
3. Test the transition sequence with the robot stationary at the landmark first.
4. Then test with actual floor changes.
