<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: simulation-testing
description: 'Write simulation-based integration tests. Use when using launch_testing with Gazebo, headless CI testing, or automated navigation tests.'
---

# Simulation-Based Automated Testing

## Why Test in Simulation

Integration testing with real hardware is slow, non-repeatable, and risky. Gazebo simulation enables: automated CI testing, deterministic replay, testing edge cases (dynamic obstacles, sensor failure), and rapid iteration without physical robot access.

## launch_testing Framework

ROS 2 provides `launch_testing` for integration tests that launch nodes and make assertions. Tests run with `colcon test`.

### Test Structure

```
my_package/
├── test/
│   ├── test_navigation.py        # launch_testing integration test
│   └── test_navigation.launch.py # Optional separate launch file
├── CMakeLists.txt
└── package.xml
```

In `CMakeLists.txt`:

```cmake
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_navigation.py
    TIMEOUT 120
  )
endif()
```

In `package.xml`:

```xml
<test_depend>launch_testing</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
<test_depend>launch_testing_ros</test_depend>
<test_depend>ros_gz_sim</test_depend>
```

## Complete Integration Test

```python
#!/usr/bin/env python3
"""Integration test: navigate to a goal in simulation."""

import time
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node as LaunchNode

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math


@pytest.mark.launch_test
def generate_test_description():
    """Launch Gazebo + robot + Nav2 for testing."""

    world_file = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'), 'worlds', 'test_house.sdf'
    ])

    # Gazebo headless
    gz_sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': ['-r -s --headless-rendering ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Bridge
    bridge = LaunchNode(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Nav2
    nav2 = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([
            FindPackageShare('my_robot_bringup'), '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
    )

    return launch.LaunchDescription([
        gz_sim,
        bridge,
        nav2,
        # Signal that test nodes can start
        launch_testing.actions.ReadyToTest(),
    ])


class TestNavigation(unittest.TestCase):
    """Test that robot can navigate to a goal."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_navigation_node')
        cls.node.declare_parameter('use_sim_time', True)

        # Wait for /clock to confirm simulation is running
        cls._wait_for_clock(timeout=30.0)

        # Action client for NavigateToPose
        cls.nav_client = ActionClient(
            cls.node, NavigateToPose, 'navigate_to_pose'
        )

        # Wait for Nav2 to be ready
        assert cls.nav_client.wait_for_server(timeout_sec=60.0), \
            "NavigateToPose action server not available"

        # Set initial pose (AMCL)
        cls.initial_pose_pub = cls.node.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        time.sleep(2.0)  # Let publishers connect
        cls._set_initial_pose(x=1.0, y=1.0, yaw=0.0)
        time.sleep(5.0)  # Let AMCL converge

    @classmethod
    def _wait_for_clock(cls, timeout):
        """Wait for /clock to confirm Gazebo is running."""
        from rosgraph_msgs.msg import Clock
        clock_received = False

        def clock_cb(msg):
            nonlocal clock_received
            clock_received = True

        sub = cls.node.create_subscription(Clock, '/clock', clock_cb, 10)
        start = time.time()
        while not clock_received and (time.time() - start) < timeout:
            rclpy.spin_once(cls.node, timeout_sec=0.5)
        cls.node.destroy_subscription(sub)
        assert clock_received, "Gazebo /clock not received within timeout"

    @classmethod
    def _set_initial_pose(cls, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25  # x variance
        msg.pose.covariance[7] = 0.25  # y variance
        msg.pose.covariance[35] = 0.0685  # yaw variance
        cls.initial_pose_pub.publish(msg)

    def _send_goal(self, x, y, yaw=0.0, timeout=60.0):
        """Send a NavigateToPose goal and wait for result."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        goal_handle = future.result()
        self.assertTrue(goal_handle.accepted, "Goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=timeout
        )
        result = result_future.result()
        return result

    def test_navigate_to_goal(self):
        """Robot should reach (3.0, 3.0) within 60 seconds."""
        result = self._send_goal(x=3.0, y=3.0, yaw=0.0, timeout=60.0)
        self.assertEqual(
            result.status, GoalStatus.STATUS_SUCCEEDED,
            f"Navigation failed with status {result.status}"
        )

    def test_navigate_to_second_goal(self):
        """Robot should reach (1.0, 4.0) from current position."""
        result = self._send_goal(x=1.0, y=4.0, yaw=1.57, timeout=60.0)
        self.assertEqual(
            result.status, GoalStatus.STATUS_SUCCEEDED,
            f"Navigation failed with status {result.status}"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
```

## Running Tests

```bash
# Build with tests
colcon build --symlink-install --packages-select my_robot_bringup

# Run tests
colcon test --packages-select my_robot_bringup --event-handlers console_direct+

# See results
colcon test-result --verbose
```

## CI Pipeline Configuration

```yaml
# .github/workflows/sim_test.yaml
name: Simulation Tests
on: [push, pull_request]

jobs:
  sim-test:
    runs-on: ubuntu-24.04
    container:
      image: osrf/ros:jazzy-desktop
    env:
      # Software rendering for headless CI
      MESA_GL_VERSION_OVERRIDE: "3.3"
      LIBGL_ALWAYS_SOFTWARE: "1"
    steps:
      - uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y ros-jazzy-ros-gz ros-jazzy-nav2-bringup \
            ros-jazzy-launch-testing-ament-cmake
          rosdep install --from-paths . --ignore-src -y

      - name: Build
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build --symlink-install

      - name: Run simulation tests
        run: |
          source /opt/ros/jazzy/setup.bash
          source install/setup.bash
          colcon test --event-handlers console_direct+
          colcon test-result --verbose
```

## Headless Rendering

For CI without a GPU, Gazebo needs software rendering:

```bash
# Server-only (no GUI), headless rendering for sensors
gz sim -r -s --headless-rendering world.sdf

# Environment variables for CI
export MESA_GL_VERSION_OVERRIDE="3.3"
export LIBGL_ALWAYS_SOFTWARE="1"
```

Without `--headless-rendering`, GPU-based sensors (cameras, gpu_lidar) won't produce data.

## Recording Test Data

```python
import subprocess

# Start rosbag recording during test
rosbag_proc = subprocess.Popen([
    'ros2', 'bag', 'record', '-o', 'test_recording',
    '/scan', '/odom', '/cmd_vel', '/tf', '/tf_static',
    '--use-sim-time',
])

# ... run test ...

rosbag_proc.terminate()
rosbag_proc.wait()
```

## Test Scenarios

| Scenario | How to Test |
|----------|-------------|
| Navigate to goal | Send goal, assert STATUS_SUCCEEDED, check final position |
| Recovery from stuck | Place obstacle near path, verify recovery behavior triggers |
| Dynamic obstacles | Spawn/move model mid-navigation, verify replanning |
| Sensor failure | Kill bridge topic mid-test, verify graceful degradation |
| Multi-goal patrol | Send waypoint sequence, verify all reached |
| Path length | Record odometry, sum distance, assert within bounds |
| Navigation time | Record wall time between goal send and completion |

## Common CI Issues

- **Gazebo startup delay**: Always wait for `/clock` before sending goals (see `_wait_for_clock`)
- **AMCL convergence**: Set initial pose explicitly and wait 5+ seconds before navigating
- **Flaky timeouts**: Use generous timeouts (60s+) in CI where machines may be slow
- **Port conflicts**: Gazebo uses ports for IPC; run tests serially or use `--network host`
- **Stale processes**: CI may leave zombie gz processes; add cleanup in test teardown

## Deterministic Testing

For reproducible results:
- Set fixed random seeds in Gazebo SDF: `<seed>42</seed>` in physics plugin
- Use pause-step mode to control exact simulation progression
- Avoid time-dependent assertions (use goal status instead of position checks where possible)
- Pin Gazebo and ROS 2 versions in CI container
