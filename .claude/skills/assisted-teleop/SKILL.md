<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: assisted-teleop
description: 'Configure Nav2 assisted teleop behavior. Use when blending manual joystick control with autonomous obstacle avoidance.'
---

# AssistedTeleop Behavior

## Overview

AssistedTeleop allows a human operator to send manual velocity commands while the behavior server enforces collision avoidance by scaling down or zeroing commands that would drive the robot into obstacles. Unlike raw teleop, every command is projected forward against the costmap before execution. Plugin: `nav2_behaviors::AssistedTeleop`.

## BT Node

```xml
<AssistedTeleop server_name="behavior_server" server_timeout="10"
                error_code_id="{assisted_teleop_error_code}"/>
```

AssistedTeleop is typically invoked from the BT as a fallback mode or via an explicit operator action, not as an automatic recovery.

## How It Works

1. The behavior subscribes to a teleop velocity topic (`cmd_vel_teleop` by default).
2. Each cycle, it reads the latest incoming teleop command.
3. It projects the robot footprint forward by `projection_time` seconds at the commanded velocity.
4. If the projected footprint intersects lethal costmap cells, the command is **scaled down** proportionally. If collision is imminent, the command is zeroed entirely.
5. The (possibly scaled) command is published to `cmd_vel`.
6. The behavior runs indefinitely until cancelled by the BT or the action client.

The scaling is continuous, not binary — as the robot approaches an obstacle, commanded speed decreases smoothly, giving the operator tactile feedback through decreasing responsiveness.

## YAML Configuration

```yaml
behavior_server:
  ros__parameters:
    cycle_frequency: 10.0
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    behavior_plugins: ["spin", "backup", "wait", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    wait:
      plugin: "nav2_behaviors::Wait"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    # AssistedTeleop parameters
    projection_time: 1.0             # seconds to project command forward
    simulation_time_step: 0.1        # time granularity for projection
    cmd_vel_teleop: cmd_vel_teleop   # input topic for operator commands
    transform_tolerance: 0.1
```

## Input Topic Setup

The operator's teleop commands must be published on a **separate topic** from the navigation `cmd_vel`. The default is `cmd_vel_teleop`.

```bash
# Terminal 1: run teleop keyboard on the assisted_teleop input topic
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop

# Terminal 2: or use a joystick
ros2 launch teleop_twist_joy teleop-launch.py joy_vel:=cmd_vel_teleop
```

The behavior server reads from `cmd_vel_teleop`, applies collision filtering, and publishes the safe command to `cmd_vel`.

## BT Usage Patterns

### Manual Override Mode

Use a `Switch` or condition to enter AssistedTeleop when an operator requests control:

```xml
<Fallback name="MainControl">
  <Sequence name="OperatorTeleop">
    <Condition ID="IsManualOverrideRequested"/>
    <AssistedTeleop server_name="behavior_server"/>
  </Sequence>
  <Sequence name="AutonomousNavigation">
    <ComputePathToPose goal="{goal}" path="{path}"/>
    <FollowPath path="{path}"/>
  </Sequence>
</Fallback>
```

### Time-Limited Teleop

Use a `Timeout` decorator to limit how long the operator can control:

```xml
<Timeout msec="30000">
  <AssistedTeleop server_name="behavior_server"/>
</Timeout>
```

After 30 seconds, the BT cancels AssistedTeleop and resumes autonomous behavior.

## projection_time Tuning

| Value | Effect |
|---|---|
| 0.5 s | Minimal lookahead — operator can drive close to obstacles, higher risk |
| 1.0 s | **Recommended** — moderate safety margin |
| 2.0 s | Conservative — robot stops well away from obstacles, feels sluggish |

Lower `projection_time` gives the operator more freedom but less collision protection. For a home environment, 1.0 is a reasonable balance.

## Differences from Raw Teleop

| Feature | Raw Teleop | AssistedTeleop |
|---|---|---|
| Collision prevention | None | Costmap-based scaling |
| Costmap awareness | None | Uses local costmap |
| Speed limiting | Manual only | Automatic near obstacles |
| Requires behavior_server | No | Yes |
| Requires local_costmap | No | Yes |

## Use Cases

- **Remote monitoring**: operator views camera feed and drives the robot to investigate an alert.
- **Recovery assist**: if autonomous recovery fails repeatedly, switch to AssistedTeleop and manually drive the robot to a clear area.
- **Maintenance positioning**: drive the robot to a charging station or maintenance area with collision safety.

## Debugging

```bash
# Verify the teleop input topic is active
ros2 topic hz /cmd_vel_teleop

# Check that behavior_server is publishing filtered commands
ros2 topic echo /cmd_vel

# If commands are being zeroed, check costmap for obstacles near the robot
ros2 topic echo /local_costmap/costmap_raw --field metadata
```

If all commands are zeroed, the robot believes it is surrounded by obstacles. Check inflation radius and sensor configuration in the local costmap.
