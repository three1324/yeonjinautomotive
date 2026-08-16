<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: multi-goal-navigation-bt
description: 'Build multi-goal navigation behavior trees. Use when sequencing multiple NavigateToPose goals or waypoint following in a BT.'
---

# Multi-Goal Navigation Behavior Trees

## Approaches to Multi-Goal Navigation

Nav2 provides two main mechanisms for visiting multiple goals:

1. **NavigateThroughPoses** — Sends all goals to the planner at once, produces a single optimized path through all waypoints
2. **Sequential NavigateToPose** — BT visits each goal independently with full plan-follow cycles
3. **Waypoint Follower** — Nav2's built-in waypoint following action server with task execution plugins

Each has different trade-offs for path optimality, per-waypoint task execution, and failure handling.

## NavigateThroughPoses

Sends a vector of poses to the planner. The planner computes a single path that visits all poses in order:

```xml
<NavigateThroughPoses goals="{goal_poses}"
                      server_name="navigate_through_poses"
                      error_code_id="{nav_error}" />
```

**Pros**: Globally optimized path, smoother motion between waypoints, fewer planning calls.
**Cons**: No per-waypoint task execution (can't stop at each waypoint to take a photo), all-or-nothing failure handling—if one waypoint is unreachable, the entire action fails.

### Setting Up Goal Poses

Goals must be a `vector<PoseStamped>` on the blackboard. Set them from a parameter or programmatically:

```xml
<SetBlackboard output_key="goal_poses" value="
  1.0;2.0;0.0;0.0;0.0;0.0;1.0|
  3.0;-1.0;0.0;0.0;0.0;0.7;0.7|
  5.0;0.5;0.0;0.0;0.0;1.0;0.0" />
<NavigateThroughPoses goals="{goal_poses}" />
```

The format is `x;y;z;qx;qy;qz;qw` separated by `|` for each pose.

## Sequential NavigateToPose with Per-Goal Tasks

For executing different actions at each waypoint, use a Sequence of NavigateToPose calls with interleaved task nodes:

```xml
<BehaviorTree ID="SequentialPatrol">
  <Sequence name="VisitAllRooms">

    <!-- Waypoint 1: Living Room — take panoramic photo -->
    <NavigateToPose goal="{living_room_pose}" error_code_id="{nav_error}" />
    <Spin spin_dist="6.28" server_name="spin" />
    <CaptureImage camera_topic="/camera/image" />

    <!-- Waypoint 2: Kitchen — check for smoke -->
    <NavigateToPose goal="{kitchen_pose}" error_code_id="{nav_error}" />
    <CheckSmokeSensor sensor_topic="/smoke_detector" result="{smoke_detected}" />

    <!-- Waypoint 3: Front Door — verify locked -->
    <NavigateToPose goal="{front_door_pose}" error_code_id="{nav_error}" />
    <CheckDoorSensor door_id="front" result="{door_locked}" />
    <Wait wait_duration="3.0" server_name="wait" />

    <!-- Waypoint 4: Garage — temperature check -->
    <NavigateToPose goal="{garage_pose}" error_code_id="{nav_error}" />
    <ReadTemperature sensor_topic="/garage/temp" value="{garage_temp}" />

  </Sequence>
</BehaviorTree>
```

### Adding Recovery Per Waypoint

Wrap each navigation in a recovery sub-tree:

```xml
<Retry num_attempts="2">
  <Sequence>
    <NavigateToPose goal="{kitchen_pose}" error_code_id="{nav_error}" />
    <CheckSmokeSensor sensor_topic="/smoke_detector" />
  </Sequence>
</Retry>
```

Or use `ForceSuccess` to skip unreachable waypoints without aborting the patrol:

```xml
<ForceSuccess>
  <Retry num_attempts="2">
    <NavigateToPose goal="{kitchen_pose}" error_code_id="{nav_error}" />
  </Retry>
</ForceSuccess>
<!-- Continue to next waypoint even if kitchen navigation fails -->
<NavigateToPose goal="{front_door_pose}" error_code_id="{nav_error}" />
```

## Dynamic Goal Lists

### Pattern: External Goal Updates via Blackboard

A ROS 2 node publishes goals to a topic. A custom BT condition or action reads the topic and updates the blackboard:

```xml
<ReactiveSequence name="DynamicNavigation">
  <!-- Custom node: reads /patrol_goals topic, updates blackboard -->
  <UpdateGoalsFromTopic topic="/patrol_goals"
                        goals_key="dynamic_goals"
                        current_index_key="goal_idx" />

  <!-- Navigate to the current goal -->
  <NavigateToPose goal="{current_goal}" error_code_id="{nav_error}" />
</ReactiveSequence>
```

### Pattern: Index-Based Waypoint Cycling

Use Script nodes to cycle through waypoints by index:

```xml
<BehaviorTree ID="IndexedPatrol">
  <KeepRunningUntilFailure>
    <Sequence>
      <!-- Select current waypoint based on index -->
      <Script code="
        wp := waypoints[goal_idx];
        goal_idx := (goal_idx + 1) % num_waypoints
      " />

      <!-- Navigate to selected waypoint -->
      <SubTree ID="NavigateWithRecovery" target_pose="{wp}" />

      <!-- Per-waypoint task -->
      <Wait wait_duration="2.0" server_name="wait" />
    </Sequence>
  </KeepRunningUntilFailure>
</BehaviorTree>
```

**Note**: BT.CPP v4's script engine supports array indexing only if the blackboard variable holding the array is properly typed. In practice, many teams use a custom action node to fetch the i-th waypoint from a parameter server.

## Waypoint Follower

Nav2's `FollowWaypoints` action server provides built-in waypoint following with task execution plugins:

```xml
<FollowWaypoints goals="{waypoint_list}"
                 server_name="follow_waypoints"
                 error_code_id="{follower_error}" />
```

Configure waypoint task executor plugins in `nav2_params.yaml`:

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 2000  # ms
```

Available plugins:
- `WaitAtWaypoint` — Pauses for a configured duration at each waypoint
- `PhotoAtWaypoint` — Takes a photo at each waypoint
- `InputAtWaypoint` — Waits for external input at each waypoint

### Waypoint Follower vs BT-Based Sequencing

| Feature                    | FollowWaypoints         | BT Sequencing           |
|----------------------------|-------------------------|-------------------------|
| Per-waypoint tasks         | Plugin-based (limited)  | Full BT flexibility     |
| Skip failed waypoints      | `stop_on_failure: false`| ForceSuccess wrapper    |
| Custom recovery per wp     | Standard Nav2 recovery  | Custom per-waypoint BT  |
| Dynamic goal insertion     | Must restart action     | Blackboard update       |
| Groot2 visibility          | Single action node      | Full tree visible       |
| Implementation effort      | Minimal                 | More XML authoring      |

## Complete Multi-Goal with Recovery Example

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="MultiGoalPatrol">

  <BehaviorTree ID="MultiGoalPatrol">
    <SequenceWithMemory name="PatrolAllGoals">

      <!-- Goal 1 with recovery (required — failure aborts patrol) -->
      <SubTree ID="GoToWithRecovery"
               target="{goal_1}" max_retries="3" />

      <!-- Goal 2 (optional — skip on failure) -->
      <ForceSuccess>
        <SubTree ID="GoToWithRecovery"
                 target="{goal_2}" max_retries="2" />
      </ForceSuccess>

      <!-- Goal 3 with task execution -->
      <Sequence>
        <SubTree ID="GoToWithRecovery"
                 target="{goal_3}" max_retries="3" />
        <Wait wait_duration="5.0" server_name="wait" />
        <Spin spin_dist="6.28" server_name="spin" />
      </Sequence>

      <!-- Goal 4 (required) -->
      <SubTree ID="GoToWithRecovery"
               target="{goal_4}" max_retries="3" />

    </SequenceWithMemory>
  </BehaviorTree>

  <BehaviorTree ID="GoToWithRecovery">
    <RecoveryNode number_of_retries="{max_retries}" name="NavRecovery">
      <Sequence>
        <ComputePathToPose goal="{target}" path="{path}" planner_id="GridBased"
                           error_code_id="{plan_err}" />
        <FollowPath path="{path}" controller_id="FollowPath"
                    error_code_id="{follow_err}" />
      </Sequence>
      <Sequence>
        <ClearEntireCostmap
          server_name="global_costmap/clear_entirely_global_costmap" />
        <ClearEntireCostmap
          server_name="local_costmap/clear_entirely_local_costmap" />
        <Wait wait_duration="2.0" server_name="wait" />
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>

</root>
```

**Key pattern**: `SequenceWithMemory` remembers completed waypoints. If the tree is interrupted (e.g., battery check) and resumed, it picks up from the last incomplete waypoint rather than restarting from goal 1. This is essential for multi-goal patrols to make forward progress.
