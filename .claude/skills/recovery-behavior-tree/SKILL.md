<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: recovery-behavior-tree
description: 'Build recovery behavior trees. Use when designing fallback recovery sequences for stuck detection and resolution.'
---

# Recovery Behavior Tree Design

## The Nav2 Recovery Pattern

Nav2's default recovery architecture uses a **RecoveryNode** (a special control node) that wraps normal navigation in one branch and recovery actions in the other. When navigation fails, recovery actions are tried in sequence with escalating aggressiveness.

## RecoveryNode Semantics

`RecoveryNode` has exactly two children:
1. **Child 1 (left)**: The normal operation branch
2. **Child 2 (right)**: The recovery branch

Behavior:
- Ticks child 1 (normal operation)
- If child 1 returns FAILURE, ticks child 2 (recovery)
- If child 2 returns SUCCESS, retries child 1
- Repeats up to `number_of_retries` times
- Returns FAILURE if retries are exhausted or child 2 fails

```xml
<RecoveryNode number_of_retries="6" name="NavigateRecovery">
  <!-- Child 1: Normal navigation pipeline -->
  <PipelineSequence name="NavigateWithReplanning">
    ...
  </PipelineSequence>

  <!-- Child 2: Recovery actions -->
  <ReactiveFallback name="RecoveryFallback">
    ...
  </ReactiveFallback>
</RecoveryNode>
```

## Complete Recovery Tree (Nav2 Default Pattern)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">

  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">

      <!-- ========== NORMAL OPERATION ========== -->
      <PipelineSequence name="NavigateWithReplanning">
        <!-- Rate-limited replanning during path following -->
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathRecovery">
            <ComputePathToPose goal="{goal}" path="{path}"
                               planner_id="GridBased"
                               error_code_id="{compute_path_error_code}" />
            <Sequence name="ClearAndReplan">
              <ClearEntireCostmap
                server_name="global_costmap/clear_entirely_global_costmap" />
              <ComputePathToPose goal="{goal}" path="{path}"
                                 planner_id="GridBased"
                                 error_code_id="{compute_path_error_code}" />
            </Sequence>
          </RecoveryNode>
        </RateController>

        <!-- Follow the computed path -->
        <RecoveryNode number_of_retries="1" name="FollowPathRecovery">
          <FollowPath path="{path}" controller_id="FollowPath"
                      error_code_id="{follow_path_error_code}" />
          <Sequence name="ClearAndFollow">
            <ClearEntireCostmap
              server_name="local_costmap/clear_entirely_local_costmap" />
            <FollowPath path="{path}" controller_id="FollowPath"
                        error_code_id="{follow_path_error_code}" />
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>

      <!-- ========== RECOVERY ACTIONS ========== -->
      <ReactiveFallback name="RecoveryFallback">
        <!-- If the goal was updated, skip recovery entirely -->
        <GoalUpdated />

        <!-- RoundRobin cycles through recovery strategies -->
        <RoundRobin name="RecoveryStrategies">

          <!-- Strategy 1: Clear costmaps -->
          <Sequence name="ClearCostmaps">
            <ClearEntireCostmap
              server_name="global_costmap/clear_entirely_global_costmap" />
            <ClearEntireCostmap
              server_name="local_costmap/clear_entirely_local_costmap" />
          </Sequence>

          <!-- Strategy 2: Spin in place -->
          <Spin spin_dist="1.57" server_name="spin"
                error_code_id="{spin_error_code}" />

          <!-- Strategy 3: Wait for obstacles to clear -->
          <Wait wait_duration="5.0" server_name="wait" />

          <!-- Strategy 4: Back up -->
          <BackUp backup_dist="0.3" backup_speed="0.15"
                  server_name="backup"
                  error_code_id="{backup_error_code}" />

        </RoundRobin>
      </ReactiveFallback>

    </RecoveryNode>
  </BehaviorTree>

</root>
```

## Execution Trace: Recovery in Action

### Scenario: Robot blocked by unexpected obstacle

| Step | Component           | Action                                       | Result    |
|------|---------------------|----------------------------------------------|-----------|
| 1    | NavigateRecovery    | Ticks PipelineSequence                       | —         |
| 2    | ComputePathToPose   | Plans path to goal                           | SUCCESS   |
| 3    | FollowPath          | Follows path, hits obstacle                  | FAILURE   |
| 4    | FollowPathRecovery  | Clears local costmap, retries FollowPath     | FAILURE   |
| 5    | PipelineSequence    | Returns FAILURE (follow failed twice)        | FAILURE   |
| 6    | NavigateRecovery    | Ticks RecoveryFallback (attempt 1/6)         | —         |
| 7    | GoalUpdated         | No new goal                                  | FAILURE   |
| 8    | RoundRobin [1]      | ClearCostmaps                                | SUCCESS   |
| 9    | NavigateRecovery    | Retries PipelineSequence                     | —         |
| 10   | ComputePathToPose   | Replans (costmaps cleared, may find new path)| SUCCESS   |
| 11   | FollowPath          | Follows new path... still blocked            | FAILURE   |
| 12   | NavigateRecovery    | Ticks RecoveryFallback (attempt 2/6)         | —         |
| 13   | RoundRobin [2]      | Spin 1.57 radians                            | SUCCESS   |
| 14   | NavigateRecovery    | Retries PipelineSequence                     | —         |
| 15   | FollowPath          | Following... SUCCESS (obstacle cleared)      | SUCCESS   |

RoundRobin cycles through strategies: clear → spin → wait → backup → clear → spin → ...

## Escalating Recovery Pattern

For more aggressive recovery, build a fallback chain that escalates:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="EscalatingRecovery">

  <BehaviorTree ID="EscalatingRecovery">
    <RecoveryNode number_of_retries="4" name="MainRecovery">

      <!-- Normal navigation -->
      <SubTree ID="NavigationPipeline" goal="{goal}" path="{path}" />

      <!-- Escalating recovery fallback -->
      <Sequence name="EscalatingRecovery">

        <!-- Level 1: Gentle — clear costmaps -->
        <ForceSuccess>
          <Sequence>
            <ClearEntireCostmap
              server_name="global_costmap/clear_entirely_global_costmap" />
            <ClearEntireCostmap
              server_name="local_costmap/clear_entirely_local_costmap" />
          </Sequence>
        </ForceSuccess>

        <!-- Level 2: Try planning again with cleared maps -->
        <Fallback name="TryReplan">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
                             error_code_id="{plan_error}" />

          <!-- Level 3: Physical recovery — spin to clear surroundings -->
          <Sequence>
            <ForceSuccess>
              <Spin spin_dist="3.14" server_name="spin"
                    error_code_id="{spin_error}" />
            </ForceSuccess>
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
                               error_code_id="{plan_error}" />
          </Sequence>
        </Fallback>

        <!-- If we still can't plan, the Fallback above fails.
             Next RecoveryNode retry will escalate further: -->

        <!-- Level 4 (on retry 3): Back up and try again -->
        <!-- Level 5 (on retry 4): Relocalization as last resort -->

      </Sequence>

    </RecoveryNode>
  </BehaviorTree>

  <BehaviorTree ID="NavigationPipeline">
    <PipelineSequence>
      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
                         error_code_id="{plan_error}" />
      <FollowPath path="{path}" controller_id="FollowPath"
                  error_code_id="{follow_error}" />
    </PipelineSequence>
  </BehaviorTree>

</root>
```

## GoalUpdated: The Recovery Circuit Breaker

`GoalUpdated` is critical inside recovery fallbacks. Without it, if the user sends a new goal while recovery is running, the robot wastes time completing recovery for the old goal:

```xml
<ReactiveFallback name="RecoveryFallback">
  <!-- SHORT CIRCUIT: new goal cancels recovery -->
  <GoalUpdated />
  <!-- Only run recovery if goal hasn't changed -->
  <RoundRobin>
    <ClearEntireCostmap ... />
    <Spin ... />
    <Wait ... />
  </RoundRobin>
</ReactiveFallback>
```

When `GoalUpdated` returns SUCCESS, the ReactiveFallback returns SUCCESS immediately. The RecoveryNode then retries the navigation pipeline with the new goal.

## Common Mistakes

### Infinite Recovery Loop

```xml
<!-- BAD: Recovery always succeeds, navigation always fails → loops forever -->
<RecoveryNode number_of_retries="999">
  <NavigateToPose goal="{unreachable_goal}" />
  <ForceSuccess>
    <ClearEntireCostmap ... />
  </ForceSuccess>
</RecoveryNode>
```

Fix: Use a reasonable `number_of_retries` (3–6) and let the tree propagate failure to a higher level that can choose a different goal.

### Recovery That Blocks Indefinitely

```xml
<!-- BAD: Wait with no timeout in recovery -->
<Wait wait_duration="300.0" />
```

Fix: Keep recovery wait times short (5–15 seconds). If the obstacle hasn't cleared after a few waits, escalate.

### Not Checking GoalUpdated

```xml
<!-- BAD: No GoalUpdated check — recovery runs even after goal changes -->
<RoundRobin>
  <Spin ... />
  <Wait ... />
  <BackUp ... />
</RoundRobin>
```

Fix: Always wrap recovery in a `ReactiveFallback` with `GoalUpdated` as the first child.

## PipelineSequence vs Sequence

`PipelineSequence` is a Nav2-specific control node that differs from `Sequence`:

- In `PipelineSequence`, when child N is ticked, children 1..N-1 are also re-ticked
- This enables the planner (child 1) to re-plan while the controller (child 2) is following

```xml
<!-- PipelineSequence: planner re-runs alongside controller -->
<PipelineSequence>
  <RateController hz="1.0">
    <ComputePathToPose goal="{goal}" path="{path}" />
  </RateController>
  <FollowPath path="{path}" />
</PipelineSequence>
```

As `FollowPath` returns RUNNING, `ComputePathToPose` is re-ticked (rate-limited to 1 Hz) to update the path. This is Nav2's continuous replanning mechanism.

## Integrating Custom Recovery Actions

Add custom recovery behaviors (e.g., "call for help") to the RoundRobin:

```xml
<RoundRobin name="RecoveryStrategies">
  <ClearEntireCostmap ... />
  <Spin spin_dist="1.57" ... />
  <Wait wait_duration="5.0" ... />
  <BackUp backup_dist="0.3" ... />
  <!-- Custom: send distress signal after standard recoveries fail -->
  <SendNotification message="Navigation stuck, requesting assistance"
                    priority="high" />
</RoundRobin>
```

The RoundRobin ensures each strategy is tried once per recovery cycle before repeating.
