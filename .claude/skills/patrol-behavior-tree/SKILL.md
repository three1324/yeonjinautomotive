<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: patrol-behavior-tree
description: 'Build a patrol behavior tree. Use when creating a BT for cyclic waypoint patrol with recovery and battery monitoring.'
---

# Patrol Behavior Tree Design

## Architecture Overview

A house patrol BT needs four layers: mission control (loop + battery monitoring), waypoint sequencing, navigation with recovery, and anomaly response. The tree uses `ReactiveSequence` at the top level to continuously monitor battery state while `KeepRunningUntilFailure` drives the infinite patrol loop.

## Complete Patrol XML

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="HousePatrol">

  <!-- ============================================================
       MAIN TREE: House Patrol with Battery Monitoring
       ============================================================ -->
  <BehaviorTree ID="HousePatrol">
    <ReactiveFallback name="MissionControl">

      <!-- PRIORITY 1: Battery critical — go charge immediately -->
      <ReactiveSequence name="BatteryEmergency">
        <IsBatteryLow battery_topic="/battery_state" min_battery="0.10"
                      is_voltage="false" />
        <NavigateToPose goal="{charger_pose}" server_name="navigate_to_pose"
                        error_code_id="{nav_error}" />
      </ReactiveSequence>

      <!-- PRIORITY 2: Intruder detected — handle anomaly -->
      <ReactiveSequence name="IntruderResponse">
        <IsIntruderDetected detection_topic="/oak_d/detections"
                           max_distance="5.0" target_class="person"
                           intruder_distance="{intruder_dist}" />
        <SubTree ID="HandleAnomaly" />
      </ReactiveSequence>

      <!-- PRIORITY 3: Normal patrol loop -->
      <ReactiveSequence name="PatrolWithBatteryGuard">
        <!-- Abort patrol if battery gets low (but not critical) -->
        <Inverter>
          <IsBatteryLow battery_topic="/battery_state" min_battery="0.20"
                        is_voltage="false" />
        </Inverter>
        <SubTree ID="PatrolLoop" />
      </ReactiveSequence>

      <!-- PRIORITY 4: Battery low (non-critical) — go charge -->
      <Sequence name="GoCharge">
        <NavigateToPose goal="{charger_pose}" server_name="navigate_to_pose"
                        error_code_id="{nav_error}" />
        <!-- Wait at charger until battery is above 80% -->
        <Inverter>
          <IsBatteryLow battery_topic="/battery_state" min_battery="0.80"
                        is_voltage="false" />
        </Inverter>
      </Sequence>

    </ReactiveFallback>
  </BehaviorTree>

  <!-- ============================================================
       PATROL LOOP: Cycle through all rooms indefinitely
       ============================================================ -->
  <BehaviorTree ID="PatrolLoop">
    <KeepRunningUntilFailure>
      <Sequence name="FullPatrolCycle">

        <!-- Initialize patrol cycle -->
        <Script code="patrol_cycle := patrol_cycle + 1" />

        <!-- Room 1: Living Room -->
        <SubTree ID="NavigateWithRecovery"
                 target_pose="{living_room_pose}" room_name="living_room" />

        <!-- Room 2: Kitchen -->
        <SubTree ID="NavigateWithRecovery"
                 target_pose="{kitchen_pose}" room_name="kitchen" />

        <!-- Room 3: Bedroom -->
        <SubTree ID="NavigateWithRecovery"
                 target_pose="{bedroom_pose}" room_name="bedroom" />

        <!-- Room 4: Hallway -->
        <SubTree ID="NavigateWithRecovery"
                 target_pose="{hallway_pose}" room_name="hallway" />

      </Sequence>
    </KeepRunningUntilFailure>
  </BehaviorTree>

  <!-- ============================================================
       NAVIGATE WITH RECOVERY: Navigate to a single goal with retries
       ============================================================ -->
  <BehaviorTree ID="NavigateWithRecovery">
    <Retry num_attempts="3">
      <Sequence name="PlanAndFollow">

        <!-- Plan the path -->
        <ComputePathToPose goal="{target_pose}" path="{nav_path}"
                           planner_id="GridBased"
                           server_name="compute_path_to_pose"
                           error_code_id="{plan_error}" />

        <!-- Optionally smooth it -->
        <SmoothPath unsmoothed_path="{nav_path}" smoothed_path="{nav_path}"
                    smoother_id="simple_smoother"
                    server_name="smooth_path"
                    error_code_id="{smooth_error}" />

        <!-- Follow with path validity monitoring -->
        <ReactiveSequence name="MonitoredFollow">
          <RateController hz="2.0">
            <IsPathValid path="{nav_path}" server_name="compute_path_to_pose" />
          </RateController>
          <FollowPath path="{nav_path}" controller_id="FollowPath"
                      server_name="follow_path"
                      error_code_id="{follow_error}" />
        </ReactiveSequence>

      </Sequence>
    </Retry>
  </BehaviorTree>

  <!-- ============================================================
       HANDLE ANOMALY: Response when intruder is detected
       ============================================================ -->
  <BehaviorTree ID="HandleAnomaly">
    <Sequence name="AnomalyResponse">
      <!-- Stop and face the detection -->
      <Wait wait_duration="2.0" server_name="wait" />

      <!-- Log the event (custom action node) -->
      <LogEvent event_type="intruder_detected"
                distance="{intruder_dist}"
                timestamp="{current_time}" />

      <!-- Send notification (custom action node) -->
      <SendNotification message="Intruder detected at {intruder_dist}m"
                        priority="high" />

      <!-- Take a photo for evidence (custom action node) -->
      <CaptureImage output_path="/patrol_logs/detections/"
                    camera_topic="/oak_d/rgb/image_raw" />

      <!-- Wait and re-evaluate — the intruder may leave -->
      <Wait wait_duration="10.0" server_name="wait" />
    </Sequence>
  </BehaviorTree>

  <!-- ============================================================
       NODE MODELS (for Groot2)
       ============================================================ -->
  <TreeNodesModel>
    <Condition ID="IsIntruderDetected">
      <input_port name="detection_topic" type="std::string" />
      <input_port name="max_distance" type="double" />
      <input_port name="target_class" type="std::string" />
      <output_port name="intruder_distance" type="double" />
    </Condition>
    <Action ID="LogEvent">
      <input_port name="event_type" type="std::string" />
      <input_port name="distance" type="double" />
      <input_port name="timestamp" type="std::string" />
    </Action>
    <Action ID="SendNotification">
      <input_port name="message" type="std::string" />
      <input_port name="priority" type="std::string" />
    </Action>
    <Action ID="CaptureImage">
      <input_port name="output_path" type="std::string" />
      <input_port name="camera_topic" type="std::string" />
    </Action>
  </TreeNodesModel>

</root>
```

## Blackboard Initialization

The tree expects these blackboard variables to be set before execution. Typically done in the BT navigator node or via `<SetBlackboard>` at the tree start:

```xml
<Sequence name="Initialize">
  <Script code="patrol_cycle := 0" />
  <SetBlackboard output_key="charger_pose" value="0.5;0.0;0.0;0.0;0.0;0.0;1.0" />
  <SetBlackboard output_key="living_room_pose" value="3.0;1.0;0.0;0.0;0.0;0.7;0.7" />
  <SetBlackboard output_key="kitchen_pose" value="5.0;-1.0;0.0;0.0;0.0;0.0;1.0" />
  <SetBlackboard output_key="bedroom_pose" value="1.0;4.0;0.0;0.0;0.0;-0.7;0.7" />
  <SetBlackboard output_key="hallway_pose" value="2.5;2.0;0.0;0.0;0.0;1.0;0.0" />
</Sequence>
```

In practice, waypoints are loaded from a YAML parameter file in the BT navigator configuration and injected into the blackboard programmatically.

## Execution Flow Analysis

### Normal Operation
1. `MissionControl` (ReactiveFallback) ticks children in order
2. `BatteryEmergency`: IsBatteryLow(0.10) → FAILURE (battery OK) → Sequence FAILURE
3. `IntruderResponse`: IsIntruderDetected → FAILURE (no intruder) → Sequence FAILURE
4. `PatrolWithBatteryGuard`: Inverter(IsBatteryLow(0.20)) → SUCCESS → PatrolLoop ticked
5. PatrolLoop navigates through rooms via KeepRunningUntilFailure

### Battery Gets Low (< 20%)
1. `BatteryEmergency`: FAILURE (above 10%)
2. `IntruderResponse`: FAILURE (no intruder)
3. `PatrolWithBatteryGuard`: Inverter(IsBatteryLow(0.20)) → FAILURE (battery IS low)
4. **ReactiveSequence halts PatrolLoop** mid-navigation
5. `GoCharge`: NavigateToPose to charger, then wait until battery > 80%

### Intruder Detected During Patrol
1. `BatteryEmergency`: FAILURE (battery OK)
2. `IntruderResponse`: IsIntruderDetected → SUCCESS → HandleAnomaly runs
3. **ReactiveFallback returns SUCCESS** — patrol is halted
4. HandleAnomaly logs, notifies, captures image, waits
5. After HandleAnomaly completes, ReactiveFallback re-evaluates from top

### Battery Critical (< 10%) During Anomaly Response
1. `BatteryEmergency`: IsBatteryLow(0.10) → SUCCESS → NavigateToPose to charger
2. **ReactiveFallback returns SUCCESS immediately** — anomaly handling is preempted
3. Robot goes to charger regardless of intruder (safety first)

## Design Decisions

- **ReactiveFallback at the top**: Ensures battery emergency always takes priority over all other behaviors, re-evaluated every tick
- **Two battery thresholds**: 10% (emergency, interrupt everything) and 20% (graceful, finish current action then charge)
- **Retry(3) on navigation**: Handles transient obstacles. After 3 failures, the patrol moves to the next room
- **IsPathValid at 2 Hz**: Balances replanning responsiveness vs computational cost
- **KeepRunningUntilFailure for patrol**: Loops indefinitely. Only stops if a navigation failure persists through all retries
