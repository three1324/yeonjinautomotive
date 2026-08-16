<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: range-sensor-layer
description: 'Configure Nav2 range sensor layer. Use when integrating ultrasonic, IR, or VL53L0X time-of-flight range sensors into the costmap.'
---

# RangeSensorLayer

## Purpose

RangeSensorLayer integrates short-range sensors (ultrasonic, IR, VL53L0X time-of-flight) that publish `sensor_msgs/Range` messages. These sensors detect obstacles that lidar may miss: low obstacles below the lidar plane, transparent surfaces (glass), and very close objects inside the lidar's minimum range.

## Configuration

```yaml
range_sensor_layer:
  plugin: "nav2_costmap_2d::RangeSensorLayer"
  enabled: true
  topics: ["/range/front_left", "/range/front_right", "/range/rear"]
  phi: 0.087                    # Sensor cone half-angle in radians
  inflate_cone: 1.0             # Multiplier for inflating the cone width
  no_readings_timeout: 1.0      # Seconds before marking sensor as stale
  clear_threshold: 0.2          # Probability below this → cell is cleared
  mark_threshold: 0.8           # Probability above this → cell is marked
  clear_on_max_reading: true    # Max range reading clears cells in cone
  combination_method: 1         # Max — prevents lidar from clearing range marks
```

## Parameters Explained

### phi (sensor cone half-angle)

The angular width of the sensor's detection cone, in radians. Each range reading marks/clears cells within this cone.

| Sensor type | Typical phi | Notes |
|---|---|---|
| VL53L0X (ToF) | 0.087 (5°) | Very narrow beam, use small phi |
| VL53L1X (ToF) | 0.15 (8.5°) | Slightly wider FoV |
| HC-SR04 (sonar) | 0.26 (15°) | Wide cone, may cause false positives |
| Sharp IR | 0.04 (2.3°) | Very narrow |

**Common issue**: Setting phi too wide creates false-positive wedges in the costmap. For VL53L0X sensors, use `phi: 0.087` or narrower.

### inflate_cone

Multiplier applied to phi when marking obstacles. At `1.0`, the marked region matches the sensor cone. Values > 1.0 widen the mark zone for extra safety margin. Values < 1.0 narrow it.

### clear_threshold and mark_threshold

The range sensor layer maintains a probabilistic model for each cell in its observation zone:

- Each range reading updates the probability based on the sensor model
- Reading < max_range at distance D: cells at D get increased probability (toward occupied), cells between sensor and D get decreased probability (toward free)
- Cells with probability ≥ `mark_threshold` are marked as obstacles
- Cells with probability ≤ `clear_threshold` are cleared
- Cells between thresholds retain their current state

Higher `mark_threshold` (e.g., 0.9) requires more consistent readings before marking — reduces false positives but slower to react.
Lower `clear_threshold` (e.g., 0.1) requires stronger evidence to clear — more conservative.

### clear_on_max_reading

When `true`, a reading at the sensor's maximum range is interpreted as "nothing detected in cone" and clears all cells in the cone. When `false`, max-range readings are ignored.

Set `true` for sensors that reliably return max_range when nothing is present (most ToF sensors). Set `false` for sensors that return noisy max-range values (some ultrasonics).

### no_readings_timeout

If no Range message is received for this duration (seconds), the sensor is marked stale and its data is no longer used. Set to ~2× the expected publish period. If the sensor publishes at 10 Hz, set to `0.5` or `1.0`.

## combination_method: MUST Be 1 (Maximum)

This is critical. RangeSensorLayer typically runs alongside VoxelLayer or ObstacleLayer. If combination_method is 0 (Overwrite), range sensor data would be overwritten by the next layer, or this layer would overwrite lidar data.

With `combination_method: 1` (Maximum), range sensor marks persist even if the lidar doesn't see the obstacle (e.g., glass, low object). The maximum of all layers is kept.

## topics Parameter

An array of Range topics. Each topic is subscribed independently. All range data feeds into the same probabilistic model.

```yaml
topics: ["/range/front_left", "/range/front_right",
         "/range/front_center", "/range/rear_left", "/range/rear_right"]
```

Each topic must publish `sensor_msgs/Range` with:
- `header.frame_id`: TF frame of the sensor
- `radiation_type`: 0 (ultrasound) or 1 (infrared)
- `field_of_view`: Used if phi is not set (but phi parameter overrides)
- `min_range`, `max_range`: Operating range of the sensor
- `range`: Current distance reading

## Complete Example with VL53L0X Sensors

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "range_sensor_layer", "inflation_layer"]

      range_sensor_layer:
        plugin: "nav2_costmap_2d::RangeSensorLayer"
        enabled: true
        topics: ["/range/front_left", "/range/front_right", "/range/rear"]
        phi: 0.087
        inflate_cone: 1.0
        no_readings_timeout: 1.0
        clear_threshold: 0.2
        mark_threshold: 0.8
        clear_on_max_reading: true
        combination_method: 1
```

## Debugging

- **Wide obstacle marks**: phi is too large. Reduce to match actual sensor beam width.
- **Obstacles don't clear**: Check `clear_on_max_reading: true`. Verify sensor actually publishes max_range when clear.
- **No marks appear**: Check sensor topic is publishing: `ros2 topic echo /range/front_left --once`. Verify the sensor frame is in the TF tree.
- **Marks cleared by lidar**: This happens when lidar's raytrace passes through the range sensor's obstacle. Ensure range_sensor_layer is AFTER voxel_layer in plugins list and uses `combination_method: 1`.
- **Stale warnings**: Sensor driver not running or topic misconfigured. Check `no_readings_timeout` value.

## Use Case: Glass Door Detection

Lidar passes through glass. Range sensors (ToF, ultrasonic) detect it. Configuration:

```yaml
# In local costmap only — global costmap uses the static map for doors
range_sensor_layer:
  plugin: "nav2_costmap_2d::RangeSensorLayer"
  topics: ["/range/front_center"]
  phi: 0.087
  mark_threshold: 0.7    # Slightly lower threshold for faster detection
  clear_threshold: 0.3
  clear_on_max_reading: true
  combination_method: 1
```

Place the ToF sensor at the height of the glass panel. The range sensor marks the glass as an obstacle even though lidar sees through it.
