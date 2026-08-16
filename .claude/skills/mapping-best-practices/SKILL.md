<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: mapping-best-practices
description: 'Follow SLAM mapping best practices. Use when planning a mapping run, choosing SLAM parameters, or improving map quality.'
---

# Indoor Mapping Best Practices

## Driving During Mapping

### Speed

Drive at **0.2–0.3 m/s**. Faster speeds cause:
- Scan distortion from motion (especially with low-rate lidars).
- Insufficient scan overlap between frames.
- Odometry drift from wheel slip on smooth floors.

### Straight Lines

Keep a straight path where possible. Constant turning generates scans at many angles, making scan matching harder. Long straight hallways provide excellent constraints.

### Loop Closure

**Always close loops.** Drive the robot back to the starting point before finishing. This gives the optimizer a constraint that locks the map endpoints together and redistributes drift.

For multi-room environments, drive through each doorway in both directions and return to a central location (e.g., the hallway) multiple times.

### Coverage Strategy

Map systematically:
1. Start at the charging dock or a known landmark.
2. Drive the perimeter of each room.
3. Return to the hallway (close the loop for that room).
4. Move to the next room.
5. After all rooms, return to start (close the global loop).

### Wall Distance

Stay **0.3–0.5m from walls**. Closer than 0.3m:
- Lidar returns are noisy at extreme angles.
- Min-range filtering may discard useful data.
- The robot's footprint may occlude part of the room.

## Sensor Configuration

### Use the Same Sensors for Mapping and Navigation

The map encodes occupancy as seen by your lidar. If you map with a 360° lidar but navigate with a 270° one (or vice versa), the costmap will have blind spots or phantom obstacles where the coverage differs.

**Rule: The sensor setup during mapping must match the setup during navigation.** Same lidar, same mounting height, same scan parameters.

### IMU Considerations

If your robot has an IMU:
- Calibrate it before mapping (magnetometer especially).
- Verify the IMU data in RViz (`/imu/data` topic) — watch for drift when stationary.
- If the IMU is noisy or uncalibrated, disable it in SLAM (`use_imu_data: false` for Cartographer).
- A bad IMU is worse than no IMU.

### Lidar Configuration

```yaml
# Verify before mapping:
# - scan_topic publishes at expected rate (10-20 Hz typical)
# - min_range and max_range match your SLAM config
# - No dead zones in the scan (check in RViz)
```

## Lighting and Environment

### Lidar-Only Mapping

Lidar is not affected by lighting. However:
- **Glass**: Lidar passes through glass windows/doors, producing returns from outside the building. Cap `max_range` to avoid mapping outdoor objects.
- **Mirrors**: Specular reflections create phantom walls. Cover mirrors during mapping or clean up the map afterward.
- **Transparent furniture**: Glass tables and shelves may not appear in the map. Add them manually if they're permanent obstacles.

### With Camera (Visual SLAM)

If using visual features for loop closure or SLAM:
- Map in the same lighting conditions you'll navigate in.
- Avoid mapping at night with artificial lights if the robot will run during the day.
- Consistent lighting produces better feature descriptors.

## Post-Mapping Cleanup

### GIMP Workflow

After saving the map (PGM + YAML), open the PGM in GIMP for manual cleanup:

1. **Remove exterior artifacts**: Flood-fill everything outside the building boundary with value 205 (unknown → free) or 254 (unknown). Use the bucket tool with a threshold of ~15 to select connected exterior regions.

2. **Fix wall gaps**: Zoom to doorways and thin walls. Use the pencil tool (1-3 px wide) with black (value 0) to fill small gaps. Doors that should be closed in the map can be drawn shut.

3. **Remove ghost obstacles**: Stray black pixels inside free space (from lidar noise or temporary objects during mapping). Use the pencil tool with value 205 (free) to erase them.

4. **Clean up unknown regions**: Areas just outside the lidar's reach often show as unknown (gray). If you know these are free (e.g., inside a room the lidar partially covered), fill with 205.

5. **Export**: File → Export As → PGM (Raw encoding) or PNG.

### What to Keep

| Artifact | Keep? | Reason |
|----------|-------|--------|
| Permanent walls | Yes | Critical for navigation |
| Furniture | Depends | Keep permanent furniture; remove temporary items |
| Door frames | Yes | Keep as walls; open/close state handled by costmap |
| Stray noise pixels | No | Causes phantom obstacles |
| Exterior returns (through windows) | No | Confuses path planning |
| Construction artifacts | No | Temporary |

## Resolution Tradeoff

| Resolution | Pixel Size | Pros | Cons |
|-----------|-----------|------|------|
| 0.025 m | 2.5 cm | High detail, narrow passage support | Large files, higher CPU for costmaps |
| 0.05 m | 5 cm | Good balance (recommended) | May miss very narrow gaps |
| 0.1 m | 10 cm | Small files, fast costmap updates | Low detail, wide corridors only |

**Recommended: 0.05m** for indoor home/office environments. Use 0.025m only if the robot must navigate through passages narrower than 0.5m.

## Saving Artifacts

After a successful mapping session, save:

1. **PGM/PNG + YAML**: The processed static map for navigation.
2. **Serialized graph** (SLAM Toolbox `.posegraph` + `.data`): For lifelong mode or re-mapping.
3. **Raw (unedited) PGM**: Before GIMP cleanup, in case you need to re-edit.
4. **Bag file** (optional): `ros2 bag record /scan /odom /tf /tf_static /imu/data` during mapping. Allows replaying the mapping session with different SLAM parameters.

```bash
# Complete save workflow:
ros2 run nav2_map_server map_saver_cli -f /home/robot/maps/house_raw

# Serialize graph:
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/robot/maps/house_graph'}"

# Copy raw for backup:
cp /home/robot/maps/house_raw.pgm /home/robot/maps/house_raw_backup.pgm

# Edit in GIMP, then save processed version:
# (Manual step in GIMP → export as house_clean.pgm)
```

## Validation Checklist

Before deploying a map for navigation:

- [ ] All walls are continuous (no gaps except doorways).
- [ ] No artifacts outside the building boundary.
- [ ] Robot starting position (origin) is correct.
- [ ] Map loads in RViz without errors.
- [ ] AMCL converges when the robot is placed at the start location.
- [ ] Navigation goals in all rooms are reachable (run a planner test).
- [ ] The map resolution matches the costmap resolution or is finer.
- [ ] The map is saved in PGM or PNG format (not JPEG).

## Iterative Improvement

Mapping is rarely perfect on the first try. Plan for 2-3 iterations:
1. **First pass**: Complete coverage, assess quality.
2. **Second pass**: Focus on areas with poor scan matching or missing walls.
3. **Third pass**: Drive the robot on the final map using AMCL to verify localization quality. If AMCL drifts in specific areas, re-map those areas with slower driving.
