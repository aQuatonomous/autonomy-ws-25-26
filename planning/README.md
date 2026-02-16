# Planning

High-level task planning and goal generation for autonomous navigation. Consumes perception (detections, fused buoys) and produces goals for the control stack.

---

## Structure

- **TaskMaster.py** — Main lifecycle manager: creates the task manager (Task1/2/3), runs a loop that updates pose/detections, calls `mgr.tick()`, publishes goals, and optionally runs potential-fields planning. Exits when `mgr.done()` → SUCCESS.
- **Global/** — Task managers (`Task1.py`, `Task2.py`, `Task3.py`) and shared entities (`entities.py`). Load task entities from JSON and drive goal queues.
- **Local/** — `potential_fields_planner.py`: potential-fields planner used when planning is enabled.
- **Input_Entities/** — JSON entity files per task (`task1_entities.json`, `task2_entities.json`, `task3_entities.json`).

---

## What the planning stack expects

- **Pose:** Current (x, y, heading) from the robot.
- **Detections:** List of detected entities (e.g. buoys, debris, indicators) with position and type. In the full system these come from the **mapping** and **computer vision** stacks: e.g. **`/fused_buoys`** (CV–LiDAR fusion) or **`/combined/detection_info_with_distance`** (CV only). The planning code expects a list of `DetectedEntity` (entity_id, entity_type, position).

---

## How to run

- **With real/sim perception:** Run the CV and mapping pipelines so `/fused_buoys` or `/combined/detection_info_with_distance` is published; then run the planning node or main script that subscribes to those topics and calls `TaskMaster` with `get_pose` and `get_detections` callbacks.
- **Standalone test (no ROS):** Use the test harness with JSON entity files:
  ```bash
  cd ~/autonomy-ws-25-26/planning
  python test.py
  ```
  `test.py` uses `Input_Entities/task3_entities.json` by default and a `FakePerception` that reveals entities within a radius; adjust `task` and `json_file` at the bottom of `test.py` for Task1/2.

---

## Monitoring and Debugging

### Input topics (perception)

The planning stack expects these topics from perception systems:

| Topic | Type | Description |
|-------|------|-------------|
| **`/fused_buoys`** | `pointcloud_filters/FusedBuoyArray` | **Preferred**: CV–LiDAR fusion output with class_id and class_name |
| **`/combined/detection_info_with_distance`** | `std_msgs/String` (JSON) | CV-only output (fallback if fusion not available) |

### Monitoring input topics

**Check if perception topics are publishing:**
```bash
# List all topics
ros2 topic list

# Check topic frequency
ros2 topic hz /fused_buoys --window 20
ros2 topic hz /combined/detection_info_with_distance --window 20

# Check topic type
ros2 topic type /fused_buoys
ros2 topic type /combined/detection_info_with_distance

# See topic info
ros2 topic info /fused_buoys --verbose
ros2 topic info /combined/detection_info_with_distance --verbose
```

**View topic messages:**

**Basic echo:**
```bash
ros2 topic echo /fused_buoys
ros2 topic echo /combined/detection_info_with_distance
```

**View long messages (no truncation):**
```bash
# Use --no-arr to see full arrays/lists without truncation
ros2 topic echo /fused_buoys --no-arr
ros2 topic echo /combined/detection_info_with_distance --no-arr

# For JSON topics, extract and pretty-print with jq
ros2 topic echo /combined/detection_info_with_distance --once --field data | jq .

# View full message continuously without truncation
ros2 topic echo /combined/detection_info_with_distance --no-arr --field data | jq .
```

**Single message:**
```bash
# Get one message and exit
ros2 topic echo /fused_buoys --once
ros2 topic echo /combined/detection_info_with_distance --once

# Single message, specific field, pretty-printed (for JSON topics)
ros2 topic echo /combined/detection_info_with_distance --once --field data | jq .
```

### Output topics (goals)

When integrated with ROS, the planning stack would publish goals. Check for goal-related topics:

```bash
# List all topics (look for goal-related topics)
ros2 topic list | grep -i goal

# Monitor goal topics if they exist
ros2 topic echo /planning/goals --no-arr
ros2 topic hz /planning/goals --window 20
```

### Debugging

**Check if planning node is running:**
```bash
ros2 node list | grep -i plan
ros2 node info /planning_node  # if node exists
```

**View node parameters (if ROS node exists):**
```bash
ros2 param list /planning_node
ros2 param get /planning_node task_id
```

**Common debugging commands:**
```bash
# Check all planning-related topics
ros2 topic list | grep -E "(plan|goal|fused|detection)"

# Monitor pipeline health
ros2 topic hz /fused_buoys --window 20
ros2 topic hz /combined/detection_info_with_distance --window 20

# Save topic messages to bag file for later analysis
ros2 bag record /fused_buoys /combined/detection_info_with_distance

# Play back bag file
ros2 bag play <bag_file_name>
```

### Troubleshooting

- **No perception data:** Ensure CV and mapping pipelines are running. Check `/fused_buoys` or `/combined/detection_info_with_distance` are publishing
- **Truncated output:** Use `--no-arr` flag with `ros2 topic echo` to see full messages without truncation
- **No planning node:** Planning may be integrated differently (callbacks, not ROS nodes). Check how your system integrates TaskMaster
- **Standalone testing:** Use `test.py` for testing without ROS dependencies

## Links

- **Mapping** ([../mapping/README.md](../mapping/README.md)) — LiDAR pipeline, `/tracked_buoys`, fusion with CV → `/fused_buoys`.
- **Computer vision** ([../computer_vision/README.md](../computer_vision/README.md)) — CV pipeline, `/combined/detection_info_with_distance`.
