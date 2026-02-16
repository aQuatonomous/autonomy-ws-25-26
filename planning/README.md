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

## Links

- **Mapping** ([../mapping/README.md](../mapping/README.md)) — LiDAR pipeline, `/tracked_buoys`, fusion with CV → `/fused_buoys`.
- **Computer vision** ([../computer_vision/README.md](../computer_vision/README.md)) — CV pipeline, `/combined/detection_info_with_distance`.
