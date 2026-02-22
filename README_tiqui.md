# Tiqui Branch - Changes Made on 2026-02-22

This document summarizes the updates made today before pushing branch `tiqui`.

## Files Changed

- `planning/Global/Task4.py`
- `planning/TaskMaster.py`
- `planning/Global_Planner/global_planner/global_planner_node.py`
- `planning/Global_Planner/launch/global_planner.launch.py`
- `planning/Global/Task6.py` (new file)

## Summary of Changes

### 1) Task 4 redesign as interrupt watchdog (`planning/Global/Task4.py`)
- Reworked Task 4 into a persistent service-boat interrupt flow.
- Added queueing of `yellow_supply_drop` targets with location-based deduplication.
- Added pre-interrupt pose capture and return-to-pre-interrupt behavior.
- Added heading realignment phase after return.
- Added direct `Twist` override support (`current_twist_override`) for in-place heading alignment.
- Updated planner obstacle handling and approach/retreat behavior.
- Task now remains active (`done() == False`) as a watchdog behavior.

### 2) TaskMaster integration for primary + interrupt execution (`planning/TaskMaster.py`)
- Added support for Task 6 manager construction.
- Added dual-manager orchestration: primary task + Task 4 interrupt overlay (except when primary task is 4).
- Added manager switching logic with yield conditions and deferred handover.
- Added active task tracking (`active_task_id`) in run results.
- Forwarded sound signals to managers that implement `on_sound_signal`.
- Updated completion criteria to account for overlay activity.

### 3) Global planner node updates for Task 6 + dynamic task reporting (`planning/Global_Planner/global_planner/global_planner_node.py`)
- Updated sound interrupt callback behavior:
  - Task 6: forwards signal values to TaskMaster/manager.
  - Other tasks: keeps stop-on-1 behavior.
- Published dynamic `curr_task` based on `active_task_id` from TaskMaster.
- Added Task 6 phase logging.
- Added support for manager-driven twist override publishing.

### 4) Launch argument docs update (`planning/Global_Planner/launch/global_planner.launch.py`)
- Updated `task_id` launch argument description to include tasks `1, 2, 3, 4, 6`.

### 5) New Task 6 implementation (`planning/Global/Task6.py`)
- Added Harbor Alert behavior:
  - Patrol by default.
  - On sound signal: divert to east-most (1) or west-most (2) yellow buoy.
  - Hold at target for fixed duration.
  - Return to pre-interrupt pose and heading.
  - Resume patrol.
- Includes obstacle-aware planning and optional heading twist override.

### 6) Left-right buoy navigation integration for Task 1/2 (TaskMaster-level)
- Added `planning/Global/left_right_buoy_nav.py`:
  - New `LeftRightBuoyNavigator` midpoint steering helper (red-left / green-right gate logic).
  - Outputs direct twist-style override (`linear.x`, `linear.y`, `angular.z`).
- Updated `planning/TaskMaster.py`:
  - Instantiates `LeftRightBuoyNavigator`.
  - Applies override only when active task is Task 1 or Task 2.
  - Leaves `planning/Global/Task1.py` and `planning/Global/Task2.py` unchanged.
- Added documentation:
  - `planning/LEFT_RIGHT_BUOY_NAV_INTEGRATION.md`
  - `planning/README.md` section + link
- Confirmed no dependency on external folder `left-right-buoy-nav/` at runtime.

## Notes
- Branch created: `tiqui`
- Remote expected: `origin`
