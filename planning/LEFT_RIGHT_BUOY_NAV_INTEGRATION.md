# Left-Right Buoy Navigation Integration

This document describes how left-right buoy midpoint steering is integrated into the planning stack for Task 1 and Task 2.

## What Was Added

- `Global/left_right_buoy_nav.py`
  - Contains `LeftRightBuoyNavigator` and `LeftRightBuoyConfig`.
  - Computes a `Twist`-style override:
    - `linear.x` forward speed
    - `linear.y` (kept at `0.0`)
    - `angular.z` yaw rate
  - Steering target is the midpoint of a valid red-left/green-right gate pair.
  - Uses proportional heading control with yaw-rate clamp.
  - Reduces forward speed when heading error is large.

## Where It Is Used

- `TaskMaster.py`
  - Instantiates one `LeftRightBuoyNavigator`.
  - On each planning tick, if `active_task_id` is `1` or `2`, it computes a buoy-based steering override using current gates from `EntityList`.
  - Sets `manager.current_twist_override` for the active manager.

- `Global_Planner/global_planner/global_planner_node.py`
  - Already supports manager-provided `current_twist_override`.
  - If override exists, node publishes override velocity directly and skips planner velocity for that tick.

## Important Constraints

- `Task1.py` and `Task2.py` were intentionally not modified.
- Integration is injected at the `TaskMaster` level.
- No runtime dependency on the external repo folder `left-right-buoy-nav/`.

## Dependencies

`Global/left_right_buoy_nav.py` uses:

- Python stdlib: `math`, `dataclasses`, `typing`
- Third-party: `numpy`

No imports from:

- `left-right-buoy-nav/ColorDetection.py`
- `left-right-buoy-nav/MotorControl.py`
- `left-right-buoy-nav/MotorDriverNode.py`
- `left-right-buoy-nav/navigation_launch.py`

## Behavior Summary

When Task 1 or Task 2 is active:

1. Get candidate gates from `EntityList.get_gates(...)`.
2. Keep only gates that satisfy red-left/green-right in the boat frame.
3. Select the best forward gate.
4. Compute heading error to gate midpoint.
5. Output `current_twist_override` for direct control.

If no valid gate is found, no override is applied and normal planner behavior continues.
