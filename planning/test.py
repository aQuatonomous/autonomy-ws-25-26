#!/usr/bin/env python3
"""
Test harness for Task1/Task2 using JSON entity file.

Adds:
- persistent managers (no reset)
- better debug prints (what is actually detected/stored)
- prints final position on SUCCESS/FAIL
"""

import os
import sys
import json
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]

sys.path.insert(0, os.path.abspath("."))

from Global.entities import load_entities_from_json
from Global.Task1 import Task1Manager
from Global.Task2 import Task2Manager
from Global.Task3 import Task3Manager
from Local.potential_fields_planner import PotentialFieldsPlanner


@dataclass
class DetectedEntity:
    entity_id: int
    entity_type: str
    position: Vec2


def dist(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FakePerception:
    def __init__(self, json_path: str, reveal_radius: float = 40.0):
        self.reveal_radius = reveal_radius
        self.catalog = self._load_catalog(json_path)
        self.revealed_ids = set()

    def _load_catalog(self, json_path: str) -> List[DetectedEntity]:
        with open(json_path, "r") as f:
            data = json.load(f)

        out: List[DetectedEntity] = []
        eid = 1
        ent = data.get("entities", {})

        for b in ent.get("buoys", []):
            out.append(DetectedEntity(eid, b["type"], tuple(b["position"])))
            eid += 1

        for d in ent.get("debris", []):
            d_type = d.get("type", "debris")
            out.append(DetectedEntity(eid, d_type, tuple(d["position"])))
            eid += 1

        for ind in ent.get("indicators", []):
            out.append(DetectedEntity(eid, ind["type"], tuple(ind["position"])))
            eid += 1

        return out

    def get_detections(self, boat_xy: Vec2) -> List[DetectedEntity]:
        dets: List[DetectedEntity] = []
        for d in self.catalog:
            if d.entity_id in self.revealed_ids:
                continue
            if dist(boat_xy, d.position) <= self.reveal_radius:
                self.revealed_ids.add(d.entity_id)
                dets.append(d)
        return dets


class GoalFollowerSim:
    def __init__(self, start: Vec2, map_bounds: Vec2):
        self.x = float(start[0])
        self.y = float(start[1])
        self.hdg = 1.57
        self.map_bounds = map_bounds
        self.speed = 3.0
        self.turn_rate = 0.25

    def pose(self) -> Pose:
        return (self.x, self.y, self.hdg)

    def step_toward(self, goal: Vec2):
        gx, gy = goal
        dx = gx - self.x
        dy = gy - self.y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return

        desired = math.atan2(dy, dx)
        diff = (desired - self.hdg + math.pi) % (2 * math.pi) - math.pi
        diff = clamp(diff, -self.turn_rate, self.turn_rate)
        self.hdg += diff

        # KEY: don't overshoot the goal
        step = min(self.speed, d)
        self.x += step * math.cos(self.hdg)
        self.y += step * math.sin(self.hdg)

        # Optional: keep heading bounded
        self.hdg = (self.hdg + math.pi) % (2 * math.pi) - math.pi

        self.x = clamp(self.x, 0.0, self.map_bounds[0])
        self.y = clamp(self.y, 0.0, self.map_bounds[1])



def _count_types(entities, t: str) -> int:
    # works with either entities.get_by_type or direct entities.entities list
    if hasattr(entities, "get_by_type"):
        return len(entities.get_by_type(t))
    return sum(1 for e in getattr(entities, "entities", []) if getattr(e, "type", None) == t)


def run_test(task_id: int, json_path: str, loop_steps: int = 5000, use_pf: bool = True):
    entities, map_bounds = load_entities_from_json(json_path, infer_goals=False)

    start = entities.get_start()
    sim = GoalFollowerSim(start, map_bounds)
    perception = FakePerception(json_path, reveal_radius=45.0)
    planner = PotentialFieldsPlanner(resolution=0.5) if use_pf else None

    if task_id == 1:
        mgr = Task1Manager(entities, map_bounds, (float(start[0]), float(start[1]), 0.0))
    elif task_id == 2:
        mgr = Task2Manager(entities, map_bounds, (float(start[0]), float(start[1]), 0.0))
    elif task_id == 3:
        mgr = Task3Manager(entities, map_bounds, (float(start[0]), float(start[1]), 0.0))
    else:
        raise ValueError("task_id must be 1, 2, or 3")

    print(f"Loaded map_bounds={map_bounds}, start={start}")
    print(f"Initial JSON entities: red={_count_types(entities,'red_buoy')} green={_count_types(entities,'green_buoy')}")

    for t in range(loop_steps):
        boat_xy = (sim.x, sim.y)
        dets = perception.get_detections(boat_xy)

        # update pose
        x, y, hdg = sim.pose()
        mgr.update_pose(x, y, hdg)

        # add detections
        for d in dets:
            mgr.add_detected(d)

        mgr.tick()

        if use_pf and planner is not None:
            mgr.plan(planner)

        status = "SUCCESS" if mgr.done() else "RUNNING"
        goals = list(getattr(mgr, "goal_queue", []))
        phase = getattr(mgr, "phase", None)
        phase_str = phase.value if phase is not None else ""

        # Debug print
        if t % 25 == 0:
            red_n = _count_types(entities, "red_buoy")
            green_n = _count_types(entities, "green_buoy")
            gates = entities.get_gates() if hasattr(entities, "get_gates") else []
            msg = f"t={t:04d} pos=({sim.x:.1f},{sim.y:.1f}) status={status}"
            if phase_str:
                msg += f" phase={phase_str}"
            msg += f" goals={goals[:1]}  red={red_n} green={green_n} gates={len(gates)}"
            print(msg)

            if task_id == 1 and hasattr(mgr, "next_gate_index"):
                print(f"   next_gate_index={mgr.next_gate_index}")
                print(f"   gate_centers={getattr(mgr,'gate_centers',None)}")
                # show actual gate pairs too
                if gates:
                    centers = [((r[0]+g[0])/2, (r[1]+g[1])/2) for (r,g) in gates]
                    print(f"   entity.get_gates() centers={centers}")
            
            if task_id == 3:
                indicator = getattr(mgr, 'indicator_color', None)
                entry_cross = getattr(mgr, 'entry_crossed', False)
                exit_cross = getattr(mgr, 'exit_crossed', False)
                print(f"   indicator={indicator} entry_crossed={entry_cross} exit_crossed={exit_cross}")

        # Stop condition
        if status == "SUCCESS":
            print("\n✅ SUCCESS reached!")
            print(f"FINAL pos=({sim.x:.2f}, {sim.y:.2f}) heading={sim.hdg:.2f}rad")
            if task_id == 1:
                print(f"FINAL next_gate_index={getattr(mgr,'next_gate_index',None)}")
                print(f"FINAL gate_centers={getattr(mgr,'gate_centers',None)}")
            if task_id == 3:
                indicator = getattr(mgr, 'indicator_color', None)
                start_t = getattr(mgr, 'start_time', None)
                finish_t = getattr(mgr, 'finish_time', None)
                entry_cross = getattr(mgr, 'entry_crossed', False)
                exit_cross = getattr(mgr, 'exit_crossed', False)
                elapsed = finish_t - start_t if (start_t and finish_t) else None
                print(f"FINAL indicator_color={indicator}")
                print(f"FINAL entry_crossed={entry_cross} exit_crossed={exit_cross}")
                print(f"FINAL start_time={start_t} finish_time={finish_t}")
                print(f"FINAL completion_time={elapsed:.2f}s" if elapsed else "FINAL completion_time=N/A")

            if hasattr(mgr, "report"):
                print("Report:", mgr.report.to_dict())
            return

        # Move toward current goal
        if goals:
            sim.step_toward(goals[0])
        else:
            sim.step_toward((sim.x + 10.0, sim.y + 10.0))

    print("\n❌ FAILED: max steps reached without SUCCESS.")
    print(f"FINAL pos=({sim.x:.2f}, {sim.y:.2f}) heading={sim.hdg:.2f}rad")

if __name__ == "__main__":
    task = 3
    json_file = "Input_Entities/task3_entities.json"
    run_test(task, json_file, loop_steps=5000, use_pf=False)
