#!/usr/bin/env python3
"""
Location: src/Planner/Global/

Entity definitions + helper methods required by Global Tasks.

Supports:
- Start position (stored on EntityList)
- Gate pairing (red_buoy + green_buoy) => get_gates()
- Obstacle extraction => get_obstacles()
- Goals written by tasks as type="goal" and names goal_wp1..N
"""

from __future__ import annotations
import math
from typing import List, Tuple, Optional, Any

from Global.types import Vec2, Vec3, _to_vec3, DetectedEntity

# No-go zone from gate edges: extent (m) along gate normal and along gate toward next pair
GATE_NO_GO_EXTENT_NORMAL = 15.0
GATE_NO_GO_EXTENT_ALONG = 15.0
GATE_NO_GO_SAMPLE_SPACING = 1.0

# When filtering gates by boat heading: max |dot(gate_vec, forward)| / gate_length (0 = strict perpendicular)
# ~0.25 allows ~15 deg tilt from perpendicular (gate left/right of boat)
GATE_PERPENDICULAR_THRESHOLD = 0.25

# Matches computer_vision cv_scripts/class_mapping.yaml (0-22). class_id 255 = unknown (unmapped).
CLASS_ID_TO_ENTITY_TYPE = {
    0: "black_buoy",
    1: "green_buoy",
    2: "green_pole_buoy",
    3: "red_buoy",
    4: "red_pole_buoy",
    5: "yellow_buoy",
    6: "cross",
    7: "dock",
    8: "triangle",
    9: "red_indicator",
    10: "green_indicator",
    11: "yellow_supply_drop",
    12: "black_supply_drop",
    20: "digit_1",
    21: "digit_2",
    22: "digit_3",
}

# When fusion sends color/class_name string (legacy or from yaml class name)
COLOR_TO_ENTITY_TYPE = {
    "red": "red_buoy",
    "green": "green_buoy",
    "yellow": "yellow_buoy",
    "black": "black_buoy",
    "black_buoy": "black_buoy",
    "unknown": "unknown",
    "red_indicator": "red_indicator",
    "green_indicator": "green_indicator",
    "red_indicator_buoy": "red_indicator",
    "green_indicator_buoy": "green_indicator",
    "green_pole_buoy": "green_buoy",
    "red_pole_buoy": "red_buoy",
    "yellow_supply_drop": "yellow_supply_drop",
    "black_supply_drop": "black_supply_drop",
}


def _color_to_entity_type(color: str) -> str:
    """Map color/class_name string to entity type. Unmapped inputs return 'unknown', not black."""
    return COLOR_TO_ENTITY_TYPE.get((color or "unknown").strip().lower(), "unknown")


def _get_attr(obj: Any, key: str, default: Any = None) -> Any:
    """Get key from object (ROS msg) or dict."""
    if hasattr(obj, key):
        return getattr(obj, key)
    if isinstance(obj, dict):
        return obj.get(key, default)
    return default


def apply_tracked_buoys(entity_list: "EntityList", buoys: List[Any], tolerance: float = 1.0) -> None:
    """
    Update EntityList from tracked-buoy-like objects (ROS TrackedBuoy or dict).
    Each item must have: id (object_id for tracking), x, y; and either class_id (int, from
    class_mapping.yaml) or color (string). Same id = same entity.
    """
    for b in buoys:
        x = float(_get_attr(b, "x", 0.0))
        y = float(_get_attr(b, "y", 0.0))
        bid = int(_get_attr(b, "id", 0))
        class_id = _get_attr(b, "class_id")
        if class_id is not None:
            try:
                cid = int(class_id)
                entity_type = CLASS_ID_TO_ENTITY_TYPE.get(cid, "unknown")
            except (TypeError, ValueError):
                entity_type = _color_to_entity_type(str(_get_attr(b, "color") or "unknown"))
        else:
            color = _get_attr(b, "color") or "unknown"
            entity_type = _color_to_entity_type(str(color))
        name = f"{entity_type}_{bid}"
        entity_list.add_or_update(entity_type, (x, y, 0.0), entity_id=bid, name=name)


class Entity:
    """Base class for all entities in the environment."""
    def __init__(
        self,
        entity_type: str,
        position: Tuple[float, ...],
        name: Optional[str] = None,
        entity_id: Optional[int] = None,
    ):
        self.type = entity_type
        self.position = _to_vec3(position)
        self.name = name or f"{entity_type}_{id(self)}"
        self.entity_id = entity_id  # from TrackedBuoy.id; None for goals

    def __repr__(self) -> str:
        return f"{self.type}({self.name}) at {self.position}"


class EntityList:
    """Container for all entities with helper methods used by global planner tasks."""
    def __init__(self, start_position: Tuple[float, ...] = (0.0, 0.0)):
        self.entities: List[Entity] = []
        self.start_position: Vec3 = _to_vec3(start_position)
        self._last_no_go_points: List[Vec2] = []
        # Stateful no-go: 1 or 2 gates along channel; updated when get_gates() set changes
        self._no_go_gates: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = []

    # -------------------------
    # core CRUD
    # -------------------------

    def set_start_position(self, position: Tuple[float, ...]) -> None:
        self.start_position = _to_vec3(position)

    def add(
        self,
        entity_type: str,
        position: Tuple[float, ...],
        name: Optional[str] = None,
        entity_id: Optional[int] = None,
    ) -> Entity:
        ent = Entity(entity_type, position, name, entity_id=entity_id)
        self.entities.append(ent)
        return ent

    def add_or_update(
        self,
        entity_type: str,
        position: Tuple[float, ...],
        entity_id: int,
        name: Optional[str] = None,
    ) -> Entity:
        """
        Add or update entity by id. Same id = same entity.
        When the same entity_id is seen again, updates position (x,y) to track buoy shifts
        in global frame (e.g. sensor refinement, small drift, or real movement).
        """
        pos = _to_vec3(position)
        for existing in self.entities:
            if existing.entity_id is not None and existing.entity_id == entity_id:
                existing.position = pos
                existing.type = entity_type
                if name is not None:
                    existing.name = name
                return existing
        return self.add(entity_type, position, name=name or f"{entity_type}_{entity_id}", entity_id=entity_id)

    def get_by_type(self, entity_type: str) -> List[Entity]:
        return [e for e in self.entities if e.type == entity_type]

    def get_positions_by_type(self, entity_type: str) -> List[Vec3]:
        return [e.position for e in self.entities if e.type == entity_type]

    # -------------------------
    # global planner helpers
    # -------------------------

    def get_start(self) -> Vec3:
        return self.start_position

    def get_goals(self) -> List[Vec3]:
        return self.get_positions_by_type("goal")

    def clear_goals(self) -> None:
        self.entities = [e for e in self.entities if e.type != "goal"]

    def _get_gate_entity_pairs(
        self,
        max_gate_width: float = 15.0,
        max_y_diff: Optional[float] = 5.0,
        min_x_separation: Optional[float] = 2.0,
        boat_heading_rad: Optional[float] = None,
    ) -> List[Tuple["Entity", "Entity"]]:
        """
        Pair red with nearest green within max_gate_width.
        If boat_heading_rad is set: keep only pairs where gate line is roughly perpendicular
        to boat (left/right of boat). Otherwise use max_y_diff and min_x_separation (same-Y, side-by-side).
        """
        red_ents = self.get_by_type("red_buoy")
        green_ents = self.get_by_type("green_buoy")
        if not red_ents or not green_ents:
            return []
        red_ents = sorted(red_ents, key=lambda e: e.position[1])
        pairs: List[Tuple[Entity, Entity]] = []
        used = set()
        use_perp = boat_heading_rad is not None
        for red in red_ents:
            rp = red.position
            best_green = None
            best_idx = -1
            best_dist = float("inf")
            for i, green in enumerate(green_ents):
                if i in used:
                    continue
                dx = rp[0] - green.position[0]
                dy = rp[1] - green.position[1]
                dist = math.hypot(dx, dy)
                if dist > max_gate_width:
                    continue
                if not use_perp:
                    if max_y_diff is not None and abs(dy) > max_y_diff:
                        continue
                    if min_x_separation is not None and abs(dx) < min_x_separation:
                        continue
                else:
                    # Gate vector red -> green; forward = (cos(h), sin(h)). Perpendicular if |dot| small.
                    gx = green.position[0] - rp[0]
                    gy = green.position[1] - rp[1]
                    if dist < 1e-6:
                        continue
                    fwd_x = math.cos(boat_heading_rad)
                    fwd_y = math.sin(boat_heading_rad)
                    dot = abs(gx * fwd_x + gy * fwd_y) / (dist + 1e-9)
                    if dot > GATE_PERPENDICULAR_THRESHOLD:
                        continue
                if dist < best_dist:
                    best_dist = dist
                    best_green = green
                    best_idx = i
            if best_green is not None:
                pairs.append((red, best_green))
                used.add(best_idx)
        return pairs

    def get_gates(
        self,
        max_gate_width: float = 15.0,
        max_y_diff: Optional[float] = 5.0,
        min_x_separation: Optional[float] = 2.0,
        boat_heading_rad: Optional[float] = None,
    ) -> List[Tuple[Vec2, Vec2]]:
        """
        Return list of (red_pos, green_pos) gate pairs. Pairs red with nearest green
        within max_gate_width.
        If boat_heading_rad is set: only pairs where gate is roughly perpendicular to boat (left/right).
        Otherwise: same Y within max_y_diff and separated on X by min_x_separation (side-by-side).
        """
        pairs = self._get_gate_entity_pairs(
            max_gate_width, max_y_diff, min_x_separation, boat_heading_rad=boat_heading_rad
        )
        return [(r.position, g.position) for r, g in pairs]  # Vec3, Vec3

    def _gate_no_go_segments(
        self,
        gates: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
        extent_normal_m: float,
        extent_along_m: float,
    ) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """For each gate, return left and right wall segments (no-go boundaries). Uses (x,y) only."""
        segments: List[Tuple[Vec2, Vec2]] = []
        for red, green in gates:
            rx, ry = float(red[0]), float(red[1])  # Vec3, use x,y
            gx, gy = float(green[0]), float(green[1])
            dx = gx - rx
            dy = gy - ry
            L = math.hypot(dx, dy)
            if L < 1e-6:
                continue
            ux, uy = dx / L, dy / L
            lnx, lny = -uy, ux
            rnx, rny = uy, -ux
            left_a = (rx - extent_along_m * ux + extent_normal_m * lnx, ry - extent_along_m * uy + extent_normal_m * lny)
            left_b = (gx + extent_along_m * ux + extent_normal_m * lnx, gy + extent_along_m * uy + extent_normal_m * lny)
            segments.append((left_a, left_b))
            right_a = (rx - extent_along_m * ux + extent_normal_m * rnx, ry - extent_along_m * uy + extent_normal_m * rny)
            right_b = (gx + extent_along_m * ux + extent_normal_m * rnx, gy + extent_along_m * uy + extent_normal_m * rny)
            segments.append((right_a, right_b))
        return segments

    def _sample_segment(self, a: Vec2, b: Vec2, spacing_m: float) -> List[Vec2]:
        """Sample points along segment (a, b) every spacing_m."""
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return [a]
        n = max(1, int(length / spacing_m) + 1)
        return [(ax + (i / n) * dx, ay + (i / n) * dy) for i in range(n + 1)]

    def _gate_center(self, gate: Tuple[Tuple[float, float, float], Tuple[float, float, float]]) -> Tuple[float, float]:
        """Gate center (x, y) for ordering and comparison."""
        r, g = gate[0], gate[1]
        return ((r[0] + g[0]) * 0.5, (r[1] + g[1]) * 0.5)

    def _no_go_gates_equal(
        self,
        a: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
        b: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
        tol: float = 0.5,
    ) -> bool:
        """True if the two gate lists represent the same set (by center distance)."""
        if len(a) != len(b):
            return False
        for ga, gb in zip(a, b):
            ca, cb = self._gate_center(ga), self._gate_center(gb)
            if math.hypot(ca[0] - cb[0], ca[1] - cb[1]) > tol:
                return False
        return True

    def get_no_go_obstacle_points(
        self,
        extent_normal_m: float = GATE_NO_GO_EXTENT_NORMAL,
        extent_along_m: float = GATE_NO_GO_EXTENT_ALONG,
        sample_spacing_m: float = GATE_NO_GO_SAMPLE_SPACING,
        map_bounds: Optional[Tuple[float, float]] = None,
        boat_heading_rad: Optional[float] = None,
    ) -> List[Vec2]:
        """
        No-go zone from 1 or 2 red-green gate pairs along the channel, plus optional map bounds.
        When new gates are brought in, no-go is updated by comparing gate centers (current vs
        stored _no_go_gates); when they differ, the stored set is updated and segments are
        rebuilt, so the no-go zone orientation/angle reflects the current gate set (each gate's
        segment uses that gate's red-to-green direction for the wall angle).
        If boat_heading_rad is set: use get_gates(boat_heading_rad=...) and order gates by
        projection along forward from start (same as Task1). Otherwise: get_gates() and order by
        gate center Y. Same sample_spacing_m for segments and map boundary. All units in meters.
        """
        if boat_heading_rad is not None:
            gates_now = self.get_gates(boat_heading_rad=boat_heading_rad)
            # Order along channel by projection onto forward from start (match Task1)
            start = self.get_start()
            s_x, s_y = float(start[0]), float(start[1])
            fwd_x = math.cos(boat_heading_rad)
            fwd_y = math.sin(boat_heading_rad)
            def key_along_fwd(g):
                cx = (g[0][0] + g[1][0]) * 0.5
                cy = (g[0][1] + g[1][1]) * 0.5
                return (cx - s_x) * fwd_x + (cy - s_y) * fwd_y
            ordered = sorted(gates_now, key=key_along_fwd)
        else:
            gates_now = self.get_gates()
            ordered = sorted(gates_now, key=lambda g: (g[0][1] + g[1][1]) * 0.5)
        current_set = ordered[:2] if ordered else []
        if not current_set:
            self._no_go_gates = []
            points = list(self._last_no_go_points)
        else:
            if not self._no_go_gates_equal(current_set, self._no_go_gates):
                self._no_go_gates = list(current_set)
            points = []
            segments = self._gate_no_go_segments(self._no_go_gates, extent_normal_m, extent_along_m)
            for seg_a, seg_b in segments:
                points.extend(self._sample_segment(seg_a, seg_b, sample_spacing_m))
            self._last_no_go_points = points
        if map_bounds is not None:
            w, h = float(map_bounds[0]), float(map_bounds[1])
            if w > 0 and h > 0:
                y = 0.0
                while y <= h + 1e-9:
                    points.append((0.0, y))
                    points.append((w, y))
                    y += sample_spacing_m
                if y - sample_spacing_m < h - 1e-9:
                    points.append((0.0, h))
                    points.append((w, h))
                x = 0.0
                while x <= w + 1e-9:
                    points.append((x, 0.0))
                    points.append((x, h))
                    x += sample_spacing_m
                if x - sample_spacing_m < w - 1e-9:
                    points.append((w, 0.0))
                    points.append((w, h))
                points = list(dict.fromkeys(points))
        return points

    def get_map_bounds(
        self,
        padding_m: float = 20.0,
    ) -> Optional[Tuple[float, float]]:
        """
        Map bounds derived from no-go zones (and entities if no no-go yet).
        Returns (max_x, max_y) in meters for clipping and planner.
        """
        points: List[Vec2] = list(self._last_no_go_points)
        if not points:
            for e in self.entities:
                if e.type != "goal":
                    points.append((e.position[0], e.position[1]))  # x,y for bounds
        if not points:
            return None
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        return (max_x + padding_m, max_y + padding_m)

    def get_obstacles(self) -> List[Vec3]:
        """
        Return obstacle positions for potential fields.
        Includes *everything* that should repel the boat.
        Excludes: goals and start.
        """
        obstacle_types = {
            "red_buoy", "green_buoy", "green_pole_buoy", "red_pole_buoy",
            "black_buoy",
            "yellow_buoy",
            "red_indicator", "green_indicator",
            "yellow_supply_drop", "black_supply_drop",
            "unknown",
        }
        out: List[Vec3] = []
        for e in self.entities:
            if e.type in obstacle_types:
                out.append(e.position)  # Vec3 (x, y, 0.0)
        return out

    def get_black_buoys(self) -> List[Tuple[int, Vec3]]:
        """
        Return list of (entity_id, position) for all black_buoy entities (debris). Position in meters.
        Used for Task 2 debris reporting (lat/long to /gs_message_send).
        """
        out: List[Tuple[int, Vec2]] = []
        for e in self.entities:
            if e.type != "black_buoy":
                continue
            eid = e.entity_id if e.entity_id is not None else 0
            out.append((eid, e.position))
        return out

    def to_detections(self) -> List[DetectedEntity]:
        """
        Return list of DetectedEntity for all non-goal entities.
        Used by TaskMaster.get_detections. Uses Entity.entity_id when set, else parses from name.
        """
        out: List[DetectedEntity] = []
        for e in self.entities:
            if e.type == "goal":
                continue
            bid = e.entity_id if e.entity_id is not None else 0
            if bid == 0 and e.name:
                parts = e.name.split("_")
                if len(parts) >= 2 and parts[-1].isdigit():
                    bid = int(parts[-1])
            out.append(DetectedEntity(entity_id=bid, entity_type=e.type, position=e.position))
        return out
