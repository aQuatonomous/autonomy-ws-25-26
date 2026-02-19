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

Vec2 = Tuple[float, float]

# No-go zone from gate edges: extent (m) along gate normal and along gate toward next pair
GATE_NO_GO_EXTENT_NORMAL = 15.0
GATE_NO_GO_EXTENT_ALONG = 15.0
GATE_NO_GO_SAMPLE_SPACING = 1.0

# class_mapping.yaml: 0 black_buoy, 1 green_buoy, 2 green_pole, 3 red_buoy, 4 red_pole, 5 yellow_buoy,
# 9 red_indicator_buoy, 10 green_indicator_buoy. id in message = object_id (tracking).
CLASS_ID_TO_ENTITY_TYPE = {
    0: "black_buoy",
    1: "green_buoy",
    2: "green_buoy",
    3: "red_buoy",
    4: "red_buoy",
    5: "yellow_buoy",
    6: "black_buoy",
    7: "black_buoy",
    8: "black_buoy",
    9: "red_indicator",
    10: "green_indicator",
    11: "black_buoy",
    12: "black_buoy",
    20: "black_buoy",
    21: "black_buoy",
    22: "black_buoy",
}

# When fusion sends color/class_name string (legacy or from yaml class name)
COLOR_TO_ENTITY_TYPE = {
    "red": "red_buoy",
    "green": "green_buoy",
    "yellow": "yellow_buoy",
    "unknown": "black_buoy",
    "red_indicator": "red_indicator",
    "green_indicator": "green_indicator",
    "red_indicator_buoy": "red_indicator",
    "green_indicator_buoy": "green_indicator",
    "green_pole_buoy": "green_buoy",
    "red_pole_buoy": "red_buoy",
}


def _color_to_entity_type(color: str) -> str:
    return COLOR_TO_ENTITY_TYPE.get((color or "unknown").strip().lower(), "black_buoy")


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
    When a new red-green gate pair is seen, push current entity ids to a cohort queue;
    when queue exceeds GATE_COHORT_QUEUE_SIZE, the oldest cohort is dropped.
    """
    entity_list._last_pruned_ids = set()
    for b in buoys:
        x = float(_get_attr(b, "x", 0.0))
        y = float(_get_attr(b, "y", 0.0))
        bid = int(_get_attr(b, "id", 0))
        class_id = _get_attr(b, "class_id")
        if class_id is not None:
            try:
                cid = int(class_id)
                entity_type = CLASS_ID_TO_ENTITY_TYPE.get(cid, "black_buoy")
            except (TypeError, ValueError):
                entity_type = _color_to_entity_type(str(_get_attr(b, "color") or "unknown"))
        else:
            color = _get_attr(b, "color") or "unknown"
            entity_type = _color_to_entity_type(str(color))
        name = f"{entity_type}_{bid}"
        entity_list.add_or_update(entity_type, (x, y), entity_id=bid, name=name)

    pairs = entity_list._get_gate_entity_pairs()
    current_keys = {
        frozenset((min(r.entity_id, g.entity_id), max(r.entity_id, g.entity_id)))
        for r, g in pairs
        if r.entity_id is not None and g.entity_id is not None
    }
    if current_keys and current_keys != entity_list._last_gate_pair_keys:
        entity_list._last_gate_pair_keys = current_keys
        current_ids = {
            e.entity_id for e in entity_list.entities
            if e.entity_id is not None and e.type != "goal"
        }
        entity_list._cohorts.append(current_ids)
        entity_list._prune_oldest_cohort()


class Entity:
    """Base class for all entities in the environment."""
    def __init__(
        self,
        entity_type: str,
        position: Vec2,
        name: Optional[str] = None,
        entity_id: Optional[int] = None,
    ):
        self.type = entity_type
        self.position = (float(position[0]), float(position[1]))
        self.name = name or f"{entity_type}_{id(self)}"
        self.entity_id = entity_id  # from TrackedBuoy.id; None for goals

    def __repr__(self) -> str:
        return f"{self.type}({self.name}) at {self.position}"


# How many gate-pair "cohorts" to keep before dropping the oldest (pair + its obstacles)
GATE_COHORT_QUEUE_SIZE = 2


class EntityList:
    """Container for all entities with helper methods used by global planner tasks."""
    def __init__(self, start_position: Vec2 = (0.0, 0.0)):
        self.entities: List[Entity] = []
        self.start_position: Vec2 = (float(start_position[0]), float(start_position[1]))
        self._last_no_go_points: List[Vec2] = []
        self._cohorts: List[set] = []  # each element = set of entity_id for that gate-pass
        self._last_gate_pair_keys: set = set()  # frozenset of (min_id, max_id) per pair
        self._last_pruned_ids: set = set()  # entity ids dropped in last cohort prune (for logging)

    # -------------------------
    # core CRUD
    # -------------------------

    def set_start_position(self, position: Vec2) -> None:
        self.start_position = (float(position[0]), float(position[1]))

    def add(
        self,
        entity_type: str,
        position: Vec2,
        name: Optional[str] = None,
        entity_id: Optional[int] = None,
    ) -> Entity:
        ent = Entity(entity_type, position, name, entity_id=entity_id)
        self.entities.append(ent)
        return ent

    def add_or_update(
        self,
        entity_type: str,
        position: Vec2,
        entity_id: int,
        name: Optional[str] = None,
    ) -> Entity:
        """
        Add or update entity by id (from fused_buoys message). Same id = same entity.
        """
        pos = (float(position[0]), float(position[1]))
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

    def get_positions_by_type(self, entity_type: str) -> List[Vec2]:
        return [e.position for e in self.entities if e.type == entity_type]

    # -------------------------
    # global planner helpers
    # -------------------------

    def get_start(self) -> Vec2:
        return self.start_position

    def get_goals(self) -> List[Vec2]:
        return self.get_positions_by_type("goal")

    def clear_goals(self) -> None:
        self.entities = [e for e in self.entities if e.type != "goal"]

    def _get_gate_entity_pairs(
        self,
        max_gate_width: float = 15.0,
        max_y_diff: Optional[float] = 5.0,
        min_x_separation: Optional[float] = 2.0,
    ) -> List[Tuple["Entity", "Entity"]]:
        """Same pairing as get_gates() but returns (red_entity, green_entity) for cohort tracking."""
        red_ents = self.get_by_type("red_buoy")
        green_ents = self.get_by_type("green_buoy")
        if not red_ents or not green_ents:
            return []
        red_ents = sorted(red_ents, key=lambda e: e.position[1])
        pairs: List[Tuple[Entity, Entity]] = []
        used = set()
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
                if max_y_diff is not None and abs(dy) > max_y_diff:
                    continue
                if min_x_separation is not None and abs(dx) < min_x_separation:
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
    ) -> List[Tuple[Vec2, Vec2]]:
        """
        Return list of (red_pos, green_pos) gate pairs. Pairs red with nearest green
        within max_gate_width. Optional filter: same Y within max_y_diff and separated
        on X by at least min_x_separation (side-by-side gates only).
        """
        pairs = self._get_gate_entity_pairs(max_gate_width, max_y_diff, min_x_separation)
        return [(r.position, g.position) for r, g in pairs]

    def _prune_oldest_cohort(self) -> None:
        """When we have more than GATE_COHORT_QUEUE_SIZE cohorts, drop oldest and remove those entities.
        Never prune red/green buoys that are part of any current gate pair (so Task3 and others keep the gate).
        """
        if len(self._cohorts) <= GATE_COHORT_QUEUE_SIZE:
            return
        oldest = self._cohorts[0]
        next_old = self._cohorts[1]
        to_remove = oldest - next_old  # ids in oldest that are not in next
        # Protect gate entities: never remove red_buoy or green_buoy that are in a current gate pair
        pairs = self._get_gate_entity_pairs()
        protected_ids = set()
        for r, g in pairs:
            if r.entity_id is not None:
                protected_ids.add(r.entity_id)
            if g.entity_id is not None:
                protected_ids.add(g.entity_id)
        to_remove = to_remove - protected_ids
        self._last_pruned_ids = to_remove
        self.entities = [
            e for e in self.entities
            if e.type == "goal" or e.entity_id is None or e.entity_id not in to_remove
        ]
        self._cohorts.pop(0)

    def _gate_no_go_segments(
        self,
        gates: List[Tuple[Vec2, Vec2]],
        extent_normal_m: float,
        extent_along_m: float,
    ) -> List[Tuple[Vec2, Vec2]]:
        """For each gate, return left and right wall segments (no-go boundaries)."""
        segments: List[Tuple[Vec2, Vec2]] = []
        for red, green in gates:
            rx, ry = float(red[0]), float(red[1])
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

    def get_no_go_obstacle_points(
        self,
        extent_normal_m: float = GATE_NO_GO_EXTENT_NORMAL,
        extent_along_m: float = GATE_NO_GO_EXTENT_ALONG,
        sample_spacing_m: float = GATE_NO_GO_SAMPLE_SPACING,
        map_bounds: Optional[Tuple[float, float]] = None,
    ) -> List[Vec2]:
        """
        No-go zone from gate edges (walls along gate) plus optional map bounds boundary.
        Uses same sample_spacing_m for gate segments and for map boundary when map_bounds is set.
        All units in meters.
        """
        gates = self.get_gates()
        points: List[Vec2] = []
        if gates:
            segments = self._gate_no_go_segments(gates, extent_normal_m, extent_along_m)
            for a, b in segments:
                points.extend(self._sample_segment(a, b, sample_spacing_m))
            self._last_no_go_points = points
        else:
            points = list(self._last_no_go_points)
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
                    points.append(e.position)
        if not points:
            return None
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        return (max_x + padding_m, max_y + padding_m)

    def get_obstacles(self) -> List[Vec2]:
        """
        Return obstacle positions for potential fields.
        Includes *everything* that should repel the boat.
        Excludes: goals and start.
        """
        obstacle_types = {
            "red_buoy", "green_buoy",
            "black_buoy", "debris",
            "yellow_buoy",
            "red_indicator", "green_indicator",
        }
        out: List[Vec2] = []
        for e in self.entities:
            if e.type in obstacle_types:
                out.append(e.position)
        return out

    def get_black_buoys(self) -> List[Tuple[int, Vec2]]:
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


def entity_list_to_detections(entity_list: "EntityList"):
    """
    Build a list of (entity_id, entity_type, position) from non-goal entities.
    Caller can wrap these in DetectedEntity for TaskMaster.get_detections().
    Uses Entity.entity_id when set, else parses from name.
    """
    out = []
    for e in entity_list.entities:
        if e.type == "goal":
            continue
        bid = e.entity_id if e.entity_id is not None else 0
        if bid == 0 and e.name:
            parts = e.name.split("_")
            if len(parts) >= 2 and parts[-1].isdigit():
                bid = int(parts[-1])
        out.append((bid, e.type, e.position))
    return out
