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
import json
from typing import List, Tuple, Optional

Vec2 = Tuple[float, float]


class Entity:
    """Base class for all entities in the environment"""
    def __init__(self, entity_type: str, position: Vec2, name: Optional[str] = None):
        self.type = entity_type
        self.position = (float(position[0]), float(position[1]))
        self.name = name or f"{entity_type}_{id(self)}"
        self.charge = self._assign_charge(entity_type)

    def _assign_charge(self, entity_type: str) -> float:
        # Potential fields: goals attract, everything else repels.
        return 1.0 if entity_type == "goal" else -1.0

    def __repr__(self) -> str:
        return f"{self.type}({self.name}) at {self.position}"


class EntityList:
    """Container for all entities with helper methods used by global planner tasks."""
    def __init__(self, start_position: Vec2 = (0.0, 0.0)):
        self.entities: List[Entity] = []
        self.start_position: Vec2 = (float(start_position[0]), float(start_position[1]))

    # -------------------------
    # core CRUD
    # -------------------------

    def set_start_position(self, position: Vec2) -> None:
        self.start_position = (float(position[0]), float(position[1]))

    def add(self, entity_type: str, position: Vec2, name: Optional[str] = None) -> Entity:
        ent = Entity(entity_type, position, name)
        self.entities.append(ent)
        return ent

    def add_or_update(self, entity_type: str, position: Vec2, name: Optional[str] = None, 
                     tolerance: float = 1.0) -> Entity:
        """
        Add entity or update existing one if found within tolerance distance.
        This prevents duplicate entities from accumulating over multiple perception updates.
        """
        pos = (float(position[0]), float(position[1]))
        
        # Look for existing entity of same type within tolerance
        for existing in self.entities:
            if existing.type == entity_type:
                dx = existing.position[0] - pos[0]
                dy = existing.position[1] - pos[1]
                dist = (dx*dx + dy*dy)**0.5
                if dist <= tolerance:
                    # Update existing entity
                    existing.position = pos
                    if name:
                        existing.name = name
                    return existing
        
        # No existing entity found, add new one
        return self.add(entity_type, position, name)

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

    def get_gates(self) -> List[Tuple[Vec2, Vec2]]:
        """
        Return list of (red_pos, green_pos) gate pairs using robust nearest-neighbor pairing.
        Each red buoy is paired with its nearest green buoy within 15ft max distance.
        This prevents gates from forming across large distances and handles uneven counts.
        """
        red_buoys = self.get_positions_by_type("red_buoy")
        green_buoys = self.get_positions_by_type("green_buoy")
        
        if not red_buoys or not green_buoys:
            return []

        gates = []
        used_green = set()
        
        # Sort reds by y-coordinate for consistent ordering
        red_buoys = sorted(red_buoys, key=lambda p: p[1])
        
        for red in red_buoys:
            closest_green = None
            min_dist = float('inf')
            closest_idx = -1
            
            for i, green in enumerate(green_buoys):
                if i in used_green:
                    continue
                    
                dx = red[0] - green[0]
                dy = red[1] - green[1]
                dist = (dx*dx + dy*dy)**0.5
                
                if dist < min_dist and dist <= 15.0:  # Max gate width of 15ft
                    min_dist = dist
                    closest_green = green
                    closest_idx = i
            
            if closest_green is not None:
                gates.append((red, closest_green))
                used_green.add(closest_idx)
        
        return gates

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


def load_entities_from_json(json_path: str, infer_goals: bool = False):
    """
    Load entities and map bounds from a JSON file.

    If infer_goals=True:
      - build gate centers from red/green buoys and add them as goal_wp1..N
      - tasks will overwrite these later anyway, but it helps for quick demos
    """
    with open(json_path, "r") as f:
        data = json.load(f)

    map_bounds_data = data.get("map_bounds", {})
    map_bounds = (
        float(map_bounds_data.get("width", 0.0)),
        float(map_bounds_data.get("height", 0.0)),
    )

    entities = EntityList()

    entity_data = data.get("entities", {})

    # Start
    start = entity_data.get("start")
    if start and "position" in start:
        entities.set_start_position(tuple(start["position"]))

    # Buoys
    for buoy in entity_data.get("buoys", []):
        entities.add(buoy["type"], tuple(buoy["position"]), buoy.get("name"))

    # Goals (optional)
    for goal in entity_data.get("goals", []):
        entities.add("goal", tuple(goal["position"]), goal.get("name"))

    # Debris list (legacy style - type "debris")
    for debris in entity_data.get("debris", []):
        entities.add("debris", tuple(debris["position"]), debris.get("name"))

    # Optional inference: gate centers
    if infer_goals and not entities.get_goals():
        for i, (red, green) in enumerate(entities.get_gates(), start=1):
            center = ((red[0] + green[0]) / 2.0, (red[1] + green[1]) / 2.0)
            entities.add("goal", center, name=f"goal_wp{i}")

    return entities, map_bounds
