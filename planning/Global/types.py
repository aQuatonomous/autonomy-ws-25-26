#!/usr/bin/env python3
"""
Shared types for Global tasks (Task1/2/3) and TaskMaster.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

Vec2 = Tuple[float, float]


@dataclass
class DetectedEntity:
    """A single detected entity (buoy, indicator, etc.) from perception."""
    entity_id: int
    entity_type: str
    position: Vec2
