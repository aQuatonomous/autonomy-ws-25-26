#!/usr/bin/env python3
"""Entry point for LR_Bouy node. Adds planning to path and runs main from Left_Right_Bouy."""

import os
import sys

# Resolve planning path (same logic as global_planner.launch.py)
planning_path = os.environ.get("PLANNING_PATH", "")
if not planning_path or not os.path.isdir(planning_path):
    # Fallback: infer from package share location
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("global_planner")
        for rel in [
            os.path.join(share, "..", "..", "..", "src", "planning"),
            os.path.join(share, "..", "..", "..", "..", "planning"),
            os.path.join(os.path.dirname(__file__), "..", "..", "..", "planning"),
        ]:
            p = os.path.abspath(rel)
            if os.path.isdir(p):
                planning_path = p
                break
    except Exception:
        pass

if planning_path and os.path.isdir(planning_path):
    if planning_path not in sys.path:
        sys.path.insert(0, planning_path)

from Global.Left_Right_Bouy import main


if __name__ == "__main__":
    main()
