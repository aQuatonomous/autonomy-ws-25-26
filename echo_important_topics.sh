#!/bin/bash
# Echo important pipeline topics with filtered output (compact, key fields only).
# Run from autonomy-ws-25-26 root. No node or Python changes; uses ros2 topic echo + filter.
# Requires: jq (for JSON), ros2, mapping + computer_vision + planning workspaces built.
# Source same workspaces as comp_single_camera_task3.sh so you can run this in another
# terminal while the competition pipeline is running and see all topics.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"
PLANNING_WS="${SCRIPT_DIR}/planning"

echo "Sourcing ROS2 and workspaces (same as comp task3)..."
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"
source "${PLANNING_WS}/install/setup.bash"

INTERVAL="${INTERVAL:-0.3}"
TIMEOUT_ECHO=2.5

# --- Filters: take ros2 topic echo stdout, print only key fields ---
filter_boat_pose() {
  grep -E "^\s*(east|north|heading_rad):" 2>/dev/null | sed 's/^[[:space:]]*//' || true
}

filter_cmd_vel() {
  awk '
    /^linear:/ { l=1; a=0; next }
    /^angular:/ { l=0; a=1; next }
    l && /^\s*x:/ { gsub(/^[^:]+:[[:space:]]*/,""); lx=$0 }
    l && /^\s*y:/ { gsub(/^[^:]+:[[:space:]]*/,""); ly=$0 }
    l && /^\s*z:/ { gsub(/^[^:]+:[[:space:]]*/,""); lz=$0 }
    a && /^\s*z:/ { gsub(/^[^:]+:[[:space:]]*/,""); az=$0 }
    END { if (lx!=="" || az!=="") print "  linear: x="lx" y="ly" z="lz"  angular.z="az }
  ' 2>/dev/null || grep -E "^\s*(x|y|z):" 2>/dev/null | sed 's/^[[:space:]]*//'
}

filter_global_detections() {
  grep -E "^\s+(east|north|class_name|class_id|id|source):" 2>/dev/null | sed 's/^[[:space:]]*//' || true
}

filter_combined_detection() {
  sed -n 's/^data: //p' 2>/dev/null | tr -d "'" | jq -r '
    .detections[]? |
    "  class_id=\(.class_id // "?") class_name=\(.class_name // "?") distance_m=\(.distance_m // "?") bearing_deg=\(.bearing_deg // "?") bbox=\(.bbox // "?")"
  ' 2>/dev/null || true
}

echo "Pipeline echo (filtered). Interval: ${INTERVAL}s. Ctrl+C to stop."
echo "  Topics: /boat_pose, /global_detections, /combined/detection_info_with_distance, /mavros/setpoint_velocity/cmd_vel_unstamped"
echo ""

while true; do
  echo "========== $(date '+%H:%M:%S') =========="

  echo "[boat_pose]"
  out=$(timeout "$TIMEOUT_ECHO" ros2 topic echo --once /boat_pose 2>/dev/null | filter_boat_pose)
  [[ -n "$out" ]] && echo "$out" || echo "  (no message)"
  echo ""

  echo "[cmd_vel (planner → MAVROS)]"
  out=$(timeout "$TIMEOUT_ECHO" ros2 topic echo --once /mavros/setpoint_velocity/cmd_vel_unstamped 2>/dev/null | filter_cmd_vel)
  [[ -n "$out" ]] && echo "$out" || echo "  (no message)"
  echo ""

  echo "[global_detections]"
  out=$(timeout "$TIMEOUT_ECHO" ros2 topic echo --once /global_detections 2>/dev/null | filter_global_detections)
  [[ -n "$out" ]] && echo "$out" || echo "  (no message)"
  echo ""

  echo "[combined/detection_info_with_distance]"
  out=$(timeout "$TIMEOUT_ECHO" ros2 topic echo --once /combined/detection_info_with_distance 2>/dev/null | filter_combined_detection)
  [[ -n "$out" ]] && echo "$out" || echo "  (no message)"
  echo ""

  sleep "$INTERVAL"
done
