#!/bin/bash
# Keep checking whether planning topics are published (global_planner must be running)
# Run from autonomy-ws-25-26 root. Logging goes to this terminal.

set -e
source /opt/ros/humble/setup.bash

TOPICS=(
  "/mavros/setpoint_velocity/cmd_vel_unstamped"
  "/planned_path"
  "/curr_task"
)

echo "Checking planning topics every 2s (Ctrl+C to stop)."
echo ""

while true; do
  echo "--- $(date '+%H:%M:%S') ---"
  for topic in "${TOPICS[@]}"; do
    if ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
      echo "  OK   $topic"
    else
      echo "  --   $topic (no publisher)"
    fi
  done
  echo ""
  sleep 2
done
