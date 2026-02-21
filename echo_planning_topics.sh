#!/bin/bash
# Keep checking whether planning topics are published (global_planner must be running)

TOPICS=(
  "/mavros/setpoint_velocity/cmd_vel_unstamped"
  "/planned_path"
  "/curr_task"
)

echo "Checking planning topics every 2s (Ctrl+C to stop). Source ROS first if needed."
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
