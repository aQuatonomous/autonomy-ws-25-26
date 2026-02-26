#!/usr/bin/env bash
# Echo planning-related topics to the terminal.
# Run this in a separate terminal; run the comp (e.g. task_test_comp.sh) in another.
#
# Usage:
#   ./planning_echo.sh
#   ./planning_echo.sh /mavros/setpoint_velocity/cmd_vel_unstamped   # custom cmd_vel topic
#
# Topics echoed: /planned_path (global waypoint set), /curr_task, /gs_message_send, /messages/gate_pass, cmd_vel

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${SCRIPT_DIR}"

source /opt/ros/humble/setup.bash
[ -f "${WS_ROOT}/install/setup.bash" ] && source "${WS_ROOT}/install/setup.bash"

echo "=== Planning echo (Ctrl+C to stop) ==="
echo "Topics: /planned_path (with waypoints), /curr_task, /gs_message_send, /messages/gate_pass, cmd_vel"
echo ""

exec python3 "${SCRIPT_DIR}/planning_echo.py" "$@"
