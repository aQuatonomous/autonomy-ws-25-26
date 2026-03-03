#!/bin/bash
# Fix ArduPilot SITL build permission errors:
#   PermissionError: [Errno 13] Permission denied: '.../build/sitl/.wafpickle-...'
# Run this on the machine where ~/ardupilot lives (e.g. Jetson). Do not run with sudo.
# See SIMULATION.md § "SITL build fails: Permission denied on `.wafpickle` (Jetson)".

set -e
ARDUPILOT="${ARDUPILOT:-$HOME/ardupilot}"

if [[ ! -d "$ARDUPILOT" ]]; then
  echo "Error: ArduPilot dir not found: $ARDUPILOT"
  echo "Set ARDUPILOT if it is elsewhere, e.g. ARDUPILOT=/path/to/ardupilot $0"
  exit 1
fi

echo "Fixing ownership and cleaning SITL build in: $ARDUPILOT"
sudo chown -R "$USER:$USER" "$ARDUPILOT"
rm -rf "$ARDUPILOT/build/sitl"
cd "$ARDUPILOT"
./waf configure
./waf rover
echo "Done. You can run run_full_simulation_tmux.bash again."
