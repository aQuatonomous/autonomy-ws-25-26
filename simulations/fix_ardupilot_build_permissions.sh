#!/bin/bash
# Fix ArduPilot SITL build when Waf fails with:
#   PermissionError: [Errno 13] Permission denied: '.../build/sitl/.wafpickle-...'
# Run this on the machine where ~/ardupilot lives (e.g. Jetson). Do not run with sudo.
# See simulations/README.md § "SITL build fails: Permission denied on .wafpickle".

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
echo "Done. You can run simulation_full.bash again."
