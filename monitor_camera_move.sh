#!/bin/bash
# Show where each camera is plugged in (by-path). Run after changing cables to see current paths.
# Camera positions used by set_camera_fps.sh and launch_cv: Cam0=1.2, Cam1=1.1, Cam2=1.4.2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Camera positions (by-path) ==="
echo ""

# Expected mapping (from set_camera_fps.sh)
echo "Expected: Cam0 = 1.2, Cam1 = 1.1, Cam2 = 1.4.2"
echo ""

if [ ! -d /dev/v4l/by-path ]; then
    echo "No /dev/v4l/by-path found."
    exit 0
fi

# List capture devices (video-index0) with path and resolved video device
for f in /dev/v4l/by-path/*-video-index0; do
    [ -e "$f" ] || continue
    path="/dev/v4l/by-path/$(basename "$f")"
    target=$(readlink -f "$f" 2>/dev/null || echo "?")
    # Extract port from path (e.g. platform-3610000.usb-usb-0:1.2:1.0 -> 1.2)
    port=$(echo "$path" | sed -n 's/.*usb-0:\([0-9.]*\):1.0.*/\1/p')
    name=""
    if command -v v4l2-ctl &>/dev/null; then
        name=$(v4l2-ctl -d "$path" --info 2>/dev/null | sed -n 's/^.*Card type: *//p' || true)
    fi
    echo "  $path"
    echo "    -> $target  (port $port)  ${name:+$name}"
    echo ""
done

echo "To override at launch: CAMERA_DEVICES=\"/path1,/path2,/path3\" ./comp.sh"
