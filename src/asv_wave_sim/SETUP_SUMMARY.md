# asv_wave_sim Setup Summary (ROS 2 Jazzy)

This document summarizes the Jazzy compatibility changes and provides quick start instructions.

---

## What Was Done

1. **Fixed Humble → Jazzy references:**
   - `aquatonomous_simulation_full.bash`: Fixed line 26 to source Jazzy instead of Humble
   - `AQUATONOMOUS_SIM.md`: Already had Jazzy references (lines 238, 317, 549)

2. **Created standalone test script:**
   - `test_gazebo_standalone.sh`: Run Gazebo in a single terminal (no tmux) for easy testing and debugging

3. **Created documentation:**
   - `MANUAL_LAUNCH.md`: Instructions for running each component in separate terminals
   - `EXTERNAL_SETUP.md`: Documentation of external dependencies (SITL_Models, ardupilot_gazebo, etc.)

---

## Quick Start

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y libcgal-dev libfftw3-dev
```

### 2. Build asv_wave_sim

```bash
cd ~/autonomy-ws-25-26/src/asv_wave_sim
colcon build --symlink-install --merge-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=ON \
  -DCMAKE_CXX_STANDARD=17
```

This creates:
- `install/setup.bash` (source this to use the package)
- `install/lib/` (waves/hydrodynamics plugin libraries)

### 3. Test Gazebo (Minimal – No Boat)

```bash
cd ~/autonomy-ws-25-26
bash src/asv_wave_sim/test_gazebo_standalone.sh
```

**Expected result:**
- Gazebo opens with water surface (waves)
- Course models load: buoys, gates, rope holds
- Error "Unable to find uri[model://ourboat]" (expected; no boat model yet)

This confirms the wave sim works!

### 4. Full Stack Setup (Optional)

For the complete simulation with the boat, see:
- `EXTERNAL_SETUP.md` – How to set up SITL_Models, ardupilot_gazebo, bridge_ws, etc.
- `MANUAL_LAUNCH.md` – How to run all components in separate terminals

Or use the tmux-based launcher:
```bash
bash ~/autonomy-ws-25-26/src/asv_wave_sim/aquatonomous_simulation_full.bash
```

---

## Files Created/Modified

| File | Status | Description |
|------|--------|-------------|
| `aquatonomous_simulation_full.bash` | Modified | Fixed Humble→Jazzy on line 26 |
| `AQUATONOMOUS_SIM.md` | Already Jazzy | No changes needed (already had jazzy refs) |
| `test_gazebo_standalone.sh` | NEW | Standalone Gazebo test (no tmux) |
| `MANUAL_LAUNCH.md` | NEW | Separate terminal launch instructions |
| `EXTERNAL_SETUP.md` | NEW | External dependencies documentation |
| `SETUP_SUMMARY.md` | NEW | This file |

---

## Testing Checklist

- [ ] Install CGAL and FFTW: `sudo apt install -y libcgal-dev libfftw3-dev`
- [ ] Build asv_wave_sim: `cd ~/autonomy-ws-25-26/src/asv_wave_sim && colcon build ...`
- [ ] Run test script: `bash ~/autonomy-ws-25-26/src/asv_wave_sim/test_gazebo_standalone.sh`
- [ ] Verify Gazebo opens with water + course models
- [ ] (Optional) Set up external dependencies and test full stack

---

## Next Steps

1. **Minimal test (recommended first):**
   - Build asv_wave_sim
   - Run `test_gazebo_standalone.sh`
   - Verify water and course models load

2. **Full boat simulation:**
   - Follow `EXTERNAL_SETUP.md` to set up boat model, SITL, bridges, etc.
   - Use `MANUAL_LAUNCH.md` to run components in separate terminals
   - Or use `aquatonomous_simulation_full.bash` for tmux-based launch

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `libcgal-dev` or `libfftw3-dev` not found | `sudo apt update && sudo apt install -y libcgal-dev libfftw3-dev` |
| Build fails | Check you're using C++17 and have Gazebo Harmonic installed |
| Gazebo won't start | Ensure ROS 2 Jazzy is sourced: `source /opt/ros/jazzy/setup.bash` |
| "Unable to find uri[model://waves]" | Check that `GZ_SIM_RESOURCE_PATH` includes `gz-waves-models/models` (test script does this) |
| "Unable to find uri[model://ourboat]" | Expected without `~/SITL_Models`; see `EXTERNAL_SETUP.md` |
| "Missing COLLADA tag" + segmentation fault | The `ourboat` model uses STL for the hull visual to avoid COLLADA loader crashes. Other visuals are commented out in `~/SITL_Models/Gazebo/models/ourboat/model.sdf`. To restore full appearance, export meshes as STL/OBJ instead of DAE. |

---

## Summary

The asv_wave_sim package is now compatible with ROS 2 Jazzy. You can:

1. **Test immediately:** Build asv_wave_sim and run the standalone test script to see water + course models
2. **Add boat later:** Follow EXTERNAL_SETUP.md to set up the full boat simulation stack

All scripts and docs are in `~/autonomy-ws-25-26/src/asv_wave_sim/`.
