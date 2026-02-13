#!/usr/bin/env python3
"""Extract 200 calibration frames from my_video-3.mkv into calibrate_frames_200/."""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "camera_grid_calibration_1"))
from extract_frames import extract_frames

here = os.path.dirname(os.path.abspath(__file__))
video = os.path.join(here, "my_video-3.mkv")
out_dir = os.path.join(here, "calibrate_frames_200")
extract_frames(video, out_dir, num_frames=200)
