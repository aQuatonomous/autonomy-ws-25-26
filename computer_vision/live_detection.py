#!/usr/bin/env python3
"""
Live object detection using YOLO model on ArduCam feed.
Displays bounding boxes and class labels in real-time.
Press 'q' to quit.
"""

from ultralytics import YOLO
import cv2
import os
import time
import argparse
from pathlib import Path

# Parse command line arguments
parser = argparse.ArgumentParser(description="Live object detection using YOLO on ArduCam")
parser.add_argument(
    "--camera",
    "-c",
    type=int,
    default=None,
    help="Camera index to use (default: auto-detect)",
)
parser.add_argument(
    "--backend",
    "-b",
    type=str,
    default=None,
    choices=["dshow", "default"],
    help="Camera backend to use: 'dshow' for DirectShow (Windows) or 'default'",
)
args = parser.parse_args()

# Get the script directory to find weights.pt
script_dir = Path(__file__).parent
weights_path = script_dir / "weights.pt"

# Check if weights file exists
if not weights_path.exists():
    print(f"Error: weights.pt not found at {weights_path}")
    print("Please ensure weights.pt is in the computer_vision directory")
    exit(1)

print(f"Loading YOLO model from {weights_path}")
model = YOLO(str(weights_path))
print("Model loaded successfully!")

# Try to open camera (ArduCam might be on different indices or need specific backend)
camera_index = 0
cap = None

# If user specified camera index, try that first
if args.camera is not None:
    print(f"\nAttempting to open camera at index {args.camera}...")
    
    # Determine backend
    if args.backend == "dshow":
        backend_id = cv2.CAP_DSHOW
        backend_name = "DirectShow (Windows)"
    elif args.backend == "default":
        backend_id = cv2.CAP_ANY
        backend_name = "Default"
    else:
        # Try DirectShow first on Windows, then default
        backend_id = cv2.CAP_DSHOW
        backend_name = "DirectShow (Windows)"
    
    cap = cv2.VideoCapture(args.camera, backend_id)
    if cap.isOpened():
        time.sleep(0.2)
        ret, frame = cap.read()
        if ret and frame is not None and frame.size > 0:
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"✓ Camera opened at index {args.camera} ({backend_name}) - {width}x{height}")
            camera_index = args.camera
        else:
            cap.release()
            cap = None
    
    if cap is None:
        print(f"❌ Could not open camera at index {args.camera}")
        print("Trying auto-detection instead...")
        args.camera = None  # Fall back to auto-detection

# Auto-detect camera if not specified or manual attempt failed
if args.camera is None:
    print("\nAttempting to open camera...")
    print("Scanning for available cameras...")

    # On Windows, try DirectShow backend first (CAP_DSHOW = 700)
    # Then try default backend
    backends_to_try = [
        (cv2.CAP_DSHOW, "DirectShow (Windows)"),
        (cv2.CAP_ANY, "Default"),
    ]

    found_cameras = []

    for backend_id, backend_name in backends_to_try:
        print(f"\nTrying {backend_name} backend...")
        for idx in range(10):  # Try indices 0-9
            try:
                test_cap = cv2.VideoCapture(idx, backend_id)
                if test_cap.isOpened():
                    # Give camera a moment to initialize
                    time.sleep(0.1)
                    
                    # Try to read a frame
                    ret, frame = test_cap.read()
                    if ret and frame is not None and frame.size > 0:
                        # Get camera name/info
                        backend = test_cap.getBackendName()
                        width = int(test_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        height = int(test_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        print(f"  ✓ Camera found at index {idx} ({backend_name}) - {width}x{height}")
                        found_cameras.append((idx, backend_id, backend_name, width, height))
                    test_cap.release()
            except Exception as e:
                pass

    if not found_cameras:
        print("\n❌ No cameras detected!")
        print("\nTroubleshooting tips:")
        print("1. Make sure your ArduCam is plugged in via USB")
        print("2. Check Device Manager to see if the camera is recognized")
        print("3. Try unplugging and replugging the camera")
        print("4. Close any other applications using the camera")
        print("5. On Windows, you may need to grant camera permissions")
        print("\nYou can also try specifying a camera index manually:")
        print("  python live_detection.py --camera 0")
        print("  python live_detection.py --camera 1 --backend dshow")
        exit(1)

    # Use the first found camera, or let user choose
    if len(found_cameras) == 1:
        camera_index, backend_id, backend_name, width, height = found_cameras[0]
        print(f"\n✓ Using camera at index {camera_index} ({backend_name})")
    else:
        print(f"\nFound {len(found_cameras)} camera(s):")
        for i, (idx, bid, name, w, h) in enumerate(found_cameras):
            print(f"  {i+1}. Index {idx} ({name}) - {w}x{h}")
        # Use first camera by default, but could add selection here
        camera_index, backend_id, backend_name, width, height = found_cameras[0]
        print(f"\n✓ Using camera at index {camera_index} ({backend_name})")

    # Open the selected camera with the working backend
    cap = cv2.VideoCapture(camera_index, backend_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera at index {camera_index}")
        exit(1)

# Give camera time to fully initialize
time.sleep(0.2)

# Set camera properties for better performance
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

print("\nStarting live detection...")
print("Press 'q' to quit")
print("-" * 50)

frame_count = 0

try:
    while True:
        ret, frame = cap.read()
        
        if not ret or frame is None:
            print("Warning: Could not read frame from camera")
            continue
        
        frame_count += 1
        
        # Run YOLO inference
        results = model.predict(
            source=frame,
            device=0,  # Use GPU if available, otherwise CPU
            imgsz=640,
            conf=0.6,  # Confidence threshold
            verbose=False,
            stream=False,
        )
        
        # Get the first result (since we're processing single frames)
        result = results[0]
        
        # Draw bounding boxes and labels on the frame
        annotated_frame = result.plot()
        
        # Display detection count on frame
        num_detections = len(result.boxes)
        cv2.putText(
            annotated_frame,
            f"Detections: {num_detections}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        # Display FPS (approximate)
        if frame_count % 30 == 0:
            print(f"Frame {frame_count} - Detections: {num_detections}")
        
        # Show the annotated frame
        cv2.imshow('Live Detection - ArduCam', annotated_frame)
        
        # Break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nQuitting...")
            break

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    # Release camera and close windows
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    print("Camera released. Goodbye!")

