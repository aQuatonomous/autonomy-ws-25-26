#!/usr/bin/env python3
"""
Check all triangle class IDs to see which one is detected.
"""

import cv2
import os
from ultralytics import YOLO

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    frame1_path = os.path.join(script_dir, "frame.png")
    model_path = os.path.join(script_dir, "weights.pt")
    
    img = cv2.imread(frame1_path)
    if img is None:
        raise FileNotFoundError(f"Could not load: {frame1_path}")
    
    model = YOLO(model_path)
    class_names = model.names
    
    print(f"Image shape: {img.shape}")
    print(f"Looking for triangles in frame.png (yellow boat)")
    print()
    
    # Check all triangle class IDs: 3 (black), 8 (blue), 14 (green)
    triangle_ids = [3, 8, 14]
    triangle_names = {3: "black_triangle", 8: "blue_triangle", 14: "green_triangle"}
    
    print("Testing at conf=0.001 to see all detections:")
    results = model.predict(img, conf=0.001, verbose=False)
    
    all_dets = []
    for r in results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            cls = int(box.cls.cpu().numpy().item())
            score = float(box.conf.cpu().numpy().item())
            class_name = class_names.get(cls, f"Class {cls}")
            all_dets.append((class_name, cls, score))
    
    all_dets.sort(key=lambda x: x[2], reverse=True)
    
    print(f"\nAll detections ({len(all_dets)} total):")
    for class_name, cls, score in all_dets[:20]:
        marker = " ← TRIANGLE" if cls in triangle_ids else ""
        print(f"  {class_name} (ID: {cls}): {score:.3f} ({score*100:.1f}%){marker}")
    
    # Check specifically for triangles
    print("\n" + "="*60)
    print("TRIANGLE DETECTIONS:")
    print("="*60)
    triangles_found = [d for d in all_dets if d[1] in triangle_ids]
    if triangles_found:
        for class_name, cls, score in triangles_found:
            print(f"  {class_name} (ID: {cls}): {score:.3f} ({score*100:.1f}%)")
    else:
        print("  NO TRIANGLES DETECTED")
        print("\n  This confirms the model file is likely different from Roboflow")
        print("  Roboflow shows 87% for triangle, but this model shows nothing")

if __name__ == "__main__":
    main()
