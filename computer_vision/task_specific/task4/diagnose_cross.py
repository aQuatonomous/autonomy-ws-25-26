#!/usr/bin/env python3
"""
Diagnose why cross confidence is low compared to triangle.
"""

import cv2
import os
import numpy as np
from ultralytics import YOLO

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    frame1_path = os.path.join(script_dir, "frame.png")
    frame2_path = os.path.join(script_dir, "frame2.png")
    model_path = os.path.join(script_dir, "weights.pt")
    
    model = YOLO(model_path)
    class_names = model.names
    
    print("="*60)
    print("DIAGNOSING CROSS vs TRIANGLE CONFIDENCE")
    print("="*60)
    
    # Test frame1 (triangle)
    print("\n1. FRAME.PNG (Triangle):")
    img1 = cv2.imread(frame1_path)
    results1 = model.predict(img1, conf=0.001, verbose=False)
    triangle_dets = []
    for r in results1:
        if r.boxes is None:
            continue
        for box in r.boxes:
            cls = int(box.cls.cpu().numpy().item())
            score = float(box.conf.cpu().numpy().item())
            if cls == 3:  # black_triangle
                xyxy = box.xyxy.cpu().numpy().reshape(-1)
                triangle_dets.append({
                    "conf": score,
                    "bbox": (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]))
                })
    
    if triangle_dets:
        best_t = max(triangle_dets, key=lambda x: x["conf"])
        print(f"   Best triangle: {best_t['conf']:.3f} ({best_t['conf']*100:.1f}%)")
        print(f"   Bbox: {best_t['bbox']}")
        print(f"   Image size: {img1.shape[1]}x{img1.shape[0]}")
        
        # Extract triangle region
        t_x1, t_y1, t_x2, t_y2 = best_t['bbox']
        t_region = img1[t_y1:t_y2, t_x1:t_x2]
        t_area = (t_x2 - t_x1) * (t_y2 - t_y1)
        print(f"   Triangle region size: {t_region.shape}")
        print(f"   Triangle area: {t_area} pixels²")
        print(f"   Triangle size relative to image: {t_area / (img1.shape[0] * img1.shape[1]) * 100:.1f}%")
    
    # Test frame2 (cross)
    print("\n2. FRAME2.PNG (Cross):")
    img2 = cv2.imread(frame2_path)
    results2 = model.predict(img2, conf=0.001, verbose=False)
    cross_dets = []
    for r in results2:
        if r.boxes is None:
            continue
        for box in r.boxes:
            cls = int(box.cls.cpu().numpy().item())
            score = float(box.conf.cpu().numpy().item())
            if cls == 2:  # black_cross
                xyxy = box.xyxy.cpu().numpy().reshape(-1)
                cross_dets.append({
                    "conf": score,
                    "bbox": (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]))
                })
    
    if cross_dets:
        best_c = max(cross_dets, key=lambda x: x["conf"])
        print(f"   Best cross: {best_c['conf']:.3f} ({best_c['conf']*100:.1f}%)")
        print(f"   Bbox: {best_c['bbox']}")
        print(f"   Image size: {img2.shape[1]}x{img2.shape[0]}")
        
        # Extract cross region
        c_x1, c_y1, c_x2, c_y2 = best_c['bbox']
        c_region = img2[c_y1:c_y2, c_x1:c_x2]
        c_area = (c_x2 - c_x1) * (c_y2 - c_y1)
        print(f"   Cross region size: {c_region.shape}")
        print(f"   Cross area: {c_area} pixels²")
        print(f"   Cross size relative to image: {c_area / (img2.shape[0] * img2.shape[1]) * 100:.1f}%")
    
    # Comparison
    print("\n" + "="*60)
    print("COMPARISON:")
    print("="*60)
    if triangle_dets and cross_dets:
        print(f"Triangle confidence: {best_t['conf']*100:.1f}%")
        print(f"Cross confidence: {best_c['conf']*100:.1f}%")
        print(f"Difference: {abs(best_t['conf'] - best_c['conf'])*100:.1f} percentage points")
        print()
        print("Possible reasons for lower cross confidence:")
        print("  1. Cross might be smaller relative to image")
        print("  2. Cross might be less clear/sharp in frame2.png")
        print("  3. Model might have been trained better on triangles")
        print("  4. Image quality/resolution differences")
        print("  5. Cross might be partially occluded or at different angle")
    
    # Check if we can improve cross detection by testing different image sizes
    print("\n" + "="*60)
    print("TESTING DIFFERENT IMAGE SIZES FOR CROSS:")
    print("="*60)
    if cross_dets:
        for imgsz in [640, 1280, None]:  # None = auto
            if imgsz:
                results = model.predict(img2, conf=0.001, imgsz=imgsz, verbose=False)
            else:
                results = model.predict(img2, conf=0.001, verbose=False)
            
            for r in results:
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    cls = int(box.cls.cpu().numpy().item())
                    score = float(box.conf.cpu().numpy().item())
                    if cls == 2:
                        imgsz_str = f"imgsz={imgsz}" if imgsz else "auto"
                        print(f"  {imgsz_str}: cross confidence = {score:.3f} ({score*100:.1f}%)")

if __name__ == "__main__":
    main()
