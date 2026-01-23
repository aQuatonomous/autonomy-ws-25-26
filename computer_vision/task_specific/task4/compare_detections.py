#!/usr/bin/env python3
"""
Compare triangle and cross detections to see why confidence differs.
"""

import cv2
import os
from ultralytics import YOLO

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(script_dir, "frame2.png")
    model_path = os.path.join(script_dir, "weights.pt")
    
    # Load image
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {image_path}")
    
    print(f"Image shape: {img.shape}")
    print()
    
    # Load model
    model = YOLO(model_path)
    class_names = model.names
    
    # Test with different confidence thresholds
    print("=" * 60)
    print("Testing detections at different confidence levels:")
    print("=" * 60)
    
    for conf_thresh in [0.25, 0.15, 0.10, 0.05, 0.01, 0.005, 0.001]:
        results = model.predict(img, conf=conf_thresh, verbose=False)
        
        triangles = []
        crosses = []
        
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls = int(box.cls.cpu().numpy().item())
                score = float(box.conf.cpu().numpy().item())
                
                if cls == 3:  # black_triangle
                    triangles.append(score)
                elif cls == 2:  # black_cross
                    crosses.append(score)
        
        if triangles or crosses:
            print(f"\nConfidence threshold: {conf_thresh}")
            if triangles:
                print(f"  Triangles detected: {[f'{s:.3f} ({s*100:.1f}%)' for s in sorted(triangles, reverse=True)]}")
            if crosses:
                print(f"  Crosses detected: {[f'{s:.3f} ({s*100:.1f}%)' for s in sorted(crosses, reverse=True)]}")
    
    # Now test on full image with very low confidence to see all detections
    print("\n" + "=" * 60)
    print("All detections at conf=0.001:")
    print("=" * 60)
    results = model.predict(img, conf=0.001, verbose=False)
    
    all_dets = []
    for r in results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            xyxy = box.xyxy.cpu().numpy().reshape(-1)
            cls = int(box.cls.cpu().numpy().item())
            score = float(box.conf.cpu().numpy().item())
            class_name = class_names.get(cls, f"Class {cls}")
            all_dets.append({
                "class": class_name,
                "cls_id": cls,
                "conf": score,
                "bbox": (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]))
            })
    
    # Sort by confidence
    all_dets.sort(key=lambda x: x["conf"], reverse=True)
    
    print(f"\nTotal detections: {len(all_dets)}")
    print("\nTop 15 detections:")
    for i, det in enumerate(all_dets[:15], 1):
        x1, y1, x2, y2 = det["bbox"]
        print(f"  {i}. {det['class']} (ID: {det['cls_id']}): {det['conf']:.3f} ({det['conf']*100:.1f}%) "
              f"bbox=({x1},{y1})-({x2},{y2})")
    
    # Check if triangle and cross are in similar locations
    triangle_dets = [d for d in all_dets if d["cls_id"] == 3]
    cross_dets = [d for d in all_dets if d["cls_id"] == 2]
    
    print("\n" + "=" * 60)
    print("Triangle vs Cross comparison:")
    print("=" * 60)
    if triangle_dets:
        best_triangle = triangle_dets[0]
        print(f"Best triangle: {best_triangle['conf']:.3f} ({best_triangle['conf']*100:.1f}%)")
        print(f"  Bbox: {best_triangle['bbox']}")
    if cross_dets:
        best_cross = cross_dets[0]
        print(f"Best cross: {best_cross['conf']:.3f} ({best_cross['conf']*100:.1f}%)")
        print(f"  Bbox: {best_cross['bbox']}")
    
    # Check image regions
    print("\n" + "=" * 60)
    print("Image analysis:")
    print("=" * 60)
    print(f"Image size: {img.shape[1]}x{img.shape[0]}")
    
    # Extract and check the regions where triangle and cross are detected
    if triangle_dets and cross_dets:
        t_bbox = best_triangle['bbox']
        c_bbox = best_cross['bbox']
        
        # Extract regions
        t_region = img[t_bbox[1]:t_bbox[3], t_bbox[0]:t_bbox[2]]
        c_region = img[c_bbox[1]:c_bbox[3], c_bbox[0]:c_bbox[2]]
        
        print(f"\nTriangle region: {t_region.shape}")
        print(f"Cross region: {c_region.shape}")
        
        # Save regions for inspection
        cv2.imwrite(os.path.join(script_dir, "triangle_region.png"), t_region)
        cv2.imwrite(os.path.join(script_dir, "cross_region.png"), c_region)
        print(f"\nSaved regions to triangle_region.png and cross_region.png")

if __name__ == "__main__":
    main()
