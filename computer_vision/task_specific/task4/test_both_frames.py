#!/usr/bin/env python3
"""
Test both frame.png (triangle) and frame2.png (cross) to compare confidence levels.
"""

import cv2
import os
from ultralytics import YOLO

def test_image(image_path, model, class_names, target_class_id, target_name):
    """Test a single image and return detection results."""
    img = cv2.imread(image_path)
    if img is None:
        print(f"Could not load: {image_path}")
        return None
    
    print(f"\n{'='*60}")
    print(f"Testing: {os.path.basename(image_path)}")
    print(f"Image shape: {img.shape}")
    print(f"Looking for: {target_name} (class_id {target_class_id})")
    print("="*60)
    
    # Test at different confidence levels
    best_detection = None
    best_conf = 0.0
    
    for conf_thresh in [0.25, 0.15, 0.10, 0.05, 0.01, 0.001]:
        results = model.predict(img, conf=conf_thresh, verbose=False)
        
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls = int(box.cls.cpu().numpy().item())
                score = float(box.conf.cpu().numpy().item())
                
                if cls == target_class_id:
                    if score > best_conf:
                        best_conf = score
                        xyxy = box.xyxy.cpu().numpy().reshape(-1)
                        best_detection = {
                            "conf": score,
                            "bbox": (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]))
                        }
    
    if best_detection:
        print(f"✓ {target_name} detected!")
        print(f"  Confidence: {best_detection['conf']:.3f} ({best_detection['conf']*100:.1f}%)")
        print(f"  Bounding box: {best_detection['bbox']}")
        return best_detection
    else:
        print(f"✗ {target_name} NOT detected at any confidence level")
        return None

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "weights.pt")
    
    # Load model
    model = YOLO(model_path)
    class_names = model.names
    
    print("="*60)
    print("COMPARING TRIANGLE vs CROSS DETECTIONS")
    print("="*60)
    print(f"Model classes: {len(class_names)}")
    print(f"Model file: {model_path}")
    
    # Test frame.png (should have triangle)
    frame1_path = os.path.join(script_dir, "frame.png")
    triangle_result = test_image(frame1_path, model, class_names, 8, "triangle (class_id 8)")
    
    # Test frame2.png (should have cross)
    frame2_path = os.path.join(script_dir, "frame2.png")
    cross_result = test_image(frame2_path, model, class_names, 2, "black_cross (class_id 2)")
    
    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    if triangle_result:
        print(f"Triangle confidence: {triangle_result['conf']*100:.1f}%")
    else:
        print("Triangle: NOT DETECTED")
    
    if cross_result:
        print(f"Cross confidence: {cross_result['conf']*100:.1f}%")
    else:
        print("Cross: NOT DETECTED")
    
    print("\n" + "="*60)
    print("ANALYSIS")
    print("="*60)
    print("If Roboflow shows:")
    print("  - Triangle: ~87%")
    print("  - Cross: ~80%")
    print("\nBut we're seeing much lower confidence, this suggests:")
    print("  1. Model file (weights.pt) is different from Roboflow version")
    print("  2. Image preprocessing differs (normalization, resizing)")
    print("  3. Model might need to be re-exported from Roboflow")
    print("\nTo fix:")
    print("  - Export the model from Roboflow in PyTorch format")
    print("  - Make sure it's the SAME model version showing 87%/80%")
    print("  - Check Roboflow export settings match your usage")

if __name__ == "__main__":
    main()
