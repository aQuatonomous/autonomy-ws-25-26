import cv2
import numpy as np
from ultralytics import YOLO

# -------------------------
# 1) Yellow blob detection
# -------------------------
def find_yellow_blobs(bgr, min_area=300):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Tune these for your lighting
    lower = np.array([20, 120, 120])   # H, S, V
    upper = np.array([40, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)

    # Clean mask
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        blobs.append((x, y, w, h, area))

    # Biggest first can be helpful
    blobs.sort(key=lambda t: t[4], reverse=True)
    return blobs, mask

# -----------------------------------
# 2) ROI above each blob
# -----------------------------------
def roi_above_blob(img_shape, blob_xywh, above_scale=2.0, pad=20):
    H, W = img_shape[:2]
    x, y, w, h, _ = blob_xywh

    roi_h = int(h * above_scale)

    x1 = max(0, x - pad)
    x2 = min(W, x + w + pad)

    y2 = max(0, y + int(0.2 * h))          # slightly into blob area if you want
    y1 = max(0, y - roi_h - pad)           # above blob

    # Safety: if blob is near top, ROI can collapse
    if y2 <= y1 or x2 <= x1:
        return None

    return (x1, y1, x2, y2)

# -----------------------------------
# 3) Run YOLO on each ROI
# -----------------------------------
def run_yolo_on_rois(bgr, rois, model, conf=0.25):
    detections = []
    for roi in rois:
        x1, y1, x2, y2 = roi
        crop = bgr[y1:y2, x1:x2]

        # Ultralytics returns results list
        results = model.predict(crop, conf=conf, verbose=False)

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                xyxy = box.xyxy.cpu().numpy().reshape(-1)  # [x1,y1,x2,y2] in crop coords
                cls = int(box.cls.cpu().numpy().item())
                score = float(box.conf.cpu().numpy().item())

                # Convert back to full-image coords
                bx1, by1, bx2, by2 = xyxy
                fx1 = int(bx1 + x1)
                fy1 = int(by1 + y1)
                fx2 = int(bx2 + x1)
                fy2 = int(by2 + y1)

                detections.append({
                    "bbox": (fx1, fy1, fx2, fy2),
                    "cls": cls,
                    "conf": score,
                    "roi": roi
                })
    return detections

def draw_debug(bgr, blobs, mask, rois, dets, class_names=None):
    out = bgr.copy()

    for x, y, w, h, area in blobs:
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(out, f"yellow area={int(area)}", (x, max(0, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    for roi in rois:
        x1, y1, x2, y2 = roi
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 0, 0), 2)

    for d in dets:
        x1, y1, x2, y2 = d["bbox"]
        cls = d["cls"]
        conf = d["conf"]
        label = f"{cls} {conf:.2f}"
        if class_names and 0 <= cls < len(class_names):
            label = f"{class_names[cls]} {conf:.2f}"

        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(out, label, (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    return out, mask_bgr

def main():
    # --- inputs
    image_path = "frame.png"
    model_path = "weights.pt"

    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(image_path)

    model = YOLO(model_path)
    class_names = model.names if hasattr(model, "names") else None

    blobs, yellow_mask = find_yellow_blobs(img, min_area=300)

    rois = []
    for b in blobs[:10]:  # limit how many you process
        roi = roi_above_blob(img.shape, b, above_scale=2.0, pad=30)
        if roi:
            rois.append(roi)

    dets = run_yolo_on_rois(img, rois, model, conf=0.25)

    debug_img, debug_mask = draw_debug(img, blobs, yellow_mask, rois, dets, class_names)

    cv2.imwrite("debug_out.png", debug_img)
    cv2.imwrite("yellow_mask.png", debug_mask)
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"SUMMARY: blobs={len(blobs)} rois={len(rois)} detections={len(dets)}")
    print(f"{'='*60}\n")
    
    # Print detailed blob information
    print(f"YELLOW BLOBS DETECTED: {len(blobs)}")
    for i, (x, y, w, h, area) in enumerate(blobs):
        center_x = x + w // 2
        center_y = y + h // 2
        print(f"  Blob {i+1}:")
        print(f"    Position: ({x}, {y})")
        print(f"    Size: {w}x{h} pixels")
        print(f"    Center: ({center_x}, {center_y})")
        print(f"    Area: {int(area)} pixels²")
        print()
    
    # Print detailed ROI information
    print(f"ROIs CREATED: {len(rois)}")
    for i, roi in enumerate(rois):
        x1, y1, x2, y2 = roi
        roi_w = x2 - x1
        roi_h = y2 - y1
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        print(f"  ROI {i+1}:")
        print(f"    Coordinates: ({x1}, {y1}) to ({x2}, {y2})")
        print(f"    Size: {roi_w}x{roi_h} pixels")
        print(f"    Center: ({center_x}, {center_y})")
        print()
    
    # Print detailed detection information
    print(f"YOLO DETECTIONS: {len(dets)}")
    for i, det in enumerate(dets):
        x1, y1, x2, y2 = det["bbox"]
        cls = det["cls"]
        conf = det["conf"]
        roi = det["roi"]
        
        # Get class name
        class_name = f"Class {cls}"
        if class_names and 0 <= cls < len(class_names):
            class_name = class_names[cls]
        
        # Calculate dimensions and center
        det_w = x2 - x1
        det_h = y2 - y1
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        print(f"  Detection {i+1}:")
        print(f"    Class: {class_name} (ID: {cls})")
        print(f"    Confidence: {conf:.3f} ({conf*100:.1f}%)")
        print(f"    Bounding Box: ({x1}, {y1}) to ({x2}, {y2})")
        print(f"    Size: {det_w}x{det_h} pixels")
        print(f"    Center: ({center_x}, {center_y})")
        print(f"    ROI: ({roi[0]}, {roi[1]}) to ({roi[2]}, {roi[3]})")
        print()
    
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
