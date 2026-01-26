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

# -------------------------
# 1b) Black blob detection (for black boats)
# -------------------------
def find_black_blobs(bgr, min_area=300):
    """
    Detect black/dark grey blobs in the image.
    Uses HSV color space with low value (brightness) threshold.
    Captures black, dark grey, and near-black colors.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Black/dark grey detection: low brightness (V channel)
    # H and S can vary, but V should be low for black/dark grey objects
    # Increased V threshold to capture dark greys (up to ~100 for better coverage)
    # This captures pure black (V=0) through dark greys (V up to 100)
    lower = np.array([0, 0, 0])        # H, S, V - very dark (pure black)
    upper = np.array([180, 255, 100])  # Allow all hues, all saturations, but low-medium brightness for dark greys

    mask = cv2.inRange(hsv, lower, upper)
    
    # Also try LAB color space for better dark color detection
    # LAB L channel is more perceptually uniform for dark colors
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l_channel = lab[:, :, 0]  # L channel (lightness)
    
    # Create a mask for dark colors in LAB space (L < 50 for dark greys/blacks)
    lab_mask = (l_channel < 50).astype(np.uint8) * 255
    
    # Combine both masks (union) for better coverage
    mask = cv2.bitwise_or(mask, lab_mask)

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

def draw_debug(bgr, blobs, mask, rois, dets, class_names=None, boat_color="yellow"):
    out = bgr.copy()

    # Choose color based on boat type
    if boat_color == "black":
        blob_color = (0, 0, 255)  # Red for black boats (BGR)
        text_color = (0, 0, 255)  # Red text
        blob_label = "black"
    else:
        blob_color = (0, 255, 255)  # Yellow for yellow boats (BGR)
        text_color = (0, 255, 255)  # Yellow text
        blob_label = "yellow"

    # Draw boxes around detected blobs with labels
    for i, (x, y, w, h, area) in enumerate(blobs):
        # Draw rectangle around blob (thicker line for visibility)
        cv2.rectangle(out, (x, y), (x + w, y + h), blob_color, 3)
        
        # Draw label above the blob with background for better visibility
        label_text = f"{blob_label} area={int(area)}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        
        # Get text size for background rectangle
        (text_width, text_height), baseline = cv2.getTextSize(label_text, font, font_scale, thickness)
        
        # Position label above the blob
        label_x = x
        label_y = max(text_height + 5, y - 5)
        
        # Draw background rectangle for text (black with some transparency effect)
        cv2.rectangle(out, 
                     (label_x - 2, label_y - text_height - 2), 
                     (label_x + text_width + 2, label_y + baseline + 2), 
                     (0, 0, 0), -1)  # Black background
        
        # Draw text
        cv2.putText(out, label_text, (label_x, label_y),
                    font, font_scale, text_color, thickness, cv2.LINE_AA)

    # Draw ROI boxes above blobs (green rectangles)
    for i, roi in enumerate(rois):
        x1, y1, x2, y2 = roi
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green ROI box
        # Optionally label the ROI
        cv2.putText(out, f"ROI {i+1}", (x1, max(y1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # Draw detection boxes (cyan/magenta for YOLO detections to distinguish from ROI boxes)
    for d in dets:
        x1, y1, x2, y2 = d["bbox"]
        cls = d["cls"]
        conf = d["conf"]
        label = f"{cls} {conf:.2f}"
        if class_names and 0 <= cls < len(class_names):
            label = f"{class_names[cls]} {conf:.2f}"

        # Draw cyan rectangle around detection (to distinguish from green ROI boxes)
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 255, 0), 3)  # Cyan in BGR
        
        # Draw label with background
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)
        
        label_x = x1
        label_y = max(text_height + 5, y1 - 5)
        
        # Background for text
        cv2.rectangle(out,
                     (label_x - 2, label_y - text_height - 2),
                     (label_x + text_width + 2, label_y + baseline + 2),
                     (0, 0, 0), -1)
        
        cv2.putText(out, label, (label_x, label_y),
                    font, font_scale, (255, 255, 0), thickness, cv2.LINE_AA)  # Cyan text

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    return out, mask_bgr

def main():
    # --- inputs
    import os
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.join(script_dir, "..")
    image_path = os.path.join(parent_dir, "frame2.png")
    model_path = os.path.join(parent_dir, "weights.pt")
    boat_color = "black"  # Changed to black for frame2
    target_shape = "cross"  # Changed to cross for frame2

    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {image_path}\nCurrent working directory: {os.getcwd()}\nScript directory: {script_dir}")

    model = YOLO(model_path)
    class_names = model.names if hasattr(model, "names") else None

    # Use black blob detection for black boats
    # Increased min_area for black boats to filter out small noise (e.g., 561 pixels is too small)
    if boat_color == "black":
        blobs, color_mask = find_black_blobs(img, min_area=2000)  # Increased from 300 to filter small blobs
        mask_filename = "black_mask.png"
    else:
        blobs, color_mask = find_yellow_blobs(img, min_area=300)
        mask_filename = "yellow_mask.png"

    rois = []
    for b in blobs[:10]:  # limit how many you process
        roi = roi_above_blob(img.shape, b, above_scale=3.0, pad=50)  # Increased scale and pad for better coverage
        if roi:
            rois.append(roi)
    
    # Run YOLO on full image (works better than cropped ROIs for low-confidence detections)
    # Use imgsz=1280 for better cross detection (increases confidence from 21% to 82%)
    print("\nRunning YOLO on full image (using imgsz=1280 for better cross detection)...")
    full_image_results = model.predict(img, conf=0.001, imgsz=1280, verbose=False)  # Higher resolution for better detection
    full_image_dets = []
    for r in full_image_results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            xyxy = box.xyxy.cpu().numpy().reshape(-1)
            cls = int(box.cls.cpu().numpy().item())
            score = float(box.conf.cpu().numpy().item())
            full_image_dets.append({
                "bbox": (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])),
                "cls": cls,
                "conf": score,
                "roi": None
            })
    print(f"Full image detections: {len(full_image_dets)}")
    for det in full_image_dets:
        cls = det["cls"]
        class_name = f"Class {cls}"
        if class_names and 0 <= cls < len(class_names):
            class_name = class_names[cls]
        print(f"  - {class_name} (ID: {cls}) conf={det['conf']:.3f}")
    print()

    # Use full image detections and filter by ROI location
    dets = []
    for det in full_image_dets:
        # Check if detection center falls within any ROI
        x1, y1, x2, y2 = det["bbox"]
        det_center_x = (x1 + x2) // 2
        det_center_y = (y1 + y2) // 2
        
        matched_roi = None
        for roi in rois:
            rx1, ry1, rx2, ry2 = roi
            if rx1 <= det_center_x <= rx2 and ry1 <= det_center_y <= ry2:
                matched_roi = roi
                break
        
        # Include detection if it's in an ROI or if we want to see all detections
        if matched_roi:
            det["roi"] = matched_roi
            dets.append(det)

    # Print all detections before filtering
    print(f"\nALL YOLO DETECTIONS (in ROIs): {len(dets)}")
    for i, det in enumerate(dets):
        cls = det["cls"]
        conf = det["conf"]
        class_name = f"Class {cls}"
        if class_names and 0 <= cls < len(class_names):
            class_name = class_names[cls]
        print(f"  Detection {i+1}: {class_name} (ID: {cls}) conf={conf:.3f}")
    print()

    # Filter detections to only include the target shape
    if target_shape == "cross":
        # For black boats: black_cross is class_id 2
        # For other boats: could be blue_cross (6), green_cross (12), red_cross (17)
        if boat_color == "black":
            # Black cross is class_id 2
            dets_filtered = [d for d in dets if d["cls"] == 2]
            print(f"Filtered to BLACK_CROSS (class_id 2): {len(dets_filtered)} detections")
        else:
            # For yellow boats, could be any cross - check all
            cross_ids = [2, 6, 12, 17]  # black, blue, green, red crosses
            dets_filtered = [d for d in dets if d["cls"] in cross_ids]
            print(f"Filtered to CROSS (class_ids {cross_ids}): {len(dets_filtered)} detections")
        dets = dets_filtered
    elif target_shape == "triangle":
        # For black boats: black_triangle is class_id 3
        # For yellow boats: could be any triangle
        if boat_color == "black":
            dets_filtered = [d for d in dets if d["cls"] == 3]  # black_triangle
            print(f"Filtered to BLACK_TRIANGLE (class_id 3): {len(dets_filtered)} detections")
        else:
            triangle_ids = [3, 8, 14]  # black, blue, green triangles
            dets_filtered = [d for d in dets if d["cls"] in triangle_ids]
            print(f"Filtered to TRIANGLE (class_ids {triangle_ids}): {len(dets_filtered)} detections")
        dets = dets_filtered

    debug_img, debug_mask = draw_debug(img, blobs, color_mask, rois, dets, class_names, boat_color)

    # Save outputs in the same directory as the script
    output_debug = os.path.join(script_dir, "debug_out_frame2.png")
    output_mask = os.path.join(script_dir, mask_filename)
    cv2.imwrite(output_debug, debug_img)
    cv2.imwrite(output_mask, debug_mask)
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"PROCESSING: {image_path}")
    print(f"BOAT COLOR: {boat_color.upper()}")
    print(f"TARGET SHAPE: {target_shape.upper()}")
    print(f"{'='*60}")
    print(f"SUMMARY: blobs={len(blobs)} rois={len(rois)} detections={len(dets)}")
    print(f"{'='*60}\n")
    
    # Print detailed blob information
    print(f"{boat_color.upper()} BLOBS DETECTED: {len(blobs)}")
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
    print(f"YOLO DETECTIONS ({target_shape.upper()}): {len(dets)}")
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
