#!/usr/bin/env python3
"""
Blob + shape boat detector: combines color blobs with YOLO shape inference
to detect and label boats with a single bounding box around both:

  - Water delivery boat: yellow blob + triangle on top
  - Ball delivery boat:   black blob + cross on top

The final bounding box is the union of the blob and the shape (marker) on top.
"""

import argparse
import cv2
import numpy as np
import os
from ultralytics import YOLO


# ----- Class IDs from weights.pt -----
TRIANGLE_IDS = [3, 8, 14]   # black, blue, green triangle
CROSS_IDS = [2, 6, 12, 17]  # black, blue, green, red cross


# ---------------------------------------------------------------------------
# 1) Yellow blob detection
# ---------------------------------------------------------------------------
def find_yellow_blobs(bgr, min_area=300):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([20, 120, 120])
    upper = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
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
    blobs.sort(key=lambda t: t[4], reverse=True)
    return blobs, mask


# ---------------------------------------------------------------------------
# 2) Black blob detection
# ---------------------------------------------------------------------------
def find_black_blobs(bgr, min_area=300):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 0, 0])
    upper = np.array([180, 255, 100])
    mask = cv2.inRange(hsv, lower, upper)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lab_mask = (lab[:, :, 0] < 50).astype(np.uint8) * 255
    mask = cv2.bitwise_or(mask, lab_mask)
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
    blobs.sort(key=lambda t: t[4], reverse=True)
    return blobs, mask


# ---------------------------------------------------------------------------
# 3) YOLO inference for triangles and crosses
# ---------------------------------------------------------------------------
def run_yolo_shapes(bgr, model, conf=0.15, imgsz=1280):
    results = model.predict(bgr, conf=conf, imgsz=imgsz, verbose=False)
    triangles = []
    crosses = []
    for r in results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            cls = int(box.cls.cpu().numpy().item())
            score = float(box.conf.cpu().numpy().item())
            xyxy = box.xyxy.cpu().numpy().reshape(-1)
            bbox = (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]))
            if cls in TRIANGLE_IDS:
                triangles.append({"bbox": bbox, "cls": cls, "conf": score})
            elif cls in CROSS_IDS:
                crosses.append({"bbox": bbox, "cls": cls, "conf": score})
    return triangles, crosses


# ---------------------------------------------------------------------------
# 4) Check if a shape is "on top" of a blob
# ---------------------------------------------------------------------------
def shape_on_top_of_blob(shape_bbox, blob_xywh, x_pad_frac=0.3, max_above_frac=2.5):
    """
    shape_bbox: (x1, y1, x2, y2)
    blob_xywh:  (x, y, w, h, area)

    Returns True if the shape is plausibly on top of the blob (e.g. marker on deck).
    """
    sx1, sy1, sx2, sy2 = shape_bbox
    bx, by, bw, bh, _ = blob_xywh
    blob_x1, blob_y1 = bx, by
    blob_x2, blob_y2 = bx + bw, by + bh
    blob_cx = bx + bw // 2
    blob_cy = by + bh // 2
    shape_cx = (sx1 + sx2) // 2
    shape_cy = (sy1 + sy2) // 2

    # 1) Horizontal overlap: shape's x-range overlaps blob's x-range (with padding)
    pad = int(bw * x_pad_frac)
    if (sx2 < blob_x1 - pad) or (sx1 > blob_x2 + pad):
        return False

    # 2) Shape is in the upper half relative to blob (shape center above blob center)
    if shape_cy >= blob_cy:
        return False

    # 3) Shape is not way above the blob (within max_above_frac * blob height)
    if sy2 < by - int(bh * max_above_frac):
        return False

    return True


# ---------------------------------------------------------------------------
# 5) Merge blob bbox and shape bbox into one combined box
# ---------------------------------------------------------------------------
def merge_bbox(blob_xywh, shape_bbox):
    """Return (x1, y1, x2, y2) encompassing both blob and shape."""
    bx, by, bw, bh, _ = blob_xywh
    blob_x1, blob_y1 = bx, by
    blob_x2, blob_y2 = bx + bw, by + bh
    sx1, sy1, sx2, sy2 = shape_bbox
    x1 = min(blob_x1, sx1)
    y1 = min(blob_y1, sy1)
    x2 = max(blob_x2, sx2)
    y2 = max(blob_y2, sy2)
    return (x1, y1, x2, y2)


# ---------------------------------------------------------------------------
# 6) Match blobs to shapes and build boat detections
# ---------------------------------------------------------------------------
def match_blob_to_shape(blobs, shapes, on_top_fn):
    """
    blobs: list of (x, y, w, h, area)
    shapes: list of {"bbox": (x1,y1,x2,y2), "cls", "conf"}
    on_top_fn(shape_bbox, blob_xywh) -> bool

    Returns list of (blob_xywh, shape_dict, combined_bbox).
    Each shape used at most once (best blob match); each blob at most one shape.
    """
    used_shapes = set()
    used_blobs = set()
    matches = []

    # Sort shapes by confidence (use best shapes first)
    shapes_sorted = sorted(shapes, key=lambda s: s["conf"], reverse=True)

    for shp in shapes_sorted:
        if id(shp) in used_shapes:
            continue
        best_blob = None
        best_blob_idx = -1
        for i, blob in enumerate(blobs):
            if i in used_blobs:
                continue
            if not on_top_fn(shp["bbox"], blob):
                continue
            best_blob = blob
            best_blob_idx = i
            break

        if best_blob is not None:
            used_blobs.add(best_blob_idx)
            used_shapes.add(id(shp))
            combined = merge_bbox(best_blob, shp["bbox"])
            matches.append((best_blob, shp, combined))

    return matches


# ---------------------------------------------------------------------------
# 7) Main detection: water boats (yellow+triangle) and ball boats (black+cross)
# ---------------------------------------------------------------------------
def detect_boats(bgr, model, conf=0.15, imgsz=1280,
                 yellow_min_area=300, black_min_area=2000):
    """
    Returns:
      water_boats: list of {"label": "water_delivery_boat", "bbox": (x1,y1,x2,y2),
                         "blob": (x,y,w,h,area), "shape": shape_dict, "conf": float}
      ball_boats:  list of {"label": "ball_delivery_boat", ...}
    """
    yellow_blobs, _ = find_yellow_blobs(bgr, min_area=yellow_min_area)
    black_blobs, _ = find_black_blobs(bgr, min_area=black_min_area)
    triangles, crosses = run_yolo_shapes(bgr, model, conf=conf, imgsz=imgsz)

    water_boats = []
    ball_boats = []

    for blob, shp, combined in match_blob_to_shape(
            yellow_blobs, triangles, shape_on_top_of_blob):
        water_boats.append({
            "label": "water_delivery_boat",
            "bbox": combined,
            "blob": blob,
            "shape": shp,
            "conf": shp["conf"],
        })

    for blob, shp, combined in match_blob_to_shape(
            black_blobs, crosses, shape_on_top_of_blob):
        ball_boats.append({
            "label": "ball_delivery_boat",
            "bbox": combined,
            "blob": blob,
            "shape": shp,
            "conf": shp["conf"],
        })

    return water_boats, ball_boats


# ---------------------------------------------------------------------------
# 8) Draw results
# ---------------------------------------------------------------------------
def draw_boats(bgr, water_boats, ball_boats, class_names=None):
    out = bgr.copy()
    # Water: cyan box
    for b in water_boats:
        x1, y1, x2, y2 = b["bbox"]
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 255, 0), 3)
        label = f"water_delivery_boat {b['conf']:.2f}"
        _draw_label(out, label, x1, y1, (255, 255, 0))
    # Ball: magenta box
    for b in ball_boats:
        x1, y1, x2, y2 = b["bbox"]
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 0, 255), 3)
        label = f"ball_delivery_boat {b['conf']:.2f}"
        _draw_label(out, label, x1, y1, (255, 0, 255))
    return out


def _draw_label(img, text, x, y, color, font_scale=0.6, thickness=2):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    ly = max(th + 5, y - 5)
    cv2.rectangle(img, (x - 2, ly - th - 2), (x + tw + 2, ly + baseline + 2), (0, 0, 0), -1)
    cv2.putText(img, text, (x, ly), font, font_scale, color, thickness, cv2.LINE_AA)


# ---------------------------------------------------------------------------
# 9) CLI and main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Detect water (yellow+triangle) and ball (black+cross) boats with combined bbox.")
    parser.add_argument("--image", "-i", type=str, default=None,
                        help="Input image (default: frame.png then frame2.png)")
    parser.add_argument("--model", "-m", type=str, default=None,
                        help="YOLO weights (default: weights.pt in script dir)")
    parser.add_argument("--conf", type=float, default=0.15,
                        help="YOLO confidence threshold (default: 0.15)")
    parser.add_argument("--imgsz", type=int, default=1280,
                        help="YOLO inference size (default: 1280)")
    parser.add_argument("--out", "-o", type=str, default=None,
                        help="Output debug image path")
    parser.add_argument("--yellow-min-area", type=int, default=300)
    parser.add_argument("--black-min-area", type=int, default=2000)
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.join(script_dir, "..")
    model_path = args.model or os.path.join(parent_dir, "weights.pt")

    if args.image:
        image_path = os.path.abspath(args.image)
    else:
        # Try frame.png then frame2.png (in task4 parent)
        for name in ("frame.png", "frame2.png"):
            p = os.path.join(parent_dir, name)
            if os.path.isfile(p):
                image_path = p
                break
        else:
            raise FileNotFoundError("No --image and neither frame.png nor frame2.png found in parent task4/")

    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Could not load: {image_path}")

    model = YOLO(model_path)
    class_names = getattr(model, "names", None)

    water_boats, ball_boats = detect_boats(
        img, model,
        conf=args.conf, imgsz=args.imgsz,
        yellow_min_area=args.yellow_min_area,
        black_min_area=args.black_min_area,
    )

    out_img = draw_boats(img, water_boats, ball_boats, class_names)

    out_path = args.out
    if not out_path:
        base = os.path.splitext(os.path.basename(image_path))[0]
        out_path = os.path.join(script_dir, f"boats_out_{base}.png")
    cv2.imwrite(out_path, out_img)
    print(f"Saved: {out_path}")

    print("\n" + "=" * 60)
    print(f"Image: {image_path}")
    print("=" * 60)
    print(f"Water delivery boats (yellow + triangle): {len(water_boats)}")
    for i, b in enumerate(water_boats):
        print(f"  [{i+1}] bbox={b['bbox']} conf={b['conf']:.3f}")
    print(f"Ball delivery boats (black + cross): {len(ball_boats)}")
    for i, b in enumerate(ball_boats):
        print(f"  [{i+1}] bbox={b['bbox']} conf={b['conf']:.3f}")
    print("=" * 60)


if __name__ == "__main__":
    main()
