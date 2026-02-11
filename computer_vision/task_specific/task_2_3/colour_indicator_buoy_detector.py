"""
Colour Indicator Buoy Detector

Pipeline:
  1) Use the diamond detector to confidently detect the black diamonds on the
     white colour-indicator buoy body (Task 3).
  2) From the union of the diamond boxes, define a region *above* the buoy
     where the red/green indicator cylinder sits.
  3) Run the colour indicator detector on that ROI to classify RED / GREEN.
  4) Draw diamonds, the inferred indicator ROI, and the final label on output.
"""

import argparse
import sys
from pathlib import Path
from typing import Tuple, Optional

import cv2
import numpy as np

# ============================================================
# Core CV utilities: edge detection + diamond detection
# (inlined here so this file is the single source of truth for
# the entire diamond + indicator pipeline).
# ============================================================

# Edge detection parameters
BLUR_KSIZE = (5, 5)
CANNY_LOW = 50
CANNY_HIGH = 150


def run_edge_detection(
    img: np.ndarray,
    *,
    canny_low: int = CANNY_LOW,
    canny_high: int = CANNY_HIGH,
    blur_ksize: tuple = BLUR_KSIZE,
) -> np.ndarray:
    """Grayscale → Gaussian blur → Canny. Returns single-channel edge image (0–255)."""
    if img is None or img.size == 0:
        return np.array([])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, blur_ksize, 0)
    edges = cv2.Canny(blurred, canny_low, canny_high)
    return edges


# Minimum contour area (avoid noise)
MIN_CONTOUR_AREA = 200
# Approx polygon: epsilon = eps_ratio * perimeter.
# Slightly larger epsilon makes contours a bit smoother so rotated / mildly
# warped diamonds are more likely to approximate to 4 vertices.
EPS_RATIO = 0.06
# Black diamond filter: mean grayscale inside contour must be below this (0–255)
MAX_BLACK_BRIGHTNESS = 100
# Default confidence threshold for accepting diamonds (0–1).
DEFAULT_CONF_THRESHOLD = 0.3


def _is_diamond(pts: np.ndarray) -> bool:
    """
    Heuristic: 4-point contour is a diamond if it looks like a rotated square or kite.
    - 4 vertices (already guaranteed by caller).
    - Sides roughly similar in length (or two pairs), and diagonals not too skewed.
    """
    if pts is None or len(pts) != 4:
        return False
    pts = pts.reshape(4, 2).astype(np.float64)
    # Side lengths
    sides = []
    for i in range(4):
        a = pts[i]
        b = pts[(i + 1) % 4]
        sides.append(np.linalg.norm(b - a))
    sides = np.array(sides)
    # For a diamond (rotated square): 4 roughly equal sides.
    # For a kite / perspective view: allow some asymmetry.
    mean_side = np.mean(sides)
    if mean_side < 1e-6:
        return False
    ratios = sides / mean_side
    # Looser tolerance so slightly rotated / warped diamonds still pass,
    # while rejecting extremely elongated quads.
    if np.any(ratios < 0.2) or np.any(ratios > 2.5):
        return False
    return True


def _mean_brightness(gray: np.ndarray, contour: np.ndarray) -> float:
    """Mean grayscale intensity inside a contour (0–255). Returns 255.0 on failure."""
    if gray.size == 0 or contour is None:
        return 255.0
    mask = np.zeros_like(gray, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    return float(cv2.mean(gray, mask=mask)[0])


def _shape_name(pts: np.ndarray) -> str:
    """Classify contour by vertex count and return shape name."""
    n = len(pts)
    if n == 3:
        return "triangle"
    if n == 4:
        return "diamond" if _is_diamond(pts) else "quad"
    if n == 5:
        return "pentagon"
    if n == 6:
        return "hexagon"
    if n > 6:
        return "circle"
    return "unknown"


def detect_shapes(
    img: np.ndarray,
    *,
    canny_low: int = CANNY_LOW,
    canny_high: int = CANNY_HIGH,
    min_area: float = MIN_CONTOUR_AREA,
    eps_ratio: float = EPS_RATIO,
    filter_diamond_only: bool = False,
    filter_black_only: bool = False,
    max_black_brightness: float = MAX_BLACK_BRIGHTNESS,
    conf_threshold: float = 0.0,
) -> list[dict]:
    """
    Run edge-based shape detection on a BGR image.

    When filter_diamond_only and filter_black_only are True, only diamonds whose
    interior is dark (mean grayscale <= max_black_brightness) are returned.

    Returns list of dicts:
      - "shape": str (e.g. "diamond", "triangle", "quad")
      - "bbox": (x, y, w, h)
      - "center": (cx, cy)
      - "area": float
      - "contour": np.ndarray (optional, for drawing)
    """
    if img is None or img.size == 0:
        return []

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges_raw = run_edge_detection(img, canny_low=canny_low, canny_high=canny_high)
    if edges_raw.size == 0:
        return []

    # Make edges more "solid" so contours form closed loops, and retrieve all
    # contours (including inner ones like the printed diamond), not just the
    # outer hull.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    edges = cv2.morphologyEx(edges_raw, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    results: list[dict] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, eps_ratio * peri, True)
        n = len(approx)
        if n < 3:
            continue

        shape = _shape_name(approx)
        if filter_diamond_only and shape != "diamond":
            continue

        # Bounding box and center (needed for confidence and output)
        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        cx = int(M["m10"] / M["m00"]) if M["m00"] else x + w // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] else y + h // 2

        # Brightness inside the full contour (used for black filter and confidence)
        mean_brightness = _mean_brightness(gray, c)
        if filter_black_only and mean_brightness > max_black_brightness:
            continue

        # Heuristic confidence score (0–1)
        confidence = 1.0
        if shape == "diamond":
            # Side-length uniformity (how "diamond-like" the polygon is)
            pts = approx.reshape(-1, 2).astype(np.float64)
            side_lengths = []
            for i in range(len(pts)):
                a = pts[i]
                b = pts[(i + 1) % len(pts)]
                side_lengths.append(np.linalg.norm(b - a))
            side_lengths = np.array(side_lengths, dtype=np.float64)
            if side_lengths.size >= 2:
                mean_side = side_lengths.mean()
                std_side = side_lengths.std()
                if mean_side > 1e-6:
                    # 0 when std is large, 1 when all sides equal
                    shape_score = max(0.0, 1.0 - (std_side / (mean_side * 2.0)))
                else:
                    shape_score = 0.0
            else:
                shape_score = 0.0

            # Darkness score (1 = perfectly black up to threshold, 0 = bright)
            if max_black_brightness > 0:
                black_score = float(
                    np.clip(
                        (max_black_brightness - mean_brightness) / max_black_brightness,
                        0.0,
                        1.0,
                    )
                )
            else:
                black_score = 1.0

            # Background whiteness score: measure brightness outside contour within bbox
            gray_roi = gray[y:y + h, x:x + w]
            mask_full = np.zeros_like(gray, dtype=np.uint8)
            cv2.drawContours(mask_full, [c], -1, 255, -1)
            mask_roi = mask_full[y:y + h, x:x + w]
            bg_mask = cv2.bitwise_not(mask_roi)
            # Avoid division by zero if bg_mask has no pixels
            if np.count_nonzero(bg_mask) > 0:
                bg_mean = cv2.mean(gray_roi, mask=bg_mask)[0]
            else:
                bg_mean = 255.0

            # Map background brightness to [0,1]; expect white buoy ~ 200-255
            min_white = 150.0
            max_white = 255.0
            white_score = float(
                np.clip((bg_mean - min_white) / max(1.0, max_white - min_white), 0.0, 1.0)
            )

            # Final confidence = mixture of shape, blackness, and white background
            confidence = float(
                (shape_score + black_score + white_score) / 3.0
            )

            # Apply confidence threshold for diamonds if requested
            if conf_threshold > 0.0 and confidence < conf_threshold:
                continue

        results.append({
            "shape": shape,
            "bbox": (x, y, w, h),
            "center": (cx, cy),
            "area": area,
            "contour": approx,
            "confidence": confidence,
            "mean_brightness": mean_brightness,
        })

    return results


def detect_diamonds(
    img: np.ndarray,
    *,
    black_only: bool = True,
    max_black_brightness: float = MAX_BLACK_BRIGHTNESS,
    conf_threshold: float = DEFAULT_CONF_THRESHOLD,
    **kwargs,
) -> list[dict]:
    """Return only diamond detections. By default only black/dark diamonds (Task 3 logo)."""
    return detect_shapes(
        img,
        filter_diamond_only=True,
        filter_black_only=black_only,
        max_black_brightness=max_black_brightness,
        conf_threshold=conf_threshold,
        **kwargs,
    )


def draw_detections(img: np.ndarray, detections: list[dict], labels: bool = True) -> np.ndarray:
    """Draw bounding boxes and shape labels on a copy of the image."""
    out = img.copy()
    for d in detections:
        x, y, w, h = d["bbox"]
        box_color = (0, 255, 0) if d["shape"] == "diamond" else (200, 200, 200)
        cv2.rectangle(out, (x, y), (x + w, y + h), box_color, 2)
        if d.get("contour") is not None:
            cv2.drawContours(out, [d["contour"]], -1, box_color, 2)
        if labels:
            conf = float(d.get("confidence", 0.0))
            label = f"{conf:.2f}" if d["shape"] == "diamond" else d["shape"]
            text_color = (0, 0, 255)
            cv2.putText(
                out,
                label,
                (x, max(10, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                text_color,
                1,
                cv2.LINE_AA,
            )
    return out


def _load_indicator_detector():
    """
    Import detect_indicator from the task_specific/indicator folder.
    Returns the function object.
    """
    script_dir = Path(__file__).resolve().parent
    indicator_dir = script_dir.parent / "indicator"
    if str(indicator_dir) not in sys.path:
        sys.path.append(str(indicator_dir))
    try:
        from indicator_detector import detect_indicator  # type: ignore
    except ImportError as e:
        raise RuntimeError(f"Failed to import indicator_detector.py from {indicator_dir}") from e
    return detect_indicator


def _bbox_iou(b1: Tuple[int, int, int, int], b2: Tuple[int, int, int, int]) -> float:
    """Intersection-over-union of two (x, y, w, h) boxes."""
    x1, y1, w1, h1 = b1
    x2, y2, w2, h2 = b2
    xa1, ya1 = x1, y1
    xa2, ya2 = x1 + w1, y1 + h1
    xb1, yb1 = x2, y2
    xb2, yb2 = x2 + w2, y2 + h2

    inter_x1 = max(xa1, xb1)
    inter_y1 = max(ya1, yb1)
    inter_x2 = min(xa2, xb2)
    inter_y2 = min(ya2, yb2)

    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    if inter_area == 0:
        return 0.0

    area_a = w1 * h1
    area_b = w2 * h2
    union = area_a + area_b - inter_area
    if union <= 0:
        return 0.0
    return inter_area / union


def _compute_indicator_roi(
    img_shape: Tuple[int, int, int],
    diamond_boxes: list[Tuple[int, int, int, int]],
    height_factor: float = 1.0,
) -> Optional[Tuple[int, int, int, int]]:
    """
    Given a list of diamond bounding boxes (x, y, w, h), compute a single
    ROI above the buoy body where the red/green indicator should be.

    height_factor controls how tall the ROI is relative to buoy height.
    """
    if not diamond_boxes:
        return None

    img_h, img_w = img_shape[:2]
    num_diamonds = len(diamond_boxes)

    # If we only have one diamond, grow a box centered on that diamond
    # that is wider and significantly taller than its own bbox.
    if num_diamonds == 1:
        x, y, w, h = diamond_boxes[0]
        center_x = x + w // 2

        # Slightly wider than diamond bbox
        roi_w = int(w * 1.5)
        # Search column above the diamond: e.g. 2x its height
        # (scaled by height_factor).
        roi_h = int(h * 2.0 * height_factor)

        roi_x1 = max(0, center_x - roi_w // 2)
        roi_x2 = min(img_w - 1, roi_x1 + roi_w)

        roi_y2 = max(0, y)  # bottom at top of diamond bbox
        roi_y1 = max(0, roi_y2 - roi_h)
    else:
        # Two or more diamonds: use the union box, but grow it a bit wider and taller.
        x1s = [x for (x, y, w, h) in diamond_boxes]
        y1s = [y for (x, y, w, h) in diamond_boxes]
        x2s = [x + w for (x, y, w, h) in diamond_boxes]
        y2s = [y + h for (x, y, w, h) in diamond_boxes]

        buoy_x1 = max(0, min(x1s))
        buoy_x2 = min(img_w - 1, max(x2s))
        buoy_y1 = max(0, min(y1s))
        buoy_y2 = min(img_h - 1, max(y2s))
        buoy_h = max(1, buoy_y2 - buoy_y1)
        buoy_w = buoy_x2 - buoy_x1
        buoy_center_x = (buoy_x1 + buoy_x2) // 2

        # "Slightly bigger" than total left-right coverage: e.g. 1.2x width.
        roi_w = int(buoy_w * 1.2)
        roi_x1 = max(0, buoy_center_x - roi_w // 2)
        roi_x2 = min(img_w - 1, roi_x1 + roi_w)

        # Height significantly higher than buoy height above the diamonds.
        roi_h = int(buoy_h * height_factor * 1.6)
        roi_y2 = max(0, buoy_y1)  # bottom of ROI = top of diamonds
        roi_y1 = max(0, roi_y2 - roi_h)

    if roi_y1 >= roi_y2 or roi_x1 >= roi_x2:
        return None

    return roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1


def classify_colour_indicator_buoy(
    img_bgr: np.ndarray,
    *,
    conf_threshold: float = DEFAULT_CONF_THRESHOLD,
    max_black_brightness: float = MAX_BLACK_BRIGHTNESS,
    roi_conf_threshold: float = 0.6,
) -> Tuple[np.ndarray, dict]:
    """
    Full pipeline on a single BGR frame.

    Returns:
      - Annotated BGR image.
      - Info dict with:
          {
            "diamonds": [...],
            "indicator_state": "red"/"green"/"none",
            "indicator_conf": float,
            "indicator_bbox": (x, y, w, h) or None
          }
    """
    detect_indicator = _load_indicator_detector()

    # 1) Detect *high-confidence* diamonds (for visualization and final decisions)
    diamonds = detect_diamonds(
        img_bgr,
        black_only=True,
        max_black_brightness=max_black_brightness,
        conf_threshold=conf_threshold,
    )

    annotated = draw_detections(img_bgr, diamonds, labels=True)

    info = {
        "diamonds": diamonds,
        "indicator_state": "none",
        "indicator_conf": 0.0,
        "indicator_bbox": None,
    }

    # 1b) Also detect black/dark diamonds used purely for ROI computation.
    #     We typically want these to be fairly reliable, so we use a (configurable)
    #     confidence threshold, default 0.6.
    all_black_diamonds = detect_shapes(
        img_bgr,
        filter_diamond_only=True,
        filter_black_only=True,
        max_black_brightness=max_black_brightness,
        conf_threshold=roi_conf_threshold,
    )

    if not all_black_diamonds:
        return annotated, info

    # First, merge out near-duplicate detections that share almost the same box
    # (e.g. inner/outer contours of the same printed diamond). Keep the larger,
    # higher-confidence one for ROI computation.
    sorted_by_quality = sorted(
        all_black_diamonds,
        key=lambda d: (-(d.get("area", 0.0)), -(d.get("confidence", 0.0))),
    )
    unique_diamonds: list[dict] = []
    for d in sorted_by_quality:
        bbox = d["bbox"]
        if all(_bbox_iou(bbox, u["bbox"]) < 0.7 for u in unique_diamonds):
            unique_diamonds.append(d)

    # If there are more than two distinct black diamonds, prefer the biggest,
    # straightest ones (area + confidence). This should give us the two logo
    # diamonds on the buoy rather than small noisy blobs.
    roi_diamonds = unique_diamonds
    if len(unique_diamonds) > 2:
        roi_diamonds = unique_diamonds[:2]

    diamond_boxes = [d["bbox"] for d in roi_diamonds]
    roi_box = _compute_indicator_roi(img_bgr.shape, diamond_boxes)
    if roi_box is None:
        return annotated, info

    rx, ry, rw, rh = roi_box
    # Draw the search ROI for the colour indicator (for visualization)
    cv2.rectangle(
        annotated,
        (rx, ry),
        (rx + rw, ry + rh),
        (0, 165, 255),  # orange box for indicator search region
        2,
    )

    roi = img_bgr[ry : ry + rh, rx : rx + rw]

    # 2) Run colour indicator detector on ROI
    state, state_conf, bbox = detect_indicator(roi)
    info["indicator_state"] = state
    info["indicator_conf"] = float(state_conf)

    # 3) Project indicator bbox back to full image and draw
    if bbox is not None:
        bx, by, bw, bh = bbox
        gx1 = rx + bx
        gy1 = ry + by
        gx2 = gx1 + bw
        gy2 = gy1 + bh
        cv2.rectangle(
            annotated,
            (gx1, gy1),
            (gx2, gy2),
            (255, 0, 0),  # blue box for indicator
            2,
        )
        info["indicator_bbox"] = (gx1, gy1, bw, bh)

    # 4) Draw final label near top of buoy
    label = f"{state.upper()} ({state_conf:.2f})"
    color = (0, 0, 255) if state == "red" else (0, 255, 0) if state == "green" else (255, 255, 255)
    # Place text slightly above the buoy union box
    _, _, _, _ = roi_box
    text_x = max(0, rx)
    text_y = max(10, ry - 10)
    cv2.putText(
        annotated,
        label,
        (text_x, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        2,
        cv2.LINE_AA,
    )

    return annotated, info


def main():
    parser = argparse.ArgumentParser(
        description="Colour indicator buoy detector: diamonds + white buoy + red/green indicator."
    )
    parser.add_argument("image", nargs="?", help="Input image path (PNG/JPEG).")
    parser.add_argument("--out", default="", help="Output image path.")
    parser.add_argument(
        "--conf-threshold",
        type=float,
        default=DEFAULT_CONF_THRESHOLD,
        help="Minimum confidence for diamonds (0–1); default 0.7",
    )
    parser.add_argument(
        "--roi-conf-threshold",
        type=float,
        default=0.6,
        help=(
            "Minimum confidence for diamonds contributing to the indicator ROI (0–1); "
            "default 0.6"
        ),
    )
    parser.add_argument(
        "--max-black",
        type=float,
        default=100.0,
        help="Max mean brightness considered black (0–255).",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open a window; just save/print results.",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    if args.image:
        image_path = Path(args.image)
        if not image_path.is_absolute():
            image_path = script_dir / image_path
    else:
        # Autoselect first image in this folder if none provided
        candidates = []
        for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp"):
            candidates.extend(script_dir.glob(ext))
        if not candidates:
            print("No image specified and no image found in script directory.")
            return 1
        image_path = sorted(candidates)[0]
        print(f"Using image: {image_path.name}")

    img = cv2.imread(str(image_path))
    if img is None:
        print(f"Failed to load image: {image_path}")
        return 1

    annotated, info = classify_colour_indicator_buoy(
        img,
        conf_threshold=args.conf_threshold,
        max_black_brightness=args.max_black,
        roi_conf_threshold=args.roi_conf_threshold,
    )

    print(f"Diamonds detected: {len(info['diamonds'])}")
    for i, d in enumerate(info["diamonds"]):
        print(
            f"  [{i+1}] center={d['center']} bbox={d['bbox']} "
            f"area={d['area']:.0f} conf={d.get('confidence', 0.0):.3f}"
        )
    print(
        f"Indicator: state={info['indicator_state']} "
        f"conf={info['indicator_conf']:.3f} "
        f"bbox={info['indicator_bbox']}"
    )

    if args.out:
        out_path = Path(args.out)
        if not out_path.is_absolute():
            out_path = script_dir / out_path
        cv2.imwrite(str(out_path), annotated)
        print(f"Saved: {out_path}")

    if not args.no_show:
        cv2.imshow("Colour Indicator Buoy", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

