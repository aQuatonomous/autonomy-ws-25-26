"""
Full pipeline: edge detection + shape/diamond detection.

Takes an input image and produces an output image with shapes (diamonds) drawn.
Uses the same edge-detection step as edge_detection.py, then contours + polygon
approx + shape classification. For Task 3 Color Indicator Buoy ("White with black diamonds").
"""

import argparse
import cv2
import numpy as np
from pathlib import Path

from edge_detection import run_edge_detection, CANNY_LOW, CANNY_HIGH

# Minimum contour area (avoid noise)
MIN_CONTOUR_AREA = 200
# Approx polygon: epsilon = eps_ratio * perimeter
EPS_RATIO = 0.04


def _angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
    """Angle in degrees between two 2D vectors."""
    a = np.arctan2(v1[1], v1[0])
    b = np.arctan2(v2[1], v2[0])
    deg = np.degrees(np.abs(a - b))
    return min(deg, 360 - deg)


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
    # For a diamond (rotated square): 4 equal sides.
    # For a kite: two pairs of equal sides. Allow some tolerance.
    mean_side = np.mean(sides)
    if mean_side < 1e-6:
        return False
    ratios = sides / mean_side
    if np.any(ratios < 0.3) or np.any(ratios > 2.0):
        return False
    # Optional: check that opposite angles are roughly equal (parallelogram-like)
    # For a diamond we expect two acute and two obtuse, or all 90 for a square.
    return True


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
) -> list[dict]:
    """
    Run edge-based shape detection on a BGR image.

    Returns list of dicts:
      - "shape": str (e.g. "diamond", "triangle", "quad")
      - "bbox": (x, y, w, h)
      - "center": (cx, cy)
      - "area": float
      - "contour": np.ndarray (optional, for drawing)
    """
    if img is None or img.size == 0:
        return []

    edges = run_edge_detection(img, canny_low=canny_low, canny_high=canny_high)
    if edges.size == 0:
        return []
    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    results = []
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

        x, y, w, h = cv2.boundingRect(c)
        M = cv2.moments(c)
        cx = int(M["m10"] / M["m00"]) if M["m00"] else x + w // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] else y + h // 2

        results.append({
            "shape": shape,
            "bbox": (x, y, w, h),
            "center": (cx, cy),
            "area": area,
            "contour": approx,
        })

    return results


def detect_diamonds(img: np.ndarray, **kwargs) -> list[dict]:
    """Convenience: return only diamond detections."""
    return detect_shapes(img, filter_diamond_only=True, **kwargs)


def draw_detections(img: np.ndarray, detections: list[dict], labels: bool = True) -> np.ndarray:
    """Draw bounding boxes and shape labels on a copy of the image."""
    out = img.copy()
    for d in detections:
        x, y, w, h = d["bbox"]
        color = (0, 255, 0) if d["shape"] == "diamond" else (200, 200, 200)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        if d.get("contour") is not None:
            cv2.drawContours(out, [d["contour"]], -1, color, 2)
        if labels:
            label = d["shape"]
            cv2.putText(
                out, label, (x, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA
            )
    return out


def main():
    ap = argparse.ArgumentParser(description="Edge-based shape detection; find diamonds (e.g. Task 3 logo).")
    ap.add_argument("image", nargs="?", help="Path to image (optional if one image in folder)")
    ap.add_argument("--all", action="store_true", help="Show all shapes; default is diamonds only")
    ap.add_argument("--no-show", action="store_true", help="Do not open result window")
    ap.add_argument("--out", default="", help="Save result image to this path")
    ap.add_argument("--min-area", type=float, default=MIN_CONTOUR_AREA, help="Min contour area")
    ap.add_argument("--canny-low", type=int, default=CANNY_LOW)
    ap.add_argument("--canny-high", type=int, default=CANNY_HIGH)
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    if args.image:
        image_path = Path(args.image)
        if not image_path.is_absolute():
            image_path = script_dir / image_path
    else:
        exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
        candidates = []
        for ext in exts:
            candidates.extend(script_dir.glob(ext))
        if not candidates:
            print("No image path given and no image found in script directory.")
            print("Usage: python diamond_detector.py [image_path] [--all] [--out result.png]")
            return 1
        image_path = sorted(candidates)[0]
        print(f"Using: {image_path.name}")

    img = cv2.imread(str(image_path))
    if img is None:
        print(f"Failed to load image: {image_path}")
        return 1

    if args.all:
        detections = detect_shapes(img, min_area=args.min_area, canny_low=args.canny_low, canny_high=args.canny_high)
    else:
        detections = detect_diamonds(img, min_area=args.min_area, canny_low=args.canny_low, canny_high=args.canny_high)

    print(f"Detected {len(detections)} diamond(s)" if not args.all else f"Detected {len(detections)} shape(s)")
    for i, d in enumerate(detections):
        print(f"  [{i+1}] {d['shape']} at {d['center']} bbox={d['bbox']} area={d['area']:.0f}")

    vis = draw_detections(img, detections)
    if args.out:
        out_path = Path(args.out)
        if not out_path.is_absolute():
            out_path = script_dir / out_path
        cv2.imwrite(str(out_path), vis)
        print(f"Saved: {out_path}")

    if not args.no_show:
        cv2.imshow("Shapes", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    exit(main())
