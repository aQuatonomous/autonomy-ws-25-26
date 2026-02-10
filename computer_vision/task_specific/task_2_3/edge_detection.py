"""
Edge detection only: take an input image, run blur + Canny, show and/or save the edge image.
"""

import argparse
import cv2
import numpy as np
from pathlib import Path

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


def main():
    ap = argparse.ArgumentParser(description="Run edge detection on an image; show and/or save result.")
    ap.add_argument("image", nargs="?", help="Input image path")
    ap.add_argument("--out", default="", help="Save edge image to this path")
    ap.add_argument("--no-show", action="store_true", help="Do not open result window")
    ap.add_argument("--canny-low", type=int, default=CANNY_LOW)
    ap.add_argument("--canny-high", type=int, default=CANNY_HIGH)
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    if args.image:
        image_path = Path(args.image)
        if not image_path.is_absolute():
            image_path = script_dir / image_path
    else:
        candidates = []
        for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
            candidates.extend(script_dir.glob(ext))
        if not candidates:
            print("No image path given and no image in script directory.")
            print("Usage: python edge_detection.py <image> [--out edges.png]")
            return 1
        image_path = sorted(candidates)[0]
        print(f"Using: {image_path.name}")

    img = cv2.imread(str(image_path))
    if img is None:
        print(f"Failed to load image: {image_path}")
        return 1

    edges = run_edge_detection(img, canny_low=args.canny_low, canny_high=args.canny_high)
    if edges.size == 0:
        return 1

    if args.out:
        out_path = Path(args.out)
        if not out_path.is_absolute():
            out_path = script_dir / out_path
        cv2.imwrite(str(out_path), edges)
        print(f"Saved: {out_path}")

    if not args.no_show:
        cv2.imshow("Edges", edges)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    exit(main())
