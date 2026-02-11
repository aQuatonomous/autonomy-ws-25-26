"""
Diamond Detection Debug Pipeline

For each input image, this script saves intermediate images for every major
stage of the diamond detection:

  0) input.png                 – original BGR image
  1) gray.png                  – grayscale
  2) edges_raw.png             – Canny edges (raw)
  2) edges_processed.png        – Canny edges (after morphological close)
  3) contours.png               – all contours overlay
  4) diamonds_stage1_shape.png  – quads classified as diamonds (no black/filter/conf)
  5) diamonds_stage2_black.png  – diamonds after black/dark filter
  6) diamonds_stage3_conf.png  – final diamonds after confidence threshold
  7) colour_indicator_buoy.png  – full pipeline: diamonds + indicator ROI + colour label

Outputs are written under:
  debug_outputs/<image_basename>/<stage>.png
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from colour_indicator_buoy_detector import (
    run_edge_detection,
    CANNY_LOW,
    CANNY_HIGH,
    detect_shapes,
    detect_diamonds,
    MAX_BLACK_BRIGHTNESS,
    DEFAULT_CONF_THRESHOLD,
    MIN_CONTOUR_AREA,
    EPS_RATIO,
    classify_colour_indicator_buoy,
)


def save_image(path: Path, img):
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def debug_one_image(img_path: Path, out_root: Path):
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"[WARN] Failed to load {img_path}")
        return

    base = img_path.stem
    out_dir = out_root / base
    out_dir.mkdir(parents=True, exist_ok=True)

    # 0) Original
    save_image(out_dir / "0_input.png", img)

    # 1) Grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    save_image(out_dir / "1_gray.png", gray)

    # 2) Edges (raw + processed)
    edges_raw = run_edge_detection(img, canny_low=CANNY_LOW, canny_high=CANNY_HIGH)
    save_image(out_dir / "2_edges_raw.png", edges_raw)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    edges = cv2.morphologyEx(edges_raw, cv2.MORPH_CLOSE, kernel, iterations=1)
    save_image(out_dir / "2_edges_processed.png", edges)

    # 3) All contours (from processed edges, with RETR_TREE)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    vis_contours = img.copy()
    cv2.drawContours(vis_contours, contours, -1, (0, 255, 255), 2)
    save_image(out_dir / "3_contours.png", vis_contours)

    # 4) Stage 1 – diamonds by shape only (no black, no confidence)
    stage1 = detect_shapes(
        img,
        canny_low=CANNY_LOW,
        canny_high=CANNY_HIGH,
        min_area=MIN_CONTOUR_AREA,
        eps_ratio=EPS_RATIO,
        filter_diamond_only=True,
        filter_black_only=False,
        max_black_brightness=MAX_BLACK_BRIGHTNESS,
        conf_threshold=0.0,
    )
    vis_stage1 = img.copy()
    for d in stage1:
        x, y, w, h = d["bbox"]
        cv2.rectangle(vis_stage1, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(
            vis_stage1,
            "shape",
            (x, max(10, y - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    save_image(out_dir / "4_diamonds_stage1_shape.png", vis_stage1)

    # 5) Stage 2 – diamonds after black filter (but before conf)
    stage2 = detect_shapes(
        img,
        canny_low=CANNY_LOW,
        canny_high=CANNY_HIGH,
        min_area=MIN_CONTOUR_AREA,
        eps_ratio=EPS_RATIO,
        filter_diamond_only=True,
        filter_black_only=True,
        max_black_brightness=MAX_BLACK_BRIGHTNESS,
        conf_threshold=0.0,
    )
    vis_stage2 = img.copy()
    for d in stage2:
        x, y, w, h = d["bbox"]
        cv2.rectangle(vis_stage2, (x, y), (x + w, y + h), (0, 255, 0), 2)
        txt = f"black {d.get('mean_brightness', 0):.1f}"
        cv2.putText(
            vis_stage2,
            txt,
            (x, max(10, y - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    save_image(out_dir / "5_diamonds_stage2_black.png", vis_stage2)

    # 6) Stage 3 – final diamonds after confidence threshold (what detector uses)
    stage3 = detect_diamonds(
        img,
        black_only=True,
        max_black_brightness=MAX_BLACK_BRIGHTNESS,
        conf_threshold=DEFAULT_CONF_THRESHOLD,
    )
    vis_stage3 = img.copy()
    for d in stage3:
        x, y, w, h = d["bbox"]
        conf = float(d.get("confidence", 0.0))
        cv2.rectangle(vis_stage3, (x, y), (x + w, y + h), (0, 0, 255), 2)
        txt = f"{conf:.2f}"
        cv2.putText(
            vis_stage3,
            txt,
            (x, max(10, y - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
    save_image(out_dir / "6_diamonds_stage3_conf.png", vis_stage3)

    # 7) Stage 4 – Full colour indicator buoy pipeline (diamonds + ROI + indicator)
    vis_stage4, info = classify_colour_indicator_buoy(
        img,
        conf_threshold=DEFAULT_CONF_THRESHOLD,
        max_black_brightness=MAX_BLACK_BRIGHTNESS,
        roi_conf_threshold=0.6,
    )
    # Save detailed debug view inside this image's debug folder
    save_image(out_dir / "7_colour_indicator_buoy.png", vis_stage4)

    # Also save a clean final output (only final detections) into a top-level folder.
    final_root = out_root.parent / "final_outputs"
    final_path = final_root / f"{base}_colour_buoy.png"
    save_image(final_path, vis_stage4)

    print(
        f"[INFO] Debug stages saved under {out_dir} "
        f"(stage1={len(stage1)}, stage2={len(stage2)}, stage3={len(stage3)}, "
        f"indicator={info['indicator_state']}); "
        f"final saved to {final_path}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Save intermediate images for diamond detection stages."
    )
    parser.add_argument(
        "image",
        nargs="*",
        help="Image files or directories. If omitted, uses input_images/*.png",
    )
    parser.add_argument(
        "--out-dir",
        default="debug_outputs",
        help="Root directory for debug stage outputs.",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    out_root = (script_dir / args.out_dir).resolve()

    targets = []
    if args.image:
        for p in args.image:
            path = Path(p)
            if not path.is_absolute():
                path = script_dir / path
            if path.is_dir():
                targets.extend(sorted(path.glob("*.png")))
                targets.extend(sorted(path.glob("*.jpg")))
                targets.extend(sorted(path.glob("*.jpeg")))
            elif path.is_file():
                targets.append(path)
    else:
        # Default: all PNGs in input_images
        input_dir = script_dir / "input_images"
        targets = sorted(input_dir.glob("*.png"))

    if not targets:
        print("[WARN] No input images found.")
        return 1

    print(f"[INFO] Running debug pipeline on {len(targets)} images...")
    for img_path in targets:
        debug_one_image(img_path, out_root)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

