# Task 2 / 3 — Edge Detection & Diamond Detection

Two scripts:

1. **`edge_detection.py`** — Edge detection only: input image → blur + Canny → show/save edge image.
2. **`diamond_detector.py`** — Full pipeline: input image → edge detection + shape/diamond detection → output image with detections drawn.

---

## 1. Edge detection only

Takes an input image and produces the edge map (no shape classification).

```bash
python edge_detection.py path/to/image.jpg
python edge_detection.py image.png --out edges.png
python edge_detection.py image.png --no-show --out edges.png
```

- **Input:** image path (or first image in this folder if omitted).
- **Output:** shows the edge image in a window; use `--out` to save.
- **Options:** `--canny-low`, `--canny-high`; `--no-show` to skip the window.

---

## 2. Edge detection + shape/diamond detection

Takes an input image and produces an output image with shapes (or diamonds only) drawn.

```bash
# Diamonds only (default)
python diamond_detector.py image.png --out result.png

# All shapes (triangle, quad, diamond, etc.)
python diamond_detector.py image.png --all --out result.png
```

- **Input:** image path (or first image in folder if omitted).
- **Output:** window with annotated image; use `--out` to save. Green = diamond, gray = other shapes.
- **Options:** `--all`, `--min-area`, `--canny-low`, `--canny-high`, `--no-show`.

The diamond step uses the same edge-detection logic as `edge_detection.py`, then contours → polygon approx → shape classification (diamond = 4-sided with suitable geometry).

---

## Use as a module

```python
# Edge image only
from edge_detection import run_edge_detection
edges = run_edge_detection(bgr_image)

# Full pipeline: shapes/diamonds
from diamond_detector import detect_diamonds, detect_shapes, draw_detections
diamonds = detect_diamonds(bgr_image)
vis = draw_detections(bgr_image, diamonds)
```

Each detection dict has: `shape`, `bbox`, `center`, `area`, `contour`.
