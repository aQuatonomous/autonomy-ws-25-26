# Task 2 / 3 — Diamond + Colour Indicator Buoy Detection

This folder now has **two Python scripts**:

1. **`colour_indicator_buoy_detector.py`**  
   End‑to‑end pipeline for a **single image**:
   - Canny edge detection  
   - Black diamond detection on the buoy faces  
   - ROI computation above the diamonds  
   - Red/green light classification using `indicator/indicator_detector.py`  
   - Final annotated output (diamonds, ROI, indicator bbox + label)

2. **`diamond_debug_pipeline.py`**  
   Batch **debugging pipeline** that:
   - Runs the full detector on one or many images  
   - Saves all intermediate stages (edges, contours, diamond stages)  
   - Saves final annotated outputs into a separate `final_outputs/` folder  

The folders `debug_outputs/` and `final_outputs/` are git‑ignored.

**ROS integration:** The same pipeline is used by the ROS 2 node `indicator_buoy_processor`. When the CV pipeline is launched with `enable_indicator_buoy:=true`, that node subscribes to `/camera{N}/image_preprocessed`, runs `classify_colour_indicator_buoy` on each frame, and publishes to `/camera{N}/indicator_detections`. The combiner merges these into `/combined/detection_info` with `class_name` `red_indicator_buoy` or `green_indicator_buoy`.

---

## 1. Running the main detector (`colour_indicator_buoy_detector.py`)

Basic usage on a single image:

```bash
cd computer_vision/task_specific/task_2_3

python colour_indicator_buoy_detector.py input_images/ChatGPT\ Image\ Feb\ 11,\ 2026,\ 12_03_21\ PM.png \
  --out final_outputs/example_colour_buoy.png \
  --no-show
```

- **Input:** positional `image` path (PNG/JPEG). If omitted, the script auto‑selects
  the first image in this folder.
- **Output:** annotated image shown in a window (unless `--no-show`); if `--out` is
  given, it is also written to that path.
- **Console:** prints each diamond’s center/bbox/area/confidence and the final
  indicator state + confidence + bbox.

**Key CLI parameters**

- `--conf-threshold`  
  Minimum confidence for **diamonds used for final decisions and drawing**.  
  - Default: `0.3`

- `--roi-conf-threshold`  
  Minimum confidence for **diamonds allowed to shape the ROI** above the buoy.  
  - Default: `0.6`  
  - These diamonds are filtered, deduplicated, and the best 1–2 are used to compute
    the orange ROI where the indicator is searched.

- `--max-black`  
  Maximum mean grayscale brightness (0–255) still considered “black” for diamonds.  
  - Default: `100.0`

- `--no-show`  
  Do not open an OpenCV window; useful for batch runs / SSH.

---

## 2. Running the debug pipeline (`diamond_debug_pipeline.py`)

This script is for **development and tuning**. It saves step‑by‑step visualizations.

### Run on all default input images

```bash
cd computer_vision/task_specific/task_2_3
python diamond_debug_pipeline.py
```

By default it looks for all `*.png` in `input_images/` and, for each image, writes:

- `debug_outputs/<image_stem>/0_input.png` – original BGR  
- `1_gray.png` – grayscale  
- `2_edges_raw.png` – raw Canny edges  
- `2_edges_processed.png` – edges after morphological close  
- `3_contours.png` – all contours overlay  
- `4_diamonds_stage1_shape.png` – diamonds by **shape only**  
- `5_diamonds_stage2_black.png` – diamonds after **blackness** filter  
- `6_diamonds_stage3_conf.png` – final diamonds after **confidence threshold**  
- `7_colour_indicator_buoy.png` – full pipeline (diamonds + ROI + indicator label)  

It also saves a clean final output for each image:

- `final_outputs/<image_stem>_colour_buoy.png`

### Run on a specific image or directory

```bash
# Single image
python diamond_debug_pipeline.py input_images/example.png

# All images in a directory
python diamond_debug_pipeline.py input_images/

# Custom debug output root
python diamond_debug_pipeline.py input_images/ --out-dir my_debug
```

---

## 3. Adjustable internal parameters (for development)

These are defined inside `colour_indicator_buoy_detector.py` and used by both scripts:

- **Edge detection**
  - `CANNY_LOW = 50`  
  - `CANNY_HIGH = 150`  
  - `BLUR_KSIZE = (5, 5)`

- **Contours / shapes**
  - `MIN_CONTOUR_AREA = 200` – minimum contour area to consider  
  - `EPS_RATIO = 0.06` – polygon approximation ratio for `cv2.approxPolyDP`

- **Blackness / confidence**
  - `MAX_BLACK_BRIGHTNESS = 100` – mean grayscale threshold for “black” diamonds  
  - `DEFAULT_CONF_THRESHOLD = 0.3` – default diamond confidence threshold

- **ROI rules (inside `_compute_indicator_roi`)**
  - **1 diamond**:
    - ROI width ≈ `1.5 ×` diamond width  
    - ROI height ≈ `2.0 ×` diamond height (scaled by `height_factor`)  
    - ROI sits directly **above** that diamond.
  - **2+ diamonds**:
    - Use the union of their boxes, expanded:
      - Width ≈ `1.2 ×` total diamond span (centered)  
      - Height ≈ `1.6 ×` buoy height above the diamonds.  

You can tweak these directly in `colour_indicator_buoy_detector.py` if you need to
change how strict the diamond detection or ROI sizing is during development. The
debug pipeline will immediately reflect any changes. 
