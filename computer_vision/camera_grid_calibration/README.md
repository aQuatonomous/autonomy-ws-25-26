# Camera Grid Calibration

Calibrate a camera using a printed chessboard. Outputs camera matrix **K** and distortion coefficients (saved as `.npz`).

## Setup

```bash
pip install opencv-python numpy
```

Use a **flat** chessboard. You need the **inner corner** count (e.g. 9×6) and the **square size in meters** (e.g. 0.035 for 35 mm). Take at least 10–25 images from different angles and positions; keep the board in focus and well lit.

## Usage

```bash
python calibrate.py --images ./calib/ --square 0.035
```

**Arguments**

- `--images` — Path to calibration images: a folder (`./calib/`) or a glob (`./calib/*.jpg`).
- `--square` — Length of one chessboard square in **meters** (e.g. `0.035` for 35 mm). Used to scale 3D points.
- `--pattern` — Inner corner grid as `COLSxROWS` (e.g. `9x6`). Must match your printed board; default `9x6`.
- `--show` — Display each image with detected corners drawn (for debugging).
- `--out` — Filename for the saved calibration (default: `camera_calib.npz`).

## Output file

Saves a `.npz` with:

- **K** — 3×3 camera matrix. `K[0,0]=fx`, `K[1,1]=fy` (focal lengths in px); `K[0,2]=cx`, `K[1,2]=cy` (principal point in px).
- **dist** — Distortion: `[k1, k2, p1, p2, k3, ...]`. Radial (k1,k2,k3) and tangential (p1,p2); use with `cv2.undistort`.
- **image_size** — `(width, height)` of the calibration images.
- **square_size** — The `--square` value you passed (meters).
- **pattern** — `[cols, rows]` inner corners used.

The script also prints **K**, **dist**, and **mean reprojection error** in pixels (lower is better; &lt; 0.5 is good).
