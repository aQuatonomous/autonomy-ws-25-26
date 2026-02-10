import argparse
import glob
import os
import sys
import numpy as np
import cv2

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="Folder or glob, e.g. ./calib/*.jpg")
    ap.add_argument("--pattern", default="9x6", help="Inner corners as COLSxROWS, e.g. 9x6 or 8x6")
    ap.add_argument("--square", type=float, required=True, help="Square size in meters (e.g. 0.035)")
    ap.add_argument("--show", action="store_true", help="Show detected corners while processing")
    ap.add_argument("--out", default="camera_calib.npz", help="Output file to save K and dist")
    args = ap.parse_args()

    # Parse pattern
    try:
        cols, rows = args.pattern.lower().split("x")
        cols, rows = int(cols), int(rows)
    except Exception:
        print("Invalid --pattern. Use like 9x6 (inner corners).")
        sys.exit(1)

    # Resolve images
    paths = sorted(glob.glob(args.images))
    if len(paths) == 0 and os.path.isdir(args.images):
        # If folder passed, try common extensions
        for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
            paths.extend(glob.glob(os.path.join(args.images, ext)))
        paths = sorted(paths)

    if len(paths) < 10:
        print(f"Found {len(paths)} images. You want at least 10, ideally 25+.")
        sys.exit(1)

    # Prepare object points in real units (meters)
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= args.square

    objpoints = []
    imgpoints = []

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    img_size = None
    good = 0

    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"Skipping unreadable: {p}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])  # (w, h)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not ret:
            print(f"No corners: {os.path.basename(p)}")
            continue

        # Refine to subpixel
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners2)
        good += 1

        if args.show:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, (cols, rows), corners2, ret)
            cv2.imshow("corners", vis)
            cv2.waitKey(200)

    if args.show:
        cv2.destroyAllWindows()

    if good < 10:
        print(f"Only {good} usable images. Improve lighting, sharpness, angles, or board size.")
        sys.exit(1)

    print(f"Usable images: {good}/{len(paths)}")

    # Calibrate
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None
    )

    # Compute mean reprojection error
    total_err = 0.0
    total_points = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
        total_err += err
        total_points += len(proj)

    mean_err = total_err / max(total_points, 1)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    print("\n=== Calibration Results ===")
    print("K (camera matrix):\n", K)
    print("dist (k1,k2,p1,p2,k3...):\n", dist.ravel())
    print(f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
    print(f"Mean reprojection error (px): {mean_err:.4f}")

    # Save
    np.savez(args.out, K=K, dist=dist, image_size=np.array(img_size), square_size=args.square, pattern=np.array([cols, rows]))
    print(f"\nSaved calibration to: {args.out}")

if __name__ == "__main__":
    main()
