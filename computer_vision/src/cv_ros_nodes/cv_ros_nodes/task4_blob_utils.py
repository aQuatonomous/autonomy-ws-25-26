"""
Blob detection and ROI utilities for Task 4 supply drop.
Used by task4_supply_processor to find yellow/black vessels and ROIs above them.
"""

import cv2
import numpy as np


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
    lower = np.array([0, 0, 0])        # H, S, V - very dark (pure black)
    upper = np.array([180, 255, 100])  # Allow all hues, all saturations, low-medium brightness

    mask = cv2.inRange(hsv, lower, upper)

    # LAB color space for better dark color detection
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l_channel = lab[:, :, 0]  # L channel (lightness)
    lab_mask = (l_channel < 50).astype(np.uint8) * 255
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

    blobs.sort(key=lambda t: t[4], reverse=True)
    return blobs, mask


# -----------------------------------
# 2) ROI above each blob
# -----------------------------------
def roi_above_blob(img_shape, blob_xywh, above_scale=2.0, pad=20):
    """
    Compute ROI above a blob where we expect to find the shape (triangle/cross).
    Returns (x1, y1, x2, y2) or None if the ROI would collapse.
    """
    H, W = img_shape[:2]
    x, y, w, h, _ = blob_xywh

    roi_h = int(h * above_scale)

    x1 = max(0, x - pad)
    x2 = min(W, x + w + pad)

    y2 = max(0, y + int(0.2 * h))          # slightly into blob area
    y1 = max(0, y - roi_h - pad)           # above blob

    # Safety: if blob is near top, ROI can collapse
    if y2 <= y1 or x2 <= x1:
        return None

    return (x1, y1, x2, y2)
