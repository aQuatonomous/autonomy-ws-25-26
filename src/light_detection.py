# light_detector.py
 
import cv2
import numpy as np
from collections import deque
 
 
# Rolling window for temporal smoothing
state_history = deque(maxlen=5)
 
 
def detect_light(roi: np.ndarray):
    """
    Detects red/green/none from the cropped top region of a buoy.
    Input: roi = BGR numpy image
    Output: (state_string, confidence_float)
    """
 
    # Handle empty region
    if roi is None or roi.size == 0:
        state_history.append("none")
        return "none", 0.0
 
    
    # STEP 1 — BRIGHTNESS FILTER
  
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    avg_bright = gray.mean()
    bright_mask = gray > avg_bright * 1.25   # brighter than buoy body
 
   # if the top 25 is greater than a threshold, consider it none
    # STEP 2 — COLOR MASKING
    
    B, G, R = cv2.split(roi)
 
    red_mask   = (R > 180) & (G < 100) & (B < 100)
    green_mask = (G > 180) & (R < 100) & (B < 100)
 
    combined_mask = (red_mask | green_mask) & bright_mask
 
    
    # STEP 3 — RE-BOX THE LIGHT REGION
    
    ys, xs = np.where(combined_mask)
 
    if len(xs) > 0:
        x1, x2 = xs.min(), xs.max()
        y1, y2 = ys.min(), ys.max()
        roi_refined = roi[y1:y2, x1:x2]
    else:
        roi_refined = roi  # fallback
 
    
    # STEP 4 — CLASSIFICATION
    
    r_votes = red_mask.sum()
    g_votes = green_mask.sum()
 
    conf_red = r_votes / (r_votes + g_votes + 1e-6) # avoid div by zero
    conf_green = g_votes / (g_votes + r_votes + 1e-6)
    conf_none = 1 - max(conf_red, conf_green)
 
    
    # STEP 5 — CREATE FINAL DECISION
    
    if max(conf_red, conf_green) < 0.4:
        state = "none"
        conf = conf_none
    else:
        if conf_red > conf_green:
            state = "red"
            conf = conf_red
        else:
            state = "green"
            conf = conf_green
 
    
    # STEP 6 — TEMPORAL SMOOTHING
    
    state_history.append(state)
    final_state = max(set(state_history), key=state_history.count)
 
    return final_state, conf