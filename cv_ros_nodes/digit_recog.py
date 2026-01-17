"""
Digit Recognition System for Black Text on White Signs

Recognizes digits 1-3 from black text on white signs using feature-based recognition.
Designed for dock slip number recognition with robustness to varying lighting and angles.

Usage:
    import cv2
    from digit_recog import DigitRecognizer
    
    recognizer = DigitRecognizer()
    image = cv2.imread('sign.jpg')
    results = recognizer.recognize(image)
    
    for result in results:
        print(f"Digit: {result['digit']}, Confidence: {result['confidence']:.2f}")
        print(f"Position: {result['bbox']}")
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional


class DigitRecognizer:
    """
    Recognizes digits 1-3 from black text on white signs.
    
    Uses feature-based recognition with contour analysis for robustness.
    """
    
    def __init__(self, min_digit_height: int = 20, max_digit_height: int = 200,
                 min_aspect_ratio: float = 0.3, max_aspect_ratio: float = 1.5):
        """
        Initialize digit recognizer.
        
        Args:
            min_digit_height: Minimum height in pixels for a digit region
            max_digit_height: Maximum height in pixels for a digit region
            min_aspect_ratio: Minimum width/height ratio for digit regions
            max_aspect_ratio: Maximum width/height ratio for digit regions
        """
        self.min_digit_height = min_digit_height
        self.max_digit_height = max_digit_height
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
    
    def recognize(self, image: np.ndarray) -> List[Dict]:
        """
        Recognize digits 1-3 in image.
        
        Args:
            image: Input image (BGR or grayscale) as numpy array
            
        Returns:
            List of dicts with keys:
                - 'digit' (int): Recognized digit (1, 2, or 3)
                - 'confidence' (float): Confidence score (0.0-1.0)
                - 'bbox' (tuple): Bounding box (x, y, w, h)
                - 'center' (tuple): Center coordinates (x, y)
        """
        if image is None or image.size == 0:
            return []
        
        # Preprocess image
        processed = self._preprocess(image)
        
        # Find potential digit regions
        regions = self._find_digit_regions(processed)
        
        # Recognize each region
        results = []
        for region in regions:
            roi, bbox = region
            digit, confidence = self._recognize_digit(roi)
            
            if digit is not None and confidence > 0.5:  # Filter low confidence
                x, y, w, h = bbox
                center_x = x + w // 2
                center_y = y + h // 2
                
                results.append({
                    'digit': digit,
                    'confidence': confidence,
                    'bbox': bbox,
                    'center': (center_x, center_y)
                })
        
        return results
    
    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image for digit recognition.
        
        Steps:
        1. Convert to grayscale if needed
        2. Apply adaptive thresholding (handles varying lighting)
        3. Morphological operations (denoise, connect broken lines)
        
        Args:
            image: Input image (BGR or grayscale)
            
        Returns:
            Binary processed image (white digits on black background)
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # Apply adaptive thresholding (better than global threshold for varying lighting)
        # Invert so we have white text on black background
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )
        
        # Morphological operations to clean up the image
        # Close small gaps in digits
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)
        
        # Remove small noise
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)
        
        return binary
    
    def _find_digit_regions(self, binary_image: np.ndarray) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        Find potential digit regions using contour detection.
        
        Args:
            binary_image: Preprocessed binary image
            
        Returns:
            List of tuples: (roi_image, bbox) where bbox is (x, y, w, h)
        """
        regions = []
        
        # Find contours
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Filter by size
            if h < self.min_digit_height or h > self.max_digit_height:
                continue
            
            # Filter by aspect ratio
            aspect_ratio = w / h if h > 0 else 0
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue
            
            # Extract region of interest with padding
            padding = 5
            x_start = max(0, x - padding)
            y_start = max(0, y - padding)
            x_end = min(binary_image.shape[1], x + w + padding)
            y_end = min(binary_image.shape[0], y + h + padding)
            
            roi = binary_image[y_start:y_end, x_start:x_end]
            
            if roi.size > 0:
                regions.append((roi, (x, y, w, h)))
        
        return regions
    
    def _extract_features(self, roi: np.ndarray) -> Dict:
        """
        Extract geometric features from digit region for classification.
        
        Args:
            roi: Region of interest (binary image)
            
        Returns:
            Dictionary of features:
                - aspect_ratio: width/height
                - contour_count: number of separate contours
                - horizontal_lines: number of horizontal line segments
                - vertical_lines: number of vertical line segments
                - top_curve: presence of curve at top
                - bottom_curve: presence of curve at bottom
                - fill_ratio: ratio of white pixels to total pixels
        """
        h, w = roi.shape
        if h == 0 or w == 0:
            return {}
        
        # Aspect ratio
        aspect_ratio = w / h
        
        # Find contours in ROI
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_count = len(contours)
        
        # Fill ratio (white pixels / total pixels)
        fill_ratio = np.sum(roi == 255) / (h * w)
        
        # Detect horizontal and vertical lines using HoughLinesP
        horizontal_lines = 0
        vertical_lines = 0
        
        lines = cv2.HoughLinesP(roi, 1, np.pi/180, threshold=int(min(w, h) * 0.3),
                                minLineLength=int(min(w, h) * 0.4), maxLineGap=5)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx = abs(x2 - x1)
                dy = abs(y2 - y1)
                
                # Horizontal line (more horizontal than vertical)
                if dx > dy * 2:
                    horizontal_lines += 1
                # Vertical line (more vertical than horizontal)
                elif dy > dx * 2:
                    vertical_lines += 1
        
        # Detect curves at top and bottom
        # Check top and bottom thirds of the image for curved regions
        top_third = roi[:h//3, :]
        bottom_third = roi[2*h//3:, :]
        
        # Use contour approximation to detect curves
        top_curve = False
        bottom_curve = False
        
        if len(contours) > 0:
            # Find largest contour (main digit shape)
            main_contour = max(contours, key=cv2.contourArea)
            
            # Approximate contour to detect curves
            epsilon = 0.02 * cv2.arcLength(main_contour, True)
            approx = cv2.approxPolyDP(main_contour, epsilon, True)
            
            # Check if approximation has many points (indicates curves)
            if len(approx) > 6:
                # Check if top portion has curve
                top_points = [pt[0] for pt in approx if pt[0][1] < h//3]
                if len(top_points) > 2:
                    top_curve = True
                
                # Check if bottom portion has curve
                bottom_points = [pt[0] for pt in approx if pt[0][1] > 2*h//3]
                if len(bottom_points) > 2:
                    bottom_curve = True
        
        return {
            'aspect_ratio': aspect_ratio,
            'contour_count': contour_count,
            'horizontal_lines': horizontal_lines,
            'vertical_lines': vertical_lines,
            'top_curve': top_curve,
            'bottom_curve': bottom_curve,
            'fill_ratio': fill_ratio,
            'height': h,
            'width': w
        }
    
    def _recognize_digit(self, roi: np.ndarray) -> Tuple[Optional[int], float]:
        """
        Recognize a single digit (1, 2, or 3) from region of interest.
        
        Uses feature-based classification:
        - Digit 1: High aspect ratio, mostly vertical line, minimal width
        - Digit 2: Two horizontal segments, top curve, bottom curve
        - Digit 3: Two curves (top and bottom), no horizontal segments in middle
        
        Args:
            roi: Region of interest (binary image)
            
        Returns:
            Tuple of (digit, confidence) where digit is 1, 2, 3, or None
        """
        features = self._extract_features(roi)
        
        if not features:
            return None, 0.0
        
        aspect_ratio = features['aspect_ratio']
        vertical_lines = features['vertical_lines']
        horizontal_lines = features['horizontal_lines']
        top_curve = features['top_curve']
        bottom_curve = features['bottom_curve']
        fill_ratio = features['fill_ratio']
        
        # Classification logic
        scores = {
            1: 0.0,
            2: 0.0,
            3: 0.0
        }
        
        # Digit 1: Single vertical line
        # Characteristics: High aspect ratio (tall and narrow), strong vertical line, minimal horizontal
        if aspect_ratio < 0.6:  # Narrow
            if vertical_lines >= 1:
                scores[1] += 0.4
            if horizontal_lines == 0:
                scores[1] += 0.3
            if fill_ratio < 0.3:  # Not too filled
                scores[1] += 0.3
        
        # Digit 2: Two horizontal segments + curves
        # Characteristics: Top curve, bottom curve, horizontal segments
        if top_curve:
            scores[2] += 0.3
        if bottom_curve:
            scores[2] += 0.3
        if horizontal_lines >= 1:
            scores[2] += 0.2
        if aspect_ratio > 0.5 and aspect_ratio < 1.2:  # Moderate width
            scores[2] += 0.2
        
        # Digit 3: Two curves (top and bottom)
        # Characteristics: Both top and bottom curves, similar shapes
        if top_curve and bottom_curve:
            scores[3] += 0.5
        if horizontal_lines == 0:  # No strong horizontal lines
            scores[3] += 0.2
        if aspect_ratio > 0.5 and aspect_ratio < 1.2:  # Moderate width
            scores[3] += 0.3
        
        # Find best match
        best_digit = max(scores, key=scores.get)
        best_score = scores[best_digit]
        
        # Normalize confidence (0.0 to 1.0)
        confidence = min(1.0, best_score)
        
        # Only return if confidence is reasonable
        if confidence < 0.3:
            return None, 0.0
        
        return best_digit, confidence
