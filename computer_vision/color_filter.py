#!/usr/bin/env python3
"""
Color filter script that keeps only red and green colors, replacing others with white.
Allows adjustable threshold for color similarity.
"""

import cv2
import numpy as np
import argparse
from pathlib import Path


def filter_red_green_colors(image, threshold=100):
    """
    Filter image to keep only colors similar to red or green.
    Replace all other colors with white.
    
    Args:
        image: Input image (BGR format from OpenCV)
        threshold: Maximum distance from red/green to keep (0-441, where 441 is max RGB distance)
    
    Returns:
        Filtered image with non-red/green colors replaced with white
    """
    # Define target colors in RGB (will convert from BGR)
    # Pure red: (255, 0, 0)
    # Pure green: (0, 255, 0)
    red_rgb = np.array([255, 0, 0], dtype=np.float32)
    green_rgb = np.array([0, 255, 0], dtype=np.float32)
    
    # Convert BGR to RGB for processing
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32)
    
    # Calculate distance to red and green for all pixels at once
    # Reshape to (height*width, 3) for vectorized operations
    pixels = image_rgb.reshape(-1, 3)
    
    # Calculate Euclidean distance to red and green
    dist_to_red = np.sqrt(np.sum((pixels - red_rgb) ** 2, axis=1))
    dist_to_green = np.sqrt(np.sum((pixels - green_rgb) ** 2, axis=1))
    
    # Create mask: keep pixels that are similar to either red or green
    mask = (dist_to_red <= threshold) | (dist_to_green <= threshold)
    
    # Create output image (start with white)
    output = np.ones_like(image_rgb) * 255
    
    # Apply mask: keep original pixels where mask is True, white elsewhere
    output_flat = output.reshape(-1, 3)
    output_flat[mask] = pixels[mask]
    output = output_flat.reshape(image_rgb.shape)
    
    # Convert back to BGR and uint8 for saving
    output_bgr = cv2.cvtColor(output.astype(np.uint8), cv2.COLOR_RGB2BGR)
    
    return output_bgr


def main():
    parser = argparse.ArgumentParser(
        description="Filter image to keep only red and green colors, replace others with white"
    )
    parser.add_argument(
        "--input",
        "-i",
        default="image.png",
        help="Input image file (default: image.png)",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output image file (default: input_filtered.png)",
    )
    parser.add_argument(
        "--threshold",
        "-t",
        type=float,
        default=100.0,
        help="Color similarity threshold (0-441, lower = stricter, default: 100)",
    )
    
    args = parser.parse_args()
    
    # Set output filename if not provided
    if args.output is None:
        input_path = Path(args.input)
        args.output = str(input_path.parent / f"{input_path.stem}_filtered{input_path.suffix}")
    
    # Validate threshold
    if args.threshold < 0 or args.threshold > 441:
        print("Warning: Threshold should be between 0 and 441. Clamping to valid range.")
        args.threshold = max(0, min(441, args.threshold))
    
    # Load image
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: Input image not found: {args.input}")
        return 1
    
    print(f"Loading image: {args.input}")
    image = cv2.imread(str(input_path))
    
    if image is None:
        print(f"Error: Could not load image: {args.input}")
        return 1
    
    print(f"Filtering with threshold: {args.threshold}")
    print("Processing image...")
    
    # Filter colors
    filtered_image = filter_red_green_colors(image, threshold=args.threshold)
    
    # Save result
    output_path = Path(args.output)
    print(f"Saving filtered image: {args.output}")
    cv2.imwrite(str(output_path), filtered_image)
    
    print("Done!")
    return 0


if __name__ == "__main__":
    exit(main())

