import cv2
import numpy as np

def create_white_mask(bgr_img, v_min=180, s_max=60):
    """
    Detects white-ish pixels using HSV
    White tends to have:
    - low saturation (S)
    - high brightness/value (V)
    """
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 0, v_min], dtype=np.uint8)
    upper = np.array([179, s_max, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    return mask

def main():
    img_path = "test_numbers.png"
    img = cv2.imread(img_path)

    if img is None:
        print(f"Could not read image: {img_path}")
        return

    mask = create_white_mask(img, v_min=180, s_max=60)

    kernel = np.ones((5,5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Apply mask to original image
    white_only = cv2.bitwise_and(img, img, mask=mask_clean)

    # Save the cleaned mask to a new file
    mask_output_path = "test_numbers_white_mask.png"
    cv2.imwrite(mask_output_path, mask_clean)
    print(f"Saved cleaned mask to: {mask_output_path}")

    cv2.imshow("Original", img)
    cv2.imshow("White Mask (raw)", mask)
    cv2.imshow("White Mask (clean)", mask_clean)
    cv2.imshow("White Only", white_only)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()