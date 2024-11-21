import cv2
import numpy as np

def callback(x):
    pass  # Callback function for trackbars, required by OpenCV

# Read and resize the input image
image = cv2.imread('image.jpg')
image = cv2.resize(image, (320, 320))

# Convert to HSV and apply a color threshold to detect brownish areas
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_brown = np.array([10, 100, 20])  # Adjust these values for the brownish area
upper_brown = np.array([20, 255, 200])  # Adjust these values for the brownish area
mask_brown = cv2.inRange(hsv_image, lower_brown, upper_brown)
highlighted_image = cv2.bitwise_and(image, image, mask=mask_brown)

# Convert to grayscale and apply histogram equalization
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
equalized_image = cv2.equalizeHist(gray_image)

# Create a window and trackbars for adjusting Canny thresholds
cv2.namedWindow('image')
cv2.createTrackbar('L', 'image', 0, 255, callback)  # Lower threshold trackbar
cv2.createTrackbar('U', 'image', 0, 255, callback)  # Upper threshold trackbar

while True:
    # Get current positions of the trackbars
    l = cv2.getTrackbarPos('L', 'image')
    u = cv2.getTrackbarPos('U', 'image')

    # Apply Canny edge detection with current threshold values
    cannied_image = cv2.Canny(equalized_image, l, u)

    # Concatenate the original, equalized, and Canny images side by side
    original_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    equalized_bgr = cv2.cvtColor(equalized_image, cv2.COLOR_GRAY2BGR)
    cannied_bgr = cv2.cvtColor(cannied_image, cv2.COLOR_GRAY2BGR)
    numpy_horizontal_concat = np.concatenate((original_bgr, equalized_bgr, cannied_bgr), axis=1)

    # Display the concatenated image
    cv2.imshow('image', numpy_horizontal_concat)

    # Break the loop when the Escape key is pressed
    k = cv2.waitKey(1) & 0xFF
    if k == 27:  # Escape key
        break

# Destroy all windows
cv2.destroyAllWindows()

