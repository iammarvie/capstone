import matplotlib.pylab as plt
import cv2
import numpy as np
import os
import math

os.environ["QT_QPA_PLATFORM"] = "offscreen"


# Read the input image
image = cv2.imread('image.jpg')
image = cv2.resize(image, (320, 320))
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
cv2.imwrite('hsv.jpg', image)
height, width = image.shape[:2]
print('Image shape:', image.shape)
kernel = np.ones((3, 3), np.float32) / 9
#denoised_image = cv2.filter2D(image, -1, kernel)
height, width = image.shape[:2]
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
(thresh, im_bw) = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
cv2.imwrite('bw_image.png', im_bw)
cannied_image = cv2.Canny(gray_image, 30, 90)
cv2.imwrite('GRAy.jpg', gray_image)
cv2.imwrite('canny.jpg', cannied_image)

hpush_b = 0.99
hpush_t = 0.80
bleft = (int(width * 0), int(height*hpush_b))  # Bottom-left
bright = (int(width), int(height*hpush_b))# Bottom-right]
tleft = (int(width * 0.20), int(height * hpush_t))  # Top-left
tright = (int(width * 0.80), int(height * hpush_t))  # Top-right

region_of_interest_coor = [ bleft, tleft, tright,bright]
center_mask = [
    (int(0.5 * width), int(0.5 * height)),  # Top-left
    (int(0.20 * width), int(hpush_b * height)),  # Bottom-left
    (int(0.80 * width), int(hpush_b * height)),  # Bottom-right
    (int(0.5 * width), int(0.5 * height))   # Top-right
]
mask = np.zeros_like(cannied_image)
cv2.fillPoly(mask, [np.array(region_of_interest_coor)], 255)
cv2.imwrite('mask_debug.jpg', mask)  # Save the mask image for debugging
cropped_image = cv2.bitwise_and(cannied_image, mask)
cv2.fillPoly(cropped_image, [np.array(center_mask)],0)
cv2.imwrite('pon.jpg', cropped_image)
# Apply Hough Line Transform to detect lines
lines = cv2.HoughLinesP(cropped_image, 1, np.pi / 60,10, np.array([]), minLineLength=20, maxLineGap=150)

# Check if lines were detected before processing them
# Check if lines were detected before processing them
'''
left_line = []
right_line = []

if lines is not None:  # Ensure lines are detected
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1)
            if slope < 0:
                left_line.append(line)
            else:
                right_line.append(line)

    # Check if there are any left or right lines before calculating averages
    if len(left_line) > 0:
        left_line = np.mean(left_line, axis=0, dtype=np.int32)
        print ("a")
    else:
        print ("b")
        left_line = None  # Set to None if no left lines

    if len(right_line) > 0:
        print ("ba")
        right_line = np.mean(right_line, axis=0, dtype=np.int32)
    else:
        print ("baa")
        right_line = None  # Set to None if no right lines

    # Draw the lines on the image only if they are not None
    if left_line is not None:
        cv2.line(image, (left_line[0][0], left_line[0][1]), (left_line[0][2], left_line[0][3]), (0, 255, 0), 5)

    if right_line is not None:
        cv2.line(image, (right_line[0][0], right_line[0][1]), (right_line[0][2], right_line[0][3]), (0, 255, 0), 5)

    # Check if both lines are not None before calculating the lane center and offset
    if left_line is not None and right_line is not None:
        #return 0.0
        # Calculate the lane center (midpoint of left and right lines at bottom)
        lane_center = (
            (left_line[0][2] + right_line[0][2]) // 2,
            (left_line[0][3] + right_line[0][3]) // 2
        )

        # Calculate the image center
        image_center = (width // 2, height)

        # Calculate the horizontal offset between lane center and image center
        offset = lane_center[0] - image_center[0]

        # Predict the turn angle (in degrees) using arctangent
        angle = np.arctan2(offset, height) * (180.0 / np.pi)
        # Print turn prediction based on angle threshold
        if abs(angle) < 5:
            print("Keep straight")
        elif angle > 5:
            print(f"Turn right by {angle:.2f} degrees")
        else:
            print(f"Turn left by {abs(angle):.2f} degrees")
        print(f'Offset: {offset}, Angle: {angle:.2f} degrees')

        # Visualize the lane center and the offset line
        cv2.circle(image, lane_center, 5, (0, 0, 255), -1)  # Lane center
        cv2.line(image, image_center, lane_center, (255, 0, 255), 2)  # Offset line

    # Draw the region of interest polygon
    cv2.polylines(image, [np.array(region_of_interest_coor)], True, (0, 255, 255), 2)
    cv2.polylines(image, [np.array(center_mask)], True, (0, 255, 180), 2)

else:
    # No lines detected, draw the region of interest polygon only
    cv2.polylines(image, [np.array(region_of_interest_coor)], True, (0, 255, 255), 2)
    cv2.polylines(image, [np.array(center_mask)], True, (0, 255, 180), 2)
    print("No lines were detected.")
'''
left_line_x = []
left_line_y = []
right_line_x = []
right_line_y = []

# Check if lines were detected before processing them
if lines is not None:
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter out lines with a slope magnitude that is too small
            if math.fabs(slope) < 0.5:
                continue

            if slope < 0:
                left_line_x.extend([x1, x2])
                left_line_y.extend([y1, y2])
            else:
                right_line_x.extend([x1, x2])
                right_line_y.extend([y1, y2])

    # Only proceed if there are enough points to fit lines
    if left_line_x and left_line_y and right_line_x and right_line_y:
        min_y = int(image.shape[0] * (3 / 5))  # Just below the horizon
        max_y = image.shape[0]  # Bottom of the image

        # Fit lines to the left and right points
        poly_left = np.poly1d(np.polyfit(left_line_y, left_line_x, deg=1))
        poly_right = np.poly1d(np.polyfit(right_line_y, right_line_x, deg=1))

        # Calculate the x-coordinates of the lines at min_y and max_y
        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))
        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))

        # Draw the left and right lines on the image
        cv2.line(image, (left_x_start, max_y), (left_x_end, min_y), (0, 255, 0), 5)
        cv2.line(image, (right_x_start, max_y), (right_x_end, min_y), (0, 255, 0), 5)

        # Calculate the lane center
        lane_center_x = (left_x_end + right_x_end) // 2
        lane_center = (lane_center_x, min_y)

        # Calculate the image center
        image_center_x = width // 2
        offset = lane_center_x - image_center_x

        # Calculate the steering angle
        angle = np.arctan2(offset, height) * (180.0 / np.pi)
        if abs(angle) < 5:
            print("Keep straight")
        elif angle > 5:
            print(f"Turn right by {angle:.2f} degrees")
        else:
            print(f"Turn left by {abs(angle):.2f} degrees")
        print(f'Offset: {offset}, Angle: {angle:.2f} degrees')

        # Visualize the lane center and the offset line
        cv2.circle(image, lane_center, 5, (0, 0, 255), -1)
        cv2.line(image, (image_center_x, max_y), lane_center, (255, 0, 255), 2)

    else:
        print("Not enough points to fit lines.")

# Draw the region of interest
cv2.polylines(image, [np.array(region_of_interest_coor)], True, (0, 255, 255), 2)

cv2.imwrite('lane_detect.jpg', image)
# cv2.imwrite('cropped.jpg', cropped_image)

'''
  elif lane_info > 2:
            self.get_logger().info('Steering right')
            return -0.20
        else:
            self.get_logger().info('Steering left')
            return 0.20
'''


