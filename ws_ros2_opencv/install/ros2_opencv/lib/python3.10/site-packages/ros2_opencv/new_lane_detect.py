import cv2
import numpy as np
import math

class LaneDetection:
    def __init__(self):
        # Parameters for tuning
        self.min_line_length = 40
        self.max_line_gap = 150
        self.canny_threshold1 = 100
        self.canny_threshold2 = 200

    def preprocess_image(self, cv_image):
        """Preprocess the image with grayscale, equalization, blurring, and Canny edge detection."""
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        equalized_image = cv2.equalizeHist(gray_image)
        blur_image = cv2.GaussianBlur(equalized_image, (5, 5), 0)
        cannied_image = cv2.Canny(blur_image, self.canny_threshold1, self.canny_threshold2)
        return cannied_image

    def get_region_of_interest_coordinates(self, width, height):
        """Define region of interest coordinates as a trapezoid."""

        bleft = (int(width * 0), int (height*0.85))
        tleft = (int(width * 0.15), int(height * 0.6))
        tright = (int(width * 0.85), int(height * 0.6))
        bright = (int(width), int (height*0.85))
        return [
            bleft, tleft, tright, bright
        ]

    def classify_lines(self, lines):
        """Classify lines into left and right based on their slope."""
        left_lines, right_lines = [], []
        for line in lines:
            for x1, y1, x2, y2 in line:
                if (x2 - x1) != 0:  # Avoid division by zero
                    slope = (y2 - y1) / (x2 - x1)
                    if slope < 0:
                        left_lines.append(line)
                    else:
                        right_lines.append(line)
        return left_lines, right_lines

    def calculate_steering_angle(self, left_line, right_line, width, height):
        """Calculate the angle for steering based on the lane center."""
        if left_line is None or right_line is None:
            return None  # No valid lines detected
        
        # Calculate lane center and image center
        lane_center = (
            (left_line[0][2] + right_line[0][2]) // 2,
            (left_line[0][3] + right_line[0][3]) // 2
        )
        image_center = (width // 2, height)
        
        # Calculate the horizontal offset and the angle
        offset = lane_center[0] - image_center[0]
        angle = np.arctan2(offset, height) * (180.0 / np.pi)
        return angle

    def detect_lane(self, cv_image):
        """Main lane detection function that returns the steering angle."""
        height, width = cv_image.shape[:2]
        cannied_image = self.preprocess_image(cv_image)

        # Get region of interest and mask it
        region_of_interest_coor = self.get_region_of_interest_coordinates(width, height)
        # draw the trapezoid on the image and show
        drimg = cv_image.copy()
        drimg = cv2.line(drimg, region_of_interest_coor[0], region_of_interest_coor[1], (255, 0, 0), 2)
        drimg = cv2.line(drimg, region_of_interest_coor[1], region_of_interest_coor[2], (255, 0, 0), 2)
        drimg = cv2.line(drimg, region_of_interest_coor[2], region_of_interest_coor[3], (255, 0, 0), 2)
        drimg = cv2.line(drimg, region_of_interest_coor[3], region_of_interest_coor[0], (255, 0, 0), 2)
        cv2.imwrite('trapezoid.jpg', drimg)
        mask = np.zeros_like(cannied_image)
        cv2.fillPoly(mask, [np.array(region_of_interest_coor)], 255)
        cropped_image = cv2.bitwise_and(cannied_image, mask)
        cv2.imwrite('canny.jpg', cropped_image)
        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(cropped_image, 1, np.pi / 180, 20,
                                minLineLength=self.min_line_length, maxLineGap=self.max_line_gap)

        # Classify lines into left and right
        left_lines, right_lines = self.classify_lines(lines) if lines is not None else ([], [])
        left_line = np.mean(left_lines, axis=0, dtype=np.int32) if len(left_lines) > 0 else None
        right_line = np.mean(right_lines, axis=0, dtype=np.int32) if len(right_lines) > 0 else None

        # Draw the lines on the image
        line_image = cv_image.copy()
        if left_line is not None:
            cv2.line(line_image, (left_line[0][0], left_line[0][1]),
                     (left_line[0][2], left_line[0][3]), (0, 255, 0), 5)
        if right_line is not None:
            cv2.line(line_image, (right_line[0][0], right_line[0][1]),
                     (right_line[0][2], right_line[0][3]), (0, 255, 0), 5)
            
        cv2.imwrite('output.jpg', line_image)

        # Calculate steering angle
        angle = self.calculate_steering_angle(left_line, right_line, width, height)
        return angle

# Main function to process a single image and output the steering angle
def main(image_path):
    # Load the image
    image = cv2.imread(image_path)
    image = cv2.resize(image, (340, 340))
    if image is None:
        print("Error: Could not read image.")
        return

    # Initialize the LaneDetection class
    lane_detector = LaneDetection()
    
    # Calculate the steering angle
    angle = lane_detector.detect_lane(image)
    if angle is not None:
        print(f'Steering angle: {angle:.2f} degrees')
    else:
        print("No lanes detected")

if __name__ == '__main__':
    # Specify the path to your image
    image_path = 'image.jpg'
    main(image_path)
