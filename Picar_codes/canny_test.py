import matplotlib.pylab as plt
import cv2
import numpy as np
import os
import math

def callback(x):
    print(x)

# Read the input image
image = cv2.imread('image.jpg')
image = cv2.resize(image, (320, 320))
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
height, width = image.shape[:2]
kernel = np.ones((3, 3), np.float32) / 9
#denoised_image = cv2.filter2D(image, -1, kernel)
height, width = image.shape[:2]
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cannied_image = cv2.Canny(gray_image, 30, 70)

cv2.namedWindow('image') # make a window with name 'image'
cv2.createTrackbar('L', 'image', 0, 255, callback) #lower threshold trackbar for window 'image
cv2.createTrackbar('U', 'image', 0, 255, callback) #upper threshold trackbar for window 'image

while(1):
    numpy_horizontal_concat = np.concatenate((image, cannied_image), axis=1) # to display image side by side
    cv2.imshow('image', numpy_horizontal_concat)
    k = cv2.waitKey(1) & 0xFF
    if k == 27: #escape key
        break
    l = cv2.getTrackbarPos('L', 'image')
    u = cv2.getTrackbarPos('U', 'image')

    canny = cv2.Canny(gray_image, l, u)

cv2.destroyAllWindows()