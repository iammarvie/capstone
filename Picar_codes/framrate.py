# chnage the frame rate of a video
import cv2
import time

cap = cv2.VideoCapture('stop_sign.avi')
fps = 1
cap.set(cv2.CAP_PROP_FPS, fps)
# save the video
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('stop_sign_slow.avi', fourcc, fps, (320, 320))
cap.release()
cv2.destroyAllWindows()


