import numpy as np
import cv2 as cv

rbg_hand = np.uint8([[[134, 89, 56]]]) # Hand 
rbg_ball = np.uint8([[[133, 37, 15]]]) # Ball 

hsv_hand = cv.cvtColor(rbg_hand, cv.COLOR_RGB2HSV)
hsv_ball = cv.cvtColor(rbg_ball, cv.COLOR_RGB2HSV)

print(f"Hand hsv: {hsv_hand}")
print(f"Ball hsv: {hsv_ball}")
