import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import argparse
import math 

 
# img = cv.imread('1.jpg')
# gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
# thresh = cv.threshold(gray, 127, 255, cv.THRESH_BINARY_INV)
 
# corners = cv.goodFeaturesToTrack(gray, 3, 0.01, 300)
# corners = np.intp(corners)
 
# for i in corners:
#     x,y = i.ravel()
#     cv.circle(img,(x,y),3,255,-1)
 
# cv.imshow("hmm", img)
# cv.waitKey(0)

def thresh_callback(val):
    output = src.copy()
    corner = cv.getTrackbarPos('Max Corner:', source_window)
    quality = cv.getTrackbarPos('Quality Level:', source_window) / 100.0
    distance = cv.getTrackbarPos('Min Distance:', source_window)
    k = cv.getTrackbarPos('K:', source_window) / 100.0
    
    
    gray = cv.cvtColor(output ,cv.COLOR_BGR2GRAY)
    # smooth = cv.GaussianBlur(gray, (5, 1), 0)
    smooth = cv.bilateralFilter(gray, 5, 75, 75)
    # thresh = cv.threshold(gray, 127, 255, cv.THRESH_BINARY_INV)
    
    
    contours_raw, _ = cv.findContours(smooth, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    contours = [i for i in contours_raw if cv.arcLength(i, False) > 1000]
    
    corners = cv.goodFeaturesToTrack(smooth, corner, quality, distance, useHarrisDetector=True, k=k)
    corners = np.intp(corners)
    
    # print(corners)
    actually_good = []
    
    for i in range(len(corners) - 1):
        x1, y1 = corners[i].ravel()
        x2, y2 = corners[i + 1].ravel()
        print(f"x1, y1: {x1}, {y1} x2, y2: {x2}, {y2}")
        print(math.tanh((y2 - y1) / (x2 - x1)))
        if abs(math.tanh((y2 - y1) / (x2 - x1))) < 0.1:
            actually_good.append([x1, y1])
            actually_good.append([x2, y2])
            cv.circle(output, (x1, y1), 3, 255, -1)
            cv.circle(output, (x2, y2), 3, 255, -1)
            
    print(actually_good)
    cv.imshow("hmm", output)
    
    
# Load source image
parser = argparse.ArgumentParser(description='Code for Convex Hull tutorial.')
parser.add_argument('--input', help='Path to input image.', default='1.jpg')
args = parser.parse_args()
src = cv.imread(cv.samples.findFile(args.input))
if src is None:
    print('Could not open or find the image:', args.input)
    exit(0)
# Convert image to gray and blur it
src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
src_gray = cv.GaussianBlur(src_gray, (5, 1), 0)
# Create Window
source_window = 'Source'
cv.namedWindow(source_window)
cv.imshow(source_window, src)
max_corner = 255
corner = 86
max_quality = 100
quality = 1
max_distance = 400
distance = 169
max_k = 100
k = 4
cv.createTrackbar('Max Corner:', source_window, corner, max_corner, thresh_callback)
cv.createTrackbar('Quality Level:', source_window, quality, max_quality, thresh_callback)
cv.createTrackbar('Min Distance:', source_window, distance, max_distance, thresh_callback)
cv.createTrackbar('K:', source_window, k, max_k, thresh_callback)
thresh_callback(corner)

cv.waitKey(0)
