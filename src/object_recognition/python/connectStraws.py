import cv2 as cv2
import numpy as np

# from https://answers.opencv.org/question/222388/detect-ellipses-ovals-in-images-with-opencv-pythonsolved/

# cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
# cv2.namedWindow("Thresh", cv2.WINDOW_NORMAL)

image = cv2.imread('image.jpg')

thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
thresh = cv2.inRange(thresh, (0, 150, 89), (35, 255, 255))

thresh = cv2.adaptiveThreshold(thresh, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 201, 0)

cv2.erode(thresh, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)), thresh)
cv2.erode(thresh, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)), thresh)

cv2.dilate(thresh, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), thresh)
cv2.dilate(thresh, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)), thresh)

# cv2.erode(thresh, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)), thresh)q
# cv2.dilate(thresh, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)), thresh)

cv2.imshow('Thresh', thresh)

contours, hieracky = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# contour = max(contours, key=cv2.contourArea)

for i in range(len(contours)):
    area = cv2.contourArea(contours[i])
    if(area > 1 and area < 7500):
        ellipse = cv2.fitEllipseDirect(contours[i])
        center, axes, angle = ellipse
        if (abs(axes[0] - axes[1]) < 30) and (axes[0] < 100 and axes[1] < 100) and (axes[0] > 25 and axes[1] > 25):
            cv2.ellipse(image, ellipse, (0, 255, 0), 2)
            print(f"centre {center} axes {axes} angle {angle} area {area}")
        
        # cv2.drawContours(image, contours[i], -1, (0, 255, 0), 3)
        # print(len(contours[i]))
        # print(area )

# cv2.drawContours(image, contour, -1, (0, 255, 0), 3)

# count, labels, stats, centroids = cv2.connectedComponentsWithStats(thresh)
# for i in range(1,count):
#     image = cv2.circle(image, (int(centroids[i,0]), int(centroids[i,1])), 5, (0, 255, 0, 0), 5)

cv2.imshow('Image', image)
# cv2.imshow('centers', image)
cv2.waitKey()