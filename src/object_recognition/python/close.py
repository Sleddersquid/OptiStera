import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import cv2 as cv2
import time as t
import imutils
import math

mpl.rc('axes', titlesize=14)  
mpl.rc('xtick', labelsize=12)
mpl.rc('ytick', labelsize=12)

# clean up points function: https://stackoverflow.com/questions/7263621/how-to-find-corners-on-a-image-using-opencv

# Routine to fix the image
def fix_image(image):  
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

def distance(pt1, pt2):
    (x1, y1), (x2, y2) = pt1, pt2
    dist = math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
    return dist

def clean_up_points(thresh : int, coordiantes):
    coor_tuples_copy = coordiantes
    i = 1
    for pt1 in coordiantes:
    # print(' I :', i)
        for pt2 in coordiantes[i::1]:
            # print(pt1, pt2)
            # print('Distance :', distance(pt1, pt2))
            if(distance(pt1, pt2) < thresh):
                coor_tuples_copy.remove(pt2)      
        i+=1
    return coor_tuples_copy

def clean_up_points_inverse(thresh : int, coordiantes):
    coor_tuples_copy = coordiantes.reverse()
    i = 1
    for pt1 in coordiantes:
    # print(' I :', i)
        for pt2 in coordiantes[i::1]:
            # print(pt1, pt2)
            # print('Distance :', distance(pt1, pt2))
            if(distance(pt1, pt2) > thresh):
                coor_tuples_copy.remove(pt2)      
        i+=1
    return coor_tuples_copy


def find_rectangle(thresh : int, coordinates):
    coordinates.reverse()
    rectangle_coords = [coordinates[0]]
    # print(len(rectangle_coords))
    i = 1
    for pt1 in coordinates:
    # print(' I :', i)
        for pt2 in coordinates[i::1]:
            # print(pt1, pt2)
            # print('Distance :', distance(pt1, pt2))
            if(distance(pt1, pt2) > thresh):
                rectangle_coords.append(pt2)
                if (len(rectangle_coords) == 4):
                    return rectangle_coords
        i+=1
    return rectangle_coords

def close_up_point_remove(thresh : int, coordinates, image):
    output = image.copy()
    coordinates.reverse()
    rectangle_coords = []
    for i in range(len(coordinates) - 1):
        dist = distance(coordinates[i], coordinates[i + 1])
        cv2.circle(output, (coordinates[i][1], coordinates[i][0]), 5, (0, 255, 0), -1)
        cv2.circle(output, (coordinates[i + 1][1], coordinates[i + 1][0]), 5, (0, 255, 0), -1)
        cv2.imshow('output', output)
        cv2.waitKey(1)
        if dist > thresh:
            rectangle_coords.append(coordinates[i])
            if (len(rectangle_coords) == 4):
                return rectangle_coords
    
    return rectangle_coords

def avarageing(thesh : int, coordinates):
    coordinates.reverse()
    avaregae_coord = []
    
    for i in range(len(coordinates) - 1):
        dist = distance(coordinates[i], coordinates[i + 1])
        sum_dist = 0
    


# Read image
image = cv2.imread("1.jpg")

# image = image[0:720, 200:1000]

# COnvert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# smooth the image
# gray = cv2.GaussianBlur(gray, (5, 1), 0)
bi = cv2.bilateralFilter(gray, 5, 75, 75)
# smooth = cv2.GaussianBlur(gray, (5, 7), 0)

canny = cv2.Canny(bi, 100, 300)
# plt.imshow(fix_image(canny))
# plt.show()


dst = cv2.cornerHarris(canny, 2, 3, 0.04)
# plt.imshow(dst, cmap='gray')
# plt.show()

mask = np.zeros_like(gray)

mask[dst > 0.015 * dst.max()] = 255
# plt.imshow(fix_image(mask))
# plt.show()

coordinates = np.argwhere(mask)

# print(coordinates)

coor_list = [l.tolist() for l in list(coordinates)]

coor_tuples = [tuple(l) for l in coor_list]

print(len(coor_tuples))

# rectangle_coords = close_up_point_remove(200, coor_tuples, image)

# print(len(rectangle_coords))

# coor_tuples_copy_2 = clean_up_points(240, coor_tuples_copy)

# coor_tuples_copy_3 = clean_up_points_inverse(, coor_tuples_copy_2)

# print(coor_tuples_copy_2)

# print(coor_tuples_copy)

# rectangle_coords = find_rectangle(200, coor_tuples_copy)


# print(coor_tuples)

output = image.copy()

for point in coor_tuples:
    cv2.circle(output, (point[1], point[0]), 5, (0, 255, 0), -1)
cv2.imshow("Bum as shi", output)
cv2.waitKey(0)

