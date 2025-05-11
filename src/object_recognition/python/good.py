
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import cv2 as cv2
import time as t
import imutils
import math

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

def collect_points(thresh, coordinates):
    coord_fuck_off = []
    x_sum, y_sum, n, i = 0, 0, 0, 0
    
    while(len(coordinates) - 1 > i):
        x1, y1 = coordinates[i]
        # x2, y2 = coordinates[i + 1]
        if distance(coordinates[i], coordinates[i + 1]) < thresh:
            i += 1
            print(n)
            x_sum += x1
            y_sum += y1
            n += 1
        elif(n == 0):
            i += 1 
            continue
        else:
            i += 2
            coord_fuck_off.append([int(x_sum / n), int(y_sum / n)])
            x_sum, y_sum, n = 0, 0, 0
            continue
    
    print(coord_fuck_off)
    return coord_fuck_off
    

cam = cv2.VideoCapture(0)

while(True):
    # Read image
    # image = cv2.imread("1.jpg")[0:1280, 200:1000]
    ret, image = cam.read(0)

    if not ret:
        print("Failed to capture image")
        break

    output_actually_clean_up = image.copy()
    output_actually_good = image.copy()
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

    # if(coor_tuples == []):
    #     continue

    for (x, y) in coor_tuples:
        cv2.circle(image, (y, x), 5, (0, 255, 0), -1)

    # cv2.imshow("Corners", image)
    cv2.imshow("Canny", canny)
    # cv2.imshow("Bilateral", bi)
    # cv2.waitKey(0)

    coor_tuples_copy = clean_up_points(80, coor_tuples)

    for (x, y) in coor_tuples_copy:
        cv2.circle(output_actually_clean_up, (y, x), 5, (0, 255, 0), -1)
        
    cv2.imshow("Clean up", output_actually_clean_up)
    # cv2.waitKey(0)

    print(len(coor_tuples_copy))

    # print(corners)
    actually_good = []

    for i in range(0, len(coor_tuples_copy) - 1, 1):
        x1, y1 = coor_tuples_copy[i]
        x2, y2 = coor_tuples_copy[i + 1]
        if(y2 - y1 == 0) or (x2 - x1) == 0:
            continue
        if (abs(math.tanh((x2 - x1) / (y2 - y1))) < 0.05):
            print(i)
            print(f"x1, y1: {x1}, {y1} x2, y2: {x2}, {y2}")
            print(abs(math.tanh((x2 - x1) / (y2 - y1))))
            actually_good.append([y1, x1])
            actually_good.append([y2, x2])
            cv2.circle(output_actually_good, (y1, x1), 3, (255, 0, 0), -1)
            cv2.circle(output_actually_good, (y2, x2), 3, (255, 0, 0), -1)
            cv2.line(output_actually_good, (y1, x1), (y2, x2), (255, 0, 0), 1)
            
    print(actually_good)

    cv2.imshow("hmm", output_actually_good)
# cv2.waitKey(0)

# # coor_tuples_copy_2 = clean_up_points(240, coor_tuples_copy)

# # coor_tuples_copy_3 = clean_up_points_inverse(, coor_tuples_copy_2)

# # print(coor_tuples_copy_2)

# # print(coor_tuples_copy)

# rectangle_coords = find_rectangle(200, coor_tuples_copy)

# print(rectangle_coords)

# output = image.copy()

# for point in rectangle_coords:
#     cv2.circle(output, (point[1], point[0]), 5, (0, 255, 0), -1)

# cv2.imshow("Corners", output)
    
    key = cv2.waitKey(1) & 0xFF
    
    if (key == 113):
        break
    
cam.release()
cv2.destroyAllWindows()

print("Done")
