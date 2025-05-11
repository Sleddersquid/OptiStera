import cv2 as cv2
import numpy as np

# https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html

# Global Variables

egdeThresh = 1

lowThreshold = 100
highThreshold = 300

ratio = 3

kernel_size = 3

cam = cv2.VideoCapture(0)

while(True):
    ret, src = cam.read(0)
    
    if not ret:
        print("Failed to capture image")
        break

    src_gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

    # smooth = cv2.blur(src_gray, (3, 3))
    # smooth = cv2.Sobel(src_gray, cv2.CV_8U, 0, 1, ksize=3)
    # smooth = cv2.medianBlur(src_gray, 5)
    smooth = cv2.bilateralFilter(src_gray, 5, 27, 27)
    # smooth = cv2.GaussianBlur(src_gray, (11, 11), 0)

    detected_edges = cv2.Canny(src_gray, lowThreshold, highThreshold, kernel_size)

    cv2.imshow('detected_edges', detected_edges)

    lines = cv2.HoughLines(detected_edges, 0.1, np.pi / 180, 50)

    # print(lines)
    print(src.shape)

    # if lines is not None:
    #     for i in range(len(lines) - 1):
    #         rho = lines[i][0]
    #         theta = lines[i][1]
    #         a, b = np.cos(theta), np.cos(theta)
    #         x0 = a * rho
    #         y0 = b * rho
    #         x1 = int(x0 + 1000 * (-b))
    #         y1 = int(y0 + 1000 * (a))
    #         x2 = int(x0 - 1000 * (-b))
    #         y2 = int(y0 - 1000 * (a))
    #         cv2.line(src, (x1, y1), (x2, y2), (0, 0, 255), 2, cv2.LINE_AA)
            
    if lines is not None:
        for r_theta in lines:
            arr = np.array(r_theta[0], dtype=np.float64)
            r, theta = arr
            # Stores the value of cos(theta) in a
            a = np.cos(theta)

            # Stores the value of sin(theta) in b
            b = np.sin(theta)

            # x0 stores the value rcos(theta)
            x0 = a*r

            # y0 stores the value rsin(theta)
            y0 = b*r

            # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
            x1 = int(x0 + 1000*(-b))

            # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
            y1 = int(y0 + 1000*(a))

            # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
            x2 = int(x0 - 1000*(-b))

            # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
            y2 = int(y0 - 1000*(a))

            # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
            # (0,0,255) denotes the colour of the line to be
            # drawn. In this case, it is red.
            cv2.line(src, (x1, y1), (x2, y2), (0, 0, 255), 2)        
    cv2.imshow('lines', src)
    if cv2.waitKey(1) & 0xFF == 113:
        break
    
    
cam.release()
cv2.destroyAllWindows()


