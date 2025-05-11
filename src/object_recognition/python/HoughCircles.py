import cv2
import numpy as np

# https://docs.opencv.org/4.x/da/d53/tutorial_py_houghcircles.html

cam = cv2.VideoCapture(0)

while True:
    # Read image as gray-scale
    ret, img = cam.read(0)
    # Convert to gray-scale

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Blur the image to reduce noise
    img_blur = cv2.medianBlur(gray, 5)
    cv2.imshow("Not final", img_blur)
    # Apply hough transform on the image
    circles = cv2.HoughCircles(img_blur, cv2.HOUGH_GRADIENT, 1, 40, param1=100, param2=30, minRadius=1, maxRadius=40)
    # Draw detected circles
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # Draw outer circle
            cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # Draw inner circle
            cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 5)
    # Show result
    print("Circle Detection using Hough Transform")
    cv2.imshow("Final", img)
    
    if cv2.waitKey(1) & 0xFF == 113:
        break