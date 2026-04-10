package org.firstinspires.ftc.teamcode.utils;

public class PythonSnapscriptHistory {

    /*
    4/5/26 =====
    working green ball only detection with snapscript

import cv2
import numpy as np

def runPipeline(image, llrobot):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_green_limit = np.array([60, 168, 93])
    upper_green_limit = np.array([90, 255, 255])
    mask = cv2.inRange(hsv, lower_green_limit, upper_green_limit)

    kernel = np.ones((3,3), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=4)

    output_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        area_px = cv2.contourArea(c)
        total_px = image.shape[0] * image.shape[1]
        area_percent = (area_px / total_px) * 100

        if area_percent >= 1.7749:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            # draw circle
            cv2.circle(output_img, center, int(radius), (0, 255, 0), 3)
            cv2.circle(output_img, center, 5, (0, 0, 255), -1)

            largestContour = c
            llpython[0] = 1
            llpython[1] = int(x)
            llpython[2] = int(y)

    return largestContour, output_img, llpython

    ===================== outputs rad + x + y

   import cv2
import numpy as np

def runPipeline(image, llrobot):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    lower_green_limit = np.array([60, 168, 93])
    upper_green_limit = np.array([90, 255, 255])

    kernel = np.ones((4,4), np.uint8)

    mask = cv2.inRange(hsv, lower_green_limit, upper_green_limit)


    mask = cv2.dilate(mask, kernel, iterations=3)

    output_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)





    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llresult = [0,0,0,0,0,0,0,0]

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        area_px = cv2.contourArea(c)
        total_px = image.shape[0] * image.shape[1]
        area_percent = (area_px / total_px) * 100

        if area_percent >= 1.7749:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            # draw circle
            cv2.circle(output_img, center, int(radius), (0, 255, 0), 3)
            cv2.circle(output_img, center, 5, (0, 0, 255), -1)

            largestContour = c

            llresult[0] = int(x)
            llresult[1] = int(y)
            llresult[2] = int(radius)

    return largestContour, output_img, llresult

    def detectDistance()
     */
}
