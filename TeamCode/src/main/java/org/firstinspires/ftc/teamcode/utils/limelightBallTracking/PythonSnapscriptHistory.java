package org.firstinspires.ftc.teamcode.utils.limelightBallTracking;

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
========== UNTESTED
    import cv2
import numpy as np

MIN_AREA = 100 # NEED TO CHANGE


def runPipeline(image, llrobot):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_green = np.array([60, 168, 93])
    upper_green = np.array([90, 255, 255])
    kernel = np.ones((3,3), np.uint8)


    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    output_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    balls = []
    llresult = [0.0, 0.0, 0.0]
    closest_width = 0.0
    tv = 0


    if len(contours) > 0:



        img_area = image.shape[0] * image.shape[1] # in pixel

        for c in contours:
            area = cv2.contourArea(c) # in pixel
            area_percent = (area / img_area) * 100
            if area < MIN_AREA or area_percent < 1.7749:
                continue

            # make shapes
            (cx_ball, cy_ball), radius = cv2.minEnclosingCircle(c)
            x, y, w, h = cv2.boundingRect(c)

            circularity = area / (np.pi * radius * radius)

            # distinguishing between overlapping balls
            if circularity > 0.8:
                balls.append((cx_ball, cy_ball, radius))
            else:

                balls.extend(splitBalls(mask, x, y, w, h))


        # for accurate distance calculations in java


        if len(balls) > 0:
            tv = 1
            height, width, _ = image.shape
            cx_img = width * 0.5
            cy_img = height * 0.5

            H_FOV = 63.3
            V_FOV = 49.7

            balls = sorted(balls, key=lambda b: b[2], reverse=True)

            balls = balls[:3]

            closest_radius = balls[0][2]
            closest_width = closest_radius * 2.0

            llresult[0] = 1 # tv CHANGE
            llresult[1] = float(len(balls))

            # sort by size (closest first)


            for (cx_ball, cy_ball, radius) in balls:

                # offset from center
                dx = cx_ball - cx_img
                dy = cy_img - cy_ball

                # normalize
                nx = dx / cx_img
                ny = dy / cy_img

                # convert to angles
                tx = nx * (H_FOV * 0.5)
                ty = ny * (V_FOV * 0.5)

                # clamp
                tx = max(min(tx, H_FOV/2), -H_FOV/2)
                ty = max(min(ty, V_FOV/2), -V_FOV/2)

                center = (int(cx_ball), int(cy_ball))
                # draw shapes
                cv2.circle(output_img, center, int(radius), (0, 255, 0), 3)
                cv2.circle(output_img, center, 3, (0, 0, 255), -1)

                # store
                llresult.extend([
                    float(cx_ball),
                    float(cy_ball),
                    float(radius),
                    float(tx),
                    float(ty)
                ])

        else:
            llresult = [0.0, 0.0, 0.0]

    return output_img, llresult

def splitBalls(mask, x, y, w, h):

        cropped_region = mask[y:y+h, x:x+w]

        dist_mask = cv2.distanceTransform(cropped_region, cv2.DIST_L2, 5)
        # L2 means euclidean distance
        # L1 is manhatten distance
        # C is chessboard distance

        _, peaks = cv2.threshold(dist_mask, 0.5 * dist_mask.max(), 255, cv2.THRESH_BINARY)
        peaks = peaks.astype(np.uint8)
        contours, _ = cv2.findContours(peaks, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_balls = []
        for cs in contours:
            (px, py), _ = cv2.minEnclosingCircle(cs)

            ac_radius = float(dist_mask[int(py), int(px)])

            # convert back to full img coors
            cx = px + x
            cy = py + y

            found_balls.append((cx, cy, ac_radius))

        return found_balls


     */
}
