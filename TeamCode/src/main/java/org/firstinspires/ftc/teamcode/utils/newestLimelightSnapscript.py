import cv2
import numpy as np

output_img = None

MIN_AREA = 400 # NEED TO CHANGE

H_FOV = 63.3
V_FOV = 49.7

kernel = np.ones((7,7), np.uint8)

lower_green = np.array([60, 168, 45])
    upper_green = np.array([90, 255, 255])


def runPipeline(image, llrobot):

    largestContour = np.array([[]])

    # blurred = cv2.GaussianBlur(image, (3, 3), 0)
    # blurred = cv2.bilateralFilter(image, 5, 75, 75)
   
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    

    


    mask = cv2.inRange(hsv, lower_green, upper_green)

    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations = 1)

    output_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(output_img, contours, -1, (0, 255, 0), 2)

    balls = []
    llresult = [0.0, 0.0, 0.0]
    closest_width = 0.0
    tv = 0

    if len(contours) > 0:
        largestContour = max(contours, key=cv2.contourArea)
    
        img_area = image.shape[0] * image.shape[1] # in pixel

        for c in contours:
            area = cv2.contourArea(c) # in pixel
            # cv2.putText(output_img, str(area), (1, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,0))
            
            # cv2.drawContours(output_img, c, -1, (0, 0, 255), 2)
            
            if area < MIN_AREA:
                continue
                 
            
            if len(c) >= 5:

                (cx1, cy1), (major, minor), angle1 = cv2.fitEllipse(c)
                (cx_ball, cy_ball), radius = cv2.minEnclosingCircle(c)
                center =  (int(cx_ball), int(cy_ball))

                cv2.circle(output_img, center, int(radius), (255, 255, 255), -1)
                # filled_mask = np.zeros(mask.shape, dtype=np.uint8)
                 
                if minor == 0:
                    minor = 1.0

                ratio = major / minor

                # 1 - 1.4999999 -> 1 ball
                # 1.5 - 1.99999

                # 1.5 - 2.499999 - 2 balls
                # 2 - 2.99999
                num_balls_est = int(ratio + 0.5)
                est_radius = minor / 2.0

                spacing = major / (num_balls_est + 1)

                initial_x = cx1 - major / 2

                for i in range(num_balls_est):

                    bx = initial_x + (i + 1) * spacing
                    balls.append([bx, cy1, est_radius])
            #else:
                #balls.append([cx1, cy1])
        
    if len(balls) > 0:
        height, width = image.shape[:2]
        cx_img = width * 0.5
        cy_img = height * 0.5

        balls = sorted(balls, key=lambda b: b[2], reverse=True)
        balls = balls[:3]

        llresult[0] = 1.0 # tv
        llresult[1] = float(len(balls))

        for (cx_ball, cy_ball, radius) in balls:
            dx = cx_ball - cx_img
            dy = cy_img - cy_ball

            nx = dx / cx_img
            ny = dy / cy_img

            # convert to angles
            tx = nx * (H_FOV * 0.5)
            ty = ny * (V_FOV * 0.5)

            # clamp 
            tx = max(min(tx, H_FOV/2.0), -H_FOV/2.0)
            ty = max(min(ty, V_FOV/2.0), -V_FOV/2.0)

            center = (int(cx_ball), int(cy_ball))

            # draw shapes
            cv2.circle(output_img, center, int(radius), (0, 255, 0), 3)
            cv2.circle(output_img, center, 3, (0, 0, 255), -1)

            llresult.extend([
                float(cx_ball),
                float(cy_ball),
                float(radius),
                float(tx),
                float(ty)
            ])

    return largestContour, output_img, llresult



    


def contour_centroid(contour):
    M = cv2.moments(contour)
    
    if M["m00"] == 0:
        return None  # avoid division by zero

    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    
    return cx, cy
