import cv2
import imutils
import numpy as np

def CannyEdge(img1, img2):

    canny1 = cv2.Canny(img1, 5, 50)
    canny2 = cv2.Canny(img2, 5, 50)

    return canny1, canny2

def ContourDetection(image):
    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(image=image.copy(), mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # loop over the contours
    for c in cnts:

        # Skip over small and to large contours
        if 1000 > cv2.contourArea(c) < 100000:
            continue
        ''' not needed for now
        # compute the center of the contour
        M = cv2.moments(c)
        # Check if m00 has zero value and continue loop
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            continue
        '''

        # calculate and draw oriented rectangular box
        # we use variable rect later to draw a gripper perimeter
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(image, [box], 0, (0, 0, 255), 2)

        # Retrieve the key parameters of the rotated bounding box
        center = (int(rect[0][0]), int(rect[0][1]))
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = int(rect[2])

        #swap height and width and draw gripper perimeter
        if width > height:
            rotated_box = (rect[0], (50, 120), rect[2])
        else:
            rotated_box = (rect[0], (120, 50), rect[2])

        rotated_box = cv2.boxPoints(rotated_box)
        rotated_box = np.int0(rotated_box)
        cv2.drawContours(image, [rotated_box], 0, (127, 255, 0), 3)


        #transform angle to +-90 degree system
        if width < height:
            angle = 90 - angle
        else:
           angle = -angle

        # draw everything on the image
        # cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
        string_list = ["CenterX=" + str(box[0][0]), "CenterY=" + str(box[0][1]), "alpha=" + str(angle)]
        y_Offset = box[0][1]
        for i in string_list:
            cv2.putText(image, i, (box[0][0], y_Offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            y_Offset += 20

        # show the output image
        cv2.imshow("Image", image)