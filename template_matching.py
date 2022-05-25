import cv2
import imutils
import numpy as np

def TemplateMatch(template, image, mask):
    result = cv2.matchTemplate(image, template, cv2.TM_CCORR_NORMED, mask=mask)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    return minVal, minLoc

def RotationSearch(template, image):
    MAXROTATION = 360
    ROTATIONSTEPS = [10, 5, 1]
    STARTANGLE = 0
    best_result = [1, 0, 0]

    for rotationstep in ROTATIONSTEPS:

        for angle in np.arange(STARTANGLE, MAXROTATION, rotationstep):
            rotated = imutils.rotate_bound(template, angle)

            # Calc Mask for template Matching, to ignore the black edges from rotation
            cnts = cv2.findContours(rotated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            # grab the largest contour, then draw a mask for the target template
            c = max(cnts, key=cv2.contourArea)
            mask = np.zeros(rotated.shape, dtype=np.uint8)
            cv2.drawContours(mask, [c], -1, 255, -1)

            w, h = mask.shape
            data = np.zeros((h, w, 3), dtype=np.uint8)

            Val, Loc = TemplateMatch(rotated, image, mask)
            if Val < best_result[0]:
                best_result[0] = Val
                best_result[1] = Loc
                best_result[2] = angle
            print(rotationstep)
    return best_result
