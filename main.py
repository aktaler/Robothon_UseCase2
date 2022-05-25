"""
object detection for robothon use case
"""
import cv2
import numpy as np
from datetime import datetime
from feature_detect import SIFT_matcher

"""
Get Image from Camera stream, maybe resize it and pass it to a module
"""

# start timer to estimate code execution time
startTime = datetime.now()

# switch variable for image or video stream
use_video = False

# start video
if use_video:
    cap = cv2.VideoCapture(1)
    ret = False
    while not ret:
        # load the image or video feed
        ret, Scene_Image = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame")

else:
    # load image from file or video stream
    Scene_Image = cv2.imread("Bilder/Szene/Szene1.jpg", 0)

if use_video:
    cap.release()

# TODO Accept a list of targets and modify the sift detector to look for all given targets, also classify the objects
# save target and scene image as grayscale
Target_Image = cv2.imread('Bilder/Target/Rauchmelder_Target_1.jpg', 0)

"""
Preprocessing
"""

# percent by which the image is resized
scale_percent = 40

lst = []
for image in (Target_Image, Scene_Image):
    # calculate the 50 percent of original dimensions
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)

    # dsize
    dsize = (width, height)

    # resize image
    resized = cv2.resize(image, dsize)

    lst.append(resized)

resized1, resized2 = lst

# Blurring
blurred1 = cv2.GaussianBlur(resized1, (5, 5), 0)
blurred2 = cv2.GaussianBlur(resized2, (5, 5), 0)

# Thresholding
threshold1 = cv2.threshold(blurred1, 105, 255, cv2.THRESH_BINARY)[1]
threshold2 = cv2.threshold(blurred2, 105, 255, cv2.THRESH_BINARY)[1]

"""
Display interim images
"""

# cv2.imshow("Blurred1", blurred1)
# cv2.imshow("Blurred2", blurred2)
# cv2.imshow("Threshold1", threshold1)
# cv2.imshow("Threshold2", threshold2)

"""
Different modul functions
"""
result_image, h_matrix = SIFT_matcher(blurred1, blurred2, ContrastThreshold=0.04, EdgeThreshold=100)

"""
Matrix Layout =    (cosAlpha    -sinAlpha   transX
                    sinAlpha    cosAlpha    transY
                    0           0           1)

Display
"""
cv2.imshow("Ergebnis", result_image)
print(h_matrix)
print("Durchlaufzeit SIFT Detection: " + str(datetime.now() - startTime))
cv2.waitKey(0)
