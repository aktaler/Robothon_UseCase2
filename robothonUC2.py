"""
object detection for robothon use case
"""
import cv2
import numpy as np
from datetime import datetime

import pyk4a
from pyk4a import PyK4A, Config
from feature_detect import SIFT_matcher


def device_localisation():
    """
    Get Image from Camera stream, maybe resize it and pass it to a module
    """

    # start timer to estimate code execution time
    startTime = datetime.now()

    # switch variable for image or video stream
    use_Azure_Kincet = False
    use_video = False

    if use_Azure_Kincet and use_video:
        Azure_Cam = PyK4A(
            Config(
                color_resolution=pyk4a.ColorResolution.RES_2160P,
                depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
                camera_fps=pyk4a.FPS.FPS_15,
                wired_sync_mode=pyk4a.WiredSyncMode.STANDALONE,
                synchronized_images_only=True,
            )
        )
        Azure_Cam.start()

    elif use_video:
        cap = cv2.VideoCapture(0)

    else:
        # load image from file or video stream
        Scene_Image = cv2.imread("Bilder/Szene/Szene1.jpg", 1)

    while True:
        # start timer to estimate code execution time
        startTime = datetime.now()
        # start video
        if use_Azure_Kincet and use_video:
            capture = Azure_Cam.get_capture()
            if np.any(capture.color):
                pic = capture.color[:,:,:3]
                pic = np.ascontiguousarray(pic)
                Scene_Image = pic

        elif use_video:
            ret = False
            while not ret:
                # load the image or video feed
                ret, Scene_Image = cap.read()
                # if frame is read correctly ret is True
                if not ret:
                    print("Can't receive frame")

        # TODO Accept a list of targets and modify the sift detector to look for all given targets, also classify the objects
        # save target and scene image as grayscale
        Target_Image = cv2.imread('Bilder/Target/Taschenrechner_Target_1.jpg', 0)

        """
        Preprocessing
        """

        # percent by which the image is resized
        scale_percent = 100

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
        blurred1 = cv2.GaussianBlur(resized1, (3, 3), 0)
        blurred2 = cv2.GaussianBlur(resized2, (3, 3), 0)

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
        result_image, h_matrix, edge_points = SIFT_matcher(blurred1, blurred2, ContrastThreshold=0.003, EdgeThreshold=150)

        """
        Matrix Layout =    (cosAlpha    -sinAlpha   transX
                            sinAlpha    cosAlpha    transY
                            0           0           1)
        
        Display
        """

        # cv2.imshow("Ergebnis", cv2.resize(result_image, (1920, 1080)))

        print("Durchlaufzeit SIFT Detection: " + str(datetime.now() - startTime))
        print("Homographie Matrix: \n{}\n Transformed Edge Points:\n{}\n".format(h_matrix, edge_points))




        if cv2.waitKey(1) == ord('q'):
            break


    # release video stream
    if use_Azure_Kincet:
        Azure_Cam.stop()
    elif use_video:
        cap.release()

    cv2.destroyAllWindows()