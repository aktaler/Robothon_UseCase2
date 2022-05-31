"""
script for detecting features in two given images, one is the target one is the scene, and try to match the features in
the target to the scene and draw a box around
TODO implement a return function for the results
"""


import numpy as np
import cv2
from datetime import datetime
from matplotlib import pyplot as plt

def SIFT_matcher(img1, img2, ContrastThreshold, EdgeThreshold):
    MIN_MATCH_COUNT = 10
    # Initiate SIFT detector
    detector = cv2.SIFT_create(contrastThreshold=ContrastThreshold, edgeThreshold=EdgeThreshold)

    # find the keypoints and descriptors with SIFT
    kp1, des1 = detector.detectAndCompute(img1, None)
    kp2, des2 = detector.detectAndCompute(img2, None)

    """img1_copy = img1
    img2_copy = img2
    img1_copy = cv2.drawKeypoints(img1, kp1, img1_copy, color=(255, 0, 0))
    img2_copy = cv2.drawKeypoints(img2, kp2, img2_copy, color=(255, 0, 0))
    #
    plt.imshow(img1_copy)
    plt.show()
    plt.imshow(img2_copy)
    plt.show()"""

    FLANN_INDEX_LSH = 0
    # ORB index params
    # index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=4, key_size=6, multi_probe_level=1)
    # SIFT index params
    index_params = dict(algorithm=FLANN_INDEX_LSH, trees=7)
    search_params = dict(checks=100)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1, des2, k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m, n in matches:
        if m.distance < 0.67 * n.distance:
            good.append(m)

    M = None

    print("Anzahl good matches: {}".format(len(good)))
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        w, h = img1.shape[::-1]
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, M)

        img2 = cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

    else:
        print("Not enough matches are found - %d/%d" % (len(good), MIN_MATCH_COUNT))
        matchesMask = None

    draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                       singlePointColor=None,
                       matchesMask=matchesMask,  # draw only inliers
                       flags=2)

    img3 = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)

    """# Plot with Mathplot
    plt.imshow(img3, 'gray'), plt.show()"""

    return img2, M, dst

if __name__ == "__main__":
    img1 = cv2.imread('Bilder/Target/Taschenrechner_Target_3.jpg', 0)
    img2 = cv2.imread("Bilder/Szene/Szene1.jpg", 0)
    SIFT_matcher(img1, img2)