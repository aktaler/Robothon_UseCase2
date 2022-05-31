"""
object detection for robothon use case
"""
import cv2
import numpy as np
from datetime import datetime
import math
import pyk4a
from pyk4a import PyK4A, Config
from feature_detect import SIFT_matcher

def get_image(use_kinect, use_live, cam):
    # start video
    if use_kinect:
        capture = cam.get_capture()
        if np.any(capture.color):
            pic = capture.color[:,:,:3]
            pic = np.ascontiguousarray(pic)
            Scene_Image = pic
    elif use_live:
        ret = False
        while not ret:
            # load the image or video feed
            ret, Scene_Image = cam.read()
            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame")
    else:
        # load image from file or video stream
        Scene_Image = cv2.imread("Bilder/Szene/Szene1.jpg", 1)
    return Scene_Image

def device_localisation(Scene_Image):
    """
    Get Image from Camera stream, maybe resize it and pass it to a module
    """

    # start timer to estimate code execution time
    startTime = datetime.now()
    # TODO Accept a list of targets and modify the sift detector to look for all given targets, also classify the objects
    # save target and scene image as grayscale
    Target_Image = cv2.imread('Bilder/Target/Taschenrechner_Target_5.jpg', 0)

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
    blurred1 = cv2.GaussianBlur(resized1, (1, 1), 0)
    blurred2 = cv2.GaussianBlur(resized2, (1, 1), 0)

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
    if(edge_points is None):
        return
    result_image = cv2.circle(result_image, edge_points[0][0].astype(int), radius=20, color=(0,0,255))
    #Scene_Image = cv2.cvtColor(Scene_Image,cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(resized2)
    cv2.fillPoly(mask, [np.int32(edge_points)], (255,255,255))
    masked = cv2.bitwise_and(resized2,mask)
    masked = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
    masked = cv2.threshold(masked, 80, 255, cv2.THRESH_BINARY)[1]
    kernel = np.ones((10,10),np.uint8) #TODO: Maybe 10 is too radical?
    masked = cv2.morphologyEx(masked, cv2.MORPH_CLOSE, kernel) #close gaps (black points) in the picture with a quite aggressive 20 pixel wide kernel. In many cases this can remove the Velcro
    #masked = cv2.bitwise_not(masked)
    cv2.imshow("ErgebnisMask", masked) #cv2.resize(newMask, (1920, 1080)))
    contours, hierarchy = cv2.findContours(masked, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    rubber = [contours[i] for i in range(len(contours)) if hierarchy[0][i][3] >= 0] #grab all internal contours
    rubberCenters = []
    if(len(rubber)==4):
        for h in rubber:
            M = cv2.moments(h) #get the contour moment
            rubberCenters.append([int(M["m10"] / M["m00"]),int(M["m01"] / M["m00"])]) #calculate the center
    else:
        return None
    rubberCenters = orderCounterclockwise(rubberCenters,edge_points[0][0])
    for idx, p in enumerate(rubberCenters):
        cv2.circle(result_image,p,3,[0,255,0],-1)
        cv2.putText(result_image, str(idx),p, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
    cv2.imshow("Ergebnis", cv2.resize(result_image, (1920, 1080)))
    print("Durchlaufzeit SIFT Detection: " + str(datetime.now() - startTime))
    print("Homographie Matrix: \n{}\n Transformed Edge Points:\n{}\n".format(h_matrix, edge_points))
    return rubberCenters
    
def orderCounterclockwise(pts, refpt):
    """ Orders a list of points counterclockwise, starting from the point closest to a reference point

    Args:
        pts: a list of points to order
        refpt: the reference point

    Returns:
        an ordered version of the list
    """
    minDist = np.Inf
    minId = -1
    angs = []
    for i, p in enumerate(pts):
        dist = math.sqrt(math.pow(p[0]-refpt[0],2)+math.pow(p[1]-refpt[1],2)) #calculate distance to reference point
        if dist < minDist: 
            minDist= dist
            minId = i
        centerPoint = np.divide(np.sum(pts, axis=0), len(pts)) #centerpoint of input points (for estimating the rotation point)
        angle = math.atan2(p[1]-centerPoint[1],p[0]-centerPoint[0])
        if angle <= 0:
            angle = (angle*(180/np.pi)*-1)
        else:
            angle = 360-(angle*(180/np.pi))
        angs.append(angle)
    
    normAng = np.mod(np.subtract(angs,angs[minId]),360) #normalize the angle by the angle of the starting point
    return [x for _, x in sorted(zip(normAng, pts))] #return sorted list

if __name__ == "__main__":
    use_kinect = True
    use_live = False
    if use_kinect :
        cam = PyK4A(
            Config(
                color_resolution=pyk4a.ColorResolution.RES_2160P,
                depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
                camera_fps=pyk4a.FPS.FPS_15,
                wired_sync_mode=pyk4a.WiredSyncMode.STANDALONE,
                synchronized_images_only=True,
            )
        )
        cam.start()

    elif use_live:
        cam = cv2.VideoCapture(0)

    while(True):
        img = get_image(use_kinect, use_live,cam)

        device_localisation(img)

        if cv2.waitKey(1) == ord('q'):
            break
    
    # # release video stream
    # if use_Azure_Kincet:
    #     Azure_Cam.stop()
    # elif use_video:
    #     cap.release()

    cv2.destroyAllWindows()