"""
object detection for robothon use case
"""
from json import tool
from os import system
from tkinter import Frame
from turtle import delay
import cv2
import numpy as np
from datetime import datetime
import math
import pyk4a
from pyk4a import PyK4A, Config, K4AException
from feature_detect import SIFT_matcher
import robotHelper
from robodk import robomath
import time
import sys


def moveToContact(robot,rtde_connection, maxForce):
    global HOST
    global PORT
    
    robot.setPoseFrame(UR5e_Base)
    #print("I am doing a Spiral Script")
    robot.setDO(5,0)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    f = open("moveToContact.script", "rb")   #Robotiq Gripper
    for l in f:
        stringL = l.decode()
        #print(stringL)
        if ("press_force =" in stringL):
            print(l.decode())
            l = ("press_force = "+str(maxForce)).encode()
        s.send(l)
    
    while(not rtde_connection.getDigitalOutState(5)): #wait until DO5 is set
        robot.setJoints([ji * 180.0/pi for ji in rtde_connection.getTargetQ()])
        #time.sleep(0.1)
    
    s.close()
    robot.ConnectSafe()
    time.sleep(0.2)
    robot.setDO(5,0)

def get_image(cam, use_kinect=True, use_live=False):
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

def device_localisation(Scene_Image, scaling=1):
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

    #cv2.imshow("Blurred1", blurred1)
    #cv2.imshow("Blurred2", blurred2)
    #cv2.imshow("Threshold1", threshold1)
    #cv2.imshow("Threshold2", threshold2)

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
        return None
    result_image = cv2.circle(result_image, edge_points[0][0].astype(int), radius=20, color=(0,0,255))
    #Scene_Image = cv2.cvtColor(Scene_Image,cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(resized2)
    cv2.fillPoly(mask, [np.int32(edge_points)], (255,255,255))
    masked = cv2.bitwise_and(resized2,mask)
    masked = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
    masked = cv2.threshold(masked, 80, 255, cv2.THRESH_BINARY)[1]
    kernel = np.ones((int(18*scale_percent/100),int(18*scale_percent/100)),np.uint8) #TODO: Maybe 10 is too radical?
    masked = cv2.morphologyEx(masked, cv2.MORPH_CLOSE, kernel) #close gaps (black points) in the picture with a quite aggressive 20 pixel wide kernel. In many cases this can remove the Velcro
    #masked = cv2.bitwise_not(masked)
    cv2.imshow("ErgebnisMask", cv2.resize(masked, (1920, 1080))) #cv2.resize(newMask, (1920, 1080)))
    contours, hierarchy = cv2.findContours(masked, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    rubber = [contours[i] for i in range(len(contours)) if hierarchy[0][i][3] >= 0] #grab all internal contours
    rubberCenters = []
    if(len(rubber)>=4):
        for h in rubber:
            #print(cv2.contourArea(h))
            if(cv2.contourArea(h)>250*scaling):
                print(cv2.contourArea(h))
                M = cv2.moments(h) #get the contour moment
                rubberCenters.append([int(M["m10"] / M["m00"]),int(M["m01"] / M["m00"])]) #calculate the center
    else:
        return None
    rubberCenters = np.array(orderCounterclockwise(rubberCenters,edge_points[0][0]))
    for idx, p in enumerate(rubberCenters):
        cv2.circle(result_image,p,3,[0,255,0],-1)
        cv2.putText(result_image, str(idx),p, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
    centerPoint = np.divide(np.sum(rubberCenters, axis=0), len(rubberCenters)) #calculate the middle point 
    print(rubberCenters, centerPoint)
    rubberCenters = np.append(rubberCenters, [centerPoint], axis = 0)
    cv2.circle(result_image,centerPoint.astype(int),3,[0,255,0],-1)
    
    cv2.line(result_image,(0,int(result_image.shape[0]/2)),(result_image.shape[1],int(result_image.shape[0]/2)),(255,0,0),1)
    cv2.line(result_image,(int(result_image.shape[1]/2),0),(int(result_image.shape[1]/2),result_image.shape[0]),(255,0,0),1)
    cv2.imshow("Ergebnis", cv2.resize(result_image, (1920, 1080)))
    cv2.setWindowProperty("Ergebnis", cv2.WND_PROP_TOPMOST, 1)
    #cv2.moveWindow("Ergebnis", 1920,0)
    print("Durchlaufzeit SIFT Detection: " + str(datetime.now() - startTime))
    #print("Homographie Matrix: \n{}\n Transformed Edge Points:\n{}\n".format(h_matrix, edge_points))
    
    return rubberCenters*(100/scale_percent)

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

def calculateTransform(robot, cam, numPics, scaling=1):
    cmat = cam.calibration.get_camera_matrix(1)
    dist = cam.calibration.get_distortion_coefficients(1)
    #cmat = np.array([[1.85182827e+03, 0.00000000e+00,1.93774986e+03],[0.00000000e+00, 1.85339297e+03, 1.07281515e+03],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    #dist = np.array([[ 0.09492541, -0.06206038, -0.0012638,   0.00230549, -0.01368713]])
    rubberCenters = []
    i = 0
    while(i<=numPics):
        img = get_image(cam=cam)
        cv2.imshow("Image", cv2.resize(img, (1600,900)))
        cv2.waitKey(100)
        rubberPoints = device_localisation(img,scaling)
        print(rubberPoints)
        if rubberPoints is not None and len(rubberPoints) == 5:
            rubberCenters.append(rubberPoints)
            i+=1
    rubberCenters = np.average(rubberCenters, axis=0)
    
    if rubberCenters is None:
        print("No valid pictures found...")
        return None
    # Calculate Calculator to Camera transformation
    feetHeight = 49
    feetWidth = 124
    # objectpoints = np.array([[0, 0, 0],
    #                         [feetWidth, 0, 0],
    #                         [feetWidth, feetHeight, 0],
    #                         [0, feetHeight, 0]])
    objectpoints = np.array([[-feetWidth/2, -feetHeight/2, 0],
                            [feetWidth/2, -feetHeight/2, 0],
                            [feetWidth/2, feetHeight/2, 0],
                            [-feetWidth/2, feetHeight/2, 0],
                            [0,0,0]])
    _, rvec, tvec = cv2.solvePnP(objectpoints.astype(np.double), rubberCenters.astype(np.double), cmat, dist, flags=cv2.SOLVEPNP_EPNP) #PNP gives us trans and rot to transform from taskboard to camera
    
    R, _ = cv2.Rodrigues(rvec)
    M_CAL2CAM  = np.eye(4)
    M_CAL2CAM[:3,:3] = R
    #Force the matrix to only rotate around Z
    M_CAL2CAM[2,:2], M_CAL2CAM[:2,2] = [0,0], [0,0]
    M_CAL2CAM[:3, 3:] = tvec
    print(M_CAL2CAM)
    # M_CAM2TOOL = np.array([[1,  0, 0,  33.10616],
    #                     [0, 1, 0, 60.94732],
    #                     [0, 0, 1, -53.34756],
    #                     [0,0,0,1]])
    # M_CAM2TOOL = np.array([[0.99999425,  0.00277953, -0.00194188, 32.37808],  #new calib
    #                         [-0.00282741,  0.99968078, -0.02510677, 60.58989],
    #                         [ 0.00187148,  0.02511211,  0.99968289, -53.34756],
    #                         [0,0,0,1]])

    # M_CAM2TOOL = np.array([[1,  0, 0,  32.37808], #new calib with removed rot
    #                     [0, 1, 0,  60.58989],
    #                     [0, 0, 1, -53.34756],
    #                     [0,0,0,1]])

    M_CAM2TOOL = np.array([[1,  0, 0,  33.10616],
                        [0, 1, 0, 60.94732],
                        [0, 0, 1, -53.34756],
                        [0,0,0,1]])

    M_TOOL2BASE = np.transpose(robot.Pose()) #Tool to Base
    #print(M_TOOL2BASE)
    x_offset = 0.3
    y_offset = -20
    calc_corner = np.transpose(np.array(robomath.TxyzRxyz_2_Pose(np.array([x_offset, y_offset,0,0,0,0]))))
    M = M_TOOL2BASE @ M_CAM2TOOL @ M_CAL2CAM
    calc_pose = robomath.Pose_2_TxyzRxyz(np.matmul(M,calc_corner))

    calc_pose[2] = 44.326996 #TODO: fix the offset again?
    print("posetransformm:",calc_pose)
    
    return calc_pose, rubberPoints

def find_calculator(robot,cam):
    pose, _ = calculateTransform(robot,cam,2)
    print(pose)
    rPose = np.array(robomath.Pose_2_TxyzRxyz(robot.Pose()))

    rPose[0] = pose[0] #+ cos(-pose[5])*camOffset[0] - sin(-pose[5])*camOffset[1]
    rPose[1] = pose[1] #+ sin(-pose[5])*camOffset[0] + sin(-pose[5])*camOffset[1]
    rPose[5] = -pose[5] + math.pi #+pi because it cant really turn 180 deg for some reason
    rPose = robomath.TxyzRxyz_2_Pose(rPose)
    cam_offset = robomath.TxyzRxyz_2_Pose([-18,-34,0,0,0,0])
    robot.MoveL(rPose*cam_offset)
    #targets["Second_Home"].setPose(rPose*cam_offset)
    #robotHelper.MoveJ(robot, targets["Second_Home"])
    time.sleep(0.5)
    
    pose, rubberpoints = calculateTransform(robot,cam,2)
    rPose = np.array(robomath.Pose_2_TxyzRxyz(robot.Pose()))

    rPose[0] = pose[0] #+ cos(-pose[5])*camOffset[0] - sin(-pose[5])*camOffset[1]
    rPose[1] = pose[1] #+ sin(-pose[5])*camOffset[0] + sin(-pose[5])*camOffset[1]
    rPose[5] = -pose[5] + math.pi #+pi because it cant really turn 180 deg for some reason
    rPose = robomath.TxyzRxyz_2_Pose(rPose)
    cam_offset = robomath.TxyzRxyz_2_Pose([-32,-40,0,0,0,0])
    targets["Second_Home"].setPose(rPose*cam_offset)
    robot.MoveL(rPose*cam_offset)
    time.sleep(0.5)
    pose, rubberpoints = calculateTransform(robot,cam,2)
    print("pose findCalc:",pose)

    return pose, rubberpoints

def find_batterys(Points, cam , robot, targets):
    ### Move to Picture Position
    robotHelper.MoveJ(robot, targets["Second_Home"])
    time.sleep(1)
    ### Take piture
    img = get_image(cam)

    l1 = Points[0] - Points[1]
    l2 = Points[3] - Points[2]
    #T0 = Points[1] + 1.07 * l1
    #T3 = Points[2] + 1.07 * l2
    #T1 = Points[1] + 0.73 * l1
    #T2 = Points[2] + 0.73 * l2
    # Create the Maske for the 4 Batterys
    T10 = Points[1] + 0.73 * l1
    T20 = Points[2] + 0.73 * l2
    T11 = Points[1] + (0.73 + 0.085) * l1
    T21 = Points[2] + (0.73 + 0.085) * l2
    T12 =  Points[1] + (0.73 + (2*0.085)) * l1
    T22 =  Points[2] + (0.73 + (2*0.085)) * l2
    T13 =  Points[1] + (0.73 + (3*0.085)) * l1
    T23 = Points[2] + (0.73 + (3*0.085)) * l2
    T14 = Points[1] + 1.07 * l1
    T24 = Points[2] + 1.07 * l2

    Corners1 = np.array([T10, T11, T21, T20]).astype(int)
    Corners2 = np.array([T11, T12, T22, T21]).astype(int)
    Corners3 = np.array([T12, T13, T23, T22]).astype(int)
    Corners4 = np.array([T13, T14, T24, T23]).astype(int)

    #cv2.circle(img, (int(T10[0]),int(T10[1])), 5, (255,0,255),-1)
    #cv2.circle(img, (int(T11[0]),int(T11[1])), 5, (255,0,255),-1)
    #cv2.circle(img, (int(T20[0]),int(T20[1])), 5, (255,0,255),-1)
    #cv2.circle(img, (int(T21[0]),int(T21[1])), 5, (255,0,255),-1)
    #Corners = np.array([T0, T1, T2, T3]).astype(int)
    #print("Corners: ", Corners)
    #cv2.drawContours(img, [Corners], 0, (255,0,0), -1)

    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #mask = np.zeros_like(img)
    #cv2.fillPoly(mask, [np.int32(Corners)], (255,255,255))
    #masked = cv2.bitwise_and(img,mask)
    #masked = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
    #masked = cv2.threshold(masked, 80, 255, cv2.THRESH_BINARY)[1]

    #contours, __ = cv2.findContours(masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #print("Contours: ", contours)
    #contours = cv2.drawContours(img, contours, 0 ,(255,0,0),5)
    #kernel = np.ones((int(10),int(10)),np.uint8) #TODO: Maybe 10 is too radical?
    #lower = np.array([0,0,0])
    #upper = np.array([120,255,255])

    mask1 = np.zeros_like(img)
    cv2.fillPoly(mask1, [np.int32(Corners1)], (255,255,255))
    masked1 = cv2.bitwise_and(img,mask1)
    masked1 = cv2.cvtColor(masked1, cv2.COLOR_BGR2GRAY)
    #hsv1 = cv2.inRange(masked1, lower, upper)
    masked1 = cv2.threshold(masked1, 80, 255, cv2.THRESH_BINARY)[1]
    #masked1 = cv2.morphologyEx(masked1, cv2.MORPH_CLOSE, kernel)
    mean1 = np.mean(masked1)
    mask1 = cv2.bitwise_not(mask1)
    #mean11 = cv2.meanStdDev(masked1, mask1)
    #a = []
    #masked11 = np.ma.masked_where(mask1.all == 0 and masked1.all == 0, masked1 )
    #masked11 = np.mean(masked11) 
    
    
    mask2 = np.zeros_like(img)
    cv2.fillPoly(mask2, [np.int32(Corners2)], (255,255,255))
    masked2 = cv2.bitwise_and(img,mask2)
    masked2 = cv2.cvtColor(masked2, cv2.COLOR_BGR2GRAY)
    #hsv2 = cv2.inRange(masked2, lower, upper)
    masked2 = cv2.threshold(masked2, 80, 255, cv2.THRESH_BINARY)[1]
    #masked2 = cv2.morphologyEx(masked2, cv2.MORPH_CLOSE, kernel)
    mean2 = np.mean(masked2)
    
    mask3 = np.zeros_like(img)
    cv2.fillPoly(mask3, [np.int32(Corners3)], (255,255,255))
    masked3 = cv2.bitwise_and(img,mask3)
    masked3 = cv2.cvtColor(masked3, cv2.COLOR_BGR2GRAY)
    #hsv3 = cv2.inRange(masked3, lower, upper)
    masked3 = cv2.threshold(masked3, 80, 255, cv2.THRESH_BINARY)[1]
    #masked3 = cv2.morphologyEx(masked3, cv2.MORPH_CLOSE, kernel)
    mean3 = np.mean(masked3)
    
    mask4 = np.zeros_like(img)
    cv2.fillPoly(mask4, [np.int32(Corners4)], (255,255,255))
    masked4 = cv2.bitwise_and(img,mask4)
    masked4 = cv2.cvtColor(masked4, cv2.COLOR_BGR2GRAY)
    #hsv4 = cv2.inRange(masked4, lower, upper)
    masked4 = cv2.threshold(masked4, 80, 255, cv2.THRESH_BINARY)[1]
    #masked4 = cv2.morphologyEx(masked4, cv2.MORPH_CLOSE, kernel)
    mean4 = np.mean(masked4)

    maxVal = np.max([mean1, mean2, mean3, mean4]) * 2/3 
    limit = max(maxVal, 0.3)
    
    Battery = []

    if mean1 < limit:
        Battery.append(1)
        cv2.drawContours(img, [Corners1], 0, (0,0,255), 3)
    if mean2 < limit:
        Battery.append(2)
        cv2.drawContours(img, [Corners2], 0, (0,0,255), 3)
    if mean3 < limit:
        Battery.append(3)
        cv2.drawContours(img, [Corners3], 0, (0,0,255), 3)
    if mean4 < limit:
        Battery.append(4)
        cv2.drawContours(img, [Corners4], 0, (0,0,255), 3)

    print("Mean Value 1: ", mean1, " 2: ", mean2, " 3: ", mean3, " 4: ", mean4 )
    print("Battery: ", Battery)

    cv2.destroyAllWindows()
    cv2.imshow("Image", cv2.resize(img, (1600,900)))
    cv2.setWindowProperty("Image", cv2.WND_PROP_TOPMOST, 1)
    cv2.moveWindow("Image", 1920,0)
    cv2.waitKey(10)
    #cv2.waitKey(0)
    #cv2.imshow("Mask1", cv2.resize(mask1, (1600,900)))
    #cv2.imshow("Battery2", cv2.resize(masked2, (1600,900)))
    #cv2.imshow("Battery3", cv2.resize(masked3, (1600,900)))
    #cv2.imshow("Battery1", cv2.resize(masked1, (1600,900)))
    #cv2.imshow("Battery4", cv2.resize(masked4, (1600,900)))
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    #cv2.imshow("Battery", cv2.resize(masked, (1600,900)))
    

    return Battery

### Task Positions
def Tool(targets, robot, get ,speedFactor = 1):
    #### Pick the tool form its place or place the tool at the place
    #### for take tool get == True, for place tool get == False ####
    speed = 50*speedFactor
    acceleration = 75*speedFactor
    robot.setSpeed(speed,speed/2,acceleration,acceleration)
    # Gripper
    if get == True:
        robotHelper.open_gripper_soft(robot)
    elif get == False:
        robotHelper.close_gripper(robot)
    # App Tool
    robotHelper.MoveJ(robot, targets["App_Tool"])
    # Gripper
    if get == True:
        robotHelper.open_gripper_soft(robot)
    elif get == False:
        robotHelper.close_gripper(robot)
    # Pick Tool
    robotHelper.MoveJ(robot, targets["Pick_Tool"])
    # Gripper
    if get == True:
        robotHelper.close_gripper(robot)
    elif get == False:
        robotHelper.open_gripper_soft(robot)
    # App Tool
    robotHelper.MoveJ(robot, targets["App_Tool"])

def Reload_tool(targets, robot,speedFactor = 1):
    #### Re positon the tool 

    speed = 50*speedFactor
    acceleration = 75*speedFactor
    robot.setSpeed(speed,speed/2,acceleration,acceleration)
    # Gripper

    # App reload positions
    robotHelper.MoveJ(robot, targets["App_Reload_Tool"])
    robotHelper.MoveJ(robot, targets["Reload_Tool"])
    robotHelper.Snap_gripper(robot,0.05)
    time.sleep(0.5)
    robotHelper.MoveJ(robot, targets["App_Reload_Tool"])



#def Box(targets, robot, Box, speedFactor = 1):
#    #### Move Battery to Box 1 or 2
#    #### for Box 1 Box == True, for Box 2 Box == False
#    speed = 50*speedFactor
#    acceleration = 75*speedFactor
#    robot.setSpeed(speed,speed/2,acceleration,acceleration)
#    ### Box 1
#    if Box == True:
#        robotHelper.close_gripper(robot)
#        robotHelper.MoveL(robot, targets["Box1"])
#        robotHelper.open_gripper(robot)
#    ### Box 2
#    elif Box == False:
#        robotHelper.close_gripper(robot)
#        robotHelper.MoveL(robot, targets["Box2"])
#        robotHelper.open_gripper(robot)


def Cover(targets, robot, speedFactor = 1):
    speed = 50*speedFactor
    acceleration = 75*speedFactor
    robot.setSpeed(speed,speed/2,acceleration,acceleration)
    # Get the Tool
    Tool(targets, robot, True, speed_mult)
    # Close the gripper becaus of the topl
    robotHelper.close_gripper(robot)
    # App Clip
    robotHelper.MoveJ(robot, targets["App_Clip"])
    cv2.destroyAllWindows()
    # Go into Push Position for Clip
    robotHelper.MoveJ(robot, targets["Open_Clip"])
    # Open the Clip that the Cover is free
    #robotHelper.MoveJ(robot, targets["Push_Clip"])
    robotHelper.MoveL_toolframe(robot, 2,0,0)
    robotHelper.Rotate_toolframe(robot, 0, 5*np.pi/180 ,0)
    robotHelper.MoveL_toolframe(robot, 0,0,-10)
    robotHelper.MoveL(robot, targets["App_Clip"])
    # Approche the Cover to flip it
    robotHelper.MoveL(robot, targets["App_Cover"])
    robotHelper.MoveL(robot, targets["Push_Cover"])

    # Move the Cover until it flips
    #robotHelper.MoveL_toolframe(robot, 0,0,-5)
    
    #robotHelper.MoveL_toolframe(robot, 2,0,0)
    robotHelper.MoveL_toolframe(robot, 50,0,0)
    #robotHelper.MoveL_toolframe(robot, 0,0, 5)
    robotHelper.MoveL_toolframe(robot, 0,0,10)

def pushBattery(robot, targets, number):
    # Get Battery1 out
    if(number%2 == 0):
        dir = 1
    else:
        dir = -1
    robotHelper.close_gripper(robot)
    robotHelper.MoveJ(robot, targets["App_Battery"+str(number)])
    robotHelper.MoveL_toolframe(robot, 0,0,20)
    robotHelper.MoveL_toolframe(robot, dir*3,0,0)

    robotHelper.MoveL_toolframe(robot, 0,0,-15)
    robotHelper.MoveL_toolframe(robot, dir*-10,0,0)
    robotHelper.MoveJ(robot, targets["App_Battery"+str(number)])

def pickBattery(robot, targets, number):
    robotHelper.open_gripper(robot)
    robotHelper.MoveJ(robot, targets["App_Pick_Battery"+str(number)])
    robotHelper.MoveJ(robot, targets["Pick_Battery"+str(number)])
    robotHelper.close_gripper_soft(robot)
    robotHelper.MoveJ(robot, targets["App_Pick_Battery"+str(number)])
    # Get Battery1 to box
    robotHelper.MoveJ(robot, targets["Box1"])
    robotHelper.open_gripper(robot)

def ejectBattery(targets, robot, batteries, toolInHand = False,  speedFactor = 1):
    speed = 50*speedFactor
    acceleration = 75*speedFactor
    robot.setSpeed(speed,speed/2,acceleration,acceleration)

    pairs = []
    while len(batteries)>0:
        sel1 = batteries[0]
        sel2 = -1
        for i in range(1,len(batteries)):
            if(sel1%2 != batteries[i]%2):
                sel2 = batteries[i]
                break
        batteries.remove(sel1)
        if(sel2 != -1):
            batteries.remove(sel2)
        pairs.append([sel1,sel2])

    for p in pairs:
        print("Doing Batteries ",p)
        # Get Tool 
        #### Dont forget to take it out !!!
        if(toolInHand == False):
            Tool(targets, robot, True, speed_mult)
            toolInHand = True
       
        # Get Battery1 out
        pushBattery(robot,targets,p[0])
        
        # Get Battery2 out
        if(p[1]!=-1):
            pushBattery(robot,targets,p[1])

        # Move Tool to Position 
        Tool(targets, robot, False, speed_mult)
        toolInHand = False
        # Grip Battery1 
        pickBattery(robot,targets,p[0])

        # Grip Battery2
        if(p[1] != -1):
            pickBattery(robot,targets,p[1])

   

if __name__ == "__main__":
    use_kinect = True       #Use PyK4A (Azure Kinect) for video stream capturing
    use_live = False        #Use OpenCV.VideoCapture() for video stream capturing
    move_frame = True     #Ture = attempt to move real robot
    Simulate = False
    
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
        
        while(True):
            try:
                cam.start()
                break
            except K4AException as e:
                print(e)
                time.sleep(0.5)
    
    elif use_live:
        cam = cv2.VideoCapture(0)

    robot, RDK, Ref_Frame = robotHelper.initUR( Simulate )
    targets = robotHelper.getRdkTargets(RDK)

    speed_mult = 2

    robot.setPoseFrame(robot.PoseFrame())
    robot.setPoseTool(robot.PoseTool())
    robot.setRounding(-1) # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
    robot.setSpeed(50*speed_mult) # Set linear speed in mm/s
    robot.setAcceleration(75*speed_mult)
    robot.setSpeedJoints(50*speed_mult)
    robot.setAccelerationJoints(75*speed_mult)

    robotHelper.MoveJ(robot, targets["Home"])
    time.sleep(1)
    robotHelper.activate_gripper(robot)
    
    if move_frame == True:
        pose, points = find_calculator(robot,cam)
        print("Pose: ", pose)
        print("Points: ", points)
        Ref_Frame.setPose(robomath.TxyzRxyz_2_Pose(pose))
        print("Moved Caculator Frame to: ", pose)
        #robotHelper.MoveJ(robot, targets["Home"])
        # Teache Referenze Frame TxyzRxyz:
        # cartesian [   -59.980772,  -571.301928,    44.326996,    -0.037727,     0.111536,    91.221479 ]



    #input()

    #robotHelper.MoveJ(robot, targets["App_Battery1"])
    #input()
    #sys.exit()
    # Remove the Cover
    Cover(targets, robot, speed_mult)
    #sys.exit()
    cv2.destroyAllWindows()
    #Reload_tool(targets, robot, speed_mult)

    



    batterys = find_batterys(points, cam, robot, targets)
    ejectBattery(targets, robot, batterys, True ,speed_mult)
        

    batterys2 = find_batterys(points, cam, robot, targets)
    ejectBattery(targets, robot, batterys2, False ,speed_mult)

    # Move Home    
    robotHelper.MoveJ(robot, targets["Home"])

    ########### Test Block ################
    #robotHelper.Run_prog(RDK,"Return_Tool")
    #input()
    cv2.destroyAllWindows()
    sys.exit()    
    #######################################
    
        #calculateTransform(robot,cam,1)
        #if cv2.waitKey(1) == ord('q'):
        #    break
        #input("press something to continue")
    
    # # release video stream
    # if use_Azure_Kincet:
    #     Azure_Cam.stop()
    # elif use_video:
    #     cap.release()

    cv2.destroyAllWindows()