from robodk.robolink import *
from robodk.robomath import *
import numpy as np

def CalculatePoseFrame2Object(frame, part):
    # Get both poses with respect to the station reference (wrt-world coordinate system)
    framePoseAbs = frame.PoseAbs()
    partPoseAbs = part.PoseAbs()
    # Calculate the pose of the object relative to the reference
    pose = framePoseAbs.inv() * partPoseAbs
    return pose

def getOptimalJoints(robot, target):
    ik = robot.SolveIK_All(target, robot.PoseTool())
    selConfigs = []
    for j in ik:
        conf = robot.JointsConfig(j).list()
        rear  = conf[0] # 1 if Rear , 0 if Front
        lower = conf[1] # 1 if Lower, 0 if Upper (elbow)
        flip  = conf[2] # 1 if Flip , 0 if Non flip (Flip is usually when Joint 5 is negative)
        if rear == 1 and lower == 0 and flip == 1:
            #print("Config: ",conf, j)
            #print("Caculated Joints Postion: ", j)
            selConfigs.append(j)
    #print("Found Configs: ", len(selConfigs))
    selConfigsNp = np.array(selConfigs)
    currJoints = robot.Joints()
    #print("Configs:", selConfigs)
    #print("Current Joint: ",currJoints)
    jointWeights = [4,1,1,3,2,1]
    deltaConfigs = (selConfigsNp[:,:6] - currJoints)**2
    for d in deltaConfigs:
        for i in range(6):
            d[i] = d[i]*jointWeights[i]
    normConfigs = np.sqrt(np.sum(deltaConfigs,axis=1))
    #print("Weights:", normConfigs)
    selJoint = selConfigs[np.argmin(normConfigs)]
    #print("Selected Joint is: ", selJoint)
    return selJoint

def MoveJ(robot, Target, unwind = False):
    global UR5e_Base
    optJoints= getOptimalJoints(robot, CalculatePoseFrame2Object(UR5e_Base,Target))
    if(unwind):
        optJoints[5] = np.array(Target.Joints())[0,5]
    robot.setPoseFrame(Target.Parent())
    robot.MoveJ(optJoints)
    time.sleep(0.1)

def MoveL(robot, Target):
    robot.setPoseFrame(Target.Parent())
    robot.MoveL(Target)
    time.sleep(0.1)

def MoveL_toolframe(robot, x=0,y=0,z=0):
    robot.setPoseFrame(UR5e_Base)
    robot.Joints()
    target_i = TxyzRxyz_2_Pose([x,y,z,0,0,0])
    robot.MoveL(robot.Pose()*target_i)
    time.sleep(0.1)

def Rotate_toolframe(robot, ax=0,ay=0,az=0,speed=40):
    robot.setSpeedJoints(speed)
    robot.Joints()
    target_i = TxyzRxyz_2_Pose([0,0,0,ax,ay,az])
    robot.MoveJ(robot.Pose()*target_i)
    time.sleep(0.1)

def MoveL_baseframe(robot, x=0,y=0,z=0):
    robot.setPoseFrame(UR5e_Base)
    robot.Joints()
    target_i = [x,y,z,0,0,0]
    robot.MoveL(TxyzRxyz_2_Pose(np.add(Pose_2_TxyzRxyz(robot.Pose()), target_i)))
    time.sleep(0.1)

def activate_gripper(robot):
    robot.setDO(6,0)
    time.sleep(0.1)
    robot.setDO(7,0)
    time.sleep(0.1)

    robot.setDO(6,1)
    time.sleep(0.1)
    robot.setDO(6,0)
    time.sleep(0.1)

    robot.setDO(7,1)
    time.sleep(0.1)
    robot.setDO(7,0)
    time.sleep(0.1)

def close_gripper(robot):
    robot.setDO(7,0)
    robot.setDO(6,0)
    time.sleep(0.5)

def close_gripper_soft(robot):
    robot.setDO(7,1)
    robot.setDO(6,0)
    time.sleep(1)

def open_gripper(robot):
    robot.setDO(7,0)
    robot.setDO(6,1)
    time.sleep(0.5)

def open_gripper_soft(robot):
    robot.setDO(7,1)
    robot.setDO(6,1)
    time.sleep(0.5)

def initUR(simulate = False):
    """Initializes UR5e Robodk connection

    Raises:
        Exception: Failed to connect exception

    Returns:
        robot: the robot object
        RDK: the RoboDK object
    """
    global UR5e_Base

    ############## RoboDK Connection ##############
    RDK = Robolink()
    robot = RDK.Item('UR5e')
    UR5e_Base = RDK.Item('UR5e Base')
    Frame = RDK.Item('Caculator_Frame')
    if simulate:
        return robot, RDK, Frame

    # Connect to the robot using default IP
    #success = robot.Connect() # Try to connect once
    success = robot.ConnectSafe() # Try to connect multiple times
    status, status_msg = robot.ConnectedState()
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        print(status_msg)
        raise Exception("Failed to connect: " + status_msg)
    RDK.setRunMode(RUNMODE_RUN_ROBOT)
    
    return robot, RDK, Frame

def getRdkTargets(RDK):
    """ Retrieves the targets set in RoboDk and saves them into a dictionary

    Returns:
        Dictionary containing the retrieved targets
    """
    global UR5e_Base

    targets =  {}
    #add your targets with targets["Name"] = RDK.Item("RDK_NAME")
    # Home Position
    targets["Home"] = RDK.Item('Home_target')
    # Tool Position
    targets["App_Tool"] = RDK.Item('App_Tool')
    targets["Pick_Tool"] = RDK.Item('Pick_Tool')
    # Box Position
    targets["Box1"] = RDK.Item('App_Box1')
    targets["Box2"] = RDK.Item('App_Box2')
    # Cover 
    targets["App_Clip"] = RDK.Item('App_Clip')
    targets["Open_Clip"] = RDK.Item('Open_Clip')
    targets["Push_Clip"] = RDK.Item('Push_Clip')
    targets["App_Cover"] = RDK.Item('App_Cover')
    targets["Push_Cover"] = RDK.Item('Push_Cover')
    # Battery
    targets["App_Battery1"] = RDK.Item('App_Battery1')
    targets["App_Battery2"] = RDK.Item('App_Battery2')
    targets["App_Battery3"] = RDK.Item('App_Battery3')
    targets["App_Battery4"] = RDK.Item('App_Battery4')
    targets["Pick_Battery1"] = RDK.Item('Pick_Battery1')
    targets["Pick_Battery2"] = RDK.Item('Pick_Battery2')
    targets["Pick_Battery3"] = RDK.Item('Pick_Battery3')
    targets["Pick_Battery4"] = RDK.Item('Pick_Battery4')
    targets["App_Pick_Battery1"] = RDK.Item('App_Pick_Battery1')
    targets["App_Pick_Battery2"] = RDK.Item('App_Pick_Battery2')
    targets["App_Pick_Battery3"] = RDK.Item('App_Pick_Battery3')
    targets["App_Pick_Battery4"] = RDK.Item('App_Pick_Battery4')

    UR5e_Base = RDK.Item('UR5e Base')
    return targets
    