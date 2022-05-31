from robodk.robolink import *
from robodk.robomath import *

def MoveJ(robot, Target):
    robot.setPoseFrame(Target.Parent())
    robot.MoveJ(Target.Joints())
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
    time.sleep(0.5)

def open_gripper(robot):
    robot.setDO(7,0)
    robot.setDO(6,1)
    time.sleep(0.5)

def open_gripper_soft(robot):
    robot.setDO(7,1)
    robot.setDO(6,1)
    time.sleep(0.5)

def initUR():
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

    # Connect to the robot using default IP
    #success = robot.Connect() # Try to connect once
    success = robot.ConnectSafe() # Try to connect multiple times
    status, status_msg = robot.ConnectedState()
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        print(status_msg)
        raise Exception("Failed to connect: " + status_msg)
    RDK.setRunMode(RUNMODE_RUN_ROBOT)
    
    return robot, RDK

def getRdkTargets(RDK):
    """ Retrieves the targets set in RoboDk and saves them into a dictionary

    Returns:
        Dictionary containing the retrieved targets
    """
    targets =  {}
    #add your targets with targets["Name"] = RDK.Item("RDK_NAME")
    targets["Home"] = RDK.Item('Home_target')
    return targets
    