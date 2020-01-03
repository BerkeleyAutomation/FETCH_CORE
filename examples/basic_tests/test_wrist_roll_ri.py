""" Test wrist roll of the Fetch.

We noticed that during our tests with poses, the Fetch would have its wrist roll
off by about 90 degrees.
"""
from fetch_core.robot_interface import Robot_Interface
from geometry_msgs.msg import PoseStamped
import cv2, math, os, sys, time
import numpy as np
import rospy
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi


# control speed of the robot
VEL = 0.4 

if __name__ == "__main__":
    rospy.loginfo("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface()
    rospy.loginfo("finished Initializing")
    names = robot.arm_joints.names()

    # For now just explicitly assign height and the tucked position
    # (Comment out these four lines if you've already got it in some good pose.)

    ## robot.torso.set_height(0.2)
    ## joints = [1.3200, 1.3999, -0.1998, 1.7199, 0.0, 1.6600, 0.0]
    ## joints_list = [(x,y) for (x,y) in zip(names,joints)]
    ## robot.arm.move_to_joint_goal(joints_list, velocity_factor=VEL)

    # Let's print out the tucked position just to verify, using joint reader
    rospy.sleep(2)
    joints = robot.joint_reader.get_joints(names=names)
    print("here are our joints from the joint reader:\n{} (last={:.2f})\n".format(joints, joints[-1]*RAD_TO_DEG))

    # Create a pose to go to, at zero degrees
    pose = robot.create_grasp_pose(0.7, 0, 0.7, 0*DEG_TO_RAD, intuitive=True)
    time.sleep(2)
    robot.move_to_pose(pose_name=pose, z_offset=0.0, velocity_factor=VEL) 
    time.sleep(2)
    joints = robot.joint_reader.get_joints(names=names)
    print("here are our joints from the joint reader:\n{} (last={:.2f})\n".format(joints, joints[-1]*RAD_TO_DEG))
    rospy.sleep(3)

    # Rotate gripper roll, assuming the gripper is at a pose where this will work well (i.e. not tucked position)

    joints[6] = 0*DEG_TO_RAD
    joints_list = [(x,y) for (x,y) in zip(names,joints)]
    robot.arm.move_to_joint_goal(joints_list, velocity_factor=VEL)
    joints = robot.joint_reader.get_joints(names=names)
    print("(went to 0 deg) here are our joints from the joint reader:\n{} (last={:.2f})\n".format(joints, joints[-1]*RAD_TO_DEG))
    rospy.sleep(3)

    ## joints[6] = 45*DEG_TO_RAD
    ## joints_list = [(x,y) for (x,y) in zip(names,joints)]
    ## robot.arm.move_to_joint_goal(joints_list, velocity_factor=VEL)
    ## joints = robot.joint_reader.get_joints(names=names)
    ## print("(went to 45 deg) here are our joints from the joint reader:\n{} (last={:.2f})\n".format(joints, joints[-1]*RAD_TO_DEG))
    ## rospy.sleep(3)

    ## joints[6] = 90*DEG_TO_RAD
    ## joints_list = [(x,y) for (x,y) in zip(names,joints)]
    ## robot.arm.move_to_joint_goal(joints_list, velocity_factor=VEL)
    ## joints = robot.joint_reader.get_joints(names=names)
    ## print("(went to 90 deg) here are our joints from the joint reader:\n{} (last={:.2f})\n".format(joints, joints[-1]*RAD_TO_DEG))
    ## rospy.sleep(3)

    ## joints[6] = 0*DEG_TO_RAD
    ## joints_list = [(x,y) for (x,y) in zip(names,joints)]
    ## robot.arm.move_to_joint_goal(joints_list, velocity_factor=VEL)
    ## joints = robot.joint_reader.get_joints(names=names)
    ## print("(went to 0 deg) here are our joints from the joint reader:\n{} (last={:.2f})\n".format(joints, joints[-1]*RAD_TO_DEG))
    ## rospy.sleep(3)

    rospy.spin()
