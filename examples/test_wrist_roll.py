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
VEL = 1.00 

if __name__ == "__main__":
    rospy.loginfo("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface()
    rospy.loginfo("finished Initializing")

    # For now just explicitly assign height and the tucked position
    robot.torso.set_height(0.2)
    names = robot.arm_joints.names()
    joints = [1.3200, 1.3999, -0.1998, 1.7199, 0.0, 1.6600, 0.0]
    joints_list = [(x,y) for (x,y) in zip(names,joints)]
    robot.arm.move_to_joint_goal(joints_list, velocity_factor=VEL)
    rospy.sleep(2)

    # Let's print out the ticked position just to verify, using joint reader
    joints = robot.joint_reader.get_joints(names=names)
    print("here are our joints from the joint reader:\n{}\n".format(joints))

    # Create a pose to go to
    pose = robot.create_grasp_pose(0.6, 0, 0.7, 0*DEG_TO_RAD, intuitive=True)
    time.sleep(2)
    robot.move_to_pose(pose_name=pose, z_offset=0.0, velocity_factor=VEL) 
    time.sleep(2)
    print("joints from the joint reader:\n{}\n".format(joints))

    # Now with the gripper at a pose, let's rotate the wrist roll. (If you do it
    # in the tucked position, the arm can block wrist rotation.)

    joints[6] = 30*DEG_TO_RAD
    joints_list = [(x,y) for (x,y) in zip(names,joints)]
    robot.arm.move_to_joint_goal(joints_list, velocity_factor=VEL)

    rospy.spin()
