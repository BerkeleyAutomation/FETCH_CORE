""" Use for basic testing of the Fetch."""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# Adjust to change robot's speed.
VEL = 0.5

if __name__ == "__main__":
    robot = Robot_Skeleton()
   
    robot.body_start_pose(start_height=0.20, end_height=0, velocity_factor=VEL)
    rospy.loginfo("Finished moving robot to start pose.")
    # default: robot.body_start_pose()
    robot.head_start_pose(pan=0.0, tilt=45.0*DEG_TO_RAD)
    
    rospy.spin() # yields activity to other threads