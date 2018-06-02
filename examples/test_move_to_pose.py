""" Test movement of end-effector.

Goal here is to send the robot end-effector to various locations in the task
space and more generally understanding the poses and assigning stuff to the arm.
"""
from fetch_core.robot_interface import Robot_Interface
from geometry_msgs.msg import PoseStamped
import cv2, math, os, sys, time
import numpy as np
import rospy
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

if __name__ == "__main__":
    rospy.loginfo("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface()
    rospy.loginfo("finished Initializing")
    #robot.body_start_pose(start_height=0.15, end_height=0.15)

    # W/out this, we get missing topics when creating poses
    time.sleep(2) 

    # Visualize in rviz to debug positioning and rotation
    #pose0 = robot.create_grasp_pose(1, 0, 0, 0, intuitive=True)
    #time.sleep(2)
    # Ugh, locks??
    #pose1 = robot.create_grasp_pose(1, 0, 0, 180*DEG_TO_RAD, intuitive=True)
    #time.sleep(2)

    # The one we actually move to.
    pose = robot.create_grasp_pose(0.7, 0, 0.5, 0, intuitive=True)
    time.sleep(2)

    # Check for ability to move to pose via motion planning
    print("started moving")
    robot.move_to_pose(pose, z_offset=0) 
    print("finished moving")

    # Check for reachability via inverse kinematics

    ## joints = robot.arm.compute_ik(pose_stamped=ps)
    ## if joints:
    ##     rospy.loginfo('Found IK!\n{}'.format(joints))
    ## else:
    ##     rospy.loginfo('No IK found.')
    rospy.spin()
