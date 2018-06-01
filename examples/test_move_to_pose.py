""" Test movement of end-effector.

Goal here is to send the robot end-effector to various locations in the task
space and more generally understanding the poses and assigning stuff to the arm.
"""
from fetch_core.robot_interface import Robot_Interface
from geometry_msgs.msg import PoseStamped
import cv2, math, os, sys, time
import numpy as np
import rospy
SCALE = np.pi / 180

if __name__ == "__main__":
    print("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface()
    print("finished Initializing")
    robot.body_start_pose()

    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x = 0.5
    ps.pose.position.y = 0
    ps.pose.position.z = 1
    ps.pose.orientation.w = 1

    pose = robot.create_grasp_pose(0.5, 0, 1, 1)

    print("moving")
    # Check for ability to move to pose
    robot.move_to_pose(pose, 0) 

    print("done moving")
    # Check for reachability via planning TODO

    # Check for reachability via inverse kinematics
    joints = robot.arm.compute_ik(pose_stamped=ps)
    if joints:
        rospy.loginfo('Found IK!\n{}'.format(joints))
    else:
        rospy.loginfo('No IK found.')
