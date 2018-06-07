""" Test movement of end-effector.

Goal here is to send the robot end-effector to various locations in the task
space and more generally understanding the poses and assigning stuff to the arm.

Note the velocity factor that we've added here, for speed.
"""
from fetch_core.robot_interface import Robot_Interface
from geometry_msgs.msg import PoseStamped
import cv2, math, os, sys, time
import numpy as np
import rospy
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi
VEL = 0.20 # control speed of the robot


def debug_pose_location_rviz():
    """TODO: figure out locking issues, and what rotations mean"""
    pose0 = robot.create_grasp_pose(1, 0, 0, 0, intuitive=True)
    time.sleep(2)
    pose1 = robot.create_grasp_pose(1, 0, 0, 90*DEG_TO_RAD, intuitive=True)
    time.sleep(2)


def test_motion_planning():
    """It's helpful if only one pose is created to move to."""
    pose = robot.create_grasp_pose(0.6, 0, 0.7, 0*DEG_TO_RAD, intuitive=True)
    time.sleep(2)
    robot.move_to_pose(pose_name=pose, z_offset=0.0, velocity_factor=VEL) 


def test_inverse_kinematics():
    """TODO: we haven't tested this"""
    joints = robot.arm.compute_ik(pose_stamped=ps)
    if joints:
        rospy.loginfo('Found IK!\n{}'.format(joints))
    else:
        rospy.loginfo('No IK found.')  


if __name__ == "__main__":
    rospy.loginfo("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface()
    rospy.loginfo("finished Initializing")

    # Height matters. We go to `start_height`, tuck, then `end_height`.
    robot.body_start_pose(start_height=0.20, end_height=0.20, velocity_factor=VEL)

    # w/out this, we get missing topics when creating poses
    time.sleep(2) 

    # Now test! Comment out as desired.

    #debug_pose_location_rviz()
    test_motion_planning()
    #test_inverse_kinematics()

    rospy.spin()
