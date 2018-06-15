"""Use to test the camera to base frame mapping."""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# Adjust to change robot's speed.
VEL = 1.0


def move_from_camera_pixels():
    """Move to this pose as specified by user.
    """
    pass


if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose()
    robot.head_start_pose()
    move_from_camera_pixels()
    print("done, just spinning now ...")
    rospy.spin()
