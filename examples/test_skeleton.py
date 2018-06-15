""" Use for basic testing of the Fetch."""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180


def basic_camera_grippers():
    # Get some camera images and save them.
    c_img, d_img = robot.get_img_data()
    cv2.imwrite("c_img_0.png", c_img)
    cv2.imwrite("d_img_0.png", d_img)

    # Open and close grippers, twice.
    print("now opening and closing grippers!")
    robot.close_gripper()
    robot.open_gripper()
    robot.close_gripper()
    robot.open_gripper()


def moving_to_poses():
    pass


if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose()
    robot.head_start_pose()

    basic_camera_grippers()
    moving_to_poses()

    print("done, just spinning now ...")
    rospy.spin()
