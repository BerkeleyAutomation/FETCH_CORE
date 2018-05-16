""" Use for extensive testing of the new Fetch interface. 

Tested and working for: Gazebos imulator and physical robot. :)
"""
from fetch_core.robot_interface import Robot_Interface
import cv2, os, sys, time
import numpy as np


if __name__ == "__main__":
    print("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface(simulation=True)

    # Adjust the pose to get it in tucked-in position and up tall.
    robot.body_start_pose()

    # Manually move the robot to interesting configurations. Get images.
    c_img, d_img = robot.get_img_data()
    cv2.imwrite("c_img_0.png", c_img)
    cv2.imwrite("d_img_0.png", d_img)

    # Open and close grippers, twice.
    robot.close_gripper()
    robot.open_gripper()
    robot.close_gripper()
    robot.open_gripper()

    # Head start pose
    robot.head_start_pose()
    time.sleep(2)

    c_img, d_img = robot.get_img_data()
    cv2.imwrite("c_img_1.png", c_img)
    cv2.imwrite("d_img_1.png", d_img)

    # But this is awkward for the Fetch. Let's switch back.
    robot.pan_head(tilt=0.5)
    time.sleep(2)
