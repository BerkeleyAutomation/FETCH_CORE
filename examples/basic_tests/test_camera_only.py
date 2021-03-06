""" Use for basic testing of the Fetch."""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# Adjust to change robot's speed.
VEL = 0.5

def save_camera_images():
    # Get some camera images and save them. 
    c_img, d_img = robot.get_img_data()

    cv2.imwrite("rgb_img.png", c_img)
    cv2.imwrite("depth_img.png", d_img)
    print("Saved images...check them out!")

if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose(start_height=0.20, end_height=0.20, velocity_factor=VEL)
    robot.head_start_pose(pan=0.0, tilt=45.0*DEG_TO_RAD)
    
    save_camera_images()

    rospy.spin()
