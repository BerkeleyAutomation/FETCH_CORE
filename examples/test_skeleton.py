""" Use for basic testing of the Fetch."""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# Adjust to change robot's speed.
VEL = 1.0


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


def moving_to_poses_1():
    """Moving to two poses, by defining two poses and moving in sequence."""
    # Pose 1, move here to go above target
    x, y, z             = ( 0.5,  0.0,  0.45)
    rot_x, rot_y, rot_z = ( 0.0, 90.0,  0.0)
    pose1 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)

    # Pose 2, then move here to go down, almost touching the ground
    x, y, z             = ( 0.5,  0.0,  0.25)
    rot_x, rot_y, rot_z = ( 0.0, 90.0,  0.0)
    pose2 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)

    offsets = [ 0.0,  0.0,  0.0]
    rospy.sleep(1)
    robot.move_to_pose(pose1, offsets, velocity_factor=VEL) 
    rospy.sleep(1)
    robot.move_to_pose(pose2, offsets, velocity_factor=VEL) 


def moving_to_poses_2():
    """Same as moving to two poses, but done by making one pose and having an
    offset w.r.t the base link."""
    x, y, z             = ( 0.5,  0.0,  0.25)
    rot_x, rot_y, rot_z = ( 0.0, 90.0,  0.0)
    pose1 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)

    offsets = [ 0.0,  0.0,  0.20]
    rospy.sleep(1)
    robot.move_to_pose(pose1, offsets, reference_frame='base_link', velocity_factor=VEL) 
    offsets = [ 0.0,  0.0,  0.0]
    rospy.sleep(1)
    robot.move_to_pose(pose1, offsets, reference_frame='base_link', velocity_factor=VEL) 


if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose()
    robot.head_start_pose()

    #basic_camera_grippers()
    #moving_to_poses_1()
    #moving_to_poses_2()

    print("done, just spinning now ...")
    rospy.spin()
