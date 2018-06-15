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


def moving_to_poses():
    """Move to this pose which means the gripper almost touches the ground.
    """
    x, y, z             = ( 0.5,  0.0,  0.25)
    rot_x, rot_y, rot_z = ( 0.0, 90.0,  0.0)
    pose0 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)
    rospy.sleep(1)

    # OPTIONAL: play it safe and go to `pose_0_b` first, THEN `pose_0`. This is
    # usually a good idea.
    if True:
        robot.move_to_pose(pose0+'_b', velocity_factor=VEL) 

    robot.move_to_pose(pose0, velocity_factor=VEL) 


if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose()
    robot.head_start_pose()

    #basic_camera_grippers()
    moving_to_poses()

    print("done, just spinning now ...")
    rospy.spin()
