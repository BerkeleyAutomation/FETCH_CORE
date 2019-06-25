""" Use for basic picking up of objects, assuming hard-coded positions."""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# Adjust to change robot's speed.
VEL = 0.4


def grab_item():
    """Assuming there's an object on the ground a known distance from the
    base_link of the Fetch, we move the end-effector there and grab it. 
    
    Assumes, of course, that the object is reachable. Moves via explicit
    coordinates; we need to know to go this amount in the x direction, etc.

    I always play it safe and go to `pose_0_b` first, THEN `pose_0` (so the
    fetch gripper lowers itself), and so on for poses 1, 2, etc.
    """
    x, y, z             = ( 0.5,  0.0,  0.5)
    rot_x, rot_y, rot_z = ( 0.0, 90.0,  0.0)
    pose0 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)
    rospy.sleep(1)

    # Our actual target position
    x, y, z             = ( 0.5,  0.0,  0.23)
    rot_x, rot_y, rot_z = ( 0.0, 90.0,  0.0)
    import ipdb; ipdb.set_trace()
    pose1 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)
    rospy.sleep(1)

    # Move to poses, adjust height as needed.
    robot.move_to_pose(pose0, velocity_factor=VEL) 
    rospy.loginfo("Just moved to pose: {}".format(pose0))
    robot.torso.set_height(0.15)
    robot.move_to_pose(pose1+'_b', velocity_factor=VEL) 
    rospy.loginfo("Just moved to pose: {}_b".format(pose1))
    robot.move_to_pose(pose1, velocity_factor=VEL) 

    # Grip object
    robot.close_gripper(width=0.06)
    rospy.sleep(1)

    # Move back up to the original pose
    robot.move_to_pose(pose0, velocity_factor=VEL) 


if __name__ == "__main__":
    robot = Robot_Skeleton()
    # set torso height high to start to hopefully avoid collisions with base
    robot.body_start_pose(start_height=0.25, end_height=0.25, velocity_factor=VEL)
    robot.head_start_pose(pan=0.0, tilt=0.0)
    rospy.loginfo("Finished robot body and head start poses.")
    robot.close_gripper(width=0.04)
    robot.open_gripper()

    grab_item()

    print("done, just spinning now ...")
    rospy.spin()
