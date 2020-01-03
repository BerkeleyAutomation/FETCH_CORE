""" Use for basic testing of the Fetch."""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# Adjust to change robot's speed.
VEL = 0.5


def move_grippers(times):
    print("now opening and closing grippers!")
    for i in range(times):
        robot.close_gripper()
        robot.open_gripper()

def moving_to_poses():
    """Move to this pose which means the gripper almost touches the ground.

    This moves via explicit coordinates, basically we need to know to go this
    amount in the x direction, etc.
    """
    x, y, z             = ( 0.6,  0.0,  0.8)
    rot_x, rot_y, rot_z = (90.0,  0.0,  0.0)
    pose0 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)
    print("Just created pose: {}".format(pose0))
    rospy.sleep(1)

    # OPTIONAL: play it safe and go to `pose_0_b` first, THEN `pose_0`. This is
    # usually a good idea.
    #if True:
    #    robot.move_to_pose(pose0+'_b', velocity_factor=VEL) 

    robot.move_to_pose(pose0, velocity_factor=VEL) 


def get_arm_straight():
    """Move robot so its arm extends outwards.

    I used this to test if we can reset the zero joints, according to Fetch
    support instructions.
    """
    robot.torso.set_height(0.20)
    zeros = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    zeros_list = [(x,y) for (x,y) in zip(robot.arm_joints.names(), zeros)]
    robot.arm.move_to_joint_goal(zeros_list, velocity_factor=0.3)

# IMAGE PROCESSING

def get_cam_output():
    # Gets the depth and rgb images set under ../fetch_core/camera.py
    c_img, d_img = robot.get_img_data()
    d_img = depth_to_3ch(d_img, 1400)
    d_img = depth_3ch_to_255(d_img)
    cv2.imwrite("rgb_img.png", c_img)
    cv2.imwrite("depth_img.png", d_img)

# Scales grayscale to RGB
def depth_to_3ch(d_img, cutoff):
    w,h = d_img.shape
    n_img = np.zeros([w, h, 3])
    d_img = d_img.flatten()
    d_img[d_img>cutoff] = 0.0
    d_img = d_img.reshape([w,h])
    for i in range(3):
        n_img[:, :, i] = d_img
    return n_img

# scales it up to 255
def depth_3ch_to_255(d_img):
    d_img = 255.0/np.max(d_img)*d_img
    d_img = np.array(d_img, dtype=np.uint8)
    for i in range(3):
        d_img[:, :, i] = cv2.equalizeHist(d_img[:, :, i])
    return d_img    

if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose(start_height=0.20, end_height=0.20, velocity_factor=VEL)
    robot.head_start_pose(pan=0.0, tilt=45.0*DEG_TO_RAD)
    #moving_to_poses()
    move_grippers(times=2) # grippers
    get_cam_output() # depth_raw and rgb_raw output

    # get_arm_straight()

    rospy.spin()
