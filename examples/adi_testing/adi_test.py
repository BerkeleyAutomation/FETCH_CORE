"""
Test different movement heuristics for moving the Fetch if it can't reach
something from its existing position.
"""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
import tf
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi


# Adjust to change robot's speed.
VEL = 1.0


def move_from_camera_pixels():
    """Move to this pose as specified by user.
    """
    pass

def get_distance(point):
    return np.sqrt(point[0]**2 + point[1]**2)


#def get_angle(point, quat):
 #   radian_vector = tf.transformations.euler_from_quaternion(quat)
  #  radian_angle = radian_vector[2]
#
 #   if point[0] < 0:
  #      if point[1] < 0:
   #         return  (np.pi/2) - rot_origin_robot() 
    #    else:
    #else:




if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose()
    robot.head_start_pose()
    #move_from_camera_pixels()
    robot.base.turn(-1)
    robot.base.go_forward(1)
    point, quat = robot.gripper.tl.lookupTransform('base_link', 'odom', rospy.Time(0))
    if point[0] < 0:
        radian_angle = -(np.pi - radian_angle)
    print(radian_vector)
    #radian_angles = DEG_TO_RAD * degree_angles[2]
    robot.base.turn(radian_angle)
    print("done, just spinning now ...")
    rospy.spin()

