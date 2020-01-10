"""This is how to create screwdrivers in the Gazebo world. It shows how to use a
custom model.

Note that in order for this to work the `model.sdf` file for the screwdriver
needs to refer to the corresopnding `.dae` file in the correct path.
"""
from fetch_core.robot_interface import Robot_Interface
from geometry_msgs.msg import PoseStamped
import cv2, math, os, sys, time
import numpy as np
import rospy
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

import spawn_object_script as ss


# control speed of the robot
VEL = 1.00 

if __name__ == "__main__":
    rospy.loginfo("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface()
    rospy.loginfo("finished Initializing")

    # For now just explicitly assign height and the tucked position
    robot.torso.set_height(0.2)
    names = robot.arm_joints.names()
    joints = [1.3200, 1.3999, -0.1998, 1.7199, 0.0, 1.6600, 0.0]
    joints_list = [(x,y) for (x,y) in zip(names,joints)]
    robot.arm.move_to_joint_goal(joints_list, velocity_factor=VEL)
    rospy.sleep(2)

    # --------------------------------------------------------------------------
    # Now let create the object. In order for this to work we need objects to
    # have different names, but right now the methods only spawn one object with
    # a fixed name, s weo just call the `clean_floor` to delete models, then we
    # can spawn the screwdriver, either from a uniform distribution or a
    # Gaussian distribution.
    # --------------------------------------------------------------------------

    delete_model, spawn_model, object_monitor = ss.setup_delete_spawn_service()
    #ss.spawn_from_uniform(1, spawn_model)
    ss.clean_floor(delete_model, object_monitor)
    ss.spawn_from_gaussian(1, spawn_model)
    print(ss.get_object_list(object_monitor))
    
    rospy.spin()
