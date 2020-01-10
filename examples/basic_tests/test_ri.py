""" Use for extensive testing of the new Fetch interface. 

Tested and working for: Gazebo simulator and physical robot. :)
"""
from fetch_core.robot_interface import Robot_Interface
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180


if __name__ == "__main__":
    rospy.loginfo("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface(simulation=True)
    rospy.loginfo("Finished initialization!")

    # Adjust the pose to get it in tucked-in position and (somewhat!) tall.
    robot.body_start_pose()

    # # Manually move the robot to interesting configurations. Get images.
    # c_img, d_img = robot.get_img_data()
    # cv2.imwrite("c_img_0.png", c_img)
    # cv2.imwrite("d_img_0.png", d_img)

    # Open and close grippers, twice.
    rospy.loginfo("now opening and closing grippers!")
    robot.close_gripper()
    robot.open_gripper()
    robot.close_gripper()
    robot.open_gripper()

    # # Head start pose (this method works successfully).
    # #robot.head_start_pose()
    # time.sleep(2)

    # c_img, d_img = robot.get_img_data()
    # cv2.imwrite("c_img_1.png", c_img)
    # cv2.imwrite("d_img_1.png", d_img)

    # # But this is awkward for the Fetch. Let's switch back.
    # robot.pan_head(tilt=0.5)
    # time.sleep(1)

    # # Test different pan and tilt (has different abs(min) and abs(max))
    # robot.head.pan_tilt(pan=-1.0, tilt=0.0)
    # time.sleep(1)
    # robot.head.pan_tilt(pan=1.0, tilt=0.0)
    # time.sleep(1)
    # robot.head.pan_tilt(pan=1.0, tilt=-1.0)
    # time.sleep(1)
    # robot.head.pan_tilt(pan=1.0, tilt=1.0)
    # time.sleep(1)
    # robot.head.pan_tilt(pan=0.0, tilt=0.0)

    robot.pan_head(tilt=0.5)

    rospy.loginfo("done, just spinning now ...")
    rospy.spin()
