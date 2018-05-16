""" Test movement of end-effector.

Goal here is to send the robot end-effector to various locations in the task
space.
"""
from fetch_core.robot_interface import Robot_Interface
import cv2, math, os, sys, time
import numpy as np
SCALE = np.pi / 180

print("Initializing our robot (this may take about 10 seconds) ...")
robot = Robot_Interface(simulation=True)
robot.body_start_pose()


