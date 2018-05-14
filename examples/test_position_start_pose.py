""" Test absolute movement for the Fetch's base. 

I tested this by sending the Fetch to 12 locations spread out on the unit circle
from the origin from 

    roslaunch fetch_gazebo simulation.launch

and all of them are able to (first) go to that point, _then_ call
`robot.position_start_pose()` and successfully come back to the starting
position (the origin) along with the angle (which was set at zero). This is
within noise due to the inaccuracy of the robot, i.e. the final angle is often
in the range of (-5,5) instead of exactly 0.
"""
from fetch_core.robot_interface import Robot_Interface
import cv2, math, os, sys, time
import numpy as np
SCALE = np.pi / 180

print("Initializing our robot (this may take about 10 seconds) ...")
robot = Robot_Interface(simulation=True)
robot.body_start_pose()

# Tested successfully: 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330
deg = 330 # switch this number as needed
print("\ndegree {}".format(deg))
robot.base.turn(deg*SCALE, speed=0.3)
robot.base.go_forward(distance=1.0, speed=0.2)
# Now try and get back to the starting pose.
robot.position_start_pose() 


# Can insert this in the end of `robot.position_start_pose()` for debugging help
#scale = 180/np.pi
#print("rel_x: {:.3f}".format(rel_x))
#print("rel_y: {:.3f}".format(rel_y))
#print("dist:  {:.3f}".format(dist))
#print("pp[2]: {:.3f}  ({:.1f})".format(pp[2], pp[2]*scale))
#print("desired_facing: {:.3f}  ({:.1f})".format(desired_facing, desired_facing*scale))
#print("desired_theta:  {:.3f}  ({:.1f})".format(desired_theta,  desired_theta*scale))
#print("current_theta:  {:.3f}  ({:.1f})".format(current_theta,  current_theta*scale))
#print("first angle:    {:.3f}  ({:.1f})".format(angle,          angle*scale))
#print("second angle:   {:.3f}  ({:.1f})".format(final_angle,    final_angle*scale))
