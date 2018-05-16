""" Test absolute movement for the Fetch's base. 

I tested this by sending the Fetch to 12 locations spread out on the unit circle
from the origin from 

    roslaunch fetch_gazebo simulation.launch

and all of them are able to (first) go to that point, _then_ call
`robot.position_start_pose()` and successfully come back to the starting
position (the origin) along with the angle (which was set at zero). This is
within noise due to the inaccuracy of the robot, i.e. the final angle is often
in the range of (-5,5) instead of exactly 0.

Can insert this in the end of `robot.position_start_pose()` for debugging help

  scale = 180/np.pi
  print("rel_x: {:.3f}".format(rel_x))
  print("rel_y: {:.3f}".format(rel_y))
  print("dist:  {:.3f}".format(dist))
  print("pp[2]: {:.3f}  ({:.1f})".format(pp[2], pp[2]*scale))
  print("desired_facing: {:.3f}  ({:.1f})".format(desired_facing, desired_facing*scale))
  print("desired_theta:  {:.3f}  ({:.1f})".format(desired_theta,  desired_theta*scale))
  print("current_theta:  {:.3f}  ({:.1f})".format(current_theta,  current_theta*scale))
  print("first angle:    {:.3f}  ({:.1f})".format(angle,          angle*scale))
  print("second angle:   {:.3f}  ({:.1f})".format(final_angle,    final_angle*scale))

With the physical robot, the origin seems to be at the charging station so be
careful about sending the robot back there (it's crowded).
"""
from fetch_core.robot_interface import Robot_Interface
from fetch_core.base import Base
import copy, cv2, math, os, sys, time
import numpy as np
import rospy
DEG_TO_RAD = np.pi / 180
np.set_printoptions(suppress=True, precision=4)

def distance_xy(pose1, pose2):
    return np.sqrt( (pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2 )

def get_pose(robot):
    start = copy.deepcopy(robot.base.odom.position)
    yaw = Base._yaw_from_quaternion(robot.base.odom.orientation)
    return np.array([start.x, start.y, yaw])



def simple_tests(robot):
    """Test accuracy of distances.

    Um ... this is REALLY bad.
    """
    rospy.sleep(2)
    pose1 = get_pose(robot)
    rospy.loginfo("testing basic movement with start_pose: {}".format(pose1))

    # Try to go forward 1.0 cm and measure via Fetch's odometry.
    robot.base.go_forward(distance=1.0, speed=0.2)
    rospy.sleep(2)
    pose2 = get_pose(robot)
    rospy.loginfo("pose: {}".format(pose2))
    rospy.loginfo("xy distance: {:.4f}".format(distance_xy(pose1,pose2)))




def test_sequence_rotations(robot):
    """To see accuracy of rotations in physical robot.
    
    PROBLEM: this will hang? The first turn of 120 degrees only goes about 10
    degrees before the program mysteriously hangs (it doesn't exit).
    """
    rospy.loginfo("testing basic movement with start_pose: {}".format(get_pose(robot)))
    rospy.sleep(2)

    deg = 120
    robot.base.turn(deg*DEG_TO_RAD, speed=0.3)
    rospy.loginfo("rotated 120 degrees")
    rospy.loginfo("pose: {}".format(get_pose(robot)))
    rospy.sleep(2) 

    deg = 120
    robot.base.turn(deg*DEG_TO_RAD, speed=0.3)
    rospy.loginfo("rotated 120 more degrees (240 total)")
    rospy.loginfo("pose: {}".format(get_pose(robot)))
    rospy.sleep(2) 

    deg = 120
    robot.base.turn(deg*DEG_TO_RAD, speed=0.3)
    rospy.loginfo("rotated 120 more degrees (360 total), should be back at start")
    rospy.loginfo("pose: {}".format(get_pose(robot)))



def test_return_to_start(robot):
    """Degrees tested successfully: 
    
    0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330
    """
    deg = 0 # switch this number as needed
    print("\ndegree {}".format(deg))
    robot.base.turn(deg*DEG_TO_RAD, speed=0.3)
    robot.base.go_forward(distance=1.0, speed=0.2)
    # Now try and get back to the starting pose.
    robot.position_start_pose() 



if __name__ == "__main__":
    rospy.loginfo("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface()
    robot.body_start_pose()

    #simple_tests(robot)
    #test_sequence_rotations(robot)
    #test_return_to_start(robot)
