""" Use for extensive testing of the new Fetch interface. 

Note, when you start the Fetch simulator and it tucks its arm, then we can query
the joint angles and then use those to assign the joints explicitly.

daniel@daniel-ubuntu-mac:~/FETCH_CORE/fetch_core$ rostopic echo -n 1 /joint_states
header: 
  seq: 16864
  stamp: 
    secs: 168
    nsecs: 802000000
  frame_id: ''
name: ['l_wheel_joint', 'r_wheel_joint', 'torso_lift_joint', 'bellows_joint', 'head_pan_joint', 'head_tilt_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint', 'l_gripper_finger_joint', 'r_gripper_finger_joint']
position: [0.09047392733343518, -0.13369358778150442, -2.18697800211842e-08, 0.0066365559473703555, 0.0010378453117310116, 0.002299883010696391, 1.3200038100597533, 1.3999852523364043, -0.1998457015332482, 1.7199699973496578, 3.346686083816053e-06, 1.6600001951658108, -3.4037209628579035e-06, 0.050027779840489575, 0.05006007057851839]
velocity: [4.942944851125051e-05, 0.000546069923973645, 5.6604182359433695e-05, -4.183390087798936e-05, -0.0012390733157181268, 0.0015232134186997448, -0.0003044367844006533, -0.00011788150787464381, -0.00013868005727169218, -0.00015571185110861447, -0.0002791236283017248, 3.747518656475711e-05, -6.649689469959841e-06, 0.008873553536818176, 0.008788044595090846]
effort: [0.0, 0.0, 0.0, -1.6580958730431885, -0.0016732346154877927, -0.07356613057693767, -0.04681804189616147, 11.783382476263089, -0.7641002562392838, 18.34901860857664, 0.23468216503751832, -0.3640518115793623, 0.022728049907019026, -9.967709261971184, -10.032290738028816]
---
"""

from fetch_core.robot_interface import Robot_Interface
import cv2, os, sys, time
import numpy as np


if __name__ == "__main__":
    print("Initializing our robot (this may take about 10 seconds) ...")
    robot = Robot_Interface(simulation=True)

    # Manually move the robot to interesting configurations. Get images.
    c_img, d_img = robot.get_img_data()
    cv2.imwrite("c_img.png", c_img)
    cv2.imwrite("d_img.png", d_img)

    # Open and close grippers, twice.
    robot.close_gripper()
    robot.open_gripper()
    robot.close_gripper()
    robot.open_gripper()

    # Head start pose
    robot.head_start_pose()
    time.sleep(2)

    # But this is awkward for the Fetch. Let's switch back.
    robot.pan_head(tilt=0.5)
    time.sleep(2)
