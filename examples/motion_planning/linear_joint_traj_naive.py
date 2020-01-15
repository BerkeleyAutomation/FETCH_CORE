# FETCH_CORE dependent version of linear joint trajectories. This works but initializes much more

from fetch_core.robot_interface import Robot_Interface
from moveit_python import PlanningSceneInterface
import cv2, os, sys, time, rospy
import numpy as np

#Adi: These values are from the Fetch documentation https://docs.fetchrobotics.com/robot_hardware.html#forces-and-torques
MAX_JOINT_VEL = np.array([0.1, 1.25, 1.45, 1.57, 1.52, 1.57, 2.26, 2.26])
#MAX_JOINT_VEL = MAX_JOINT_VEL / 1.5
def execute_waypoints_trajectory(waypoints, t):
    robot.arm.move_to_waypoints(waypoints, t)

def setBoundaries():
    '''
    This is a fix for the FETCH colliding with itself
    Define ground plane
    This creates objects in the planning scene that mimic the ground
    If these were not in place gripper could hit the ground
    '''
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

def calc_dt(q1, q0):
    return (15.0*np.absolute(np.subtract(q1, q0))) / (8.0*MAX_JOINT_VEL)

def calculate_optimal_dts(waypoints):
    dts = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
    for i, w in enumerate(waypoints):
        if i < len(waypoints)-1:
            dts.append(calc_dt(waypoints[i+1], waypoints[i]))
    optimal_dts = [max(dt) for dt in dts]
    return [sum(optimal_dts[:(i+1)]) for i, dt in enumerate(optimal_dts)]


if __name__ == "__main__":
    robot = Robot_Interface()
    robot.body_start_pose(start_height=0.15, end_height=0.15, velocity_factor=0.4)
    rospy.loginfo("Finished robot starting config")
    robot.close_gripper()
    robot.open_gripper()
    rospy.loginfo("Getting Starting Joint States")
    starting_joint_states = robot.joint_reader.get_joints(['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'])
    fast = True # change to true to use optimal

    waypoints = [starting_joint_states, [0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0],
            [0.155287500830913, 1.29407382142349, 1.30323208557534, -1.57238860721612, 1.65106149726505, -0.688852111001825, 1.40961929449145, 0.51701136699339],
            [0.210575000171710, 1.01735127234069, 1.03566780064439, -3.14477721443224, 1.73132662402381, -1.37770422200365, 1.2484422184766, 1.03402273398678],
            [0.265862499512507, 0.740628723257891, 0.768103515713439, -4.71716582164836, 1.81159175078257, -2.06655633300548, 1.08726514246175, 1.55103410098017],
            [0.321149998853303, 0.463906174175092, 0.500539230782487, -6.28955442886448, 1.89185687754133, -2.7554084440073, 0.926088066446906, 2.06804546797355],
            [0.329099219075529, 0.174213246266179, 1.05811081186774, -10.0336779258358, 1.35928353626556, -1.22300351166094, -1.28255428407097, 4.49824258148513]]
    
    # Brings it back in reverse to the same waypoints
    s = waypoints[::-1] #all but the top waypoint (so the last one in the initial waypoints list)
    waypoints.extend(s)
    
    # Jackson: Optimal DTS is working
    
    t = calculate_optimal_dts(waypoints) if fast else [5*i for i, w in enumerate(waypoints)]
    print("optimal dts? " + str(fast) + ": " + str(t))

    execute_waypoints_trajectory(waypoints, t)
