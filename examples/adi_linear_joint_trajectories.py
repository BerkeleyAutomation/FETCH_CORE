from fetch_core.robot_mpanna import Robot_mpanna
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
    robot = Robot_mpanna()
    # Sets boundaries to avoid self-collision
    setBoundaries()
    robot.body_start_pose(start_height=0.15, end_height=0.15, velocity_factor=0.4)
    rospy.loginfo("Finished robot starting config")
    robot.close_gripper()
    robot.open_gripper()
    rospy.loginfo("Getting Starting Joint States")
    starting_joint_states = robot.joint_reader.get_joints(['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'])
    fast = True # change to true to use optimal

    # rospy.loginfo(starting_joint_states)
    #waypoints = [starting_joint_states, [0.106235115125885, 1.37606352480331, 1.39334696175522, -0.420497336506435, 1.59227563078897, -0.52, 1.47382850504935, 0.0509967387614102], [0.357974887270042, 1.02953832290129, 1.1120673936905, -0.415498501755318, 1.79970946338367, -0.92405834419338, 1.4542944913284, -0.207940361854241], [0.200744649781226, 1.24926751395852, 0.755091164112393, -0.00268870852003267, 1.74339999539584, -0.712255651205343, 0.942508484549711, 0.619291703500511], [0.119794040645698, 1.33871095235714, 0.400098606767764, -0.373629239435118, 1.75305031308584, -0.506757666051433, 1.37533384982841, 0.90522106163889], [0.208202922038813, 1.11368441113305, -0.119901393232236, -0.152885332368672, 2.06432680267215, -0.774439324365845, 0.9890421038736, 1.1651435866182], [0.178054805603152, 1.22748137225951, 0.064635415579831, 0.843113409910906, 0.993640978710236, -1.79310541253395, 0.251641571454754, 2.19863258055732], [0.127289748562299, 0.997175518651635, 0.474512149390198, 1.36311340991091, 0.52005395609246, -1.99322761859398, 0.681872078965692, 2.67465341593641], [0.24639855138333, 1.13795487970897, 0.424511833042091, 1.86994323893161, 0.893667890715115, -2.26734423959981, 0.915675326829185, 2.28335323526018], [0.313885665453301, 0.562922266929196, -0.145252703743405, 2.4988729261165, 1.40382031551758, -2.94597753894417, 0.669219822218754, 2.50281901111157], [0.118429956522941, -0.342627788628036, 0.170802693769544, 3.17717350755562, 1.87710563661761, -3.16746335061948, 1.03917742161299, 2.14931137502374], [0.13951225411896, 0.177372211371964, 0.669570517828946, 3.16871622633316, 2.01004541446168, -3.55499255270217, 1.20856988888469, 1.83257462777608], [0.342009765026439, 0.603033629069347, 0.818687809758623, 3.31057461898144, 0.848715586415808, -4.82148658189959, 1.47040524682231, 1.54549869395757]]
    
    #Part 2:
    #waypoints = [starting_joint_states, [0.18132, 0.249076, 0.153913, 1.47009, 1.48902, -0.125871, -1.74715, -1.45724], 
    #        [0.352103, -0.267906, -0.592047, 1.61856, 1.63002, 0.767981, 0.181624, -1.41734], 
    #        [0.361203, -0.398541, -0.547667, 3.47927, 1.1809745, 1.3205005, 0.276377, -1.45165],
    #        [0.370303, -0.529175, -0.503286, 5.33998, 0.731929, 1.87302, 0.37113, -1.48596]]
    #waypoints = [starting_joint_states, [0.18132, 0.249076, 0.153913, 1.47009, 1.48902, -0.125871, -1.74715, -1.45724]] 
    #Adi: Part 1
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
