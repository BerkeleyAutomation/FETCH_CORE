#!/usr/bin/env python
import rospy, actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# Solve race condition issues with joint_state subscribers
from threading import Lock, Thread
import numpy as np

#Jackson: [Credit: Adi for finding the values]. Moves the arm from tucked, to partially out, back to tucked. 
DEFAULT_WAYPOINTS=[
        [0.331960222350222, 0.684251505175216, 0.733581945351227, -2.6916493108771, 0.769654475882819, 4.38075713095153, -1.43642147925985, 4.73268820736189], \
        [0.374525605525235, 0.626130878202896, 0.838277051846567, -2.07213165321071, 1.50637046115835, 0.993503202349398, 0.928070723700663, -1.98752086908437], \
        [0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0]]
DEFAULT_WAYPOINTS = DEFAULT_WAYPOINTS[::-1]

#Adi: These values are from the Fetch documentation https://docs.fetchrobotics.com/robot_hardware.html#forces-and-torques
MAX_JOINT_VEL = np.array([0.1, 1.25, 1.45, 1.57, 1.52, 1.57, 2.26, 2.26])
TORSO_ACTION_SERVER = 'torso_controller/follow_joint_trajectory'
JOINT_ACTION_SERVER = 'arm_with_torso_controller/follow_joint_trajectory'
JOINT_NAMES = names = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
_joint_states = dict()
lock = Lock()

def callback(msg):
    # rospy.loginfo(msg)
    
    # Subscribes to joint_states, returning a list of lists of joint positions (floats)
    lock.acquire()
    for i, name in enumerate(msg.name):
        if i >= len(msg.position):
            continue
        _joint_states[name] = msg.position[i]
    lock.release()

def get_latest_joint_state():
    """
    Returns: A list of the joint values. Values may be None if we do not
        have a value for that joint yet.
    """
    lock.acquire()

    ret = None
    if all(name in _joint_states for name in names):
        ret = [_joint_states[name] for name in names]
    lock.release()
    return ret if ret else None

def calc_dt(q1, q0):
    return (15.0*np.absolute(np.subtract(q1, q0))) / (8.0*MAX_JOINT_VEL)

def calculate_optimal_dts(waypoints):
    dts = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
    for i, w in enumerate(waypoints):
        if i < len(waypoints)-1:
            dts.append(calc_dt(waypoints[i+1], waypoints[i]))
    optimal_dts = [max(dt) for dt in dts]
    return [sum(optimal_dts[:(i+1)]) for i, dt in enumerate(optimal_dts)]

def execute_waypoints_trajectory(waypoints, t):
    ''' 
    Sends waypoint to the _joint_client (arm_with_torso_controller)
    Equivalent to our `def move_to_waypoints(waypoints, t)` method in arm.py
    The advantage of this is that it is in the same file, doesn't open an extra class, and is not going through robot.py which creates another class.
    Args:
        waypoints: A list of lists of 7 joint positions
        t: times it should take to get from waypoint a to waypoint b
    '''
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names.extend(JOINT_NAMES)
    for i, w in enumerate(waypoints):
        point = JointTrajectoryPoint()
        goal.trajectory.points.append(point)
        goal.trajectory.points[i].time_from_start = rospy.Duration(t[i])
        rospy.loginfo("waypoint " + str(i) + " = " +str(waypoints[i]))
        for j, p in enumerate(waypoints[i]):
            goal.trajectory.points[i].positions.append(waypoints[i][j])
            goal.trajectory.points[i].velocities.append(0.0)
            goal.trajectory.points[i].accelerations.append(0.0)
    _joint_client.send_goal(goal)
    _joint_client.wait_for_result(rospy.Duration(10))

def appendwp(lst, wp):
    lastwp = lst[-1]
    print(wp[-1])
    if any([abs(lastwp[j] - wp[j]) > np.pi for j in range(8)]):
        print(lastwp[j])
        appendwp(lst, [(lastwp[j] + wp[j])/2 for j in range(8)])
        appendwp(lst, wp)
    else:
        lst.append(wp)

if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/joint_states", JointState, callback)

    ''' 
    From [ros_docs](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)
    The final addition, rospy.spin() simply keeps your node from exiting until the node has been shutdown. Unlike roscpp, rospy.spin() does not affect the subscriber callback functions, as those have their own threads.
    '''
    starting_joint_states = None
    while starting_joint_states is None:
        starting_joint_states = get_latest_joint_state()
    
    # rospy.loginfo(JOINT_NAMES)
    # rospy.loginfo(str(starting_joint_states))
    
    rospy.loginfo("Setting up action server, arm_with_torso_controller ...")
    global _joint_client 
    _joint_client = actionlib.SimpleActionClient(
        JOINT_ACTION_SERVER, FollowJointTrajectoryAction)
    _joint_client.wait_for_server(timeout=rospy.Duration(10))

    # Waypoints should be the same now
    waypoints = [starting_joint_states]
    waypoints.extend(DEFAULT_WAYPOINTS)
   
    new_wp = [waypoints[0]]

    for w in range(1, len(waypoints)):
        appendwp(new_wp, waypoints[w])

    waypoints = new_wp

    #rospy.loginfo("waypoints: " + str(waypoints)) 

    # Way to get the revese of the waypoints
    waypoints += waypoints[::-1] # puts in the reverse of the same waypoints
                


    # Jackson: Optimal DTS Works, this uses optimal DTS if fast is true
    fast = True # change to true to use optimal
    rospy.loginfo("Using optimal dts? " + ("YES" if fast else "NO"))
    t = calculate_optimal_dts(waypoints) if fast else [5*i for i, w in enumerate(waypoints)]
    # rospy.loginfo("t: " + str(t))

    execute_waypoints_trajectory(waypoints, t)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
