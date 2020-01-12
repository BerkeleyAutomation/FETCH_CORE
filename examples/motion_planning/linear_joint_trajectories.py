from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib, rospy
import numpy as np

#Adi: These values are from the Fetch documentation https://docs.fetchrobotics.com/robot_hardware.html#forces-and-torques
MAX_JOINT_VEL = np.array([0.1, 1.25, 1.45, 1.57, 1.52, 1.57, 2.26, 2.26])

#Jackson: [Credit: Adi for finding the values]. Moves the arm from tucked, to partially out, back to tucked. 
DEFAULT_WAYPOINTS = [[0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0],
            [0.155287500830913, 1.29407382142349, 1.30323208557534, -1.57238860721612, 1.65106149726505, -0.688852111001825, 1.40961929449145, 0.51701136699339],
            [0.210575000171710, 1.01735127234069, 1.03566780064439, -3.14477721443224, 1.73132662402381, -1.37770422200365, 1.2484422184766, 1.03402273398678],
            [0.265862499512507, 0.740628723257891, 0.768103515713439, -4.71716582164836, 1.81159175078257, -2.06655633300548, 1.08726514246175, 1.55103410098017],
            [0.321149998853303, 0.463906174175092, 0.500539230782487, -6.28955442886448, 1.89185687754133, -2.7554084440073, 0.926088066446906, 2.06804546797355],
            [0.329099219075529, 0.174213246266179, 1.05811081186774, -10.0336779258358, 1.35928353626556, -1.22300351166094, -1.28255428407097, 4.49824258148513]]

def execute_waypoints_trajectory(waypoints, t):
    ''' Sends waypoint to the _joint_client, our Arm Joint server

    Equivalent to our `def move_to_waypoints(waypoints, t)` method in arm.py
    The advantage of this is that it is in the same file, doesn't open an extra class, and is not going through robot.py which creates another class.
    
    Args:
        waypoints: A list of lists of 7 joint positions
        t: [TODO] Jackson: I don't understand what t is

    '''
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.extend(['torso_lift_joint'] + ARM_JOINT_NAMES)
        for i, w in enumerate(waypoints):
            point = JointTrajectoryPoint()
            goal.trajectory.points.append(point)
            goal.trajectory.points[i].time_from_start = rospy.Duration(t[i])
            for j, p in enumerate(waypoints[i]):
                goal.trajectory.points[i].positions.append(waypoints[i][j])
                goal.trajectory.points[i].velocities.append(0.0)
                goal.trajectory.points[i].accelerations.append(0.0)
        _joint_client.send_goal(goal)
        _joint_client.wait_for_result(rospy.Duration(10))

def set_torso_height(height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        height = height if height <= 0.4 and height >= 0.0 else 0.0

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append('torso_lift_joint')
        point = JointTrajectoryPoint()
        point.positions.append(height)
        point.time_from_start = rospy.Duration(5)
        goal.trajectory.points.append(point)
        _torso_client.send_goal(goal)
        # Give it 10 seconds to respond with result
        _torso_client.wait_for_result(timeout=rospy.Duration(10))

# TODO: @Adi can you help me detail `calc_dt` and `calculate_optimal_dts` with comments

def calc_dt(q1, q0):
    return (15.0*np.absolute(np.subtract(q1, q0))) / (8.0*MAX_JOINT_VEL)

def calculate_optimal_dts(waypoints):
    dts = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]
    for i, w in enumerate(waypoints):
        if i < len(waypoints)-1:
            dts.append(calc_dt(waypoints[i+1], waypoints[i]))
    optimal_dts = [max(dt) for dt in dts]
    return [sum(optimal_dts[:(i+1)]) for i, dt in enumerate(optimal_dts)]

def get_current_joint_states():
    '''
    Subscribes to joint_states, returning a list of lists of joint positions (floats)
    '''

    def _callback(msg):
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                continue
            _joint_states[name] = msg.position[i]

    _sub = rospy.Subscriber('/joint_states', JointState, _callback)
     # Jackson: Need to initialize node to start running rospy stuff [http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29]
    rospy.init_node('listener', anonymous=True)
    _joint_states = {}

    return _joint_states

def get_joints(self, names):
        """Gets the latest values for a list of joint names.

        Args:
            name: list of strings, the names of the joints whose values we want
                to read.

        Returns: A list of the joint values. Values may be None if we do not
            have a value for that joint yet.
        """
        return [self.get_joint(name) for name in names]

if __name__ == "__main__":
    '''
    CAUTION : MAKE SURE TO RESET YOUR ROBOT WITH `reset_to_base_position.py` BEFORE YOU RUN THIS SCRIPT
    # body_start_pose(start_height=0.15, end_height=0.15, velocity_factor=0.4)
    '''

    # Setup the action servers for the torso and arm joints
    global _torso_client, _joint_client

    rospy.loginfo("Setting up action server for torso...")
    _torso_client = actionlib.SimpleActionClient(
            TORSO_ACTION_SERVER, FollowJointTrajectoryAction)
    _torso_client.wait_for_server(timeout=rospy.Duration(10))

    rospy.loginfo("Setting up action server for arm joints...")
    _joint_client = actionlib.SimpleActionClient(
            JOINT_ACTION_SERVER, FollowJointTrajectoryAction)
    _joint_client.wait_for_server(timeout=rospy.Duration(10))

    rospy.loginfo("Getting Starting Joint States")

    # TODO: Get rid of this dependency on robot.joint.reader and make it return a list of a list


    starting_joint_states = robot.joint_reader.get_joints(['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'])

    # Jackson: Should be a list of a list of starting_joint_states
    rospy.loginfo("[var] starting_joint_states: " + str(starting_joint_states))

    # Jackson: [Credit: Adi] Goes from start, through waypoints, back through waypoints in reverse to start.
    waypoints = starting_joint_states + DEFAULT_WAYPOINTS + DEFAULT_WAYPOINTS[::-1] + starting_joint_states

    
    # Jackson: Optimal DTS Works, this uses optimal DTS if fast is true
    fast = True # change to true to use optimal
    rospy.loginfo("Using Optimal DTS = " + str(fast))
    t = calculate_optimal_dts(waypoints) if fast else [5*i for i, w in enumerate(waypoints)]

    execute_waypoints_trajectory(waypoints, t)
