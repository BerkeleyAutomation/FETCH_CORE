#!/usr/bin/env python

'''
# Simple Sample to log

Goals

> Log joint angles
> Simplify down our fetch_core library into essentials (eg. we're not doing stuff like init classes)

'''

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion



# Global method callback
def _joint_callback(msg):
    '''
    # Sample output for the callback message when we subscribe to joint_states
    seq: 182094
    stamp:
    secs: 1578709886
    nsecs: 406410945
    frame_id: ''
    name: [l_wheel_joint, r_wheel_joint, torso_lift_joint, bellows_joint, head_pan_joint, head_tilt_joint,
      shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint,
      forearm_roll_joint, wrist_flex_joint, wrist_roll_joint]
    position: [-2.13157057762146, 1.7713346481323242, 0.2267853021621704, 0.092, -0.04775416851043701, 0.01552928824157715, 1.0288139179138183, 0.1768386896057129, 0.3019527820941162, 1.1223817453186036, 0.8019957066299438, 1.3775347105578613, 0.6618088218002319]
    velocity: [-0.141357421875, 1.1920928955078125e-07, 0.0975341796875, 0.04876708984375, -0.00011026859283447266, 0.0016613006591796875, 0.07769775390625, -0.8017578125, 0.83447265625, -0.367919921875, 0.7685546875, -0.06817626953125, 0.341552734375]
    effort: [0.004680633544921875, -0.01263427734375, 216.875, 0.0, -0.0, -0.0, 15.625, -92.0, 32.65625, -19.84375, 12.8828125, 0.0555419921875, 6.52734375]
    
    # Jackson: Able to log joint positions with a global callback, using msg.position
    '''

    rospy.loginfo(msg.name)
    rospy.loginfo(msg.position)

    # This was previous code to put it in a joint_state list inside of a class
    # for i, name in enumerate(msg.name):
    #     if i >= len(msg.position):
    #         continue
        # self._joint_states[name] = msg.position[i]
    

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

if __name__ == '__main__':
    rospy.init_node("dev")

    # Log waypoints async through rospy subscriber
    sub = rospy.Subscriber('/joint_states', JointState, _joint_callback)
    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # Sets boundaries to avoid self-collision
    setBoundaries()

    # TODO: Figure out how to implement adi's linear joint trajectories, specifically optimal DTS
    # TODO: action server move from waypoint to waypoint

    # This is the wrist link not the gripper itself
    gripper_frame = 'wrist_roll_link'
    # Position and rotation of two "wave end poses"
    gripper_poses = [Pose(Point(0.042, 0.384, 1.826),
                          Quaternion(0.173, -0.693, -0.242, 0.657)),
                     Pose(Point(0.047, 0.545, 1.822),
                          Quaternion(-0.274, -0.701, 0.173, 0.635))]

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    while not rospy.is_shutdown():
        for pose in gripper_poses:
            # Finish building the Pose_stamped message
            # If the message stamp is not current it could be ignored
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            # # Set the message pose
            gripper_pose_stamped.pose = pose

            # # Move gripper frame to the pose specified
            # move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            # result = move_group.get_move_action().get_result()

            # if result:
            #     # Checking the MoveItErrorCode
            #     if result.error_code.val == MoveItErrorCodes.SUCCESS:
            #         rospy.loginfo("Hello there!")
            #     else:
            #         # If you get to this point please search for:
            #         # moveit_msgs/MoveItErrorCodes.msg
            #         rospy.logerr("Arm goal in state: %s",
            #                      move_group.get_move_action().get_state())
            # else:
            #     rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()