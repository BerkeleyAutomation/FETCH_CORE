#! /usr/bin/env python

import actionlib
import control_msgs.msg
import rospy

import tf
import tf2_ros
import tf2_geometry_msgs

import IPython

import numpy as np
import thread

from image_geometry import PinholeCameraModel as PCM

CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
ACTION_SERVER = 'gripper_controller/gripper_action'


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self, cam):
        # Michael Laskey
        not_read = True
        while not_read:
            try:
                cam_info = cam.read_info_data()
                if(not cam_info == None):
                    not_read = False
            except:
                rospy.logerr('info not recieved')
        self.pcm = PCM()
        self.pcm.fromCameraInfo(cam_info)
        self.br = tf.TransformBroadcaster()
        self.tl = tf.TransformListener()

        # Justin Huang
        self._client = actionlib.SimpleActionClient(ACTION_SERVER, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10))


    def open(self):
        """Opens the gripper.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        self._client.send_goal_and_wait(goal, rospy.Duration(10))


    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal, rospy.Duration(10))


    def compute_trans_to_map(self,norm_pose,rot):
        """TODO: Figure out the transform reference frame by looking at camera_info"""

        pose = self.tl.lookupTransform('map','head_rgbd_sensor_rgb_frame', rospy.Time(0))

        M = tf.transformations.quaternion_matrix(pose[1])
        M_t = tf.transformations.translation_matrix(pose[0])
        M[:,3] = M_t[:,3]


        M_g = tf.transformations.quaternion_matrix(rot)
        M_g_t = tf.transformations.translation_matrix(norm_pose)
        M_g[:,3] = M_g_t[:,3]

        M_T = np.matmul(M,M_g)

        trans = tf.transformations.translation_from_matrix(M_T)

        quat = tf.transformations.quaternion_from_matrix(M_T)

        return trans,quat

    def loop_broadcast(self,norm_pose,base_rot,rot_z):
        norm_pose,rot = self.compute_trans_to_map(norm_pose,base_rot)
        print "NORM POSE ",norm_pose
        count = np.copy(self.count)
        while True:
            self.br.sendTransform((norm_pose[0], norm_pose[1], norm_pose[2]),
                    #tf.transformations.quaternion_from_euler(ai=0.0,aj=0.0,ak=0.0),
                    rot,
                    rospy.Time.now(),
                    'grasp_i_'+str(count),
                    #'head_rgbd_sensor_link')
                    'map')

            """TODO: figure out what to put as config"""
            self.br.sendTransform((0.0, 0.0, -0.05), # previously with z = config.gripper length
                    tf.transformations.quaternion_from_euler(ai=0.0,aj=0.0,ak=rot_z),
                    rospy.Time.now(),
                    'grasp_'+str(count),
                    #'head_rgbd_sensor_link')
                    'grasp_i_'+str(count))

    def broadcast_poses(self,position,rot):
        """TODO: figure out unit of depth image: if meters do nothing"""

        count = 0

        td_points = self.pcm.projectPixelTo3dRay((position[0],position[1]))
        print "DE PROJECTED POINTS ",td_points
        norm_pose = np.array(td_points)
        norm_pose = norm_pose/norm_pose[2]
        norm_pose = norm_pose*(position[2])
        # norm_pose = norm_pose*(cfg.MM_TO_M*position[2])
        print "NORMALIZED POINTS ",norm_pose

        #pose = np.array([td_points[0],td_points[1],0.001*num_pose[2]])
        a = tf.transformations.quaternion_from_euler(ai=-2.355,aj=-3.14,ak=0.0)
        b = tf.transformations.quaternion_from_euler(ai=0.0,aj=0.0,ak=1.57)

        base_rot = tf.transformations.quaternion_multiply(a,b)

        thread.start_new_thread(self.loop_broadcast,(norm_pose,base_rot,rot))

        time.sleep(0.3)

    def create_grasp_pose(self,x,y,z,rot):
        """Broadcast given pose and return its name

        Note: all x, y, z, rot are scalars (so rot =/= all Euler angles or
        quaternions)
        """
        self.broadcast_poses([x,y,z],rot)
        grasp_name = 'grasp_'+str(self.count)
        self.count += 1
        return grasp_name
