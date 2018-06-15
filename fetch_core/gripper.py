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

        self.count = 0
        self.pose_count = 0

        # Justin Huang
        self._client = actionlib.SimpleActionClient(ACTION_SERVER, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10))

        # Daniel: add a fake frame. :( TODO: need to fix and get rid of this ...
        self._create_grasp_pose_fake()
        rospy.sleep(2.0)


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
        """Used for Siemens challenge since we determine grasp pose by looking
        at a camera image.
        
        TODO: Figure out the transform reference frame by looking at camera_info
        """
        # Daniel: changed this upon seeing differences in HSR vs Fetch's coordinate frames
        #pose = self.tl.lookupTransform('odom', 'head_camera_rgb_frame', rospy.Time(0))
        pose = self.tl.lookupTransform('odom', 'fake_head2', rospy.Time(0))

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


    def create_grasp_pose(self, x, y, z, rot):
        """Broadcast given pose and return its name. Used for Siemens challenge.

        Args: 
            x,y,z,rot: all are scalars representing the pose, with `rot` as the
                rotation about the z-axis (in _radians_).
        """
        self.broadcast_poses([x,y,z], rot)
        grasp_name = 'grasp_'+str(self.count)
        self.count += 1
        return grasp_name


    def broadcast_poses(self, position, rot):
        """Broadcast pose, for Siemens challenge,
        
        TODO: figure out unit of depth image: if meters do nothing
        """
        td_points = self.pcm.projectPixelTo3dRay((position[0],position[1]))
        norm_pose = np.array(td_points)
        norm_pose = norm_pose/norm_pose[2]
        norm_pose = norm_pose*(position[2])
        #a = tf.transformations.quaternion_from_euler(ai=-2.355,aj=-3.14,ak=0.0)
        #b = tf.transformations.quaternion_from_euler(ai=0.0,aj=0.0,ak=1.57)
        a = tf.transformations.quaternion_from_euler(
                ai=-2.355, aj=-3.14, ak=0.0)
        b = tf.transformations.quaternion_from_euler(
                ai=0.0, aj=0.0, ak=1.57)
        base_rot = tf.transformations.quaternion_multiply(a,b)
        thread.start_new_thread(self.loop_broadcast,(norm_pose,base_rot,rot))
        rospy.sleep(1.0)


    def loop_broadcast(self, norm_pose, base_rot, rot_z):
        """Loop pose, used for Siemens challenge.
        
        TODO: figure out what to put as config, test this out.
        TODO: I think Zisu said that this has to be the odom frame ... should
        double-check and confirm.

        grasp_i_c: created wrt the odom frame for the object to grasp
        grasp_c: where we actually go to (pretty sure, need to test), since this
            has offset for which the gripper can then "move forward" a bit
        """
        norm_pose,rot = self.compute_trans_to_map(norm_pose,base_rot)
        count = np.copy(self.count)

        while True:
            self.br.sendTransform((norm_pose[0], norm_pose[1], norm_pose[2]),
                    rot,
                    rospy.Time.now(),
                    'grasp_i_'+str(count),
                    'odom')

            # Daniel: replacing this with the last one here so the offset is wrt
            # x coordinate, and is larger (since the movement code moves wrt the
            # wrist roll link which is some extra amount). 

            ##self.br.sendTransform((0.0, 0.0, -0.05), # previously with z = config.gripper length
            ##        tf.transformations.quaternion_from_euler(ai=0.0,aj=0.0,ak=rot_z),
            ##        rospy.Time.now(),
            ##        'grasp_'+str(count),
            ##        'grasp_i_'+str(count))

            # Really lame solution heh
            self.br.sendTransform((0.0, 0.0, 0.0),
                    tf.transformations.quaternion_from_euler(ai=0.0,aj=-np.pi/2.0,ak=0.0),
                    rospy.Time.now(),
                    'grasp_tmp_'+str(count),
                    'grasp_i_'+str(count))
            self.br.sendTransform((0.0, 0.0, 0.0),
                    tf.transformations.quaternion_from_euler(ai=np.pi,aj=0.0,ak=0.0),
                    rospy.Time.now(),
                    'grasp_fetch_'+str(count),
                    'grasp_tmp_'+str(count))
            self.br.sendTransform((-0.150, 0.0, 0.0),
                    # Not sure if we use this, or if we set rot_x=rot_z, then rot_z=0?
                    #tf.transformations.quaternion_from_euler(ai=0.0,aj=0.0,ak=rot_z),
                    tf.transformations.quaternion_from_euler(ai=0.0,aj=0.0,ak=0.0),
                    rospy.Time.now(),
                    'grasp_'+str(count),
                    'grasp_fetch_'+str(count))
 


    # --------------------------------------------------------------------------
    # Solely for fake frames, ugly workaround for Siemens challenge since we
    # need to make fake_head2 to be aligned with the HSR's camera frame for
    # maximum code compatibility.
    # --------------------------------------------------------------------------

    def _create_grasp_pose_fake(self):
        thread.start_new_thread(self._loop_fake, ())

    def _loop_fake(self):
        # Identity rotation is (0,0,0,1) in (x,y,z,w) form, NOT (w,x,y,z)
        position = [0,0,0]
        quat0 = tf.transformations.quaternion_from_euler(ai=-np.pi/2.0, aj=0.0, ak=0.0)
        quat1 = tf.transformations.quaternion_from_euler(ai=0.0, aj=np.pi/2.0, ak=0.0)
        while True:
            self.br.sendTransform(position,
                                  quat0,
                                  rospy.Time.now(),
                                  'fake_head1',
                                  'head_camera_rgb_frame')
            self.br.sendTransform(position,
                                  quat1,
                                  rospy.Time.now(),
                                  'fake_head2',
                                  'fake_head1')



    # --------------------------------------------------------------------------
    # For more intuitive grasp poses, where we define wrt the base link. This is
    # an alternative for creating poses (the other method above uses cameras).
    # --------------------------------------------------------------------------

    def create_grasp_pose_intuitive(self, x, y, z, rot_x, rot_y, rot_z):
        """Broadcast given pose and return its name.

        Args: 
            x,y,z,rot_x,rot_y,rot_z: 6 DoF pose, w/angles in radians.
        """
        self.broadcast_poses_intuitive([x,y,z], [rot_x,rot_y,rot_z])
        pose_name = 'pose_'+str(self.pose_count)
        self.pose_count += 1
        return pose_name


    def broadcast_poses_intuitive(self, position, rot):
        thread.start_new_thread(self.loop_broadcast_intuitive, (position,rot))
        rospy.sleep(2.0)


    def loop_broadcast_intuitive(self, position, rot):
        """The intuitive way to test out poses.

        Specifically, now have position and rotations be points with respect to
        the base_link frame (moves w/robot), so I can directly interpret it.
        Make a pose, pose_0 which is the TARGET, but we will first go to
        pose_0_b since that has an appropriate offset in the x-direction of
        about the gripper length.
        """
        pcount = np.copy(self.pose_count)
        quat = tf.transformations.quaternion_from_euler(ai=rot[0], aj=rot[1], ak=rot[2])
        while True:
            self.br.sendTransform(position,
                                  quat,
                                  rospy.Time.now(),
                                  'pose_'+str(pcount),
                                  'base_link')
            self.br.sendTransform((-0.050, 0, 0),
                                  (0, 0, 0, 1),
                                  rospy.Time.now(),
                                  'pose_'+str(pcount)+'_b',
                                  'pose_'+str(pcount))
