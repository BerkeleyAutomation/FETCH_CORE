import controller_manager_msgs.srv
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tf import TransformListener, TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import copy, math, rospy, time
import numpy as np
DEG_TO_RAD = np.pi / 180

from arm import Arm
from arm_joints import ArmJoints
from base import Base
from camera import RGBD
from head import Head
from gripper import Gripper
from torso import Torso
from reader import JointStateReader


class Robot_Skeleton(object):
    """Basic bare-bones solution for the Fetch robot interface.
    
    We recommend extending this class with additional convenience methods based
    on your application needs.
    """

    def __init__(self, simulation=True):
        """Initializes various aspects of the Fetch."""
        rospy.init_node("fetch")
        rospy.loginfo("initializing the Fetch...")
        self.arm = Arm()
        self.arm_joints = ArmJoints()
        self.base = Base()
        self.camera = RGBD()
        self.head = Head()
        self.gripper = Gripper(self.camera)
        self.torso = Torso()
        self.joint_reader = JointStateReader()

        # Tucked arm starting joint angle configuration
        self.names = ArmJoints().names()
        self.tucked = [1.3200, 1.3999, -0.1998, 1.7199, 0.0, 1.6600, 0.0]
        self.tucked_list = [(x,y) for (x,y) in zip(self.names, self.tucked)]

        # Initial (x,y,yaw) position of the robot wrt map origin. We keep this
        # fixed so that we can reset to this position as needed. The HSR's
        # `omni_base.pose` (i.e., the start pose) returns (x,y,yaw) where yaw is
        # the rotation about that axis (intuitively, the z axis). For the base,
        # `base.odom` supplies both `position` and `orientation` attributes.
        start = copy.deepcopy(self.base.odom.position)
        yaw = Base._yaw_from_quaternion(self.base.odom.orientation)
        self.start_pose = np.array([start.x, start.y, yaw])
        rospy.loginfo("...finished initialization!")


    def body_start_pose(self, start_height=0.10, end_height=0.10, velocity_factor=None):
        """Sets the robot's body to some initial configuration.

        Tucks the arm using motion planning. NEVER directly set joints as that
        often leads to collisions.

        Args:
            start_height: Height in meters for Fetch before arm-tuck.
            end_height: Height in meters for Fetch after arm-tuck.
            velocity_factor: controls the speed, closer to 0 means slower,
                closer to 1 means faster. (If 0.0, then it turns into 1.0 for
                some reason.) Values greater than 1.0 are cut to 1.0.
        """
        self.torso.set_height(start_height)
        self.arm.move_to_joint_goal(self.tucked_list, velocity_factor=velocity_factor)
        self.torso.set_height(end_height)


    def head_start_pose(self, pan=0.0, tilt=0.0):
        """Sets the robot's head to some initial configuration.

        Args: 
            pan: Value in radians for head sideways rotation, counterclockwise
                when looking at robot from an aerial view.
            tilt: Value in radians for head up/down movement, positive means
                looking downwards.
        """
        self.head.pan_tilt(pan=pan, tilt=tilt)


    def get_img_data(self):
        """Obtain camera and depth image.
        
        Returns:
            Tuple containing RGB camera image and corresponding depth image.
        """
        c_img = self.camera.read_color_data()
        d_img = self.camera.read_depth_data()
        return (c_img, d_img)


    def create_grasp_pose(self, x, y, z, rot_x, rot_y, rot_z):
        """Creates a pose in the world for the robot's end-effect to go to.
        
        Args:
            x, y, z, rot_x, rot_y, rot_z: A 6-D pose description.
        """
        pose_name = self.gripper.create_grasp_pose_intuitive(
                x, y, z, rot_x, rot_y, rot_z)
        return pose_name


    def move_to_pose(self, pose_name, velocity_factor=None):
        """Moves to a pose.
 
        In the HSR, moved the `hand_palm_link` to the frame named `pose_name` at
        the correct pose. For the Fetch we should be able to extract the pose
        from `pose_name` and then call the Arm's `move_to_pose` method.
        
        Args:
            pose_name: A string name for the pose to go 
            velocity_factor: controls the speed, closer to 0 means slower,
                closer to 1 means faster. (If 0.0, then it turns into 1.0 for
                some reason.) Values greater than 1.0 are cut to 1.0.
        """
        # See: 
        #   http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
        #   https://answers.ros.org/question/256354/does-tftransformlistenerlookuptransform-return-quaternion-position-or-translation-and-rotation/
        # First frame should be the reference frame, use `base_link`, not `odom`.
        point, quat = self.gripper.tl.lookupTransform('base_link', pose_name, rospy.Time(0))

        # See:
        #   https://github.com/cse481wi18/cse481wi18/blob/indigo-devel/applications/scripts/cart_arm_demo.py
        #   https://github.com/cse481wi18/cse481wi18/wiki/Lab-19%3A-Cartesian-space-manipulation
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose = Pose(
                Point(point[0], point[1], point[2]),
                Quaternion(quat[0], quat[1], quat[2], quat[3])
        )

        # See `arm.py` written by Justin Huang
        error = self.arm.move_to_pose(pose_stamped=ps, velocity_factor=velocity_factor)
        if error is not None:
            rospy.logerr(error)


    def open_gripper(self):
        self.gripper.open()


    def close_gripper(self):
        self.gripper.close()
