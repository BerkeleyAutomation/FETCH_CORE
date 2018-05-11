import controller_manager_msgs.srv
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tf import TransformListener, TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import rospy
#import config_core as cfg # ?
import numpy as np
from arm import Arm
from arm_joints import ArmJoints
from base import Base
from camera import RGBD
from head import Head
from gripper import Gripper
from torso import Torso


class Robot_Interface(object):
    """ TODO """

    def __init__(self, simulation):
        """Initializes various aspects of the Fetch.
        
        TODOs: get things working, also use `simulation` flag to change ROS
        topic names if necessary (especially for the cameras!).
        """
        rospy.init_node("fetch")
        self.arm = Arm()
        self.arm_joints = ArmJoints()
        self.base = Base()
        self.camera = RGBD()
        self.head = Head()
        self.gripper = Gripper()
        self.torso = Torso()

        # TODO Need something similar for the Fetch. For the HSR this returns
        # the (x,y,z) of the base w.r.t. the map's origin. 
        # https://docs.hsr.io/manual_en/development/base_python_interface.html
        # I think we use the 'odom' topic. Then in `position_start_pose` we need
        # a substitute for `self.omni_base.go_abs(...)` ... again, see HSR docs
        # for detials.
        #self.start_pose = self.omni_base.pose


    def body_start_pose(self):
        """Sets the robot's body to some initial configuration.
        
        The HSR uses `whole_body.move_to_go()` which initializes an appropriate
        posture so that the hand doesn't collide with movement. For the Fetch,
        we can probably set the torso height at zero, and set joints so that the
        robot is in its "tucked" position? (Or perhaps we could set all joints
        to zero, but that results in a fully extended arm out in fromt.)
        """
        raise NotImplementedError()


    def head_start_pose(self):
        """Hard-coded starting pose for the robot's head.
        
        TODO these values were taken from the HSR. The Fetch likely needs to use
        a different pan and tilt.
        """
        self.head.pan_tilt(pan=1.5, tilt=-0.8)


    def position_start_pose(self, offsets=None):
        """Assigns the robot's base to some starting position.
        
        This should be OK if we can figure out how to go to absolute poses with
        the Fetch's base. See comments above.
        """
        raise NotImplementedError()


    def get_img_data(self):
        """Obtain camera and depth image.
        
        Returns:
            Tuple containing RGB camera image and corresponding depth image.
        """
        c_img = self.camera.read_color_data()
        d_img = self.camera.read_depth_data()
        return (c_img, d_img)


    def get_depth(self, point, d_img):
        """Compute mean depth near grasp point.

        NOTE: assumes that we have a simlar `cfg.ZRANGE` as with the HSR. I'm
        not sure where exactly this comes from.
        """
        y, x = int(point[0]), int(point[1])
        z_box = d_img[y-cfg.ZRANGE:y+cfg.ZRANGE, x-cfg.ZRANGE:x+cfg.ZRANGE]
        indx = np.nonzero(z_box)
        z = np.mean(z_box[indx])
        return z


    def get_rot(self, direction):
        """Compute rotation of gripper such that given vector is grasped.

        Currently this directly follows the HSR code as there's nothing
        Fetch-dependent.
        """
        dy, dx = direction[0], direction[1]
        dx *= -1
        if dy < 0:
            dx *= -1
            dy *= -1
        rot = np.arctan2(dy, dx)
        rot = np.pi - rot
        return rot


    def create_grasp_pose(self, x, y, z, rot):
        """ TODO """
        raise NotImplementedError()

        
    def open_gripper(self):
        self.gripper.open()


    def close_gripper(self):
        self.gripper.close()


    def move_to_pose(self, pose_name, z_offset):
        """ TODO """
        raise NotImplementedError()


    def find_ar(self, ar_number):
        """ TODO """
        raise NotImplementedError()


    def pan_head(self, tilt):
        """Adjusts tilt of the robot, AND set pan at zero.
        
        Args: 
            tilt: Value in radians, positive means looking downwards.
        """
        self.head.pan_tilt(pan=0, tilt=tilt)
