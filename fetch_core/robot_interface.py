import controller_manager_msgs.srv
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tf import TransformListener, TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import copy, math, rospy
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

        # Tucked arm starting joint angle configuration
        self.tucked_arm = [1.3200, 1.3999, -0.1998, 1.7199, 3.3468e-06, 1.6600,
                -3.4037e-06]

        # Initial (x,y,yaw) position of the robot wrt map origin.  The HSR's
        # `omni_base.pose` (i.e., the start pose) returns (x,y,yaw) where yaw is
        # the rotation about that axis (which must be the z axis). For the base,
        # `base.odom` supplies both `position` and `orientation` attributes.
        start = copy.deepcopy(self.base.odom.position)
        yaw = Base._yaw_from_quaternion(self.base.odom.orientation)
        self.start_pose = np.array([start.x, start.y, yaw])


    def body_start_pose(self):
        """Sets the robot's body to some initial configuration.
        
        The HSR uses `whole_body.move_to_go()` which initializes an appropriate
        posture so that the hand doesn't collide with movement. For the Fetch,
        we should probably make the torso extend, so the arms can extend more
        easily without collisions. We should also probably keep the arm in the
        tucked position to start. We'll need to experiment.
        """
        self.torso.set_height(0.2)
        self.arm.move_to_joints( self.arm_joints.from_list(self.tucked_arm) )


    def head_start_pose(self):
        """Hard-coded starting pose for the robot's head.
        
        TODO these values were taken from the HSR. The Fetch likely needs to use
        a different pan and tilt. We'll need to experiment.
        """
        self.head.pan_tilt(pan=1.5, tilt=-0.8)


    def position_start_pose(self, offsets=None):
        """Assigns the robot's base to some pose.
        
        The HSR code used: self.omni_base.go(p[0],p[1],p[2],300,relative=False)
        which sends robot to x, y, yaw-axis, with time-out (300) and with
        absolute (not relative) coordinates. 
        
        Right now, we use an ugly workaround, where we have our target (x,y),
        compute the distance traveled, and then derive the angle. Then we turn
        according to that angle, and go forward. Finally, we do a final turn
        which corresponds to the target yaw at the end. Note that the final turn
        will only turn relative to the angle, so (naively) we first undo the
        original turn, then we go forward with the actual desired turn.
        
        This assumes the z-coordinate stays at zero, and that the yaw is
        simply the angle about the z-axis.

        Args:
            offsets: a list of length 3, indicating offsets in the x, y, and
            yaws, respectively, to be added onto the starting pose.
        """
        p = np.copy(self.start_pose)
        #start = np.copy(self.start_pose)
        if offsets:
            p += np.array(offsets)

        # We have our target position.
        targ_x = p[0]
        targ_y = p[1]
        targ_z_angle = p[2] # TODO check that this is desired semantics wrt HSR

        # TODO need to test carefully ...

        ## # Get distances and compute angles. All x and y here are absolute values.
        ## dist = np.sqrt( (start.x-targ_x)**2 + (start.y-targ_y)**2 )
        ## rel_x = targ_x - start.x
        ## rel_y = targ_y - start.y
        ## assert -1 <= rel_x / dist <= 1
        ## assert -1 <= rel_y / dist <= 1
        ## first_angle_v1 = np.arccos(rel_x / dist)
        ## first_angle_v2 = np.arcsin(rel_y / dist)
        ## # After we've gone forward we need to undo the effect of our first turn.
        ## targ_z = targ_z_angle - (first_angle_v1*(180/math.pi))

        ## # Note that the output of np.arccos, np.arcsin are in radians.
        ## print("rel_x, rel_y: {} and {}".format(rel_x, rel_y))
        ## print("first turn at angle {} (or {})".format(first_angle_v1, first_angle_v2))
        ## print("in degrees, {} (or {})".format(
        ##         first_angle_v1 * (180/math.pi), first_angle_v2 * (180/math.pi))
        ## )
        ## print("targ_z in degrees (including undoing first turn): {}".format(targ_z))

        ## # Finally, do desired movement.
        ## base.turn(first_angle_v1)
        ## base.go_forward(distance=dist, speed=0.2)
        ## base.turn(targ_z * (math.pi / 180))


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
