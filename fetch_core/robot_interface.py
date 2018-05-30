import controller_manager_msgs.srv
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tf import TransformListener, TransformBroadcaster
from cv_bridge import CvBridge, CvBridgeError
import copy, math, rospy, time
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
    """For usage with the Fetch robot."""

    def __init__(self, simulation=True):
        """Initializes various aspects of the Fetch.
        
        TODOs: get things working, also use `simulation` flag to change ROS
        topic names if necessary (especially for the cameras!). UPDATE: actually
        I don't think this is necessary now, they have the same topic names.
        """
        rospy.init_node("fetch")
        self.arm = Arm()
        self.arm_joints = ArmJoints()
        self.base = Base()
        self.camera = RGBD()
        self.head = Head()
        self.gripper = Gripper(self.camera)
        self.torso = Torso()

        # Tucked arm starting joint angle configuration
        self.tucked_arm = [1.3200, 1.3999, -0.1998, 1.7199, 3.3468e-06, 1.6600,
                -3.4037e-06]

        # Initial (x,y,yaw) position of the robot wrt map origin. We keep this
        # fixed so that we can reset to this position as needed. The HSR's
        # `omni_base.pose` (i.e., the start pose) returns (x,y,yaw) where yaw is
        # the rotation about that axis (intuitively, the z axis). For the base,
        # `base.odom` supplies both `position` and `orientation` attributes.
        start = copy.deepcopy(self.base.odom.position)
        yaw = Base._yaw_from_quaternion(self.base.odom.orientation)
        self.start_pose = np.array([start.x, start.y, yaw])
        self.turn_speed = 0.3


    def body_start_pose(self):
        """Sets the robot's body to some initial configuration.
        
        The HSR uses `whole_body.move_to_go()` which initializes an appropriate
        posture so that the hand doesn't collide with movement. For the Fetch,
        we should probably make the torso extend, so the arms can extend more
        easily without collisions. We should also probably keep the arm in the
        tucked position to start. We'll need to experiment.

        Update: for now, set height to be something small, tuck joints, then set
        height back to zero so that we can move safely.
        """
        self.torso.set_height(0.2)
        time.sleep(1)
        self.arm.move_to_joints( self.arm_joints.from_list(self.tucked_arm) )
        time.sleep(1)
        self.torso.set_height(0.03) # give a little room


    def head_start_pose(self):
        """Hard-coded starting pose for the robot's head.
        
        TODO these values were taken from the HSR. The Fetch likely needs to use
        a different pan and tilt. We'll need to experiment. Positive pan means
        rotating counterclockwise when looking at robot from an aerial view.
        """
        self.head.pan_tilt(pan=1.5, tilt=-0.8)


    def position_start_pose(self, offsets=None):
        """Assigns the robot's base to some starting pose.

        Mainly to "reset" the robot to the original starting position (and also,
        rotation about base axis) after it has moved, usually w/no offsets.
        
        Ugly workaround: we have target (x,y), and compute the distance to the
        point and the angle. We turn the Fetch according to that angle, and go
        forward. Finally, we do a second turn which corresponds to the target
        yaw at the end. This turns w.r.t. the current angle, so we undo the
        effect of the first turn.  See `examples/test_position_start_pose.py`
        for tests.
        
        Args:
            offsets: a list of length 3, indicating offsets in the x, y, and
            yaws, respectively, to be added onto the starting pose.
        """
        current_pos = copy.deepcopy(self.base.odom.position)
        current_theta = Base._yaw_from_quaternion(self.base.odom.orientation) # [-pi, pi]
        ss = np.array([current_pos.x, current_pos.y, current_theta])

        # Absolute target position and orientation specified with `pp`.
        pp = np.copy(self.start_pose)
        if offsets:
            pp += np.array(offsets)

        # Get distance to travel, critically assumes `pp` is starting position.
        dist = np.sqrt( (ss[0]-pp[0])**2 + (ss[1]-pp[1])**2 )
        rel_x = ss[0] - pp[0]
        rel_y = ss[1] - pp[1]
        assert -1 <= rel_x / dist <= 1
        assert -1 <= rel_y / dist <= 1

        # But we also need to be *facing* the correct direction, w/input [-1,1].
        # First, get the opposite view (facing "outwards"), then flip by 180.
        desired_facing = np.arctan2(rel_y, rel_x) # [-pi, pi], facing outward
        desired_theta  = math.pi + desired_facing # [0, 2*pi], flip by 180
        if desired_theta > math.pi:
            desired_theta -= 2*math.pi # [-pi, pi]
 
        # Reconcile with the current theta. Got this by basically trial/error
        angle = desired_theta - current_theta  # [-2*pi, 2*pi]
        if angle > math.pi:
            angle -= 2*math.pi 
        elif angle < -math.pi:
            angle += 2*math.pi 

        self.base.turn(angular_distance=angle, speed=self.turn_speed)
        self.base.go_forward(distance=dist, speed=0.2)

        # Back at the start x, y, but now need to consider the _second_ turn.
        # The robot is facing at `desired_theta` rads, but wants `pp[2]` rads.
        final_angle = pp[2] - desired_theta
        if final_angle > math.pi:
            final_angle -= 2*math.pi 
        elif final_angle < -math.pi:
            final_angle += 2*math.pi 
        self.base.turn(angular_distance=final_angle, speed=self.turn_speed)


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
        pose_name = self.gripper.create_grasp_pose(x, y, z, rot)
        return pose_name

        
    def open_gripper(self):
        self.gripper.open()


    def close_gripper(self):
        self.gripper.close()


    def move_to_pose(self, pose_name, z_offset):
        """Moves to a pose.
 
        In the HSR, moved the `hand_palm_link` to the frame named `pose_name` at
        the correct pose. For the Fetch we should be able to extract the pose
        from `pose_name` and then call the Arm's `move_to_pose` method.
        
        Args:
            pose_name: A string name for the pose to go 
            z_offset: Scalar offset in z-direction, offset is wrt the pose
                specified by `pose_name`.
        """
        # TODO: need to get the info for Point and Quaternion ... figure out
        # what kind of reference frame to use?
        #pose,quat = gripper.tl.lookupTransform('map', pose_name, rospy.Time(0))
        #ps = PoseStamped()
        #ps.header.frame_id = 'base_link'
        #ps.pose = Pose(Point(x,y,z+z_offset), Quaternion(quat?))
        #self.arm.move_to_pose(pose_stamped=ps)
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
