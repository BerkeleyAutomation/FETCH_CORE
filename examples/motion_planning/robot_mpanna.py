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
from reader import JointStateReader
DEG_TO_RAD = np.pi / 180
ZRANGE = 20


class Robot_mpanna(object):
    """
    For usage with the Fetch robot.
    
    Jackson:
    I disabled features we do not need for our mplambda/mpanna project (eg. head).
    We implemented the callback reader without a class in the script we run.

    # TODO:
    Create a custom gripper class that doesn't need the cameras

    """

    def __init__(self, simulation=True):
        """Initializes various aspects of the Fetch.
        
        TODOs: get things working, also use `simulation` flag to change ROS
        topic names if necessary (especially for the cameras!). UPDATE: actually
        I don't think this is necessary now, they have the same topic names.
        """
        rospy.init_node("fetch")
        self.arm = Arm()
        self.camera = RGBD()
        self.arm_joints = ArmJoints()
        self.base = Base()
        self.head = Head()
        self.gripper = Gripper(self.camera)
        self.torso = Torso()
        self.joint_reader = JointStateReader()

        # Tucked arm starting joint angle configuration
        self.names = ArmJoints().names()
        self.tucked = [1.3200, 1.3999, -0.1998, 1.7199, 0.0, 1.6600, 0.0]
        #self.tucked_list = [zip(self.names, self.tucked)]

        # Initial (x,y,yaw) position of the robot wrt map origin. We keep this
        # fixed so that we can reset to this position as needed. The HSR's
        # `omni_base.pose` (i.e., the start pose) returns (x,y,yaw) where yaw is
        # the rotation about that axis (intuitively, the z axis). For the base,
        # `base.odom` supplies both `position` and `orientation` attributes.
        start = copy.deepcopy(self.base.odom.position)
        yaw = Base._yaw_from_quaternion(self.base.odom.orientation)
        self.start_pose = np.array([start.x, start.y, yaw])
        self.TURN_SPEED = 0.3


    def body_start_pose(self, start_height=0.10, end_height=0.10, velocity_factor=0.5):
        """Sets the robot's body to some initial configuration.
        
        The HSR uses `whole_body.move_to_go()` which initializes an appropriate
        posture so that the hand doesn't collide with movement. For the Fetch,
        we should probably make the torso extend, so the arms can extend more
        easily without collisions. Use `move_to_joint_goal` since that uses
        motion planning. Do NOT directly set the joints without planning!!
        """
        self.torso.set_height(start_height)
        self.arm.move_to_joint_goal(joint_names=self.names, joint_positions=self.tucked,  velocity_factor=velocity_factor)
        self.torso.set_height(end_height)
        # Specific to the siemens challenge (actually a lot of this stuff is ...)
        # if self.num_restarts == 0:
        #     self.base.turn(angular_distance=45*DEG_TO_RAD)
        #     self.num_restarts += 1


    def position_start_pose(self, offsets=None, do_something=False):
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
        # Causing problems during my tests of the Siemens demo.
        if not do_something:
            return


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

        self.base.turn(angular_distance=angle, speed=self.TURN_SPEED)
        self.base.go_forward(distance=dist, speed=0.2)

        # Back at the start x, y, but now need to consider the _second_ turn.
        # The robot is facing at `desired_theta` rads, but wants `pp[2]` rads.
        final_angle = pp[2] - desired_theta
        if final_angle > math.pi:
            final_angle -= 2*math.pi 
        elif final_angle < -math.pi:
            final_angle += 2*math.pi 
        self.base.turn(angular_distance=final_angle, speed=self.TURN_SPEED)


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
        z_box = d_img[y-ZRANGE:y+ZRANGE, x-ZRANGE:x+ZRANGE]
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
        """ If `intuitive=True` then x,y,z,rot interpreted wrt some link in the
        world, e.g., 'odom' for the Fetch. It's False by default to maintain
        backwards compatibility w/Siemens-based code.
        """
        pose_name = self.gripper.create_grasp_pose(x, y, z, rot)
        return pose_name

        
    def open_gripper(self):
        self.gripper.open()


    def close_gripper(self):
        self.gripper.close()


    def move_to_pose(self, pose_name, z_offset, velocity_factor=None):
        """Moves to a pose.
 
        In the HSR, moved the `hand_palm_link` to the frame named `pose_name` at
        the correct pose. For the Fetch we should be able to extract the pose
        from `pose_name` and then call the Arm's `move_to_pose` method.
        
        Args:
            pose_name: A string name for the pose to go 
            z_offset: Scalar offset in z-direction, offset is wrt the pose
                specified by `pose_name`.
            velocity_factor: controls the speed, closer to 0 means slower,
                closer to 1 means faster. (If 0.0, then it turns into 1.0 for
                some reason.) Values greater than 1.0 are cut to 1.0.
        """
        # See: 
        #   http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
        #   https://answers.ros.org/question/256354/does-tftransformlistenerlookuptransform-return-quaternion-position-or-translation-and-rotation/
        # First frame should be the reference frame, use `base_link`, not `odom`.
        point, quat = self.gripper.tl.lookupTransform('base_link', pose_name, rospy.Time(0))
        z_point = point[2] + z_offset

        # See:
        #   https://github.com/cse481wi18/cse481wi18/blob/indigo-devel/applications/scripts/cart_arm_demo.py
        #   https://github.com/cse481wi18/cse481wi18/wiki/Lab-19%3A-Cartesian-space-manipulation
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose = Pose(
                Point(point[0], point[1], z_point), 
                Quaternion(quat[0], quat[1], quat[2], quat[3])
        )

        # See `arm.py` written by Justin Huang
        error = self.arm.move_to_pose(pose_stamped=ps, velocity_factor=velocity_factor)
        if error is not None:
            rospy.logerr(error)
