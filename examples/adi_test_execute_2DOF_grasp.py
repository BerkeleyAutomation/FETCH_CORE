from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi


VEL = 0.4

def grab_item():
    """Assuming there's an object on the ground a known distance from the
    base_link of the Fetch, we move the end-effector there and grab it. 
    
    Assumes, of course, that the object is reachable. Moves via explicit
    coordinates; we need to know to go this amount in the x direction, etc.

    I always play it safe and go to `pose_0_b` first, THEN `pose_0` (so the
    fetch gripper lowers itself), and so on for poses 1, 2, etc.
    """
    x, y, z             = ( 0.5,  0.0,  0.5)
    rot_x, rot_y, rot_z = ( 0.0, 90.0,  45.0)
    #pose0 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)
    #Now we will try to create a pixel to pose grasp (grasp coordinates are pixels)
    pose0 = robot.gripper.create_grasp_pose(320, 480, 1, 45)
    print(pose0)
    rospy.sleep(1)

    # Our actual target position
    #x, y, z             = ( 0.5,  0.0,  0.23)
    #rot_x, rot_y, rot_z = ( 0.0, 90.0,  45.0)
    #import ipdb; ipdb.set_trace()
    #pose1 = robot.create_grasp_pose(x, y, z, rot_x*DEG_TO_RAD, rot_y*DEG_TO_RAD, rot_z*DEG_TO_RAD)
    #rospy.sleep(1)

    # Move to poses, adjust height as needed.
    robot.move_to_pose(pose0, velocity_factor=VEL)
    rospy.loginfo("Just moved to pose: {}".format(pose0))
    robot.torso.set_height(0.15)
    #robot.move_to_pose(pose1+'_b', velocity_factor=VEL)
    #rospy.loginfo("Just moved to pose: {}_b".format(pose1))
    #robot.move_to_pose(pose1, velocity_factor=VEL)

    # Grip object
    robot.close_gripper(width=0.02)
    rospy.sleep(1)

    # Move back up to the original pose
    #robot.move_to_pose(pose0, velocity_factor=VEL)

#Returns color and depth image
def get_color_and_depth():
    c_img, d_img = robot.get_img_data()
    cv2.imwrite("c_img_0.png", c_img)
    cv2.imwrite("d_img_0.png", d_img)
    return c_img, d_img


#This function needs to be adapted for Fetch
def adjust_grasp_center(self, desired_grasp_center, actual_grasp_center):
        difference_x = desired_grasp_center.pose.position.x - actual_grasp_center.pose.position.x
        difference_y = desired_grasp_center.pose.position.y - actual_grasp_center.pose.position.y
        base_position_map_frame = self.robot.omni_base.get_pose()
        quaternion_matrix = transformations.quaternion_matrix(base_position_map_frame.ori)
        euler_angles = transformations.euler_from_matrix(quaternion_matrix)
        self.move_base(base_position_map_frame.pos.x + difference_x, base_position_map_frame.pos.y + difference_y, euler_angles[2])



def get_frame_origin(self, frame_name):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_name
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        trans = self.tfBuffer.lookup_transform('map', frame_name, rospy.Time())
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        return pose_transformed





def img_coords2pose(self, cm, dir_vec, d_img, rot=None, depth=None):
        """
        convert image coordinates to a
        grasp pose
        '''
        Parameters
        ----------
        cm : []
            x, y pixel coordinates of center of mass
        dir_vec : []
            vector of direction to singulate in
        d_img : cv2.img
            depth image from robot
        rot : boolean
            True if singulate, None otherwise
        Returns
        -------
        name of the generated grasp pose
        """
        if depth is not None:
            z = depth
        else:
            z = self.robot.get_depth(cm, d_img)

        if rot is None:
            rot = self.robot.get_rot(dir_vec)

        pose_name = self.robot.create_grasp_pose(cm[1], cm[0], z, rot)
        # time.sleep(2)
        time.sleep(0.1)
        return pose_name




def get_actual_grasp_center(self, grasp_angle_hsr):
        rotation_axis_y = 0.079
        rotation_axis_x = 0.472
        offset_grasp_to_rotational_axis = 0.0115
        x = rotation_axis_x - math.cos(grasp_angle_hsr)*offset_grasp_to_rotational_axis
        y = rotation_axis_y + math.sin(grasp_angle_hsr)*offset_grasp_to_rotational_axis
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        return pose






#The 0.32 radians will probably change for the FETCH as well as the out of bounds angle
def transform_dexnet_angle(self, grasp_angle_dexnet):
        # rgbd camera is rotated by 0.32 radian, need to compensate that in angle
        grasp_angle_dexnet -= 0.32
        # rotate angle by 180 degrees if dexnet gives angle out of bound for hsr
        if grasp_angle_dexnet < -1.92:
            grasp_angle_hsr = grasp_angle_dexnet + np.pi
        else:
            grasp_angle_hsr = grasp_angle_dexnet
        return grasp_angle_hsr


def execute_2DOF_grasp(self, grasp_center, grasp_depth_m, grasp_angle_dexnet, grasp_width, grasp_height_offset, d_img, object_label):
        print('Executing decluttering for object in class %i' %(object_label))
        grasp_start = time.time()
        grasp_angle_hsr = self.transform_dexnet_angle(grasp_angle_dexnet) 
        actual_grasp_center = self.get_actual_grasp_center(grasp_angle_hsr)
        # use dummy direction because this function needs one as argument
        dir_vec = [0, 1]
        # img_coords2pose exchanges x and y of grasp center, thus having to give them exchanged
        grasp_frame_name = self.img_coords2pose([grasp_center[1], grasp_center[0]], dir_vec, d_img, depth=grasp_depth_m*1000)
        desired_grasp_center = self.get_frame_origin(grasp_frame_name)
        #thread.start_new_thread(self.publish_pose,('desired_grasp_pose',desired_grasp_center))
        #exit_var = raw_input()
        #if exit_var == 'exit':
        #    return
        self.adjust_grasp_center(desired_grasp_center, actual_grasp_center)
        self.go_to_grasp_pose(grasp_angle_hsr)
        z = self.compute_z_value(desired_grasp_center, grasp_height_offset)
        z = self.adjust_z_based_on_grasp_width(z, grasp_width)
        self.robot.whole_body.move_to_joint_positions({'arm_lift_joint': z})
        self.robot.close_gripper()
        self.robot.whole_body.move_to_joint_positions({'arm_lift_joint': z + 0.3})
        grasp_end = time.time()
        print('Grasping took %.2f seconds' %(grasp_end - grasp_start))
        # This sleep is needed because robot otherwise tests too early when object did not drop yet
        time.sleep(0.1)
        #if self.check_if_object_grasped():
        self.go_to_drop_pose()
        self.drop_object()
        #self.drop_object_in_bin(object_label)
        drop_end = time.time()
        print('Dropping into bin took %.2f seconds' %(drop_end - grasp_end))
        self.go_to_start_pose()



if __name__ == "__main__":
    robot = Robot_Skeleton()
    # set torso height high to start to hopefully avoid collisions with base
    robot.body_start_pose(start_height=0.25, end_height=0.25, velocity_factor=VEL)
    robot.head_start_pose(pan=0.0, tilt=45.0)
    robot.base.turn(0.26)

    #Call camera_to_world stuff here

    rospy.loginfo("Finished robot body and head start poses.")
    robot.close_gripper(width=0.04)
    robot.open_gripper()
    get_color_and_depth()
    grab_item()

