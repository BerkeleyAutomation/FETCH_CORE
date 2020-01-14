from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np


def can_plan(ps, allowed_planning_time=10.0, 
                 execution_timeout=15.0, 
                 group_name='arm_with_torso', 
                 num_planning_attempts=1, 
                 orientation_constraint=None, 
                 plan_only=False,
                 replan=True, 
                 replan_attempts=20, 
                 tolerance=0.01, 
                 velocity_factor=0.4):
    error = robot.move_to_pose(pose_name=ps, 654
                     execution_timeout=execution_timeout,
                     group_name=group_name,
                     num_planning_attempts=num_planning_attempts,
                     orientation_constraint=orientation_constraint,
                     plan_only=plan_only, 
                     replan=replan,
                     replan_attempts=replan_attempts,
                     tolerance=tolerance,
                     velocity_factor=velocity_factor)
    print(error)


if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose(start_height=0.18, end_height=0.18, velocity_factor=0.4)
    rospy.loginfo("Finished robot starting config.")
    robot.close_gripper(width=0.04)
    robot.open_gripper()
    robot.arm.move_to_joint_goal()

    #map_coordinates = [0.64364554, 0.40790289, 0.26959194]
    #pose0 = robot.gripper.create_grasp_pose(320, 400, 0.7, 45, use_world_frame=True, world_coordinates=map_coordinates)
    #can_plan(pose0)
