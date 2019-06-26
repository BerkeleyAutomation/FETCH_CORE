#!/usr/bin/python
# -*- coding: utf-8 -*-
import argparse, cv2, math, os, rospy, sys, threading, time
from pprint import pprint
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from cv_bridge import CvBridge, CvBridgeError

import tf
import tf2_ros
import tf2_geometry_msgs
import IPython


class RGBD(object):

    def __init__(self):
        # rostopic list [-s for subscribers] [-p for publishers] [-v verbose]
        self.bridge = CvBridge()
        self.img_rgb_raw = None
        self.img_depth_raw = None
        self.info = None
        self.is_updated = False

        # rospy.Subscriber(name, data_msg_class, callback)
        # Use `rqt_image_view` to see a interactive GUI of the possible rostopics
        
        self.sub_rgb_raw = rospy.Subscriber('head_camera/rgb/image_raw', Image, self.callback_rgb_raw)
        self.sub_depth_raw = rospy.Subscriber("head_camera/depth_registered/image_raw", Image, self.callback_depth_raw)
        self._sub_info = rospy.Subscriber("head_camera/rgb/camera_info", CameraInfo, self.callback_cam_info)

    def callback_rgb_raw(self, data):
        try:
            self.img_rgb_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.color_time_stamped = data.header.stamp
            self.is_updated = True
        except CvBridgeError as e:
            rospy.logerr(e)
            print(e)

    def callback_depth_raw(self, data):
        try:
            # you do not need to anything more than this to get the depth imgmsg as a cv2
            # make sure that you are using the right encoding
            self.img_depth_raw = self.bridge.imgmsg_to_cv2(data, '32FC1')
        except CvBridgeError as e:
            rospy.logerr(e)

    def callback_cam_info(self,data):
        try:
            self._info = data
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)


    def read_color_data(self):
        return self.img_rgb_raw


    def read_depth_data(self):
        return self.img_depth_raw
        # return self.img_rgb_raw


    def read_info_data(self):
        return self._info
