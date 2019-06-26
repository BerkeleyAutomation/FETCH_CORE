#!/usr/bin/python
# -*- coding: utf-8 -*-
import argparse, cv2, math, os, rospy, sys, threading, time
from pprint import pprint
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from cv_bridge import CvBridge, CvBridgeError

import tf
import tf2_ros
import tf2_geometry_msgs
import IPython


class RGBD(object):

    def __init__(self):
        """Similar to the HSR version, but with Fetch topic names."""
        topic_name_c = 'head_camera/rgb/image_raw'
        topic_name_i = 'head_camera/rgb/camera_info'
        topic_name_d = 'head_camera/depth_registered/image_raw'

        # rostopic list [-s for subscribers] [-p for publishers] [-v verbose]

        self.bridge = CvBridge()
        self.img_rgb_raw = None
        self.img_depth_raw = None
        self.info = None
        self.is_updated = False

        #rospy.Subscriber(name, data_msg_class, callback)
        self.sub_rgb_raw = rospy.Subscriber(topic_name_c, Image, self.callback_rgb_raw)
        self.sub_depth_raw = rospy.Subscriber(topic_name_d, Image, self.callback_depth_raw)
        self._sub_info        = rospy.Subscriber(topic_name_i, CameraInfo, self._info_cb)


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
            #  self._input_depth_image = self._bridge.imgmsg_to_cv2(
            #         data, desired_encoding="passthrough")
            self.img_depth_raw = self.bridge.imgmsg_to_cv2(data, '32FC1')

            # could just try to do pass through
            # Convert the depth image to a Numpy array since most cv2 functions
            # require Numpy arrays.
            depth_array = np.array(depth_image, dtype=np.float32)
            # Normalize the depth image to fall between 0 (black) and 1 (white)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            # At this point you can display the result properly:
            # cv2.imshow('Depth Image', depth_display_image)
            # If you write it as it si, the result will be a image with only 0 to 1 values.
            # To actually store in a this a image like the one we are showing its needed
            # to reescale the otuput to 255 gray scale.
            cv2.imwrite('capture_depth.png',frame*255)

        except CvBridgeError as e:
            rospy.logerr(e)


    def _info_cb(self,data):
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
