#!/usr/bin/python


import argparse, cv2, math, os, rospy, sys, threading, time
from pprint import pprint
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

import message_filters
import tf
import tf2_ros
import tf2_geometry_msgs
import IPython


class RGBD(object):

    def __init__(self):
        """Similar to the HSR version, but with Fetch topic names."""
        cam_image_topic = 'head_camera/rgb/image_raw'
        cam_info_topic = 'head_camera/rgb/camera_info'
        depth_image_topic = 'head_camera/depth_registered/image_raw'

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.info = None
        self.is_updated = False
        
        self.image_sub = message_filters.Subscriber(cam_image_topic, Image)
        self.depth_sub = message_filters.Subscriber(depth_image_topic, Image)
    
    def callback(self, rgb_data, depth_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CVBridgeError, e:
            print e
    
    def read_camera_image():
        return self.color_image

    def read_depth_image():
        return self.depth_image
