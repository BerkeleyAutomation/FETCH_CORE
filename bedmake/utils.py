import cv2, os, sys
import numpy as np


def depth_to_3ch(img, cutoff=1.0):
    """HUGE NOTE: the `cutoff` assumes the units are in _meters_."""
    w,h = img.shape
    new_img = np.zeros([w,h,3])
    img = img.flatten()
    img[img>cutoff] = 0
    img = img.reshape([w,h])
    for i in range(3):
        new_img[:,:,i] = img
    return new_img


def depth_scaled_to_255(img):
    img = 255.0/np.max(img)*img
    img = np.array(img,dtype=np.uint8)
    for i in range(3):
        img[:,:,i] = cv2.equalizeHist(img[:,:,i])
    return img


def depth_to_net_dim(img, cutoff=1.0):
    """Call this from other code!"""
    assert img.shape == (480,640)
    img = depth_to_3ch(img, cutoff)
    img = depth_scaled_to_255(img)
    return img
