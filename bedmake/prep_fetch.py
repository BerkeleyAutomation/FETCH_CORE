"""
Use to position the Fetch correctly before we start.
It also saves c_img and d_img, solely for testing (and also visualizes what it'd
look like with preprocessing).
"""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy
import numpy as np
np.set_printoptions(linewidth=200, edgeitems=10)
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

# Adjust to change robot's speed.
VEL = 0.5


def depth_to_3ch(img):
    """HUGE NOTE: this (the 1.2 cutoff) assumes the units are in _meters_."""
    w,h = img.shape
    new_img = np.zeros([w,h,3])
    img = img.flatten()
    img[img>1.2] = 0
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


def depth_to_net_dim(img):
    assert img.shape == (480,640)
    img = depth_to_3ch(img)
    img = depth_scaled_to_255(img)
    return img


if __name__ == "__main__":
    robot = Robot_Skeleton()
    robot.body_start_pose(start_height=0.00, end_height=0.00, velocity_factor=VEL)
    robot.head_start_pose(pan=0.0*DEG_TO_RAD, tilt=45.0*DEG_TO_RAD)

    # Get RGB and depth images, then investigate.
    c_img, d_img = robot.get_img_data()
    print("c_img: {}\nd_img: {}".format(c_img.shape, d_img.shape))
    print("dtypes: {} and {}".format(c_img.dtype, d_img.dtype))
    print("max(c_img): {}".format(np.max(c_img)))
    non_nan = np.count_nonzero(~np.isnan(d_img))
    print("non-NaN elements in d_img: {} / {} ({:.2f} %)".format(non_nan, d_img.size, float(non_nan) / d_img.size))
    where_nan = np.isnan(d_img)
    original = d_img[~where_nan] # non-nans
    print("Considering only non-NaN, mean {}, max {}, min {}".format(np.mean(original),
            np.max(original), np.min(original)))

    # Note: NaNs usually appear in the depth at the 'borders' of the image,
    # could be too far / too deep? Convert to al black to indicate 'too far'?
    # Maybe my version is off? cv2.patchNaNs returns none, yet docs tell me to
    # do `a = cv2.patchNaNs(a, 0)` ...
    print(d_img)
    cv2.patchNaNs(d_img, 0)
    print("now:")
    print(d_img)

    d_img = depth_to_net_dim(d_img)

    cv2.imwrite("c_img_0.png", c_img)
    cv2.imwrite("d_img_0.png", d_img)
    print("saved camera, depth images!")

    while True:
        c_img, _ = robot.get_img_data()
        cv2.imshow("c_img", c_img)
        cv2.waitKey()
    rospy.spin()
