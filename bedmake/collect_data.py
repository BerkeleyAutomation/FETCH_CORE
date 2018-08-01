"""
Use to collect one rollout of data. So we save a bunch of these in succession.
All assuming we manually arrange the bed frame. See the data collection protocol
for more details.
"""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy, pickle
import numpy as np
np.set_printoptions(linewidth=200, edgeitems=10)
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi
VEL = 0.5

# Good thing I did this a lot before. :-)
ESC_KEYS = [27, 1048603]

# Adjust
OUTDIR = 'rollouts/'


def call_wait_key(nothing=None):
    """ Call this like: `call_wait_key( cv2.imshow(...) )`.

    Basically, press any key to continue, OR press ESC to kill/exit.
    """
    ESC_KEYS = [27, 1048603]
    key = cv2.waitKey(0)
    if key in ESC_KEYS:
        print("Pressed ESC key. Terminating program...")
        sys.exit()


def debug(c_img, d_img):
    """For debugging.

    Note: NaNs usually appear in the depth at the 'borders' of the image, could
    be too far / too deep? Also, maybe my cv2 version is old?  cv2.patchNaNs
    returns None, yet docs tell me `a = cv2.patchNaNs(a, 0)` ... let's do the
    patching outside this method since it's pretty critical.
    """
    print("c_img: {}\nd_img: {}".format(c_img.shape, d_img.shape))
    print("dtypes: {} and {}".format(c_img.dtype, d_img.dtype))
    print("max(c_img): {}".format(np.max(c_img)))
    non_nan = np.count_nonzero(~np.isnan(d_img))
    perc = float(non_nan) / d_img.size
    print("non-NaN in d_img: {} / {} ({:.2f} %)".format(non_nan, d_img.size, perc))
    where_nan = np.isnan(d_img)
    original = d_img[~where_nan] # non-nans
    print("Considering only non-NaN, mean {}, max {}, min {}".format(np.mean(original),
            np.max(original), np.min(original)))


def set_up_bed():
    """Use this to set up initial bed configuration.
    It's research code, please don't criticize ...
    """
    print("\n --- SET UP THE BED/SHEET ---")
    if np.random.rand() < 0.5:
        print("Try making sheet (roughly) flat")
    else:
        print("Try making sheet (roughly) wrinkled")
    if np.random.rand() < 0.5:
        print("Put red marker somewhere FAR from the target corner")
    else:
        print("Put red marker somewhere CLOSE to the target corner")
    if np.random.rand() < 0.5:
        print("Simulate same side as robot, so don't pull opposite side")
        side = 'BOTTOM'
    else:
        print("ALERT! Pretend we are the robot on the opposite side and grasp+pull the opposite corner.")
        side = 'TOP'
    print("Don't forget the desiderata in the data collection protocol!\n")
    return side


def get_pose_from_cimg(c_img):
    """ TODO it's trivial but cumbersome """
    pass


if __name__ == "__main__":
    num_rollouts = len([x for x in os.listdirs(OUTDIR) if 'rollout' in x and x[-2:] == '.p'])
    out_path = os.path.join(OUTDIR,'rollout_{}.p'.format(num_rollouts))
    print("Get things set up to save at: {}".format(out_path))
    side = set_up_bed()
    rollout = []

    # Set to stuff from `prep_fetch.py`, while we adjust the bed.
    robot = Robot_Skeleton()
    robot.body_start_pose(start_height=0.00, end_height=0.00, velocity_factor=VEL)
    robot.head_start_pose(pan=0.0*DEG_TO_RAD, tilt=45.0*DEG_TO_RAD)

    # Get RGB and depth images, then do a bunch of debug prints.
    # Note: this first one is solely as a placeholder as we may not have
    # finished with the setup.
    c_img, d_img = robot.get_img_data()
    call_wait_key(cv2.imshow("finish adjusting the sheet the move forward.",c_img))

    print("Moving forward. First image is:")
    c_img, d_img = robot.get_img_data()
    debug(c_img, d_img)
    cv2.patchNaNs(d_img, 0)
    call_wait_key(cv2.imshow("Img 1/3, input to grasp net.", c_img))
    pose = get_pose_from_cimg(c_img)
    datum = {'c_img': c_img, 'd_img': d_img, 'side': side, 'pose': pose, 'type': 'grasp'}
    rollout.append(datum)

    c_img, d_img = robot.get_img_data()
    debug(c_img, d_img)
    cv2.patchNaNs(d_img, 0)
    call_wait_key(cv2.imshow("Img 2/3, input to success net (negative) AND grasp net (for new grasp).", c_img))
    pose = get_pose_from_cimg(c_img)
    # For success net, 1 means 'negative/failure' case.
    # Also needs to be input to the grasp network, which we do right after.
    datum = {'c_img': c_img, 'd_img': d_img, 'side': side, 'class': 1, 'type': 'success'}
    rollout.append(datum)
    datum = {'c_img': c_img, 'd_img': d_img, 'side': side, 'type':'grasp', 'pose': pose}
    rollout.append(datum)

    # For success net, 0 means 'positive/success' case.
    c_img, d_img = robot.get_img_data()
    debug(c_img, d_img)
    cv2.patchNaNs(d_img, 0)
    call_wait_key(cv2.imshow("Img 3/3, input to success net (success).", c_img))
    datum = {'c_img': c_img, 'd_img': d_img, 'side': side, 'class': 0, 'type': 'success'}
    rollout.append(datum)

    # Now we save along with rollout information.
    assert len(rollout) == 4
    f = open(out_path, 'w')
    pickle.dump(rollout, f)
    f.close()
    print("just saved: {}".format(out_path))
