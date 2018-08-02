"""
Use to collect one rollout of data. So we save a bunch of these in succession.
All assuming we manually arrange the bed frame. See the data collection protocol
for more details.
"""
from fetch_core.skeleton import Robot_Skeleton
import cv2, os, sys, time, rospy, pickle, utils
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
    assert np.max(c_img) <= 255
    non_nan = np.count_nonzero(~np.isnan(d_img))
    perc = float(non_nan) / d_img.size
    print("non-NaN in d_img: {} / {} ({:.2f} %)".format(non_nan, d_img.size, perc))
    where_nan = np.isnan(d_img)
    original = d_img[~where_nan] # non-nans
    print("Considering only non-NaN, mean {:.2f}, max {:.2f}, min {:.2f}".format(
            np.mean(original), np.max(original), np.min(original)))


def set_up_bed():
    """Use this to set up initial bed configuration.
    It's research code, please don't criticize ...
    """
    print("\n --- SET UP THE BED/SHEET ---")

    if np.random.rand() < 0.5:
        print("-> Try making sheet (roughly) flat")
        style = 'Flat'
    else:
        print("-> Try making sheet (roughly) wrinkled")
        style = 'Wrinkled'

    perc = (np.random.rand() * 0.7) * 100
    perc = max(perc, 5.0)
    print("-> try putting corner roughly {:.0f}% to completion".format(perc))

    if np.random.rand() < 0.5:
        print("-> Simulate same side as robot, so don't pull opposite side")
        side = 'BOTTOM'
    else:
        print("-> ALERT! Pretend we are the robot on the opposite side and grasp+pull the opposite corner.")
        side = 'TOP'

    print("Don't forget the desiderata in the data collection protocol!\n")
    return (style, perc, side)


def get_pose_from_cimg(c_img):
    """ TODO it's trivial but cumbersome """
    pose = list(utils.red_contour(c_img))
    return pose


if __name__ == "__main__":
    num_rollouts = len([x for x in os.listdir(OUTDIR) if 'rollout_' in x])
    os.makedirs( os.path.join(OUTDIR,'rollout_{}'.format(num_rollouts)) )
    out_path = os.path.join(OUTDIR,'rollout_{}/rollout.p'.format(num_rollouts))
    print("\n\n\n\n\n\n\n\n\n\nGet things set up to save at: {}".format(out_path))
    style, perc, side = set_up_bed()
    rollout = []

    # Set to stuff from `prep_fetch.py`, while we adjust the bed. (This is
    # specific to our setup, so different code bases will likely have to adjust.
    # See our data collection protocol for more details.)
    robot = Robot_Skeleton()
    robot.body_start_pose(start_height=0.00, end_height=0.00, velocity_factor=VEL)
    robot.head_start_pose(pan=0.0*DEG_TO_RAD, tilt=45.0*DEG_TO_RAD)

    # Get RGB and depth images, then do a bunch of debug prints.  This one is a
    # placeholder as we may not have finished with the initial sheet setup.
    # ACTIONABLE: get the initial sheet set up, then press any key (except ESC).
    c_img, _ = robot.get_img_data()
    cv2.imwrite("tmp/cimg_{}.png".format(len(os.listdir('tmp/'))), c_img)
    call_wait_key(cv2.imshow("(A placeholder to stop code, finish sheet and press a key)",c_img))

    # Proceed forward, get the first c_img/d_img pair that the grasp net sees.
    print("\nFIRST IMAGE ...")
    c_img, d_img = robot.get_img_data()
    debug(c_img, d_img)
    cv2.patchNaNs(d_img, 0)
    pose = get_pose_from_cimg(c_img)
    print("pose: {}".format(pose))
    datum = {'c_img': c_img, 'd_img': d_img, 'side': side, 'pose': pose, 'type': 'grasp'}
    datum['style'] = style
    datum['perc'] = perc
    rollout.append(datum)

    # Show first image that the grasp net would see. ACTIONABLE: must simulate a
    # grasp (BUT MAKE SURE IT FAILS). Then press any key except ESC.
    call_wait_key(cv2.imshow("Img 1/3, input grasp net, pose {}. SIMULATE A GRASP".format(pose), c_img))

    # Now we've just done a grasp+pull, get new images, etc.  For success net, a
    # 1 means 'negative/failure' case.  Also needs to be input to the grasp
    # network, which we do immediately after.
    print("\nSECOND IMAGE ...")
    c_img, d_img = robot.get_img_data()
    debug(c_img, d_img)
    cv2.patchNaNs(d_img, 0)
    pose = get_pose_from_cimg(c_img)
    print("pose: {}".format(pose))
    datum = {'c_img': c_img, 'd_img': d_img, 'side': side, 'class': 1, 'type': 'success'}
    datum['style'] = style
    datum['perc'] = perc
    rollout.append(datum)
    datum = {'c_img': c_img, 'd_img': d_img, 'side': side, 'type':'grasp', 'pose': pose}
    datum['style'] = style
    datum['perc'] = perc
    rollout.append(datum)

    # ACTIONABLE: this time, actually grasp+pull and succeed at it! But make
    # sure you pull not to the exact corner but to a point 2-3 inches away from
    # it, because the sheet is wider than the bed frame.
    call_wait_key(cv2.imshow("Img 2/3, input success net (NEGATIVE) AND grasp net (for new grasp).", c_img))

    # Get final image. For success net, 0 means 'positive/success' case.
    print("\nTHIRD IMAGE ...")
    c_img, d_img = robot.get_img_data()
    debug(c_img, d_img)
    cv2.patchNaNs(d_img, 0)
    datum = {'c_img': c_img, 'd_img': d_img, 'side': side, 'class': 0, 'type': 'success'}
    datum['style'] = style
    datum['perc'] = perc
    rollout.append(datum)

    # Show what the success net sees. Press any key to exit.
    call_wait_key(cv2.imshow("Img 3/3, input success net (success). Press key to exit.", c_img))

    # Finally, save along with rollout information.
    assert len(rollout) == 4
    f = open(out_path, 'w')
    pickle.dump(rollout, f)
    f.close()
    print("just saved: {}\n".format(out_path))
