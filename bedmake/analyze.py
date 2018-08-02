"""
Use this script to inspect the raw data for quick sanity checks.
"""
import cv2, os, pickle, sys, utils
import numpy as np
np.set_printoptions(suppress=True, linewidth=200)

# ADJUST
ROLLOUTS = 'rollouts/'
IMG_PATH = 'images/'

g_total = 0
s_count_failure = 0
s_count_success = 0

for rnum in range(0, 60):
    print("\n=====================================================================")
    print("rollout {}".format(rnum))
    path = os.path.join(ROLLOUTS, 'rollout_{}/rollout.p'.format(rnum))
    if not os.path.exists(path):
        print("{} does not exist, skipping...".format(path))
        continue
    data = pickle.load(open(path,'rb'))

    # These record, for grasp and successes, the index into _this_ rollout.
    g_in_rollout = 0
    s_in_rollout = 0

    for (d_idx,datum) in enumerate(data):
        print("\ncurrently on item {} in this rollout, out of {}:".format(d_idx,len(data)))
        print('side:   {}'.format(datum['side']))
        print('type:   {}'.format(datum['type']))
        if datum['type'] == 'grasp':
            print('pose:   {}'.format(datum['pose']))
        elif datum['type'] == 'success':
            print('class:  {}'.format(datum['class']))
        else:
            raise ValueError(datum['type'])

        # Get the depth image to look better.
        datum['d_img'] = utils.depth_to_net_dim(datum['d_img'], cutoff=1.0)

        # Grasping. For these, overlay the pose to the image (red circle, black border).
        if datum['type'] == 'grasp':
            c_path = os.path.join(IMG_PATH, 'rollout_{}_grasp_{}_rgb.png'.format(rnum,g_in_rollout))
            d_path = os.path.join(IMG_PATH, 'rollout_{}_grasp_{}_depth.png'.format(rnum,g_in_rollout))
            c_img = (datum['c_img']).copy()
            d_img = (datum['d_img']).copy()
            pose = datum['pose']
            pose_int = (int(pose[0]), int(pose[1]))
            cv2.circle(img=c_img, center=pose_int, radius=7, color=(0,0,255), thickness=-1)
            cv2.circle(img=d_img, center=pose_int, radius=7, color=(0,0,255), thickness=-1)
            cv2.circle(img=c_img, center=pose_int, radius=9, color=(0,0,0), thickness=2)
            cv2.circle(img=d_img, center=pose_int, radius=9, color=(0,0,0), thickness=2)
            cv2.imwrite(c_path, c_img)
            cv2.imwrite(d_path, d_img)
            g_in_rollout += 1
            g_total += 1

        # Success (0=success, 1=failure).
        elif datum['type'] == 'success':
            result = datum['class']
            if result == 0:
                s_count_success += 1
                c_path = os.path.join(IMG_PATH,
                        'rollout_{}_success_{}_class0_rgb.png'.format(rnum,s_in_rollout))
                d_path = os.path.join(IMG_PATH,
                        'rollout_{}_success_{}_class0_depth.png'.format(rnum,s_in_rollout))
            else:
                s_count_failure += 1
                c_path = os.path.join(IMG_PATH,
                        'rollout_{}_success_{}_class1_rgb.png'.format(rnum,s_in_rollout))
                d_path = os.path.join(IMG_PATH,
                        'rollout_{}_success_{}_class1_depth.png'.format(rnum,s_in_rollout))
            cv2.imwrite(c_path, datum['c_img'])
            cv2.imwrite(d_path, datum['d_img'])
            s_in_rollout += 1

        else:
            raise ValueError(datum['type'])

    print("=====================================================================")

print("\nDone analyzing. Some stats:")
print("g_total:         {}".format(g_total)) # total images for grasping network
print("s_count_failure: {}".format(s_count_failure))
print("s_count_success: {}".format(s_count_success))
