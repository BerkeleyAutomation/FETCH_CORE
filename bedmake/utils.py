import cv2, os, sys
import numpy as np

# --- PREPROCESSING DEPTH ---

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




# Only for testing the red contours for images.
def red_contour(image, save_images=False, fname=None):
    """The HSR (and Fetch) have images in BGR mode.

    This method for detecting red is courtesy of Ron Berenstein. We tried
    HSV-based methods but it's really bad.
    """
    original = image.copy()

    b, g, r = cv2.split(image)
    bw0 = (r[:,:]>150).astype(np.uint8)*255

    bw1 = cv2.divide(r, g[:, :] + 1)
    bw1 = (bw1[:, :] > 1.5).astype(np.uint8)*255
    bw1 = np.multiply(bw1, bw0).astype(np.uint8) * 255
    bw2 = cv2.divide(r, b[:,:]+1)
    bw2 = (bw2[:, :] > 1.5).astype(np.uint8)*255

    bw = np.multiply(bw1, bw2).astype(np.uint8) * 255
    kernel = np.ones((5, 5), np.uint8)
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)
    bw = cv2.dilate(bw, kernel, iterations=1)
    _, bw = cv2.threshold(bw,0,255,0)

    # Now get the actual contours.  Note that contour detection requires a
    # single channel image. Also, we only want the max one as that should be
    # where the sewn patch is located.
    (cnts, _) = cv2.findContours(bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(cnts) == 0:
        cv2.imwrite("problem_original.png", original)
        cv2.imwrite("problem_bw.png", bw)
        print("Problem, len(cnts) == 0. See saved images...")
        sys.exit()
    cnt_largest = max(cnts, key = lambda cnt: cv2.contourArea(cnt))

    # Find the centroid in _pixel_space_. Draw it.
    try:
        M = cv2.moments(cnt_largest)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        if save_images:
            peri = cv2.arcLength(cnt_largest, True)
            approx = cv2.approxPolyDP(cnt_largest, 0.02*peri, True)
            cv2.circle(image_bgr, (cX,cY), 50, (0,0,255))
            cv2.drawContours(image_bgr, [approx], -1, (0,255,0), 2)
            cv2.putText(img=image_bgr, 
                        text="{},{}".format(cX,cY), 
                        org=(cX+10,cY+10), 
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1, 
                        color=(255,255,255), 
                        thickness=2)
            cv2.imwrite(fname.replace('.png','_cnt.png'), image_bgr)
        return (cX,cY)
    except:
        print("PROBLEM CANNOT DETECT CONTOUR ...")


if __name__ == "__main__":
    for i in range(1,11):
        fname = "cimgs/cimg_{}.png".format(i)
        img = (cv2.imread(fname)).copy()
        red_contour(img, save_images=True, fname=fname)
