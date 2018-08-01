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
def red_contour(image, save_images=False):
    """The HSR (and Fetch) have images in BGR mode."""
    lower = np.array([150, 100, 100])
    upper = np.array([180, 255, 255])
    #image = cv2.medianBlur(image, 9)
    image = cv2.bilateralFilter(image, 7, 13, 13)
    hsv   = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask  = cv2.inRange(hsv, lower, upper)
    res   = cv2.bitwise_and(image, image, mask=mask)
    res   = cv2.medianBlur(res, 9)
    if save_images:
        cv2.imwrite("tmp/c_img_0_red.png", res)

    # Now get the actual contours.  Note that contour detection requires a
    # single channel image. Also, we only want the max one as that should be
    # where the sewn patch is located.
    image = cv2.cvtColor(res, cv2.COLOR_HSV2BGR) 
    image_bgr = image.copy()
    (cnts, _) = cv2.findContours(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 
            cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            cv2.imwrite("tmp/c_img_0_cnt.png", image_bgr)
        return (cX,cY)
    except:
        print("PROBLEM CANNOT DETECT CONTOUR ...")


if __name__ == "__main__":
    #img = cv2.imread("tmp/c_img_0.png")
    img = cv2.imread("tmp/rollout_0_grasp_0_rgb.png")
    red_contour(img.copy(), save_images=True)
