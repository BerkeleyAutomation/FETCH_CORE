from core.sensors import RGBD
import hsrb_interface
import cv2

####Initialize Robot### 
robot = hsrb_interface.Robot()

cam = RGBD()

###READ COLOR AND DEPTH DATA 
c_img = cam.read_color_data()
d_img = cam.read_depth_data()
cv2.imwrite("c_img.png", c_img)
cv2.imwrite("d_img.png", d_img)


## Alternative way (by Daniel Seita)
# from core.robot_interface import Robot_Interface
# robot = Robot_Interface()
# c_img, d_img = robot.get_img_data()
# cv2.imwrite("c_img.png", c_img)
# cv2.imwrite("d_img.png", d_img)
