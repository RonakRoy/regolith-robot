#!/usr/bin/python

import rospy
import numpy as np
import cv2
import time
<<<<<<< HEAD
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
=======
import image_geometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Point32, Pose, Twist, Vector3, Quaternion
>>>>>>> 653f354443ce6d1c6e8e87ed59e530d3e35b2e1f
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('object_detector', anonymous=True)
cv_bridge = CvBridge()

thresh_pub = rospy.Publisher('/object_detector/thresholded', Image, queue_size = 1)
open_pub = rospy.Publisher('/object_detector/opened', Image, queue_size = 1)
blob_pub = rospy.Publisher('/object_detector/blobs', Image, queue_size = 1)

<<<<<<< HEAD
def main():
    raw_image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback)
    color_image_subscriber = rospy.Subscriber('/camera/rgb/image_color', Image, color_callback)
    # depth_image_subscriber = rospy.Subscriber('/camera/depth_registered/depth', Image, depth_callback)
=======
pile_pub = rospy.Publisher('/object_detector/pile_location', PointCloud, queue_size = 1)

def main():
    raw_image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback)
    color_image_subscriber = rospy.Subscriber('/camera/rgb/image_color', Image, color_callback)
    depth_image_subscriber = rospy.Subscriber('/camera/depth_registered/image', Image, depth_callback)

    depth_info_subscriber = rospy.Subscriber('/camera/depth_registered/camera_info', CameraInfo, depth_info_callback)
>>>>>>> 653f354443ce6d1c6e8e87ed59e530d3e35b2e1f

    rospy.sleep(1)    
    rospy.spin()


color_image = None
def color_callback(msg):
    try:
        global color_image
        color_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

<<<<<<< HEAD

def rgb_callback(msg):
=======
depth_image = None
def depth_callback(msg):
    try:
        global depth_image
        depth_image = cv_bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        print(e)

depth_camera_frame = None
depth_camera_model = None
def depth_info_callback(msg):
    global depth_camera_model
    global depth_camera_frame
    if depth_camera_model is None:
        depth_camera_model = image_geometry.PinholeCameraModel()
        depth_camera_model.fromCameraInfo(msg)

        depth_camera_frame = msg.header.frame_id

def rgb_callback(msg):
    raw_stamp = msg.header.stamp
>>>>>>> 653f354443ce6d1c6e8e87ed59e530d3e35b2e1f
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # convert to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # get threshold values
<<<<<<< HEAD
    lower_bound_HSV = np.array([0, 0, 230])
    upper_bound_HSV = np.array([250, 255, 255])
=======
    lower_bound_HSV = np.array([rospy.get_param("/hl",   0), rospy.get_param("/sl", 100), rospy.get_param("/vl",  30)])
    upper_bound_HSV = np.array([rospy.get_param("/hh",  50), rospy.get_param("/sh", 255), rospy.get_param("/vh", 255)])
>>>>>>> 653f354443ce6d1c6e8e87ed59e530d3e35b2e1f

    # threshold
    mask_HSV = cv2.inRange(hsv_image, lower_bound_HSV, upper_bound_HSV)

    # get display image
    disp_image_HSV = cv2.bitwise_and(cv_image, cv_image, mask=mask_HSV)

    thresh_img_msg = cv_bridge.cv2_to_imgmsg(disp_image_HSV, "bgr8")
    thresh_img_msg.header.stamp = rospy.Time.now()
    thresh_pub.publish(thresh_img_msg)

    kernel = np.ones((5,5),np.uint8)

    # open
<<<<<<< HEAD
    opened = cv2.morphologyEx(mask_HSV, cv2.MORPH_OPEN, kernel, iterations = 1)
=======
    opened = cv2.morphologyEx(mask_HSV, cv2.MORPH_OPEN, kernel, iterations = 3)
>>>>>>> 653f354443ce6d1c6e8e87ed59e530d3e35b2e1f

    open_img_msg = cv_bridge.cv2_to_imgmsg(opened)
    open_img_msg.header.stamp = rospy.Time.now()
    open_pub.publish(open_img_msg)

    # find largest contours
    found, contours, hierarchy = cv2.findContours(opened, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest = max(contours, key = cv2.contourArea)

<<<<<<< HEAD
=======
    # get points of largest contour
    cimg = np.zeros_like(cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY))
    cv2.drawContours(cimg, [largest], 0, color=255, thickness=-1)
    pts = np.where(cimg == 255)

    x_med = np.median(pts[1])
    mask = (pts[1] >= x_med-10) & (pts[1] <= x_med+10)

    y_bot = np.median(np.sort(pts[0][mask])[-50:])

>>>>>>> 653f354443ce6d1c6e8e87ed59e530d3e35b2e1f
    if color_image != None:
        cv2.drawContours(color_image, contours, -1, (0,255,0), 3)
        x,y,w,h = cv2.boundingRect(largest)

<<<<<<< HEAD
        cv2.rectangle(color_image,(x,y),(x+w,y+h),(0,0,255),2)
=======
        cv2.rectangle(color_image, (x,y), (x+w,y+h), (0,0,255), 2)

        cv2.circle(color_image, (int(x_med), int(y_bot)), 20, (255,0,255), -1)
>>>>>>> 653f354443ce6d1c6e8e87ed59e530d3e35b2e1f

        blob_img_msg = cv_bridge.cv2_to_imgmsg(color_image, "bgr8")
        blob_img_msg.header.stamp = rospy.Time.now()
        blob_pub.publish(blob_img_msg)

<<<<<<< HEAD
=======
    global depth_camera_model
    if depth_camera_model is not None:
        global depth_image

        ray = np.array(depth_camera_model.projectPixelTo3dRay((x_med,y_bot)))
        dist = 0
        radius = 9
        for dx in range(-radius,radius+1):
            for dy in range(-radius,radius+1):
                dist += depth_image[x_med + dx, y_bot + dy] / 4.0
        point = ray * dist / (radius*radius)

        print point

        pile_msg = PointCloud()
        pile_msg.header.stamp = raw_stamp
        pile_msg.header.frame_id = depth_camera_frame
        pile_point = Point32()
        pile_point.x = point[0]
        pile_point.y = point[1]
        pile_point.z = point[2]
        pile_msg.points.append(pile_point)

        pile_pub.publish(pile_msg)


>>>>>>> 653f354443ce6d1c6e8e87ed59e530d3e35b2e1f
if __name__=='__main__':
    main()
