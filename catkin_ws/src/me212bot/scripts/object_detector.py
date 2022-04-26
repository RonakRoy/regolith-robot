#!/usr/bin/python

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('object_detector', anonymous=True)
cv_bridge = CvBridge()

thresh_pub = rospy.Publisher('/object_detector/thresholded', Image, queue_size = 1)
open_pub = rospy.Publisher('/object_detector/opened', Image, queue_size = 1)
blob_pub = rospy.Publisher('/object_detector/blobs', Image, queue_size = 1)

def main():
    raw_image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback)
    color_image_subscriber = rospy.Subscriber('/camera/rgb/image_color', Image, color_callback)
    # depth_image_subscriber = rospy.Subscriber('/camera/depth_registered/depth', Image, depth_callback)

    rospy.sleep(1)    
    rospy.spin()


color_image = None
def color_callback(msg):
    try:
        global color_image
        color_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)


def rgb_callback(msg):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # convert to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # get threshold values
    lower_bound_HSV = np.array([0, 0, 230])
    upper_bound_HSV = np.array([250, 255, 255])

    # threshold
    mask_HSV = cv2.inRange(hsv_image, lower_bound_HSV, upper_bound_HSV)

    # get display image
    disp_image_HSV = cv2.bitwise_and(cv_image, cv_image, mask=mask_HSV)

    thresh_img_msg = cv_bridge.cv2_to_imgmsg(disp_image_HSV, "bgr8")
    thresh_img_msg.header.stamp = rospy.Time.now()
    thresh_pub.publish(thresh_img_msg)

    kernel = np.ones((5,5),np.uint8)

    # open
    opened = cv2.morphologyEx(mask_HSV, cv2.MORPH_OPEN, kernel, iterations = 1)

    open_img_msg = cv_bridge.cv2_to_imgmsg(opened)
    open_img_msg.header.stamp = rospy.Time.now()
    open_pub.publish(open_img_msg)

    # find largest contours
    found, contours, hierarchy = cv2.findContours(opened, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest = max(contours, key = cv2.contourArea)

    if color_image != None:
        cv2.drawContours(color_image, contours, -1, (0,255,0), 3)
        x,y,w,h = cv2.boundingRect(largest)

        cv2.rectangle(color_image,(x,y),(x+w,y+h),(0,0,255),2)

        blob_img_msg = cv_bridge.cv2_to_imgmsg(color_image, "bgr8")
        blob_img_msg.header.stamp = rospy.Time.now()
        blob_pub.publish(blob_img_msg)

if __name__=='__main__':
    main()
