#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for de-noising
# Luke Roberto Oct 2017

import rospy
import numpy as np
import cv2  # OpenCV module
from matplotlib import pyplot as plt
import time

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('canny', anonymous=True)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

def main():
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, rosHTransformCallback)
    print("Subscribing")
    rospy.spin()


def rosHTransformCallback(msg):
    # convert ROS image to opencv format
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # visualize it in a cv window
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey(3)

    #TODO: Play with these values
    lower_threshold = 10
    upper_threshold = 500
    grayIm = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    cannyIm = cv2.Canny(grayIm,lower_threshold,upper_threshold,apertureSize = 3)   # Canny edge detector to make it easier for hough transform to "agree" on lines
    cv2.imshow("Canny_Image", cannyIm)
    cv2.waitKey(3)

if __name__=='__main__':
    main()
