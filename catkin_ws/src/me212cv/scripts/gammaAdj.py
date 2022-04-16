#!/usr/bin/python

# 2.12 Lab 7 object detection: a node for adjusting gamma
# Luke Roberto Oct 2017
# Jacob Guggenheim 2019
# Jerry Ng 2019


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

rospy.init_node('gamma', anonymous=True)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

def main():
    rospy.Subscriber('/camera/rgb/image_raw', Image, rosHTransformCallback)
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

    # gamma adjust
    gamma = 0.5
    g05 = adjust_gamma(cv_image, gamma=gamma)
    cv2.imshow("Gamma = 0.5", g05)
    cv2.waitKey(3)

    gamma = 2
    g2 = adjust_gamma(cv_image, gamma=gamma)
    cv2.imshow("Gamma = 2", g2)
    cv2.waitKey(3)

    gamma = 3
    g3 = adjust_gamma(cv_image, gamma=gamma)
    cv2.imshow("Gamma = 3", g3)
    cv2.waitKey(3)

def adjust_gamma(image, gamma=1.0):

   invGamma = 1.0 / gamma
   table = np.array([((i / 255.0) ** invGamma) * 255
      for i in np.arange(0, 256)]).astype("uint8")

   return cv2.LUT(image, table)

if __name__=='__main__':
    main()
