#!/usr/bin/python

# 2.12 Lab 7 object detection: a node for otsu
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

rospy.init_node('otsu', anonymous=True)

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

    # otsu
    grayIm = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    blurIm = cv2.GaussianBlur(grayIm,(15,5),0)
    blurThresh,otsuImage = cv2.threshold(blurIm,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    # show images
    images = [blurIm, 0, otsuImage]
    for i in xrange(1):
        plt.subplot(1,3,i*3+1), plt.imshow(images[i*3], 'gray'),
        plt.subplot(1,3,i*3+2), plt.hist(images[i*3].ravel(),256),  plt.axvline(x=blurThresh, color='c')
        plt.subplot(1,3,i*3+3), plt.imshow(images[i*3+2], 'gray')
    plt.show()
    time.sleep(3)
    plt.close()

if __name__=='__main__':
    main()
