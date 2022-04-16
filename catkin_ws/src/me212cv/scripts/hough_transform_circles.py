#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for de-noising
# Luke Roberto Oct 2017
# Jacob Guggenheim March 2019  
# Jerry Ng March 2019

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

rospy.init_node('hough', anonymous=True)

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

    # gray and blur
    resizeIm = cv2.resize(cv_image, (0,0), fx=0.25, fy=0.25)
    blurIm = cv2.medianBlur(resizeIm,5)
    grayIm = cv2.cvtColor(blurIm,cv2.COLOR_BGR2GRAY) # Convert to grascale image

    # hough
    #TODO: Play with these values
    arg1 = 100      #Upper threshold for Canny edge detection
    arg2 = 50       #Threshold for center detection. 
    min_distance = 20    # Minimum distance between new circle centers. 
    dp = 1.2             # How accepting to degradation of circles are you willing to be
    circles = cv2.HoughCircles(grayIm,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2, minRadius=0,maxRadius=60)
    print(circles)
    if circles is not None:
        circles = (np.around(circles))
        for i in circles[0,:]:
            # draw circle
            cv2.circle(resizeIm,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(resizeIm,(i[0],i[1]),2,(0,0,255),3)


    cv2.imshow("Hough Circle Detection", resizeIm)
    cv2.waitKey(3)

if __name__=='__main__':
    main()
