#!/usr/bin/python

# 2.12 Lab 7 Optical Flow
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

rospy.init_node('opticalFlow', anonymous=True)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

def nothing(x):
    pass


global firstTime, old_gray, p0, lk_params, mask, color
firstTime = True
old_gray = None
p0 = None
lk_params = None
mask = None
color = None

def main():
    rospy.Subscriber('/camera/rgb/image_raw', Image, opFlowCallback)
    print("Subscribing")
    rospy.spin()


def opFlowCallback(msg):
    # some bad coding practice
    global firstTime, old_gray, p0, lk_params, mask, color

    # convert ROS image to opencv format
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # visualize it in a cv window
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey(3)

    ################ First time: establish good features to track ####################
    if firstTime:
        ################ Some stuff for Lucas Kanade Optical Flow ####################
        # params for ShiTomasi corner detection
        feature_params = dict( maxCorners = 100, qualityLevel = .7, minDistance = 7, blockSize = 7 )

        # Parameters for lucas kanade optical flow
        lk_params = dict( winSize  = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Take first frame and find corners in it
        old_frame = cv_image
        old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
        p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

        # Create a mask image for drawing purposes
        mask = np.zeros_like(old_frame)
        color = np.random.randint(0,255,(100,3))

        firstTime = False
    else:
        frame = cv_image
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

        # Select good points
        good_new = p1[st==1]
        good_old = p0[st==1]

        # draw the tracks
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
        img = cv2.add(frame,mask)

        cv2.imshow('frame',img)
        cv2.waitKey(3)

        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)



if __name__=='__main__':
    main()
