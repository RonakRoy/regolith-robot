#!/usr/bin/python

# 2.12 Lab 7 object detection: a node for observing erosion/dilation
# Jacob Guggenheim 2019
# Jerry Ng 2019

import rospy
import numpy as np
import cv2  # OpenCV module
from matplotlib import pyplot as plt
import time
from Tkinter import *
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math


rospy.init_node('morphOps', anonymous=True)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()
tk = Tk()
l_h = Scale(tk, from_ = 0, to = 255, label = 'Hue, lower', orient = HORIZONTAL)
l_h.pack()
u_h = Scale(tk, from_ = 0, to = 255, label = 'Hue, upper', orient = HORIZONTAL)
u_h.pack()
u_h.set(255)
l_s = Scale(tk, from_ = 0, to = 255, label = 'Saturation, lower', orient = HORIZONTAL)
l_s.pack()
u_s = Scale(tk, from_ = 0, to = 255, label = 'Saturation, upper', orient = HORIZONTAL)
u_s.pack()
u_s.set(255)
l_v = Scale(tk, from_ = 0, to = 255, label = 'Value, lower', orient = HORIZONTAL)
l_v.pack()
u_v = Scale(tk, from_ = 0, to = 255, label = 'Value, upper', orient = HORIZONTAL)
u_v.pack()
u_v.set(255)

def main():
    rospy.Subscriber('/camera/rgb/image_raw', Image, morphOpsCallback)
    print("Subscribing")
    mainloop()


def morphOpsCallback(msg):
    # convert ROS image to opencv format
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # visualize it in a cv window
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey(3)

    ################ HSV THRESHOLDING ####################
    # convert to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # get threshold values
    lower_bound_HSV = np.array([l_h.get(), l_s.get(), l_v.get()])
    upper_bound_HSV = np.array([u_h.get(), u_s.get(), u_v.get()])

    # threshold
    mask_HSV = cv2.inRange(hsv_image, lower_bound_HSV, upper_bound_HSV)

    # display image
    cv2.imshow("HSV_Thresholding", mask_HSV)
    cv2.waitKey(3)

    # kernel for all morphological operations
    #TODO: Change size of kernel
    # Also, try changing the shape of the kernel (places 1's in certain locations). Try making a circle/line/etc.
    kernel = np.ones((7,7),np.uint8)

    # EXAMPLE OF A VERTICAL LINE:
    #kernel = np.array([[0, 1, 0, 0],\
    #                [0, 1, 0, 0 ],\
    #                [0, 1, 0, 0 ],\
    #                [0, 1, 0, 0]], dtype=np.uint8)
    
    #TODO: Change number of iterations to see the effect.
    num_iterations = 3
    ################ Erosion ####################
    # erode blobs
    erosion = cv2.erode(mask_HSV,kernel,iterations = num_iterations)

    # display image
    cv2.imshow("Erosion", erosion)
    cv2.waitKey(3)

    ################ Dilation ####################
    # dilate blobs
    dilation = cv2.dilate(mask_HSV,kernel,iterations = num_iterations)

    # display image
    cv2.imshow("Dilation", dilation)
    cv2.waitKey(3)

    ################ Opening ####################
    # good for removing noise. its an erosion (to get rid of noise) followed by a dilation (to get back the original blobs you wanted to keep)
    opening = cv2.morphologyEx(mask_HSV, cv2.MORPH_OPEN, kernel, iterations = num_iterations)

    # display image
    cv2.imshow("Opening - Get rid of noise", opening)
    cv2.waitKey(3)

    ################ Closing ####################
    # good for filling small holes in blobs. its a dilation (to fill the holes) followed by an erosion (to get the object back to the right size)
    closing = cv2.morphologyEx(mask_HSV, cv2.MORPH_CLOSE, kernel, iterations = num_iterations)

    # display image
    cv2.imshow("Closing - Fill in blobs", closing)
    cv2.waitKey(3)


if __name__=='__main__':
    main()
