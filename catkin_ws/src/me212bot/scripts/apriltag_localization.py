#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from me212bot.msg import WheelCmdVel
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad


rospy.init_node('apriltag_navi', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()
    
def main():
    apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    
    rospy.sleep(1)    
    rospy.spin()

## apriltag msg handling function (Need to modify for Task 2)
def apriltag_callback(data):
    # use apriltag pose detection to find where is the robot
    ids = []
    poses = []
    areas = []

    max_area = 0
    i_ma = -1

    for detection in data.detections:
        if detection.id in [2,7,4,0]:
            x = [corner.x for corner in detection.corners2d]
            y = [corner.y for corner in detection.corners2d]

            print x, y

            area = -0.5 * ((x[0]*y[1] + x[1]*y[2] + x[2]*y[3] + x[3]*y[0]) - (x[1]*y[0] + x[2]*y[1] + x[3]*y[2] + x[0]*y[3]))

            print "found ", detection.id, " with area ", area

            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(lr, poselist_tag_cam, 'apriltag_ref', 'robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag_{}'.format(detection.id), 'map')

            if area > max_area:
                max_area = area
                i_ma = len(areas)

            ids.append(detection.id)
            areas.append(area)
            poses.append(poselist_base_map)

    print "Found apriltag ids ", ids

    if i_ma != -1:
        print "Using apriltag with id ", ids[i_ma]
        pubFrame(br, pose = poses[i_ma], frame_id = '/robot_base', parent_frame_id = '/map')

if __name__=='__main__':
    main()
    
