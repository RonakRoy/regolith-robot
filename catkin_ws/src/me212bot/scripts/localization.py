#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm
from Queue import Queue

from me212bot.msg import WheelCmdVel, DeltaRobotPose
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad

rospy.init_node('localization', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

odom_queue = Queue()
apriltag_queue = Queue()

X = 0.2
Y = 0.2
Th = np.pi/4

LOCALIZATION_FREQ = 500.0
    
def main():
    # rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    odom_sub = rospy.Subscriber("/delta_robot_pose", DeltaRobotPose, odom_callback, queue_size = 1)
    
    thread = threading.Thread(target = thread_target)
    thread.start()

    rospy.sleep(1)    
    rospy.spin()

def thread_target():
    global odom_queue
    global X
    global Y
    global Th

    # publish initial transfer (guess)    
    rospy.sleep(2)
    pubFrame(br, pose=[X, Y, 0, 0, 0, Th], frame_id = '/robot_base', parent_frame_id = '/map')

    r = rospy.Rate(LOCALIZATION_FREQ)
    previous_time = rospy.Time(0)
    current_time = rospy.Time(0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # if current_time - previous_time < rospy.Duration(1.0/LOCALIZATION_FREQ):
        #     rospy.sleep(0.001)
        #     continue

        while not odom_queue.empty():
            odom = odom_queue.get()

            Th += odom.delta_theta
            X += odom.distance * np.cos(Th)
            Y += odom.distance * np.sin(Th)

            if odom.header.stamp >= current_time:
                break

        pubFrame(br, pose=[X, Y, 0, 0, 0, Th], frame_id = '/robot_base', parent_frame_id = '/map')
        
        previous_time = current_time
        r.sleep()

def odom_callback(msg):
    odom_queue.put(msg)
    # global odom
    # odom = msg

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

            # 3000 is the max practical area, but looks pretty good.
            area = -0.5 * ((x[0]*y[1] + x[1]*y[2] + x[2]*y[3] + x[3]*y[0]) - (x[1]*y[0] + x[2]*y[1] + x[3]*y[2] + x[0]*y[3]))

            # print "found ", detection.id, " with area ", area

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

    if i_ma != -1:
        pubFrame(br, pose = poses[i_ma], frame_id = '/robot_base', parent_frame_id = '/map')

if __name__=='__main__':
    main()
    
