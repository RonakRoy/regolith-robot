#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm
from Queue import Queue

from me212bot.msg import WheelCmdVel, DeltaRobotPose, LocalizationMode
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad
from localization_mode import ODOM_ONLY, APRILTAG_ONLY
from std_srvs.srv import Trigger, TriggerResponse

odom_queue = Queue()
apriltag_queue = Queue()

X = 0.2
Y = 0.2
Th = np.pi/4

LOCALIZATION_FREQ = 500.0

rospy.init_node('localization', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

localization_mode = ODOM_ONLY

def main():
    rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    rospy.Subscriber("/delta_robot_pose", DeltaRobotPose, odom_callback, queue_size = 1)
    rospy.Subscriber("/localization_mode", LocalizationMode, mode_callback, queue_size = 1)
    
    thread = threading.Thread(target = thread_target)
    thread.start()

    rospy.sleep(1)
    rospy.spin()

def thread_target():
    global odom_queue
    global apriltag_queue
    global april_tagging

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

        if localization_mode == ODOM_ONLY:
            while not apriltag_queue.empty(): apriltag_queue.get()

            while not odom_queue.empty():
                odom = odom_queue.get()

                Th += odom.delta_theta
                X += odom.distance * np.cos(Th)
                Y += odom.distance * np.sin(Th)

                if odom.header.stamp >= current_time:
                    break
        elif localization_mode == APRILTAG_ONLY:
            while not odom_queue.empty(): odom_queue.get()

            april_tag_poses = []
            while not apriltag_queue.empty():
                april_tag_poses.append(apriltag_queue.get())

            avg_pose = np.average(april_tag_poses, axis=0)
            norm_quat = avg_pose[3:] / np.linalg.norm(avg_pose[3:])

            X = avg_pose[0]
            Y = avg_pose[1]
            Th = tfm.euler_from_quaternion(norm_quat)[2]
            
        pubFrame(br, pose=[X, Y, 0, 0, 0, Th], frame_id = '/robot_base', parent_frame_id = '/map')
        
        previous_time = current_time
        r.sleep()

def mode_callback(msg):
    global localization_mode
    localization_mode = msg.mode

def odom_callback(msg):
    global localization_mode
    if localization_mode != ODOM_ONLY: return

    odom_queue.put(msg)

def apriltag_callback(data):
    global localization_mode
    if localization_mode != APRILTAG_ONLY: return

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

    if i_ma == -1: return

    global apriltag_queue
    apriltag_queue.put(poses[i_ma])

if __name__=='__main__':
    main()
    
