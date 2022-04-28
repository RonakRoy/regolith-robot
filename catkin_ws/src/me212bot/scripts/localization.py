#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm
from queue import Queue

from me212bot.msg import WheelCmdVel
from apriltags.msg import AprilTagDetections
from geometry_msgs.msg import PoseStamped
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad

rospy.init_node('localization', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

odom_queue = Queue()
apriltag_queue = Queue()
    
def main():
    # rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    rospy.Subscriber("/odom", PoseStamped, odom_callback, queue_size = 1)
    
    thread = threading.Thread(target = thread_target)
    thread.start()

    rospy.sleep(1)    
    rospy.spin()

def thread_target():
    # publish initial transfer (guess)
    last_time = rospy.Time.now()
    pubFrame(br, pose=[0.18, 0.18, 0, 0, 0, 0], frame_id = '/robot_base', parent_frame_id = '/map')

    while not rospy.is_shutdown():
        (trans,rot) = lr.lookupTransform('/map', '/robot_base', rospy.Time(0))
        odom = odom_queue.get()

        rot = tfm.quaternionMultiply(tfm.quaternion_from_euler([0,0,odom.delta_theta]), rot)
        th = tfm.euler_from_quaternion(rot)[2]
        trans = np.array(trans) + np.array([odom.distance * np.cos(th), odom.distance * np.sin(th), 0])

        pubFrame(br, pose = trans.tolist() + list(rot), frame_id = '/robot_base', parent_frame_id = '/map')

def odom_callback(msg):
    odom_queue.put(msg)

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
    
