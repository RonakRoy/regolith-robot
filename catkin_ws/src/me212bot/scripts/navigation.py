#!/usr/bin/python

# 2.12 Lab 3 AprilTag Navigation: use AprilTag to get current robot (X,Y,Theta) in world frame, and to navigate to target (X,Y,Theta)
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from me212bot.msg import WheelCmdVel
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad

rospy.init_node('navigation', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

TRAJECTORY = [
    [1.15, 1.15,  1/4.0 * np.pi],
    [1.15, 1.15,  3/4.0 * np.pi],
    [0.90, 1.50,  3/4.0 * np.pi],
    [1.15, 1.15,  3/4.0 * np.pi],
    [1.15, 1.15,  1/4.0 * np.pi],
    [1.83, 1.83,  1/4.0 * np.pi],
    [1.83, 1.83,    0.0 * np.pi],
    [2.80, 1.83,    0.0 * np.pi],
]
    
def main():
    rospy.sleep(1)
    
    thread = threading.Thread(target = navi_loop)
    thread.start()
    
    rospy.spin()
    
## navigation control loop (No need to modify)
def navi_loop():
    velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)
    traj_index = 0
    traj_pose = TRAJECTORY[traj_index]
    
    rate = rospy.Rate(100) # 100hz
    
    wcv = WheelCmdVel()
    
    while not rospy.is_shutdown():
        pos, ori = lr.lookupTransform('/map', '/robot_base', rospy.Time(0))

        X = pos[0]
        Y = pos[1]
        Th = tfm.euler_from_quaternion(ori)[2]
        
        Xd  = traj_pose[0]
        Yd  = traj_pose[1]
        Thd = traj_pose[2]
        
        pos_delta         = np.array([Xd, Yd]) - np.array([X, Y])
        robot_heading_vec = np.array([np.cos(Th), np.sin(Th)])
        
        if (np.linalg.norm(pos_delta) < 0.08 and np.fabs(diffrad(Th, Thd)) < 0.05) :
            print 'Arrived at trajectory point', traj_index
            wcv.desiredWV_R = 0  
            wcv.desiredWV_L = 0

        dX = np.linalg.norm(pos_delta)
        dTh = Thd - Th

        print "dX:", dX, "dTh", dTh

        vel_desired = 0.75*dX + .25
        angVel_desired = -0.50*dTh

        wcv.desiredWV_R = vel_desired - angVel_desired
        wcv.desiredWV_L = vel_desired + angVel_desired
                
        velcmd_pub.publish(wcv)  
        
        rate.sleep()

if __name__=='__main__':
    main()
    
