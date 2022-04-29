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
from sensor_msgs.msg import PointCloud
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad

rospy.init_node('navigation', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

TRAJECTORY = [
    # [1.15, 1.15,  1/4.0 * np.pi, "fwd"],
    # [1.15, 1.15,  3/4.0 * np.pi, "inp"],
    # [0.90, 1.50,  3/4.0 * np.pi, "fwd"]
    # [1.15, 1.15,  3/4.0 * np.pi, "bwd"],
    # [1.15, 1.15,  1/4.0 * np.pi, "inp"],
    # [1.83, 1.83,  1/4.0 * np.pi, "fwd"],
    # [1.83, 1.83,    0.0 * np.pi, "inp"],
    # [2.80, 1.83,    0.0 * np.pi, "fwd"],
]
    
def main():
    rospy.sleep(1)

    pile_point_subscriber = rospy.Subscriber('/object_detector/pile_location', PointCloud, pile_callback)
    
    thread = threading.Thread(target = navi_loop)
    thread.start()
    
    rospy.spin()

pile = None
def pile_callback(msg):
    global pile
    pile = msg.points[0]
    
## navigation control loop (No need to modify)
def navi_loop():
    velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)
    # traj_index = 0
    # traj_pose = TRAJECTORY[traj_index]
    
    rate = rospy.Rate(100) # 100hz
    
    wcv = WheelCmdVel()
    
    while not rospy.is_shutdown():
        X = pos[0]
        Y = pos[1]
        Th = tfm.euler_from_quaternion(ori)[2]
        
        global pile
        if pile is None:
            continue

        pos, ori = lr.lookupTransform(pile.header.frame_id, '/map', pile.header.stamp)

        Xd = pile.x
        Yd = pile.y
        Thd = Th
        # Xd  = traj_pose[0]
        # Yd  = traj_pose[1]
        # Thd = traj_pose[2]
        
        pos_delta         = np.array([Xd, Yd]) - np.array([X, Y])
        robot_heading_vec = np.array([np.cos(Th), np.sin(Th)])

        # if traj_pose[3] == "inp":
        #     pos_delta = np.array([0,0])
        
        if (np.linalg.norm(pos_delta) < 0.1 and np.fabs(diffrad(Th, Thd)) < 0.1) :
            print 'Arrived at trajectory point', traj_index
            traj_index += 1

            try:
                traj_pose = TRAJECTORY[traj_index]
            except:
                wcv.desiredWV_R = 0
                wcv.desiredWV_L = 0
                        
                velcmd_pub.publish(wcv)  
                break

        dX = np.linalg.norm(pos_delta)
        dTh = Thd - Th

        speed_multiplier = 0.2

        vel_desired = 2.0*dX + 0.75
        # if traj_pose[3] == "inp":
        #     vel_desired = 0
        # elif traj_pose[3] == "bwd":
        #     vel_desired = -vel_desired
        angVel_desired = -1.0*dTh

        wcv.desiredWV_L = speed_multiplier*(vel_desired + angVel_desired)
        wcv.desiredWV_R = speed_multiplier*(vel_desired - angVel_desired)

        print "dX:", dX, "dTh", dTh, "wL:", wcv.desiredWV_L, "wR:", wcv.desiredWV_R
                
        velcmd_pub.publish(wcv)  
        
        rate.sleep()

    wcv.desiredWV_R = 0
    wcv.desiredWV_L = 0
            
    velcmd_pub.publish(wcv)  

if __name__=='__main__':
    main()
    
