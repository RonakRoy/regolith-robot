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

DRIVE_FWD = 0
DRIVE_BWD = 1
TURN_IN_PLACE = 2
DRIVE_TO_PILE = 3

TRAJECTORY = [
    [DRIVE_FWD,      1/4.0 * np.pi, 1.15, 1.15],
    [TURN_IN_PLACE, -1/4.0 * np.pi],
    [DRIVE_TO_PILE]
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
    traj_index = 0
    traj_cmd = TRAJECTORY[traj_index]
    
    rate = rospy.Rate(100) # 100hz
    
    wcv = WheelCmdVel()
    
    dX = None
    dTh = None
    global pile
    while not rospy.is_shutdown():
        pos, ori = lr.lookupTransform('/map', '/robot_base', rospy.Time(0))

        X = pos[0]
        Y = pos[1]
        Th = tfm.euler_from_quaternion(ori)[2]

        if traj_cmd[0] != DRIVE_TO_PILE:
            Thd = traj_cmd[1]

            if traj_cmd[0] != TURN_IN_PLACE:
                Xd  = traj_cmd[2]
                Yd  = traj_cmd[3]
        else:
            if pile is None or (dX is not None and dX <= 1.0):
                continue
            Xd = pile.x
            Yd = pile.y
            Thd = Th
            
        if traj_cmd[0] == TURN_IN_PLACE:
            pos_delta = np.array([0,0])
        else:
            pos_delta = np.array([Xd, Yd]) - np.array([X, Y])

        if (np.linalg.norm(pos_delta) < 0.1 and np.fabs(diffrad(Th, Thd)) < 0.1) :
            print 'Arrived at trajectory point', traj_index
            traj_index += 1

            try:
                traj_cmd = TRAJECTORY[traj_index]
            except:
                print 'Reached end of trajectory.'
                wcv.desiredWV_R = 0
                wcv.desiredWV_L = 0
                        
                velcmd_pub.publish(wcv)  
                break

        dX = (-1 if traj_cmd[0] == DRIVE_BWD else 1) * np.linalg.norm(pos_delta)
        dTh = Thd - Th

        vel_desired = 2.0*dX + 0.75 if traj_cmd[0] != TURN_IN_PLACE else 0
        angVel_desired = -1.5*dTh

        speed_multiplier = 1.0
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
    
