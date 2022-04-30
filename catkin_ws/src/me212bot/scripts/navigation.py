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
from std_srvs.srv import Trigger, TriggerRequest

rospy.init_node('navigation', anonymous=True)
rospy.wait_for_service('/localize')
localize_serv = rospy.ServiceProxy('/localize', Trigger)

lr = tf.TransformListener()
br = tf.TransformBroadcaster()

DRIVE_FWD = 0
DRIVE_BWD = 1
TURN_IN_PLACE = 2
LOCATE_PILE = 3
DRIVE_TO_PILE = 4
LOCALIZE = 5
BLIND = 6

TRAJECTORY = [
    [DRIVE_FWD,      1.1, 1.1],
    [TURN_IN_PLACE, -1/4.0 * np.pi],
    [LOCATE_PILE],
    [DRIVE_TO_PILE],
    [LOCALIZE],
    [BLIND,         -5, -5, 1],
    [LOCALIZE],
    [TURN_IN_PLACE,  1/4.0 * np.pi],
    [DRIVE_FWD,      1.8, 1.8],
    [LOCALIZE],
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
    velcmd_pub = rospy.Publisher("/hardware/cmd_drive", WheelCmdVel, queue_size = 1)
    traj_index = 0
    traj_cmd = TRAJECTORY[traj_index]
    
    rate = rospy.Rate(100) # 100hz
        
    dX = None
    dTh = None
    global pile

    traj_pt_start_time = rospy.Time.now()

    pile_samples = []
    while not rospy.is_shutdown():
        wcv = WheelCmdVel()

        pos, ori = lr.lookupTransform('/map', '/robot_base', rospy.Time(0))

        move_on = False

        X = pos[0]
        Y = pos[1]
        Th = tfm.euler_from_quaternion(ori)[2]

        if traj_cmd[0] == LOCATE_PILE:
            if (rospy.Time.now()-traj_pt_start_time) < rospy.Duration(0.5):
                continue

            if len(pile_samples) < 100:
                if pile is not None:
                    pile_samples.append((pile.x,pile.y))
            else:
                pile_med = np.median(pile_samples, axis=0)

                Xd = pile_med[0]
                Yd = pile_med[1]
                Thd = Th

                move_on = True

        elif traj_cmd[0] == LOCALIZE:
            if (rospy.Time.now()-traj_pt_start_time) < rospy.Duration(0.25):
                continue

            loc_req = TriggerRequest()
            result = localize_serv(loc_req)

            move_on = True

        elif traj_cmd[0] == BLIND:
            wcv.desiredWV_R = traj_cmd[2]
            wcv.desiredWV_L = traj_cmd[1]
                    
            velcmd_pub.publish(wcv)

            if (rospy.Time.now()-traj_pt_start_time) >= rospy.Duration(traj_cmd[3]):
                move_on = True
            
        else:
            if traj_cmd[0] == TURN_IN_PLACE:
                Thd = traj_cmd[1]
                done = np.fabs(diffrad(Th,Thd) < 0.1)
            else:
                Xd = traj_cmd[1]
                Yd = traj_cmd[2]
                
                dir_mult = -1 if traj_cmd[0] == DRIVE_BWD else 1

                robot_heading_vec = np.array([np.cos(Th), np.sin(Th)])
                pos_delta = np.array([Xd, Yd]) - np.array([X, Y])
                heading_err_cross = cross2d(dir_mult * robot_heading_vec, pos_delta / np.linalg.norm(pos_delta))

                done = np.linalg.norm(pos_delta) < 0.1 and abs(heading_err_cross) < 0.1

            if traj_cmd[0] == DRIVE_TO_PILE and (rospy.Time.now()-traj_pt_start_time) > rospy.Duration(5):
                done = True

            if done:
                move_on = True

            if traj_cmd[0] == TURN_IN_PLACE:
                vel_desired = 0
                angVel_desired = -1.5*diffrad(Thd, Th)
            else:
                dX = np.linalg.norm(pos_delta)
                vel_desired = dir_mult * (2.0*dX + 0.75) if traj_cmd[0] != TURN_IN_PLACE else 0
                angVel_desired = -1*heading_err_cross

            speed_multiplier = 3.0 if traj_cmd[0] == DRIVE_TO_PILE else 1.0
            wcv.desiredWV_L = speed_multiplier*(vel_desired + angVel_desired)
            wcv.desiredWV_R = speed_multiplier*(vel_desired - angVel_desired)

            # print "dX:", dX, "dTh", dTh, "wL:", wcv.desiredWV_L, "wR:", wcv.desiredWV_R
                    
            velcmd_pub.publish(wcv)

        if move_on:
            print 'Arrived at trajectory point', traj_index
            traj_index += 1

            wcv.desiredWV_R = 0
            wcv.desiredWV_L = 0
            velcmd_pub.publish(wcv)

            try:
                traj_cmd = TRAJECTORY[traj_index]
                traj_pt_start_time = rospy.Time.now()

                pile_samples = []
            except:
                print 'Reached end of trajectory.'
                break
        
        rate.sleep()

    wcv.desiredWV_R = 0
    wcv.desiredWV_L = 0
            
    velcmd_pub.publish(wcv)  

if __name__=='__main__':
    main()
    
