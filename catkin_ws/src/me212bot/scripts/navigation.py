#!/usr/bin/python

# 2.12 Lab 3 AprilTag Navigation: use AprilTag to get current robot (X,Y,Theta) in world frame, and to navigate to target (X,Y,Theta)
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm


from me212bot.msg import WheelCmdVel, LocalizationMode, PbarPose, ScoopPose
from apriltags.msg import AprilTagDetections
from sensor_msgs.msg import PointCloud
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad
from localization_mode import ODOM_ONLY, APRILTAG_ONLY
from std_srvs.srv import Trigger, TriggerRequest

rospy.init_node('navigation', anonymous=True)

lr = tf.TransformListener()
br = tf.TransformBroadcaster()

DRIVE_FWD = 0
DRIVE_BWD = 1
TURN_TO_FACE = 2
LOCATE_PILE = 3
DRIVE_TO_PILE = 4
WAIT = 5
BLIND = 6
PBAR_SCOOP = 7

face_x_pile = [TURN_TO_FACE,   2.4, 0.0,       ODOM_ONLY]
face_y_pile = [TURN_TO_FACE,   0.0, 2.4,       ODOM_ONLY]

corner_face_x_pile = [TURN_TO_FACE,   2.4, 1.2,         ODOM_ONLY]
corner_face_y_pile = [TURN_TO_FACE,   1.2, 2.4,         ODOM_ONLY]


# TRAJECTORY SEGMENTS
PILE_IN_N_OUT = [
    [WAIT,                             1.0, ODOM_ONLY],
    [LOCATE_PILE,                      2.0, ODOM_ONLY],
    [DRIVE_TO_PILE,                    5.0, ODOM_ONLY],
    [PBAR_SCOOP,     0, 0, 1700,      -1,   ODOM_ONLY],
    [BLIND,         -1, -1,            0.5, ODOM_ONLY],
    [WAIT,                             0.25,ODOM_ONLY],
    [WAIT,                             0.25,APRILTAG_ONLY],
    [PBAR_SCOOP,     0, 1000, 2700,   -1,   ODOM_ONLY],
    [DRIVE_BWD,      1.1, 1.1,        -1,   ODOM_ONLY],
]

GO_TO_BOX_AND_DEPOSIT = [
    [TURN_TO_FACE,   1.6, 1.6,        -1,   ODOM_ONLY],
    [DRIVE_FWD,      1.6, 1.6,        -1,   ODOM_ONLY],
    [WAIT,                             0.5, ODOM_ONLY],
    [WAIT,                             0.25,APRILTAG_ONLY],
    [TURN_TO_FACE,   2.0, 2.0,        -1,   ODOM_ONLY],
    [DRIVE_FWD,      2.0, 2.0,        -1,   ODOM_ONLY],
    [WAIT,                             0.5, ODOM_ONLY],
    [WAIT,                             0.25,APRILTAG_ONLY],
    [TURN_TO_FACE,   2.8, 2.0,        -1,   ODOM_ONLY],
    [BLIND,          1.0, 1.0,         3.0, APRILTAG_ONLY],
    [PBAR_SCOOP,     0, 1000, 500,    -1,   APRILTAG_ONLY],
    [PBAR_SCOOP,     0, -200, -700,  -1,    APRILTAG_ONLY],
    [PBAR_SCOOP,     0,    0, -1500,  -1,   APRILTAG_ONLY],
    [PBAR_SCOOP,     0, 1000, -1500,  -1,   APRILTAG_ONLY],
    [WAIT,                             0.25,ODOM_ONLY],
    [WAIT,                             0.25,APRILTAG_ONLY],
]

BACK_OUT_TO_MIDDLE = [
    [PBAR_SCOOP,     0, 0, -1500,     -1,     APRILTAG_ONLY],
    [PBAR_SCOOP,     0, 1300, 1300,           APRILTAG_ONLY],
    [BLIND,         -1.25, -1,         0.75,   ODOM_ONLY],
    [WAIT,                             0.25,  ODOM_ONLY],
    [WAIT,                             0.25,  APRILTAG_ONLY],
    [TURN_TO_FACE,   2.4, 2.4,        -1,     ODOM_ONLY],
    [WAIT,                             0.25,  ODOM_ONLY],
    [WAIT,                             0.25,  APRILTAG_ONLY],
    [DRIVE_BWD,      1.8, 1.8,        -1,     ODOM_ONLY],
    [WAIT,                             0.25,  ODOM_ONLY],
    [WAIT,                             0.25,  APRILTAG_ONLY],
    [DRIVE_BWD,      1.1, 1.1,        -1,     ODOM_ONLY],
]

# Build the trajectory
TRAJECTORY = [
    [DRIVE_FWD,      1.1, 1.1,        -1,   ODOM_ONLY],
    face_x_pile,
]
TRAJECTORY += PILE_IN_N_OUT + [corner_face_x_pile] + GO_TO_BOX_AND_DEPOSIT + BACK_OUT_TO_MIDDLE
TRAJECTORY += [
    face_y_pile,
    [PBAR_SCOOP,     0, 0, 0,                 ODOM_ONLY],
]
TRAJECTORY += PILE_IN_N_OUT + [corner_face_y_pile] + GO_TO_BOX_AND_DEPOSIT + BACK_OUT_TO_MIDDLE

loc_mod_pub = rospy.Publisher("/localization_mode", LocalizationMode, queue_size = 1)

velcmd_pub = rospy.Publisher("/cmd/drive", WheelCmdVel, queue_size = 1)
pbar_pub = rospy.Publisher("/cmd/pbar", PbarPose, queue_size = 1)
scoop_pub = rospy.Publisher("/cmd/scoop", ScoopPose, queue_size = 1)

def main():
    rospy.sleep(1)

    pile_point_subscriber = rospy.Subscriber('/object_detector/pile_location', PointCloud, pile_callback)

    pbar_sub = rospy.Subscriber('/hardware/pose/pbar', PbarPose, pbar_callback)
    scoop_sub = rospy.Subscriber('/hardware/pose/scoop', ScoopPose, scoop_callback)

    thread = threading.Thread(target = navi_loop)
    thread.start()
    
    rospy.spin()

pile = None
def pile_callback(msg):
    global pile
    pile = msg.points[0]

pbar_pos = 0
def pbar_callback(msg):
    global pbar_pos
    pbar_pos = msg.pbar

scoop_wrist_pos = 0
scoop_jaw_pos = 0
def scoop_callback(msg):
    global scoop_wrist_pos
    scoop_wrist_pos = msg.wrist
    global scoop_jaw_pos
    scoop_jaw_pos = msg.jaw

## navigation control loop (No need to modify)
def navi_loop():
    print "Starting trajectory navigation thread."

    rate = rospy.Rate(100) # 100hz
        
    dX = None
    global pile

    global pbar_pos
    global scoop_wrist_pos
    global scoop_jaw_pos

    pbar_target = 0

    traj_index = 0
    traj_cmd = TRAJECTORY[traj_index]

    traj_pt_start_time = rospy.Time.now()
    loc_mode = LocalizationMode()

    loc_mode.mode = traj_cmd[-1]
    loc_mod_pub.publish(loc_mode)

    err_inc_count = 0

    pile_samples = []

    while not rospy.is_shutdown():
        wcv = WheelCmdVel()

        pos, ori = lr.lookupTransform('/map', '/robot_base', rospy.Time(0))

        move_on = False

        X = pos[0]
        Y = pos[1]
        Th = tfm.euler_from_quaternion(ori)[2]

        if traj_cmd[-2] > 0 and (rospy.Time.now()-traj_pt_start_time) >= rospy.Duration(traj_cmd[-2]):
            move_on = True

        if traj_cmd[0] == PBAR_SCOOP:
            if np.abs(traj_cmd[1] - pbar_target) > 2.0:
                pbar_target += 0.50 * np.sign(traj_cmd[1] - pbar_pos)

            pbar_pose = PbarPose()
            pbar_pose.pbar = int(pbar_target)
            pbar_pub.publish(pbar_pose)

            scoop_pose = ScoopPose()
            scoop_pose.wrist = traj_cmd[2]
            scoop_pose.jaw = traj_cmd[3]
            scoop_pub.publish(scoop_pose)

            if abs(pbar_target - traj_cmd[1]) > 2.0 or scoop_wrist_pos != traj_cmd[2] or scoop_jaw_pos != traj_cmd[3]:
                continue

            move_on = True

        elif traj_cmd[0] == LOCATE_PILE:
            if len(pile_samples) < 100:
                if pile is not None:
                    pile_samples.append((pile.x,pile.y))
            else:
                pile_med = np.median(pile_samples, axis=0)

                Xd = pile_med[0]
                Yd = pile_med[1]

                move_on = True

        elif traj_cmd[0] == WAIT:
            pass

        elif traj_cmd[0] == BLIND:
            wcv.desiredWV_R = traj_cmd[2]
            wcv.desiredWV_L = traj_cmd[1]
                    
            velcmd_pub.publish(wcv)
            
        else:
            if traj_cmd[0] != DRIVE_TO_PILE:
                Xd = traj_cmd[1]
                Yd = traj_cmd[2]
            
            dir_mult = -1 if traj_cmd[0] == DRIVE_BWD else 1

            robot_heading_vec = np.array([np.cos(Th), np.sin(Th)])
            pos_delta = np.array([Xd, Yd]) - np.array([X, Y])
            heading_err_cross = cross2d(robot_heading_vec, dir_mult * pos_delta / np.linalg.norm(pos_delta))

            if traj_cmd[0] == TURN_TO_FACE:
                move_on = move_on or np.fabs(heading_err_cross) < 0.05
            else:
                move_on = move_on or np.linalg.norm(pos_delta) < 0.1 or err_inc_count >= 25

            if traj_cmd[0] == TURN_TO_FACE:
                vel_desired = 0
                angVel_desired = -np.sign(heading_err_cross)
            else:
                dX_old = dX
                dX = np.linalg.norm(pos_delta)

                if dX > dX_old:
                    err_inc_count += 1
                else:
                    err_inc_count = 0

                if traj_cmd[0] == DRIVE_TO_PILE:
                    vel_desired = 3
                else:
                    vel_desired = dir_mult * min(2.0, 10.0*dX)
                angVel_desired = -heading_err_cross

            wcv.desiredWV_L = 0.25*(vel_desired + angVel_desired)
            wcv.desiredWV_R = 0.25*(vel_desired - angVel_desired)
                    
            velcmd_pub.publish(wcv)

        if move_on:
            print 'Completed trajectory action', traj_index
            traj_index += 1

            wcv.desiredWV_R = 0
            wcv.desiredWV_L = 0
            velcmd_pub.publish(wcv)

            try:
                traj_cmd = TRAJECTORY[traj_index]
                traj_pt_start_time = rospy.Time.now()

                loc_mode.mode = traj_cmd[-1]
                loc_mod_pub.publish(loc_mode)

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
    
