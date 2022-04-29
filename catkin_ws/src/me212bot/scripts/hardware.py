#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import Quaternion

import helper
from me212bot.msg import WheelCmdVel, DeltaRobotPose

odom_publisher = rospy.Publisher('/delta_robot_pose', DeltaRobotPose, queue_size = 1)

comms = []
try:
    comms.append(serial.Serial('/dev/ttyACM0', 115200, timeout=5))
except:
    pass

try:
    comms.append(serial.Serial('/dev/ttyACM1', 115200, timeout=5))
except:
    pass

try:
    comms.append(serial.Serial('/dev/ttyACM2', 115200, timeout=5))
except:
    pass

drive_port = None
pbar_port = None
scoop_port = None

## main function (Need to modify)
def main():
    rospy.init_node('me212bot', anonymous=True)

    global drive_port
    global pbar_port
    global scoop_port
    for i in range(len(comms)):
        for j in range(10):
            serial_data = comms[i].readline().strip()
            try:
                split_data = serial_data.split(',')
                if split_data[0] == "DRIVE":
                    print "Found drive arduino."
                    drive_port = i
                    break
                elif split_data[1] == "PBAR":
                    print "Found pbar arduino."
                    pbar_port = i
                    break
                elif split_data[1] == "SCOOP":
                    print "Found scoop arduino."
                    scoop_port = i
                    break
            except:
                pass

    drive_thread = threading.Thread(target = drive_thread_target)
    drive_thread.start()

    pbar_thread = threading.Thread(target = pbar_thread_target)
    pbar_thread.start()

    scoop_thread = threading.Thread(target = scoop_thread_target)
    scoop_thread.start()

    ## 1. Initialize a subscriber
    rospy.Subscriber('/cmdvel', WheelCmdVel, cmdvel_callback)
    rospy.spin()


## msg handling function (Need to modify)
def cmdvel_callback(msg):  
    ## 2. Send msg.desiredWV_R and msg.desiredWV_L to Arduino.
    strCmd = str(msg.desiredWV_R) + ',' + str(msg.desiredWV_L) + '\n'
    comms[drive_port].write(strCmd)

def drive_thread_target():
    if drive_port is not None:
        drive_arduino = comms[drive_port]

        while not rospy.is_shutdown():
            raw_data = drive_arduino.readline().strip()
            data = raw_data.split(',')
            
            try:
                dist = float(data[1])
                dth = float(data[2])
                
                # publish odometry as Pose msg
                odom = DeltaRobotPose()
                odom.header.stamp = rospy.Time.now()
                odom.distance = dist
                odom.delta_theta = dth

                odom_publisher.publish(odom)
            except:
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', raw_data
            
def pbar_thread_target():
    pbar_arduino = None

    for i in range(3):
        try:
            serial_comm = serial.Serial('/dev/ttyACM{}'.format(i), 115200)

            serial_data = serial_comm.readline()
            if serial_data == "PBAR\n":
                pbar_arduino = serial_comm
                drive_arduino.write("LOCK\n")
                print "FOUND PBAR ARDUINO"
        except:
            pass

    prevtime = rospy.Time.now()
    if pbar_arduino is not None:
        while not rospy.is_shutdown():
            serialData = pbar_arduino.readline()

            try: 
                pbar_angle = float(splitData[0])

                print 'pbar=', pbar_angle
            except:
                print 'Cannot parse', splitData


def scoop_thread_target():
    scoop_arduino = None

    for i in range(3):
        try:
            serial_comm = serial.Serial('/dev/ttyACM{}'.format(i), 115200, timeout = 5)

            serial_data = serial_comm.readline()
            if serial_data == "SCOOP":
                scoop_arduino = serial_comm
                drive_arduino.write("LOCK\n")
                print "FOUND SCOOP ARDUINO"
        except:
            pass

    prevtime = rospy.Time.now()
    if scoop_arduino is not None:
        while not rospy.is_shutdown():
            serialData = pbar_arduino.readline()
            splitData = serialData.split(',')

            try:
                wrist = float(splitData[0])
                jaw = float(splitData[1])
                
                print 'wrist=', wrist, ' jaw=', jaw

            except:
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', splitData

if __name__=='__main__':
    main()


