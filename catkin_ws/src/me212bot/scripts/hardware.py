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

## main function (Need to modify)
def main():
    rospy.init_node('me212bot', anonymous=True)
    
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
    drive_arduino.write(strCmd)

def drive_thread_target():
    drive_arduino = None

    for i in range(3):
        try:
            serial_comm = serial.Serial('/dev/ttyACM{}'.format(i), 115200)
            print("Connected.")

            for i in range(10):
                serial_data = serial_comm.readline()
                print serial_data
                if serial_data.strip() == "DRIVE":
                    drive_arduino = serial_comm
                    drive_arduino.write("LOCK")
                    print "FOUND DRIVE ARDUINO"
                    break   
        except:
            pass

    if drive_arduino is not None:
        while not rospy.is_shutdown():
            serialData = drive_arduino.readline()
            splitData = serialData.split(',')
            
            try:
                dist = float(splitData[0])
                dth = float(splitData[1])
                
                # publish odometry as Pose msg
                odom = DeltaRobotPose()
                odom.header.stamp = rospy.Time.now()
                odom.distance = dist
                odom.delta_theta = dth

                odom_publisher.publish(odom)
            except:
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', splitData
            
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


