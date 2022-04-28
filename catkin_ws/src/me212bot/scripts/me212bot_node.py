#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion

import helper
from me212bot.msg import WheelCmdVel

drive_arduino = None
pbar_arduino = None
scoop_arduino = None

odom_publisher = rospy.Subscriber('/odom', Pose, queue_size = 1)

## main function (Need to modify)
def main():
    rospy.init_node('me212bot', anonymous=True)
    
    arduino_thread = threading.Thread(target = arduino_thread_target)
    arduino_thread.start()
    
    ## 1. Initialize a subscriber
    rospy.Subscriber('/cmdvel', WheelCmdVel, cmdvel_callback)
    
    rospy.spin()


## msg handling function (Need to modify)
def cmdvel_callback(msg):  
    ## 2. Send msg.desiredWV_R and msg.desiredWV_L to Arduino.
    strCmd = str(msg.desiredWV_R) + ',' + str(msg.desiredWV_L) + '\n'
    drive_arduino.write(strCmd)

def arduino_thread_target():
    prevtime = rospy.Time.now()

    for i in range(3):
        serial_comm = serial.Serial('/dev/ttyACM{}'.format(i), 115200, timeout = 5)

        serial_data = serial_comm.readline()
        if serial_data == "DRIVE":
            drive_arduino = serial_comm
            drive_arduino.write("LOCK\n")
            print "FOUND DRIVE ARDUINO"
        elif serial_data == "PBAR":
            pbar_arduino = serial_comm
            drive_arduino.write("LOCK\n")
            print "FOUND PBAR ARDUINO"
        elif serial_data == "SCOOP":
            scoop_arduino = serial_comm
            drive_arduino.write("LOCK\n")
            print "FOUND SCOOP ARDUINO"

    while not rospy.is_shutdown():
        if drive_arduino is not None:
            serialData = drive_arduino.readline()
            splitData = serialData.split(',')
            
            try:
                x     = float(splitData[0])
                y     = float(splitData[1])
                theta = float(splitData[2])
                
                hz    = 1.0 / (rospy.Time.now().to_sec() - prevtime.to_sec())
                prevtime = rospy.Time.now()
                
                # print 'x=', x, ' y=', y, ' theta =', theta, ' hz =', hz
                
                # publish odometry as Pose msg
                odom = Pose()
                odom.position.x = x
                odom.position.y = y
                qtuple = tfm.quaternion_from_euler(0, 0, theta)
                odom.orientation = Quaternion(qtuple[0], qtuple[1], qtuple[2], qtuple[3])

                odom_publisher.publish(odom)
            except:
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', splitData

        if pbar_arduino is not None:
            serialData = pbar_arduino.readline()

            try: 
                pbar_angle = float(splitData[0])

                print 'pbar=', pbar_angle
            except:
                print 'Cannot parse', splitData


        if scoop_arduino is not None:
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


