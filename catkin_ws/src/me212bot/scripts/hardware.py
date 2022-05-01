#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import Quaternion

import helper
from me212bot.msg import WheelCmdVel, DeltaRobotPose, PbarPose, ScoopPose

odom_publisher = rospy.Publisher('/delta_robot_pose', DeltaRobotPose, queue_size = 1)
pbar_publisher = rospy.Publisher('/pbar_pose', PbarPose, queue_size=1)
scoop_publisher = rospy.Publisher('/scoop_pose', ScoopPose, queue_size=1)

drive_arduino = None
pbar_arduino = None
scoop_arduino = None

## main function (Need to modify)
def main():
    rospy.init_node('me212bot', anonymous=True)

    print "\n\nSTARTING HARDWARE NODE..."

    global drive_arduino
    global pbar_arduino
    global scoop_arduino
    for i in range(3):
        serial_comm = None
        try:
            port = '/dev/ttyACM{}'.format(i)
            serial_comm = serial.Serial(port, 115200, timeout=5)
            print "Connected to", port
        except:
            break

        for j in range(10):
            try:
                serial_data = serial_comm.readline().strip()
                split_data = serial_data.split(',')
                if split_data[0] == "DRIVE":
                    print "Found drive arduino."
                    drive_arduino = serial_comm
                    break
                elif split_data[0] == "PBAR":
                    print "Found pbar arduino."
                    pbar_arduino = serial_comm
                    break
                elif split_data[0] == "SCOOP":
                    print "Found scoop arduino."
                    scoop_arduino = serial_comm
                    break
            except:
                pass

    if drive_arduino is not None:
        print "registering drive"
        drive_thread = threading.Thread(target = drive_thread_target)
        drive_thread.start()

    if pbar_arduino is not None:
        print "registering pbar"
        pbar_thread = threading.Thread(target = pbar_thread_target)
        pbar_thread.start()

    if scoop_arduino is not None:
        print "registering scoop"
        scoop_thread = threading.Thread(target = scoop_thread_target)
        scoop_thread.start()

    drive_cmd_sub = rospy.Subscriber('/hardware/cmd_drive', WheelCmdVel, cmd_drive_callback)
    pbar_cmd_sub = rospy.Subscriber('/hardware/cmd_pbar', PbarPose, cmd_pbar_callback)
    scoop_cmd_sub = rospy.Subscriber('/hardware/cmd_scoop', ScoopPose, cmd_scoop_callback)

    rospy.spin()

def cmd_drive_callback(msg):
    drive_arduino.write(str(msg.desiredWV_R) + ',' + str(msg.desiredWV_L) + '\n')

def drive_thread_target():
    while not rospy.is_shutdown():
        try:
            raw_data = drive_arduino.readline().strip()
            data = raw_data.split(',')

            dist = float(data[1])
            dth = float(data[2])
            
            odom = DeltaRobotPose()
            odom.header.stamp = rospy.Time.now()
            odom.distance = dist
            odom.delta_theta = dth

            odom_publisher.publish(odom)
        except serial.SerialException:
            print 'DRIVE: No serial data on read.'
        except:
            print 'DRIVE: Cannot parse \"{}\"'.format(raw_data)

pbar_d = 0
def cmd_pbar_callback(msg):
    global pbar_d
    pbar_enc = msg.pbar                  # TODO: Inverse kinematics

    if pbar_d != pbar_enc:
        pbar_arduino.write("{}\n".format(pbar_enc))
        pbar_d = pbar_enc
            
def pbar_thread_target():
    while not rospy.is_shutdown():
        try:
            raw_data = pbar_arduino.readline().strip()
            data = raw_data.split(',')

            pbar_enc = float(data[1])

            pbar_angle = pbar_enc        # TODO: Forward kinematics

            pbar = PbarPose()
            pbar.pbar = pbar_angle

            pbar_publisher.publish(pbar)
        except serial.SerialException:
            print 'PBAR: No serial data on read.'
        except:
            print 'PBAR: Cannot parse \"{}\"'.format(raw_data)

wrist_d = 0
jaw_d = 0
def cmd_scoop_callback(msg):
    global wrist_d
    global jaw_d

    wrist_enc = msg.wrist
    jaw_enc = msg.jaw

    if wrist_d != wrist_enc or jaw_d != jaw_enc:
        scoop_arduino.write("{},{}\n".format(wrist_enc, jaw_enc))

        wrist_d = wrist_enc
        jaw_d = jaw_enc

def scoop_thread_target():
    while not rospy.is_shutdown():
        try:
            raw_data = scoop_arduino.readline().strip()
            data = raw_data.split(',')

            wrist_enc = float(data[1])   # TODO: Forward kinematics
            jaw_enc = float(data[2])     # TODO: Forward kinematics

            wrist_angle = wrist_enc
            jaw_angle = jaw_enc

            scoop = ScoopPose()
            scoop.wrist = wrist_angle
            scoop.jaw = jaw_angle
            scoop_publisher.publish(scoop)                
        except serial.SerialException:
            print 'SCOOP: No serial data on read.'
        except:
            print 'SCOOP: Cannot parse \"{}\"'.format(raw_data)

if __name__=='__main__':
    main()


