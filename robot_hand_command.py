#! /urs/bin/env python

# Robot Hand Command
# Version 1.0
# Date: 12/17/20
# Author: Nikolai Tiong
#
# Robot Hand Command is the ROS publisher script for the robot hand
# This is where messages are generated to tell the hand where to move each finger
# Message is of the format "aaa bbb ccc ddd eee" - with spaces
# Where
#    aaa - Thumb
#    bbb - Index
#    ccc - Middle
#    ddd - Ring
#    eee - Pinky
# The movement range of the servos are:
#    150 - Open
#    440 - Closed
# Messages can be sent outside of these ranges, the Robot Hand Arduino sketch 
# will adjust values outside of range to the open or close position depending
# on whether the value is above or below.
#
# How to use:
#    Make sure the robot hand is powered on and connected to the network that
#    the system running ROS is on - ping the IP address of the hand.
#    Open a terminal window on the system running ROS
#    Type "roscore"
#    Open another terminal window and type:
#       "rosrun rosserial_python serial_node.py tcp"
#    Open another terminal window and navigate to the folder this script is in
#    and type "python robot_hand_command.py"
#    The topic robot_hand_command will be published and messages should appear
#    on the rosrun window.
#    Open another terminal window, navigate to the folder containing
#    robot_hand_position.py and type "python robot_hand_position"
#
#
# To Do:
#    Extend the message to have a speed value - this will be the delay() in
#    the Robot Hand sketch in between pulses. Do this on an entire hand
#    basis or on a per finger basis. Robot Hand will need to be modified
#    to handle the message in either case.
#

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node('robot_hand_command')
	pub = rospy.Publisher("/robot_hand_command", String, queue_size = 10)
	rate = rospy.Rate(0.5)
        print ("Initialized publisher")
	while not rospy.is_shutdown():
                print ("Publishing")
		msg = String()
		msg.data = "200 200 250 350 400"
                pub.publish(msg)
		rate.sleep()
                msg.data = "350 350 300 200 250"
                pub.publish(msg)
                rate.sleep()
                msg.data = "449 440 400 300 440"
                pub.publish(msg)
                rate.sleep()
                msg.data = "300 300 150 200 300"
                pub.publish(msg)
                rate.sleep()
	rospy.loginfo("node has stopped")
