#!/usr/bin/env python
#! /urs/bin/env python

# Robot Hand Position
# Version 1.0
# Date: 12/17/20
# Author: Nikolai Tiong
#
# Robot Hand Position is the subscriber to robot_hand_position sent from the
# robot hand. It uses the default subscriber template and just receives the 
# messages and prints it out. No processing has been implemented yet.
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
# To Do:
#    Feed this back into robot_hand_command so that it can make decisions
#    based on whether the fingers reached their intended position. 

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("robot_hand_position", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
