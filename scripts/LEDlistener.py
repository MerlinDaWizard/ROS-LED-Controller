#!/usr/bin/env python

## Given an LED Position, colour (RGB) and length of LED strip, sets that LED

import rospy
from led_locator.msg import LEDdata
#from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False) # Not anon as can't control multiple LED strips off a single line anyway, would just repeat 

    rospy.Subscriber('chatter', LEDdata, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
