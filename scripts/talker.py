#!/usr/bin/env python
## An example script to show the functionality of the neopixel listener

import rospy
from std_msgs.msg import String
from neopixel_controller.msg import LEDcommand
from random import randint
def getLEDCount() -> int:
    try:
        ledCount = rospy.get_param("neopixel_controller/led_count")
        print(f"Detected {ledCount} LED(s)")
    except KeyError as e:
        print("Could not find parameter for LED count...")
        print("Assuming LED count 30.")
        ledCount = 30
    return ledCount

def talker():
    ledCount = getLEDCount()

    pub = rospy.Publisher('multichannel_set_led', LEDcommand, queue_size=10)
    rospy.init_node('randomSet', anonymous=True)
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        #print(f"Active subscribers = {pub.getActiveSubscribers()}")
        count = randint(0,ledCount-1)
        rospy.loginfo(f"Tried setting LED at index {count}")
        pub.publish(count,randint(0,2),20, True)
        #count = (count + 1) % 10
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
