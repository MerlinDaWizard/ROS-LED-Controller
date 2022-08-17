#!/usr/bin/env python

## Fills the LEDs given a specific colour, uses RGBA ignoring alpha value

import rospy
from std_msgs.msg import String, ColorRGBA
from led_locator.msg import LEDdata

def filler():
    pub = rospy.Publisher('fillLEDs', ColorRGBA, queue_size=10)
    rospy.init_node('filler', anonymous=True)
    #rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        RGB = input("What colour to fill with? (R,G,B):").split(',')
        RGB = [int(num) for num in RGB]
        rospy.loginfo(f"Filling with {RGB} {rospy.get_time()}")
        pub.publish(RGB[0],RGB[1],RGB[2],0) # Unused Alpha value
        #count = (count + 1) % 10

if __name__ == '__main__':
    try:
        filler()
    except rospy.ROSInterruptException:
        pass
