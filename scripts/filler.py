#!/usr/bin/env python

## Fills the LEDs given a specific colour, uses RGBA ignoring alpha value

import rospy
from std_msgs.msg import String, ColorRGBA
from neopixel_controller.msg import LEDcommand

def filler():
    pub = rospy.Publisher('multichannel_set_led', LEDcommand, queue_size=30)
    rospy.init_node('filler', anonymous=True)
    #rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        RGB = input("What colour to fill with? (R,G,B):").split(',')
        try:
            RGB = [int(num) for num in RGB]
        except ValueError:
            print("Invalid Number, exitting")
            exit()
        rospy.loginfo(f"Filling with {RGB}")
        for channel, value in enumerate(RGB):
            pub.publish(LEDcommand.ALLLEDS, channel, value)

if __name__ == '__main__':
    try:
        filler()
    except rospy.ROSInterruptException:
        pass
