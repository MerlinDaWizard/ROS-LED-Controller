import rospy
from math import cos,asin,sqrt,tan
import math
import sys
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from neopixel_controller.msg import LEDcommand

class JoystickDisplay():
    def __init__(self, led_count: int, in_topic: str, channel: int, offset: int, out_topic: str = 'multichannel_set_led'):
        rospy.init_node('joystick_displayer', anonymous=True)
        self.offset = offset
        self.led_count = led_count
        self.channel = channel
        self.pub = rospy.Publisher(out_topic, LEDcommand, queue_size = 30)
        rospy.Subscriber(in_topic, Twist, self.recieveTwist)
        rospy.loginfo(f"Listening on {in_topic}")
        self.former_led_idx = -1
    
    def recieveTwist(self, twist: Twist) -> None:
        x = twist.linear.x
        y = twist.linear.y
        print(f"{x:.4f}, {y:.4f}")
        angle_rad = math.atan2(x,y) + math.pi
        
        led_idx = (int(self.led_count - (angle_rad / (2*math.pi)) * (self.led_count-1)) + self.offset) % self.led_count # Offset may need changing depending on where you connect to your LEDs
        print(f"{angle_rad=:0.4f}")
        print(f"{led_idx=}")
        if self.former_led_idx != led_idx:
            self.pub.publish(LEDcommand.ALLLEDS,self.channel,0,False)
            self.pub.publish(led_idx,self.channel,150,True)
            self.former_led_idx = led_idx

def getLEDCount() -> int:
    try:
        led_count = rospy.get_param("neopixel_controller/led_count")
        print(f"Detected {led_count} LED(s)")
    except KeyError as e:
        print("Could not find parameter for LED count...")
        print("Make sure listener is started first.")
        print("Assuming LED count 30.")
        led_count = 30
    return led_count

def main() -> None:
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 4:
        rospy.logfatal("Must have an intopic, channel & offset specified as an argument")
        exit()
    in_topic = args[1]
    
    try:
        channel = int(args[2])
        offset = int(args[3])
    except ValueError as e:
        print("Cannot decode channel or offset")
        throw(e)
    led_count = getLEDCount()
    disp = JoystickDisplay(led_count, in_topic, channel, offset)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
