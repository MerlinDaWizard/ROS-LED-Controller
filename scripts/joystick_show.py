import rospy
from math import cos,asin,sqrt,tan
import math
import sys
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from neopixel_controller.msg import LEDcommand

class JoystickDisplay():
    def __init__(self, led_count: int, in_topic: str, channel: int, out_topic: str = 'multichannel_set_led'):
        rospy.init_node('joystick_displayer', anonymous=True)
        self.led_count = led_count
        self.channel = channel
        self.pub = rospy.Publisher(out_topic, LEDcommand, queue_size = 30)
        rospy.Subscriber(in_topic, Twist, self.recieveTwist)
        rospy.loginfo(f"Listening on {in_topic}")

    
    def recieveTwist(self, twist: Twist) -> None:
        x = twist.linear.x
        y = twist.linear.y
        print(f"{x:.4f}, {y:.4f}")
        angle_rad = math.atan2(x,y) + math.pi
#        led_idx = (self.led_count - int( (angle_rad / (2*math.pi) ) * (self.led_count - 1) )) % self.led_count
        led_idx = (int(self.led_count - (angle_rad / (2*math.pi)) * (self.led_count-1)) + 36) % self.led_count
        print(f"{angle_rad=}")
#        rad = asin(sqrt(math.pow(tan(roll),-1)+math.pow(tan(roll),-1)) /2)
#        rad = (sqrt(math.pow(tan(roll),-1)+math.pow(tan(roll),-1)) /2)
        print(f"{led_idx}")
        self.pub.publish(LEDcommand.ALLLEDS,self.channel,0,False)
        self.pub.publish(led_idx,self.channel,150,True)

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
    if len(args) != 3:
        rospy.logfatal("Must have an intopic & channel specified as an argument")
        exit()
    in_topic = args[1]
    
    try:
        channel = int(args[2])
    except ValueError as e:
        print("Cannot decode channel")
        throw(e)
    led_count = getLEDCount()
    disp = JoystickDisplay(led_count, in_topic, channel)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
