import rospy
from math import cos,asin,sqrt,tan,atan2,sin,cos
import math
import sys
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Imu
from neopixel_controller.msg import LEDcommand

class TiltDisplay():
    def __init__(self, led_count: int, in_topic: str, channel: int, out_topic: str = 'multichannel_set_led'):
        rospy.init_node('tilt_displayer', anonymous=True)
        self.led_count = led_count
        self.channel = channel
        self.pub = rospy.Publisher(out_topic, LEDcommand, queue_size = 30)
        rospy.Subscriber(in_topic, Imu, self.recieveImu)
        rospy.loginfo(f"Listening on {in_topic}")

    
    def recieveImu(self, imu: Imu) -> None:
        orientation = imu.orientation
        print(orientation)

        roll, pitch, yaw = euler_from_quaternion((orientation.x,orientation.y,orientation.z,orientation.w))
        degree_const = (180/math.pi)
        
        d_roll = roll * degree_const
        d_pitch = pitch * degree_const
        print(f"{d_roll=}")
        print(f"{d_pitch=}")
        rad = atan2(cos(yaw),sin(pitch))
        
#        rad = asin(sqrt(math.pow(tan(roll),-1)+math.pow(tan(roll),-1)) /2)
#        rad = (sqrt(math.pow(tan(roll),-1)+math.pow(tan(roll),-1)) /2)
#        rad = cos(roll)/cos(pitch)
        print(rad)
        print(math.degrees(rad))


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
    disp = TiltDisplay(led_count, in_topic, channel)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
