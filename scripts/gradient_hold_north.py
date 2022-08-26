## This example is done in normal degrees, if you would like one in radians look at joystick_show.py
## You can also replace the large euler_from_quaternion with tf.transformations

## When started it will set a datum at this point. We then use the IMU data 
## to attempt to keep a stable LED pointing in a direction

import rospy
from std_msgs.msg import String, ColorRGBA
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import Imu
from neopixel_controller.msg import LEDcommand
import math
import sys

class HoldNorthDisplay():
    def __init__(self, led_count: int, topic_in: str, channel: int, topic_out: str = 'multichannel_set_led'):
        rospy.init_node('direction_displayer', anonymous=True)
        self.led_count = led_count
        self.channel = channel
#        self.formerLED = -1
        self.startup = True
        rospy.Subscriber(topic_in, Imu, self.recieveImu)
        rospy.loginfo(f"Listening on {topic_in}")
        self.pub = rospy.Publisher(topic_out, LEDcommand, queue_size = 30)

    def recieveImu(self, imu: Imu) -> None:
        max_brightness = 200 # Constant
        orientation = imu.orientation
        print(orientation) 
        # RADIANS
        roll, pitch, yaw = euler_from_quaternion(orientation.x,orientation.y,orientation.z,orientation.w)
        # Yaw is both negative and positive, we convert it to degrees instead of radians then make sure it wraps around to 360
        cur_angle = (yaw*(180/math.pi) + 180)%360
        
        # Calibrate start angle to calculate difference from
        if (self.startup):
            self.start_angle = cur_angle
            #self.startup = False
        
        angle_difference = self.start_angle - cur_angle
        print(f"{angle_difference=}")
        
        led_float = (self.angleToLED(angle_difference) + 77) # OFFSET
        led_intensity = int( (led_float % 1) * max_brightness)

        led_idx_primary = math.floor(led_float) % self.led_count
        led_idx_secondary = (led_idx_primary + 1) % self.led_count

        print(f"{led_intensity=}")
        print(f"{led_idx_primary=}")
        
        if self.startup == False:
            self.pub.publish(self.former_led_idx_primary, self.channel, 0, False)
            self.pub.publish(self.former_led_idx_secondary, self.channel, 0, False)
            #self.pub.publish((self.formerLED-1)%self.ledCount,self.channel,0,False)
            
        self.pub.publish( led_idx_primary, self.channel, (max_brightness - led_intensity), False)
        self.pub.publish( led_idx_secondary, self.channel, led_intensity, True)
        
        self.former_led_idx_primary = led_idx_primary
        self.former_led_idx_secondary = led_idx_secondary
        self.startup = False
        
    def angleToLED(self, angle: float) -> float:
        led_float = (angle / 360) * (self.led_count-1)
        print(f"{led_float=}")
        return (led_float)

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

def getLEDCount() -> int:
    try:
        ledCount = rospy.get_param("neopixel_controller/led_count")
        print(f"Detected {ledCount} LED(s)")
    except KeyError as e:
        print("Could not find parameter for LED count...")
        print("Make sure listener is started first.")
        print("Assuming LED count 30.")
        ledCount = 30
    return ledCount

def main():
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 3:
        
        rospy.logerr("Must have an intopic & channel specified as an argument")
        exit()
    
    inTopic = args[1]
    channel = int(args[2]) # TODO: Error catching with graceful exit
    ledCount = getLEDCount()
    velo = HoldNorthDisplay(ledCount, inTopic, channel)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
