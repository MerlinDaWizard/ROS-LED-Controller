
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
    def __init__(self, ledCount: int, inTopic: str, channel: int, outTopic: str = 'multichannel_set_led'):
        rospy.init_node('angle_displayer', anonymous=True)
        self.ledCount = ledCount
        self.channel = channel
        self.formerLED = -1
        self.startup = True
        rospy.Subscriber(inTopic, Imu, self.recieveImu)
        rospy.loginfo(f"Listening on {inTopic}")
        self.pub = rospy.Publisher(outTopic, LEDcommand, queue_size = 30)

    def recieveImu(self, imu: Imu) -> None:
        orientation = imu.orientation
        print(orientation)
        #theta = math.acos(orientation.w)*2
        #ax = orientation.x / math.sin(theta/2)
        #ay = orientation.y / math.sin(theta/2)
        #az = orientation.z / math.sin(theta/2)
        # RADIANS
        roll, pitch, yaw = euler_from_quaternion(orientation.x,orientation.y,orientation.z,orientation.w)
        angle = (yaw*(180/math.pi) + 180)%360
        if (self.startup):
            self.startAngle = angle
            #self.startup = False
        
        angleDiff = self.startAngle - angle
        print(f"{angleDiff=}")
        
        ledIndex = (self.angleToLED(angleDiff) + 77) % (self.ledCount) # Middle ish of a sector, its an even number :/
        print(f"{ledIndex=}")
        print(f"{self.formerLED=}")
        
        if (ledIndex != self.formerLED):
            if self.startup == False:
                self.pub.publish(self.formerLED,self.channel,0,False)
                self.pub.publish((self.formerLED+1)%self.ledCount,self.channel,0,False)
                self.pub.publish((self.formerLED-1)%self.ledCount,self.channel,0,False)
            
            self.pub.publish(ledIndex,self.channel,150,False)
            self.pub.publish( (ledIndex+1)%self.ledCount, self.channel, 50, False)
            self.pub.publish( (ledIndex-1)%self.ledCount, self.channel, 50, True)

            self.startup = False
            self.formerLED = ledIndex

    def angleToLED(self, angle: float) -> int:
        led_float = (angle / 360) * (self.ledCount-1)
        print(f"{led_float=}")
        return int(led_float)

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
