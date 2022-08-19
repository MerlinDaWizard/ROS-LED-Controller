## Listenings to a given topic as an argument, takes in any twists and uses
## the angular velocity to store a current rotation, will send out a signal
## pointing to starting rotation

import rospy
from std_msgs.msg import String, ColorRGBA
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import Imu
from neopixel_controller.msg import LEDcommand
import math
import sys

class VelocityDisplay():
    def __init__(self, ledCount: int, inTopic: str, outTopic = 'multichannel_set_led') -> None:
        rospy.init_node('velocity_displayer', anonymous=True)
        self.ledCount = ledCount
        self.formerLED = -1
        rospy.Subscriber(inTopic, Imu, self.recieveTwist)
        rospy.loginfo(f"Listening on {inTopic}")
        self.pub = rospy.Publisher(outTopic, LEDcommand, queue_size = 30)

    def recieveTwist(self, imu):
        
        linearAccel = imu.linear_acceleration
        radian = math.atan2(linearAccel.y,linearAccel.x)
        print(radian)
        degrees = math.degrees(radian) + 180
        print(degrees)
        ledIndex = self.angleToLED(degrees)
        if self.formerLED == ledIndex:
            return
        brightness = int(math.sqrt((linearAccel.y*linearAccel.y) + (linearAccel.x*linearAccel.x)) * 20)
        print(brightness)
        if brightness < 40:
            brightness = 0
        brightness = min(brightness,255)    
        self.pub.publish(ledIndex,LEDcommand.GREEN,brightness,False)
        if self.formerLED != -1:
            self.pub.publish(self.formerLED,LEDcommand.GREEN,0,True)
        self.formerLED = ledIndex

    def angleToLED(self,angle):
        return int((angle / 360) * (self.ledCount-1))

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
    if len(args) != 2:
        
        rospy.logerr("Must have an intopic specified as an argument")
        exit()
    
    inTopic = args[1]
    ledCount = getLEDCount()
    velo = VelocityDisplay(ledCount, inTopic) 
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
