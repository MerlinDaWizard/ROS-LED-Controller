## Listenings to a given topic as an argument, takes in any twists and uses
## the angular velocity to store a current rotation, will send out a signal
## pointing to starting rotation

import rospy
from std_msgs.msg import String, ColorRGBA
import geometry_msgs.msg
from turtlesim.msg import Pose
from neopixel_controller.msg import LEDcommand
import math

def getLEDCount() -> int:
    try:
        ledCount = rospy.get_param("neopixel_controller/led_count")
        print(f"Detected {ledCount} LED(s)")
    except KeyError as e:
        print("Could not find parameter for LED count...")
        print("Assuming LED count 30.")
        ledCount = 30
    return ledCount

def thetaToDegrees(theta):
    degrees = theta * (180/math.pi)
    if degrees < 0: # Turtle stores it as 0 through 180 using the sign as an indicator for direction, this converts it to a full 360 clockwise
        degrees = abs(degrees)
    else:
        degrees  = 360 - degrees
    return degrees

def recievePose(turtlePose):
    global ledCount
    global formerLED
    global pubIndivLED
    #print(turtlePose)
    angle = turtlePose.theta * (180/math.pi)
    #print(f"AnglePre: {angle}")
    angle = thetaToDegrees(turtlePose.theta) 
    #print(f"AnglePost: {angle}")
    led = angleToLED(angle,ledCount) # LEDs are 0 based
    #print(f"Publishing to LED {led}")
    if formerLED == led: # So that we are not constantly publishing the same LED
        return
    #print(formerLED)
    #print(led)
    
    pubIndivLED.publish(led,LEDcommand.GREEN,50,False)
    if formerLED < ledCount: # Stops a crash with listener on startup due to sneding a publish with >= ledCount, will make fix here and TODO make listener ignore calls that would crash it 
        pubIndivLED.publish(formerLED,LEDcommand.GREEN,0,True)
    formerLED = led

def angleToLED(angle, ledCount): # Assuming perfect circle etc etc.
    return int((angle / 360) * (ledCount-1))
    
def main():
    global ledCount
    global pubIndivLED 
    global fillLED
    ledCount = getLEDCount()
    pubIndivLED = rospy.Publisher('multichannel_set_led', LEDcommand, queue_size = 30)  
    #fillLED = rospy.Publisher('fillLEDs', ColorRGBA, queue_size = 10)
    global formerLED
    formerLED = ledCount + 10 # Hopefully always above whatever but keeping it unsigned int
    rospy.init_node('angularLT')
    rospy.Subscriber('turtle1/pose_throttle', Pose, recievePose)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
