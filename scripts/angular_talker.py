## Listenings to a given topic as an argument, takes in any twists and uses
## the angular velocity to store a current rotation, will send out a signal
## pointing to starting rotation

import rospy
from std_msgs.msg import String, ColorRGBA
import geometry_msgs.msg
from turtlesim.msg import Pose
from led_locator.msg import LEDdata
from led_locator.srv import *
import math

def talker():
    rospy.wait_for_service('get_LED_count')
    try:
        get_LED_count = rospy.ServiceProxy('get_LED_count', GetLEDCount)
        ledCount = get_LED_count().ledCount
    except rospy.ServiceException as e:
        print("Cannot get LED strip length from listener")
        print(f"Service call failed: {e}")
    print(f"LED Count={ledCount}")
    pub = rospy.Publisher('individualLEDset', LEDdata, queue_size=10)
    rospy.init_node('angularTalker', anonymous=True)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        #print(f"Active subscribers = {pub.getActiveSubscribers()}")
        count = randint(0,ledCount-1)
        rospy.loginfo(f"Tried setting LED at index {count}")
        pub.publish(count,(20,0,20))
        #count = (count + 1) % 10
        rate.sleep()

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
    print(turtlePose)
    angle = turtlePose.theta * (180/math.pi)
    print(f"AnglePre: {angle}")
    angle = thetaToDegrees(turtlePose.theta) 
    print(f"AnglePost: {angle}")
    led = angleToLED(angle,ledCount) # LEDs are 0 based
    print(f"Publishing to LED {led}")
    if formerLED == led: # So that we are not constantly publishing the same LED
        return
    print(formerLED)
    print(led)
    if formerLED < ledCount: # Stops a crash with listener on startup due to sneding a publish with >= ledCount, will make fix here and TODO make listener ignore calls that would crash it 
        pubIndivLED.publish(formerLED,(0,0,0))
    #fillLED.publish(0,0,0,0)
    pubIndivLED.publish(led,(20,20,0))
    formerLED = led
def angleToLED(angle, ledCount): # Assuming perfect circle etc etc.
    return int((angle / 360) * (ledCount-1))
    
def listener():
    global ledCount
    global pubIndivLED 
    global fillLED
    pubIndivLED = rospy.Publisher('individualLEDset', LEDdata, queue_size = 10)  
    #fillLED = rospy.Publisher('fillLEDs', ColorRGBA, queue_size = 10)
    global formerLED
    #print(rospy.get_param_names())
    try:
        ledCount = rospy.get_param("ledlocator/led_count")
        print(f"Detected {ledCount} LED(s)")
    except KeyError as e:
        print("Could not find parameter for LED count...")
        print("Assuming LED count 30.")
        ledCount = 30
        #print("Contacting Service to retrieve it.")
    formerLED = ledCount + 10 # Hopefully always above whatever but keeping it unsigned int
    rospy.init_node('angularLT')
    rospy.Subscriber('turtle1/pose_throttle', Pose, recievePose)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
