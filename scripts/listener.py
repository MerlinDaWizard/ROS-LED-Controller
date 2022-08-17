#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import board
import neopixel
import sys
import rospy
from std_msgs.msg import String, ColorRGBA
from led_locator.msg import LEDdata 
from led_locator.srv import GetLEDCount,GetLEDCountResponse

def setIndivLED(data):
    #print(dir(data))
    colourInts = [num for num in data.ledRGB] # As ROS uint8's in python become bytes, convert them back to ints.
    rospy.loginfo(f"{rospy.get_caller_id()}, LED Num: {data.ledPos}, LED Colour: {colourInts}")
    global pixels
    pixels[data.ledPos] = colourInts

def listener():
    rospy.init_node('listener', anonymous=False)
    service = rospy.Service('get_LED_count', GetLEDCount, handle_get_led_count)
    rospy.Subscriber('individualLEDset', LEDdata, setIndivLED)
    rospy.Subscriber('fillLEDs', ColorRGBA, fillLEDs)

    rospy.spin()

def fillLEDs(ColorRGBA): # Using RGBA as no default RGB msg and dont want to create one
    global pixels
    colourTup = (ColorRGBA.r,ColorRGBA.g,ColorRGBA.b)
    pixels.fill(colourTup)

def handle_get_led_count(the_only_here_to_make_ros_happy_variable): # Not sure why this is required
    global pixels # Replaced with ros parameter server stuff, still here, use not encouraged
    rospy.loginfo(f"{rospy.get_caller_id()}, Served LED Count={pixels._pixels}")
    return GetLEDCountResponse(pixels._pixels)
    
def validatePixelCount(num):
    if num is None:
        return f"That is an invalid LED amount [{num}]"
    if num <= 0:
        return f"Can't have below 1 LEDs [{num}]"
    
    return True

if __name__ == '__main__':
    global pixels
    pixelCount = None
    args = rospy.myargv(argv=sys.argv)
    if len(args) >= 2:
        try:
            #print(args)
            pixelCount = int(args[1])
            valid = validatePixelCount(pixelCount)
            if valid is not True:
                print(valid)
                print("Exiting...")
                exit()
        except ValueError as e:
            print("That does not seam to be a valid LED count")
            print(e)
            exit()
    else: # Asking user on startup as its missing an argument for it
        print("Missing argument for LED count.")
        while not pixelCount:
            try:
                pixelCount = int(input("How many LEDs? "))
                valid = validatePixelCount(pixelCount)
                if valid is not True:
                    print(valid)
                    pixelCount = None
            except ValueError as e:
                print("That does not seam to be a valid LED count")
                print(e)
    rospy.set_param('ledlocator/led_count', pixelCount)
    pixels = neopixel.NeoPixel(board.D18, pixelCount)
    listener()
