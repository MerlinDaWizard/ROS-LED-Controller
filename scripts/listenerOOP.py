import board
import neopixel
import sys
import rospy

from std_msgs.msg import String, ColorRGBA
from neopixel_controller.msg import LEDcommand
#from neopixel_controller.srv import GetLEDCount,GetLEDCountResponse

class LEDcontroller():
    def __init__(self, ledCount: int, pin) -> None:
        self.LEDs = neopixel.NeoPixel(pin, ledCount)
        rospy.set_param('neopixel_controller/led_count', ledCount)
        rospy.init_node('ledListener', anonymous=False)
        rospy.Subscriber('multichannel_set_led', LEDcommand, self.channelSetLED)
        #rospy.Subscriber('fillLEDs', ColorRGBA, fillLEDs)

    def getNewColour(self, oldColour, channel, level):      
        if channel == LEDcommand.ALLCHANNELS:
            return (level, level, level)

        newColour = oldColour.copy()
        newColour[channel] = level
        return newColour

    def channelFill(self, channel, level): # Optimises when filling instead of iterating through all using default setting method
              for i in range(self.LEDs._pixels):
            LED = self.LEDs[i]
            newColour = self.getNewColour(LED, channel, level)
            self.LEDs._set_item(i,newColour[0],newColour[1],newColour[2],0)
        self.LEDs.show()

    def channelSetLED(self, LEDcommand):
        ledIndex = LEDcommand.led_index
        channel = LEDcommand.channel
        level = LEDcommand.level

        if (ledIndex == LEDcommand.ALLLEDS): # Will attempt to map the colour to all LEDS for that specific channel
            self.channelFill(channel, level)
            rospy.loginfo(f"{rospy.get_caller_id()}, Filled!")
            return
        
        self.LEDs[ledIndex] = self.getNewColour(self.LEDs[ledIndex], channel, level) 
        rospy.loginfo(f"{rospy.get_caller_id()}, LED Index: {ledIndex}, Channel: {channel}, Level: {level}")

def validateLEDAmount(ledAmount: int):
    if ledAmount is None:
        return f"That is an invalid LED amount [{num}]"
    if ledAmount <= 0:
        return f"Can't have below 1 LEDs [{num}]"
    
    return True

def grabLEDAmount() -> int:
    pixelCount = None
    args = rospy.myargv(argv=sys.argv)
    if len(args) >= 2:
        try:
            #print(args)
            pixelCount = int(args[1])
            valid = validateLEDAmount(pixelCount)
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
                valid = validateLEDAmount(pixelCount)
                if valid is not True:
                    print(valid)
                    pixelCount = None
            except ValueError as e:
                print("That does not seam to be a valid LED count")
                print(e)
    return pixelCount

if __name__ == '__main__':
    amount = grabLEDAmount()
    controller = LEDcontroller(amount, (board.D18))
    rospy.spin()
