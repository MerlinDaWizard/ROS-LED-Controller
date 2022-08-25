import board
import neopixel
import sys
import rospy
import time

from std_msgs.msg import String, ColorRGBA
from neopixel_controller.msg import LEDcommand
#from neopixel_controller.srv import GetLEDCount,GetLEDCountResponse

class LEDcontroller():
    def __init__(self, ledCount: int, pin, gamma_correction: bool = True) -> None:
        self.LEDs = neopixel.NeoPixel(pin, ledCount, auto_write = False)
        self.gamma_correction = gamma_correction
        rospy.set_param('neopixel_controller/led_count', ledCount)
        rospy.init_node('ledListener', anonymous=False)
        rospy.Subscriber('multichannel_set_led', LEDcommand, self.channelSetLED)
        rospy.loginfo("Started listener...")
        #rospy.Subscriber('fillLEDs', ColorRGBA, fillLEDs)

    def getNewColour(self, old_colour: tuple, channel: int, level: int) -> tuple:
        if self.gamma_correction:
            level = self.gamma_lookup(level)

        if channel == LEDcommand.ALLCHANNELS:
            return (level, level, level)

        new_colour = old_colour.copy()
        new_colour[channel] = level
        return new_colour

    def channelFill(self, channel: int, level: int) -> None:
        '''Uses internal _set_item as it seamed to be much faster, is what the default fill method uses'''
        for i in range(self.LEDs._pixels):
            LED = self.LEDs[i]
            new_colour = self.getNewColour(LED, channel, level)
            self.LEDs._set_item(i,new_colour[0],new_colour[1],new_colour[2],0)
        self.LEDs.show()

    def channelSetLED(self, LEDcommand) -> None:
        led_idx = LEDcommand.led_index
        channel = LEDcommand.channel
        level = LEDcommand.level
        # Checking LEDs not out of range - Will pass gracefully
        if led_idx > (self.LEDs._pixels - 1) and led_idx != LEDcommand.ALLLEDS:
            rospy.logerr(f"{rospy.get_caller_id()} Tried to set LED {ledIndex+1} when we only have {self.LEDs._pixels} available")
            return

        if led_idx < 0:
            rospy.logerr(f"{rospy.get_caller_id()} Tried to set LED {ledIndex+1}, which doesn't exist") 
            return

        if (led_idx == LEDcommand.ALLLEDS): # Will attempt to map the colour to all LEDS for that specific channel
            self.channelFill(channel, level)
            rospy.loginfo(f"{rospy.get_caller_id()}, Filled!")
            return
        
        r, g, b, w = self.LEDs._parse_color(self.getNewColour(self.LEDs[led_idx], channel, level))
        self.LEDs._set_item(led_idx, r, g, b, w)
        if (LEDcommand.show):
            self.LEDs.show()
        #self.LEDs[ledIndex] = self.getNewColour(self.LEDs[ledIndex], channel, level) 
        rospy.loginfo(f"{rospy.get_caller_id()}, LED Index: {led_idx}, Channel: {channel}, Level: {level}")
        
    def gamma_lookup(self, level: int) -> int:
        '''Adjusts for the human eye's perception of colours, values below 28 will get cut off'''
        lookup_table = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213, 215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255)
        if level == 0: # as level-1, 0-1 would cause out of bounds
            return 0
        return lookup_table[level-1]
#    def channelGradientManager(self, GradientMsg): # TODO: make it work.
#        sideSize = GradientMsg.side_size
#        highest = GradientMsg.origin_level
#        idx = GradientMsg.origin_index
#        channel = GradientMsg.channel
#        self.LEDs[idx] = 
#        r, g, b, w = self.LEDs._parse_color(self.getNewColour(self.LEDs[idx], channel, highest))
#        self.LEDs._set_item(idx, r, g, b, w)
#        for i in range(1,sideSize+1):
#            brightness = (i/sideSize+1)*highest#            
            
def validateLEDAmount(led_amount: int):
    if led_amount is None:
        return f"That is an invalid LED amount [{num}]"
    if led_amount <= 0:
        return f"Can't have below 1 LEDs [{num}]"
    return True

def grabLEDAmount() -> int:
    pixel_count = None
    args = rospy.myargv(argv=sys.argv)
    if len(args) >= 2:
        try:
            #print(args)
            pixel_count = int(args[1])
            valid = validateLEDAmount(pixel_count)
            if valid is not True:
                rospy.logerr(valid)
                rospy.loginfo("Exiting...")
                exit()
        except ValueError as e:
            rospy.logerr("That does not seam to be a valid LED count")
            rospy.logerr(e)
            exit()
    else: # Asking user on startup as its missing an argument for it
        rospy.logwarn("Missing argument for LED count.")
        while not pixel_count:
            try:
                pixel_count = int(input("How many LEDs? "))
                valid = validateLEDAmount(pixel_count)
                if valid is not True:
                    rospy.logwarn(valid)
                    pixel_count = None
            except ValueError as e:
                rospy.logwarn("That does not seam to be a valid LED count")
                rospy.logwarn(e)
    return pixel_count

if __name__ == '__main__':
    amount = grabLEDAmount()
    controller = LEDcontroller(amount, (board.D18))
    rospy.spin()
