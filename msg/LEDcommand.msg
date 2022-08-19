## CONSTANTS
# Ints must be in base 10, (2^16)-1, Represents a sentinal meaning all LEDs, can be used to clear
uint16 ALLLEDS = 65535
# Constants for colours, wanted to use a bool array but you can't define constants with those, would be kind of verbose
uint8 RED = 0
uint8 GREEN = 1
uint8 BLUE = 2
uint8 ALLCHANNELS = 255

## DATA

uint16 led_index
uint8 channel
uint8 level
bool show
