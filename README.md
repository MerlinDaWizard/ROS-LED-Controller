# ROS LED STRIP Controller
Only supports strips operating in RGB mode (not RGBW)

### Usage
Publish messages to `multichannel_set_led`, specifing the index, colour channel and the level.
This package is mainly designed for having multiple overlapping displays to show debug information. E.g. The direction a robot believes is north, movement of accelerometer etc.
If you are looking to do more complicated displays requiring a large amount of LED updating, I would recommend just using the adafruit library inside your own package instead of through this one.

#### Starting
1. Make sure roscore is running
2. Start the listener - `rosrun neopixel_controller listener.py [LED AMOUNT]` The listener default runs on the PWM 0 pin on raspis, will add cmdline argument in the future
3. Run your script which publishes messages to the neopixels

### Limitations 
The listener is quite slow, from some quick tests it can run at around 20Hz, try not to make too many more messages than that as it will just start queueing them and then eventually discarding.

### Tips
To apply to all LEDs use LEDcontroller.ALLLEDS as the index
To apply to all channels use LEDcontroller.ALLCHANNELS as the channel
(LEDcontroller.RED, LEDcontroller.GREEN, LEDcontroller.BLUE also exist and just are 0,1,2 respectively)

Both of those sentinal values combined can be used to easily clear the LEDs.

### Contributing
If you have any idea how to improve the package or get sick of my bad code feel free to submit a pull request, it will most likely be accepted :)

All contributions are under the MIT license.
