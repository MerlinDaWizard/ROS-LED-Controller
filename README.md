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

### Running Examples
1. Make sure roscore is running
2. Run the listener seperately (sudo -s, source /opt/ros/noetic/setup.sh, source your catkin ws, rosrun neopixel_controller listener.py)
3. Roslaunch neopixel_controller (JoystickExample.launch|HoldNorthExample.launch)
* Hold north requires turtlebot / other imu data

### Limitations 
If you write to the listener too much it may flash to the human eye, this is especially true if you are effecting all LEDs. 
Listener must be ran as sudo, sudo -s then sourcing ros manually typically works well enough, if someone can find a way to circumvent this make a pull request / issue.

### Tips
To apply to all LEDs use LEDcontroller.ALLLEDS as the index
To apply to all channels use LEDcontroller.ALLCHANNELS as the channel
(LEDcontroller.RED, LEDcontroller.GREEN, LEDcontroller.BLUE also exist and just are 0,1,2 respectively)

Both of those sentinal values combined can be used to easily clear the LEDs.
