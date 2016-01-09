This project uses  a gy-85 9DOF accelerometer , gyro and magnetometer and an arduino uno to create a head tracker that will produce joystick outputs to a game that can be mapped to look joystick axes within a game.
	The project uses the unoJoy project ( which can be downloaded at https://code.google.com/p/unojoy/) to handle the implementation of a usb HID joystick device.

Simply move the contents of the libraries folder into your arduino libraries folder and then compile the unoJoy sample arduino sketch.
Download the program to the uno before using unojoy to turn the arduino uno into a joystick.
 buttons can be attached to pins 10 and 9 . pulling pin 10 low sets the current orientation to zero. pulling pin 9 low turns off tracking and sets all axes to zero position (effectively turning off head tracking)
