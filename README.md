HatRat, the Mouse For Your Hat!
======


This project is an attempt to make an inexpensive, easy-to-use head mouse. The shell is 3D printed and features a clip to put it on a hat, visor, or maybe even a glasses frame. 

To use the device, clip it onto something with the USB C port facing outward toward the screen. Connect a USB cable and the device will automatically turn on. Move your head up, down, left, or right to move the mouse accordingly. 


How to Use the HatRat
======
If you hold still (or even just relatively still) for 3 seconds, the mouse will enter a gesture mode:
1.  Tilt left then back to center to left-click the mouse.
2.  Tilt right then back to center to right-click the mouse.
3.  To left-click and drag, tilt to the left and keep tilted. Move your head in the direction you want to drag while keeping your head tilted. When you're done, tilt your head back to the center.
4.  To right-click and drag, tilt to the right and keep tilted. Move your head in the direction you want to drag while keeping your head tilted. When you're done, tilt your head back to the center.

If you need to reset the orientation of the mouse, look to the edge of the screen. Like, if your head is too far turned to the left, just turn all the way to the right until you're about where you want "right" to be. The same holds true for up, down, left, and right.

In the repository is the Arduino sketch, 3D print files, and KiCAD files needed to build the thing.

Bill of Materials
======
Other than the circuit board, you will need:
-	1x Arduino Pro Micro USB C
-	1x Bidirectional Logic Level Converter
-	1x AMS1117-3.3V Voltage Regulator
-	1x MPU6050 Accelerometer
-	2x Female Headers - 12 Pins - 2.54mm Pitch

You should be able to print the thing with about 25g of filament, making the entire thing cost about $15, versus the hundreds of dollars that other solutions cost.
