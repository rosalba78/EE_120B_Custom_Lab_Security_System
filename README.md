EE_120B_Custom_Lab_Security_System
==================================

For my EE 120B Embedded Systems class, I needed to create a custom lab as my final project. I chose to build a security system. The requirements were to build an embedded system with 3 "build-upons." These build-upons were a concepts that were taught to us through lab and lecture but modified to do something different than shown in lab. The 3 "build-upons" I chose are the following: 1. The use of a PIR sensor, 2. Creating a custom character on an LCD screen, and 3. using UART to implement a bluetooth module.

The use of this code is ONLY for educational purposes.
---------------------------------------------------------------------

Components used in this project:

Breadboard Power Supply
ATMega 1284
4x4 Keypad
16x2 LCD Display
PIR Sensor (from RadioShack)
5mm Common Cathode RGB LED
HC-06 Bluetooth Module

---------------------------------------------------------------------

How the system works:

DISARMED STATE
1. Default state and also enabled when entering a valid pass code followed by the 'D' key
2. LCD displays an "unlock" character
3. System waits to be armed
4. To Arm the system, you must press the 'A' key
5. The PIR Sensor is IGNORED in the disarmed state

ARMED STATE
1. Enabled by pressing the 'A' key
2. LCD displays a "lock" character
3. PIR sensor alerts the system of any detection of movement
   - When movement is sensed the RGB LED blinks Red using PWM
4. Bluetooth Module sends a signal and alerts the owner that movement has been detected by the armed system
5. To disable the alarm, you must enter a valid pass code followed by the 'D' key
   - This can also be done when the system is armed and no movement has been detected
   
Check out the demo on youtube for the latest updates on the project:
<INSERT LINK>

---------------------------------------------------------------------

Sources used:

1. Libraries provided by the TAs
2. Sources on how to create a custom character on a LCD Display
   - http://www.8051projects.net/lcd-interfacing/lcd-custom-character.php
   - http://saeedsolutions.blogspot.com/2012/12/how-to-display-custom-characters-on-lcd.html
3. Previous lab manuals for the course
4. Programming Embedded Systems (PES) ----> Course's Electronic Text

---------------------------------------------------------------------

Special thanks to my moral support team. Don't know what I would do without you guys! =]
