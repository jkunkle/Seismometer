# how to build

Check the defines at the top of the Makefile.  
In order to program, the correct USB device needs
to be addressed with the variable serialPort
The easiest way to find this is to plug in something
to the usb port and check for new devices.  On a
mac this is at /dev


To build do 

`make`

To program, plug in device and do

`make program`


# code setup

The main microcontroller software is in
controller.cpp. 

i2c_master.cpp has a simple i2c library
essentially the rest of the code is used
for communication to the SD card
