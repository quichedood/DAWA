# DAWA - Data Acquisition With Arduino
![DAWA-V4](http://dawa.panik-po.com/images/dawa-v4.jpg)

DAWA is an Arduino datalogger (and laptimer) shield for your motorbike.
It records 10 times per seconds in a CSV file lots of information.

# What information is logged ?
* __Raw data acquisition__ : On Triumph bikes (and many others) ECU values can be directly read like : RPM, SPEED, GEAR POSITION, THROTTLE and BRAKE state
* __Environement values__ : A 9-axis sensor (BNO055) is used to store G-forces and I hope soon, roll and pitch
* __Position values__ : A UBLOX 10Hz GPS chip gets realtime coordinates
* __Screen display__ : A little OLED screen is attached to view values in realtime
* __Laptimer__ : Laptimer functions are now working great ! They are based on GPS values.
* __Recorder__ : All these information are stored on a SD card 10 times per seconds in a CSV file

# How it work ?
I couldn't make it easier !  
Press the button start recording, press again stop recording :)  
One CSV file is created on each new record.

# What about laptimer functions
A little bit more difficult, you have to put a file named "TRACKS.CSV" on the sdcard.  
This file will contain track name and finishline coordinates, one line per track :  
`<trackname>;<finishline lat. A>;<finishline lon. A>;<finishline lat. B>;<finishline lon. B>`  
`CAROLE;489799930;25224350;489800230;25226330`  
To keep precision, latitude and longitude should be converted to integers (multiply by 10 000 000).  
When start recording the closer track is automatically chosen.

# Repository Contents
* /Arduino - The .ino file you need to put in the Arduino M0
* /Documentation - Some brief explanations about this shield (french - not translated)
* /Eagle Libraries - All Eagle parts I use in my schematics
* /Eagle Sources - Eagle files of this project
* /Gerber Sources - Last Gerber files, used to produce PCB
* /Partslist - A detailed BOM list

