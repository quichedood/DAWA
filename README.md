# DAWA - Data Acquisition With Arduino
![DAWA-V4](http://dawa.panik-po.com/images/dawa-v4.jpg)

DAWA is an Arduino datalogger (and laptimer) shield for your motorbike.
It records 10 times per seconds in a CSV file lots of information.

# What information is logged ?
* __Raw data acquisition__ : On Triumph bikes and many others, ECU values can be directly read (I personnaly use : RPM, SPEED, GEAR POSITION, THROTTLE and BRAKE state)
* __Environement values__ : A 9-axis sensor (BNO055) is used to store G-forces and I hope soon, roll and pitch
* __Position values__ : A UBLOX 10Hz GPS chip gets realtime coordinates
* __Infrared temperatures__ : You can plug up to 4 infrared temperature sensors (tyres or ground t° for example)
* __Analog inputs__ : You can measure up to 6 analog inputs (suspension sensors for exemple)

# Where is it logged ?
Everything is stored on a micro SD card.  
10 times per seconds, a new line is created in a CSV file. This line contains every data values separated by a semicolon.  
Current values are displayed in realtime on the OLED screen attached.

# Could it be used as a laptimer ?
YES ! Since v4 and the integration of a 10Hz GPS chip, laptimer functions are available.

# How does it work ?
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

# What's next
Some ideas :  
* __Bluetooth connectivity__ : Add Bluetooth support to get realtime data or transfer CSV files on smartphone
* __Laptimer split time__ : Add split time management

# Repository Contents
* /Arduino - The .ino file you need to put in the Arduino M0
* /Documentation - Some brief explanations about this shield (french - not translated)
* /Eagle Libraries - All Eagle parts I use in my schematics
* /Eagle Sources - Eagle files of this project
* /Gerber Sources - Last Gerber files, used to produce PCB
* /Partslist - A detailed BOM list

