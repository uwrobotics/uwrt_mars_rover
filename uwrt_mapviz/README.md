# uwrt_mapviz

## Installation

## Installing mapviz

### Install from Ros-binaries
```
$ sudo apt-get install ros-kinetic-mapviz ros-kinetic-mapviz-plugins ros-kinetic-tile-map
```

#############################################################

**WARNING1:** If you have any error after rosdep install like:
```
Err:3 http:/
/mirrors.tuna.tsinghua.edu.cn/ros/ubuntu xenial/main amd64 ros-kinetic-unique-id amd64 1.0.5-0xenial-20190320-125240-0800
  404  Not Found [IP: 2402:f000:1:408:8100::1 80]
 ```
* Run ```sudo apt update```.

**AT THE END OF WARNING1**

#############################################################


## Installing USB GPS Device

### Installing from ros binaries
```
$ sudo apt-get install ros-kinetic-nmea-msgs
$ sudo apt-get install ros-kinetic-nmea-navsat-driver
```

**WARNING:** If it gives error, no devices named /dev/ttyUSB0, then try to check whether your system detects the plugged GPS. 
```
$ lsusb
```
If you see GPS device here that's good. Now try to get device path.
```
$ dmesg | grep -i usb
```
Generally if it is not in ttyUSB path, it should be in ttyACM path. For instance, for the computer I am working on the path is ```/dev/ttyACM0```.

**WARNING:** If you have permission errors to open /dev/tty**** path, then give the permission.
```
$ sudo chmod 666 /dev/ttyACM0
```

**WARNING:** Don't forget that you need to give permission again, if you replug the device.

* Echo /fix topic to see whether you get correct coordinates
```
$ rostopic echo /fix
```
* Convert latitude and longtitude to Degree Minute Second (DMS) to check whether GPS works fine. 
For this purpose, you can use [this website](https://www.latlong.net/lat-long-dms.html). 

**WARNING:** If you don't get meaningful information or NaN when you subscribe to /fix topic, probably your device's baud rate is different. Try with different baud rates to find yours.

### Add Google Maps to Mapviz
For this purpose, basically follow the tutorial in this [link](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite).











