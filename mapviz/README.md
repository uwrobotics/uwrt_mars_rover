# mapviz
Mapviz is a ROS-based visualization tool with a plug-in system similar to rviz focused on visualization 2D data. For official ROS wiki page of mapviz, please visit [this](http://wiki.ros.org/mapviz) link.


## Installation

### Installing mapviz
**TESTED PLATFORM:** Ubuntu 16.04 with ROS Kinetic

### Install from Source
* Go your catkin workspace and find src directory.
```
$ cd catkin_ws/src
```
* Clone necessary packages
```
$ git clone https://github.com/swri-robotics/mapviz.git --branch $ROS_DISTRO-devel
$ git clone https://github.com/swri-robotics/marti_common.git --branch $ROS_DISTRO-devel
$ git clone https://github.com/swri-robotics/marti_messages.git --branch indigo-devel
```
* Go to root of your workspace and install dependencies
```
$ cd catkin_ws
$ rosdep install --from-paths src --ignore-src
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


#############################################################

**WARNING2:** If you have error after apt update like:
```
W: GPG error: http://packages.ros.org <YOUR_UBUNTU_VERSION> InRelease: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY 5523BAEEB01FA116
```
* The reason behind this is that the old ROS key has been revoked as part of the measures to deal with a recent security incident with build.ros.org (Security issue on ROS build farm). You need to change it. Here are the steps:
* Delete old key
```
$ sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
```
* Add new key
```
$ sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
* Now clean the cache and update
```
$ sudo apt clean && sudo apt update
```
* For further info, you can check [this](https://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/) link. 
  
**AT THE END OF WARNING2**

#############################################################


* Now you can build catkin workspace.
```
$ cd catkin_ws
$ catkin_make
```

### Install from Ros-binaries
```
$ sudo apt-get install ros-kinetic-mapviz ros-kinetic-mapviz-plugins ros-kinetic-tile-map
```

### Installing USB GPS Device
**TESTED PLATFORM:** Ubuntu 16.04 with ROS Kinetic


We are going to use ros package called [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) that is a ROS package reads serial data coming from USB GPS device and convert it to a ROS topic. 

### Installing from source
* Go to your src directory in catkin_ws
```
$ cd catkin_ws/src
```
* Clone necessary packages
```
$ git clone  https://github.com/ros-drivers/nmea_navsat_driver --branch $ROS_DISTRO-devel
$ git clone  https://github.com/ros-drivers/nmea_msgs --branch indigo-devel
```
* Go to root of catkin_ws and build
```
$ cd catkin_ws
$ catkin_make
```

* Source setup.bash and run the example node
```
$ source catkin_ws/devel/setup.bash
$ rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB0 _baud:=38400
```

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

* Generally GPS devices boudrate is 4800. At least, this is the case for the GPS that I am using. Here is the example prompt that I am using: 
```
$ rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyACM0 _baud:=38400
```
* Echo /fix topic to see whether you get correct coordinates
```
$ rostopic echo /fix
```
* Convert latitude and longtitude to Degree Minute Second (DMS) to check whether GPS works fine. 
For this purpose, you can use [this website](https://www.latlong.net/lat-long-dms.html). 

**WARNING:** If you don't get meaningful information or NaN when you subscribe to /fix topic, probably your device's baud rate is different. Try with different baud rates to find yours.

### Add Google Maps to Mapviz
For this purpose, basically follow the tutorial in this [link](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite).











