ROS driver for HLDS 3D TOF sensor
============================
This document and the files associated with it
are CONFIDENTIAL to Hitachi and HLDS. They are
only be to used according the signed NDA and
not shared with others.

Requirements
------------
Ubuntu 16.04 Desktop Linux
ROS 1.0 Kinetic

Installation
------------
* Install HLDS 3D TOF SDK for Linux
  $ sudo apt-get install cmake libcurl4-openssl-dev libgtk2.0-dev
  $ sudo dpkg -i libtof-dev_2.0.0-3ubuntu16_amd64.deb

* Install the HLDS 3D TOF ROS driver
  $ cd ~/catkin_ws
  $ source devel/setup.bash
  $ source /opt/ros/kinetic/setup.bash
  $ unzip hlds_3dtof-ros_1.0.1.zip
  $ catkin_make install


Running
-------

  $ roslaunch hlds_3dtof hlds_3dtof.launch


