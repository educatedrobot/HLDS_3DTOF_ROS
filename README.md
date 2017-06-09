ROS driver for HLDS 3D TOF sensor
============================
This document and the files associated with it
are CONFIDENTIAL to Hitachi and HLDS. They are
only be to used according the signed NDA and
not shared with others.

Requirements
------------
Ubuntu 16.04 Desktop Linux (http://releases.ubuntu.com/16.04/ubuntu-16.04.2-desktop-amd64.iso)  
ROS 1.0 Kinetic (http://www.ros.org/)  
HLDS 3D TOF SDK 2.0.0 or later 

Installation
------------
* Install Ubuntu if not already installed. Ubuntu 16.04 is required for ROS Kinetic  
  Download the .iso image above and install to a PC or virtual machine (Oracle Virtual Box is recommended)  
  
* Install ROS 1.0 Kinetic
  Follow instructions at http://wiki.ros.org/kinetic/Installation/Ubuntu  

* Install HLDS 3D TOF SDK for Linux  
  $ sudo apt-get install cmake libcurl4-openssl-dev libgtk2.0-dev  
  $ sudo dpkg -i libtof-dev_2.0.0-3ubuntu16_amd64.deb  

* Install the HLDS 3D TOF ROS driver  
  $ cd ~/catkin_ws  
  $ source devel/setup.bash  
  $ source /opt/ros/kinetic/setup.bash  
  $ curl https://github.com/educatedrobot/hlds_3dtof/archive/master.zip -o hlds_3dtof_ros.zip  
  $ unzip hlds_3dtof_ros.zip  
  $ rm hlds_3dtof_ros.zip  
  $ catkin_make install  


Running
-------

  $ roslaunch hlds_3dtof hlds_3dtof.launch


