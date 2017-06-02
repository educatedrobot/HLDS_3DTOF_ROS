/* ROS Driver for HLDS 3D TOF sensor
 * Copyright (C) 2017, Hitachi-LG Data Storage, Inc. (HLDS)
 * All rights reserved.
 *
 * The source code and distribution files licensed under
 * the 2-Clause BSD license as Open Source.
 */

/**
* @file     hlds_3dtof_node.cpp
* @brief    ROS Driver for HLDS 3D TOF sensor
* @author   Hitachi-LG Data Storage, Inc. (HLDS)
* @date     2017.03.15
* @version    v1.0.1
* @copyright  2017 Hitachi-LG Data Storage,Inc.
*
* @par Change History:
* - 2017.03.15 Initial
*/

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "hldstof/tof.h"

using namespace std;
using namespace hlds;

#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/publisher.h>
#include <string>

int main(int argc, char* argv[])
{
  // Initialize ROS
  ros::init (argc, argv, "hlds_3dtof");
  std::string frame_id;
  ros::NodeHandle nh;
  sensor_msgs::PointCloud2 cloud;
  std::string file_name;
  std::string cloud_topic;
  ros::Publisher cloudPublisher;

  // Initialize HLDS 3D TOF
  Result ret = Result::OK;
  int numoftof = 0;
  TofManager tm;
  const TofInfo* ptofinfo = nullptr;
  FrameDepth frame; //Create instances for reading frames
  Frame3d frame3d;  //Create instances for 3D data after conversion


  // Open TOF Manager (Read tof.ini file)
  nh.getParam("ini_path", tm.inifilepath);
  if ((ret = tm.Open()) != Result::OK) {
    ROS_ERROR_STREAM("HLDS 3D TOF initialization file \"" << tm.inifilepath  <<"tof.ini\" invalid.");
    return -1;
  }

  // Get number of TOF sensor and TOF information list
  numoftof = tm.GetTofList(&ptofinfo);

  if (numoftof == 0) {
    ROS_ERROR_STREAM("No HLDS 3D TOF sensors found.");
    return -1;
  }

  // Create Tof instances for TOF sensors
  Tof tof;

  // Open all Tof instances (Set TOF information)
  string vfpga = "";
  string vosv = "";
  string vroot = "";
  if (tof.Open(ptofinfo[0]) != Result::OK){
    ROS_ERROR_STREAM("Error connecting to HLDS 3D TOF sensor.");
    return -1;
  }

  if (tof.GetVersion(&vfpga, &vosv, &vroot) == Result::OK) {
    ROS_INFO_STREAM("HLDS 3D TOF version" << ":" << vfpga.c_str() << ":" << vosv.c_str() << ":" << vroot.c_str() );
  }

  //Set camera mode as Depth mode
  if (tof.SetCameraMode(CameraMode::CameraModeDepth) != Result::OK){
    ROS_INFO_STREAM("HLDS 3D TOF ID " << tof.tofinfo.tofid << " Set Camera Mode Error");
    return -1;
  }

  //Set camera pixel
  if (tof.SetCameraPixel(CameraPixel::w320h240) != Result::OK){
    //  if (tof.SetCameraPixel(CameraPixel::w160h120) != Result::OK){
    ROS_INFO_STREAM("HLDS 3D TOF ID " << tof.tofinfo.tofid << " Set Camera Pixel Error");
    return -1;
  }

  //Set TOF sensor angle and height
  int location_x, location_y, location_z;
  int angle_x, angle_y, angle_z;
  nh.param("sensor_location_x", location_x, 0);
  nh.param("sensor_location_y", location_y, 0);
  nh.param("sensor_location_z", location_z, -1000); // Height of sensor from ground to sensor (usually negative) in mm. 
  nh.param("sensor_angle_x", angle_x, 45); // Angle of sensor (0 pointed down, 45 pointed horizontal). 
  nh.param("sensor_angle_y", angle_y, 0);
  nh.param("sensor_angle_z", angle_z, 0);  
  if (tof.SetAttribute(location_x, location_y, location_z, angle_x, angle_y, angle_z) != Result::OK){
    ROS_INFO_STREAM("HLDS 3D TOF ID " << tof.tofinfo.tofid << " Set Camera Position Error");
    return -1;
  }

  //Noise reduction(Low signal cutoff)
  if (tof.SetLowSignalCutoff(20) != Result::OK){
    ROS_INFO_STREAM("HLDS 3D TOF ID " << tof.tofinfo.tofid << " Low Signal Cutoff Error");
    return -1;
  }

  //Start 
  if (tof.Run() != Result::OK){
    ROS_INFO_STREAM("HLDS 3D TOF ID " << tof.tofinfo.tofid << " Run Error");
    return -1;
  }

  // Once Tof instances are started, TofManager is not necessary and closed
  if (tm.Close() != Result::OK){
      ROS_ERROR_STREAM("Error closing HLDS 3D TOF manager.");
      return -1;
  }

  //Create color table
  frame.CreateColorTable(0, 65530);

  // Initialize the ROS publisher
  nh.param("frame_id", frame_id, std::string("/base_link"));
  nh.param("cloud_topic", cloud_topic, std::string("cloud"));
  cloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > (cloud_topic, 1);
  ROS_INFO_STREAM("Publishing data on topic \"" << nh.resolveName(cloud_topic) << "\" with frame_id \"" << frame_id << "\"");

  ros::Rate r(40);

  //while (nh.ok ())
  while (ros::ok())
  {
    ros::spinOnce();

    //Get the latest frame number
    long frameno;
    TimeStamp timestamp;
    tof.GetFrameStatus(&frameno, &timestamp);
    if (frameno != frame.framenumber) {
      //Read a new frame only if frame number is changed(Old data is shown if it is not changed.)

      //Read a frame of depth data
      ret = Result::OK;
      ret = tof.ReadFrame(&frame);
      if (ret != Result::OK) {
        ROS_INFO_STREAM("HLDS 3D TOF read frame error");
        break;
      }

      //3D conversion(with lens correction)
      frame3d.Convert(&frame);

      //3D rotation(to top view)
      frame3d.Rotate(angle_x, angle_y, angle_z);

      // Create the point cloud
      const size_t pixelCount = frame3d.width * frame3d.height;
      pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud(new pcl::PointCloud<pcl::PointXYZ>());
      ptCloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
      ptCloud->header.frame_id = frame_id;
      ptCloud->width = frame3d.width;
      ptCloud->height = frame3d.height;
      ptCloud->is_dense = false;
      ptCloud->points.resize(pixelCount);

      for (int i = 0; i < pixelCount; i++) {
        pcl::PointXYZ &point = ptCloud->points[i];

        //Get coordinates after 3D conversion
        point.x = frame3d.frame3d[i].x;
        point.y = frame3d.frame3d[i].y;
        point.z = frame3d.frame3d[i].z;
      }

      // Publish the point cloud
      cloudPublisher.publish(*ptCloud);
      ROS_DEBUG_STREAM_ONCE("Publishing data with " << pixelCount
                            << " points on topic \"" << nh.resolveName(cloud_topic)
                            << "\" in frame \"" << cloud.header.frame_id << "\"");
    }
    r.sleep();
  }

  // Shutdown the sensor
  if (tof.Stop() != Result::OK || tof.Close() != Result::OK) {
    ROS_INFO_STREAM("HLDS 3D TOF ID " << tof.tofinfo.tofid << " Stop Error");
  }

  return 0;
}
