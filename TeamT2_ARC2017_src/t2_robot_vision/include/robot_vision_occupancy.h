/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Toshiba Corporation, 
 *                     Toshiba Infrastructure Systems & Solutions Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Toshiba Corporation, nor the Toshiba
 *       Infrastructure Systems & Solutions Corporation, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#ifndef ROBOT_VISION_OCCUPANCY
#define ROBOT_VISION_OCCUPANCY

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <sensor_msgs/PointCloud2.h>

#include <rosbag/bag.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <catch_robot_msgs/conversions.h>

// include base nodelet
#include "robot_vision_nodelet.h"

#include "T2_robot_vision/OccupancyTarget.h"
#include <t2_msgs/OccupancyRes.h>

#define DEBUG 1
#define MAX_FILENAME 1024

namespace T2_robot_vision
{
class robot_vision_occupancy: public T2_robot_vision::robot_vision_nodelet
{
public:
  robot_vision_occupancy();
  virtual ~robot_vision_occupancy();
  
protected:
  virtual void onInit();
  void occupancyTargetCallback(const T2_robot_vision::OccupancyTargetConstPtr &notice);
  int fillUpDeadAngle(pcl::PointCloud<pcl::PointXYZ> &in_cloud,
                    pcl::PointCloud<pcl::PointXYZ> &out_cloud,
                    double fill_up_range_z);
  // Subscriber and Publisher
  ros::Subscriber occupancy_target_sub_;
  ros::Publisher occupancy_res_pub_;
  
  std::map<uint32_t, cv::Mat> matBin;
  double resolution;
  double interval;
  double remove_area;
  // double fill_up_range_z;
  std::map<uint32_t, double> fill_range_bin;
  int save_bag;
  
}; // class robot_vision_occupancy

} // namespace cat_robot_vision

#endif //ROBOT_VISION_OCCUPANCY
