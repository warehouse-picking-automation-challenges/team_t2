#include "robot_vision_recognition.h"

// Include message type
#include <T2_robot_vision/RecognizedItem.h>
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


// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_recognition, nodelet::Nodelet);

namespace T2_robot_vision
{

robot_vision_recognition::robot_vision_recognition()
{
}

/*
 * Nodelet Destructor.
 */
robot_vision_recognition::~robot_vision_recognition()
{
}

/*
 * Initialize the nodelet.
 */
void robot_vision_recognition::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  recognision_sub_ = nh.subscribe("recognized_segment", 10, &robot_vision_recognition::recognizedSegmentCallback, this);

  recognision_pub_ = nh.advertise<T2_robot_vision::RecognizedItem>("recognized_item", 10);
}

void robot_vision_recognition::recognizedSegmentCallback(const T2_robot_vision::RecognizedSegmentConstPtr &notice)
{
  ROS_INFO_STREAM("recognizedSegmentCallback");
#if 0
  // recognition process

#else
  T2_robot_vision::RecognizedItemPtr data(new T2_robot_vision::RecognizedItem);
  data->recog_target = notice->recog_target;
  data->calibrated_points.push_back(notice->calibrated_points[0]);
  data->segmented_data.push_back(notice->segmented_data[0]);

  T2_robot_vision::ItemizedData item;
  // Dex4_Orange_Tablet_RGB_5_45_1.bmp
  item.category = 0;
  item.pitch = 5;
  item.yaw = 45;
  item.roll = 0;
  item.score = 100;
  item.seg_id = 0;
  data->itemized_data.push_back(item);
#endif
  recognision_pub_.publish(data);
}

} // namespace T2_robot_vision
