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
#ifndef ROBOT_VISION_SEGMENT
#define ROBOT_VISION_SEGMENT

// include base nodelet
#include "robot_vision_nodelet.h"

// include subscribe message type
#include <T2_robot_vision/CalibratedData.h>
#include <T2_robot_vision/RecognizedSegment.h>
#include <T2_robot_vision/RecognizedItem.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "seg_common.hpp"

namespace T2_robot_vision
{
class robot_vision_segment: public T2_robot_vision::robot_vision_nodelet
{
public:
  robot_vision_segment();
  virtual ~robot_vision_segment();
  
  int SegmentAlgMain(int job_num,
		     std::vector<uint32_t> cam_id_v,
		     std::vector<cv::Mat> cMat_v,
		     std::vector<cv::Mat> dMat_v,
		     std::vector<cv::Mat> pMat_v,
		     std::vector<cv::Mat> fMat_v,
		     std::vector<uint32_t>& seg_sx,
		     std::vector<uint32_t>& seg_ex,
		     std::vector<uint32_t>& seg_sy,
		     std::vector<uint32_t>& seg_ey,
		     std::vector<cv::Mat>& mMat_v,
		     std::vector<uint32_t>& seg_imgidx
		     );

  void VarNvMap(cv::Mat pCloud, cv::Mat nvMat, cv::Mat& varMono, int t_size);
  void GuessPlane(cv::Mat pCloud, cv::Mat nvMat, cv::Mat varMono,
		  cv::Mat& maskMat, 
		  int t_size,
		  int var_th, double len_th, int area_th);

protected:
  cv::Mat areaMat;
  SegPrm seg_prm;
  
  virtual void onInit();
  void calibratedDataCallback(const T2_robot_vision::RecognizedSegmentConstPtr &notice);
  void seg_python_Callback(const T2_robot_vision::CalibratedDataConstPtr &notice);
  int DetectForegroundPoint(int cam_id, cv::Mat pMat, cv::Mat& flgMat);


  // Subscriber and Publisher
  ros::Subscriber segment_python_sub_;
  ros::Publisher segment_python_pub_;
  ros::Subscriber segment_sub_;
  ros::Publisher segment_pub_;
  ros::Publisher segment_unknown_pub_;
  
};

}

#endif //ROBOT_VISION_SEGMENT

