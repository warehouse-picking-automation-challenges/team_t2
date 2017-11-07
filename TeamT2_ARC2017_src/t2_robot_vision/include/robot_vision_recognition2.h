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
#ifndef ROBOT_VISION_RECOGNITION
#define ROBOT_VISION_RECOGNITION

// include base nodelet 
#include "robot_vision_nodelet.h"

// include subscribe message type
#include "T2_robot_vision/RecognizedItem.h"
#include "T2_robot_vision/RecognizedSegment.h" 
#include <cv_bridge/cv_bridge.h>

namespace T2_robot_vision
{
class robot_vision_recognition2: public T2_robot_vision::robot_vision_nodelet
{
public:
  robot_vision_recognition2();
  virtual ~robot_vision_recognition2();

protected:
  virtual void onInit();
  void recognizedSegmentCallback(const T2_robot_vision::RecognizedSegmentConstPtr &notice);
  void resultYOLOv2Callback(const T2_robot_vision::RecognizedItemConstPtr &notice);

typedef struct Recog2Prm Recog2Prm;

 void YoloLinemodAlgMain(std::vector<cv::Mat>& cMat_v,
	  std::vector<cv::Mat>& dMat_v,
	  std::vector<uint32_t>& seg_sx,
	  std::vector<uint32_t>& seg_ex,
	  std::vector<uint32_t>& seg_sy,
	  std::vector<uint32_t>& seg_ey,
	  std::vector<uint32_t>& seg_imgidx,
	  std::vector<cv::Mat>& mMat_v,
	  std::vector<uint32_t>& recog_category,
	  std::vector<uint32_t>& recog_yaw,
	  std::vector<int32_t>& recog_pitch,
	  std::vector<uint32_t>& recog_roll,
	  std::vector<uint32_t>& recog_score,
	  std::vector<uint32_t>& recog_seg_id,
          int step_roll,
          int jn,
          Recog2Prm recog2_prm
	  );

 void SegLinemodAlgMain(std::vector<cv::Mat>& cMat_v,
	  std::vector<cv::Mat>& dMat_v,
	  std::vector<uint32_t>& seg_sx,
	  std::vector<uint32_t>& seg_ex,
	  std::vector<uint32_t>& seg_sy,
	  std::vector<uint32_t>& seg_ey,
	  std::vector<uint32_t>& seg_imgidx,
	  std::vector<cv::Mat>& mMat_v,
	  std::vector<uint32_t>& recog_category,
	  std::vector<uint32_t>& recog_yaw,
	  std::vector<int32_t>& recog_pitch,
	  std::vector<uint32_t>& recog_roll,
	  std::vector<uint32_t>& recog_score,
	  std::vector<uint32_t>& recog_seg_id,
          int step_roll,
          int jn,
          Recog2Prm recog2_prm
	  );

  // Subscriber and Publisher
  ros::Subscriber recognition_yolo_sub_;
  ros::Subscriber recognition_seg_sub_;
  ros::Publisher recognition_yolo_pub_;
  ros::Publisher recognition_seg_pub_;
  ros::Publisher recognition_seg_2plane_pub_;
};

}

#endif //ROBOT_VISION_RECOGNITION

