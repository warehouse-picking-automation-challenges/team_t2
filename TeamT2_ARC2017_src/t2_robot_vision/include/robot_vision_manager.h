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
#ifndef ROBOT_VISION_MANAGER
#define ROBOT_VISION_MANAGER

// Include base nodelet
#include "robot_vision_nodelet.h"

// Include message type
#include <catch_robot_msgs/CaptureReq.h>
#include <catch_robot_msgs/RecognizeReq.h>
#include <T2_robot_vision/CancelCaptureReq.h>
#include <T2_robot_vision/RecognitionTarget.h>
#include <t2_msgs/OccupancyReq.h>
#include <T2_robot_vision/OccupancyTarget.h>

namespace T2_robot_vision
{
	class robot_vision_manager: public T2_robot_vision::robot_vision_nodelet
	{
	public:
		robot_vision_manager();
		virtual ~robot_vision_manager();

	protected:
		virtual void onInit();

		void captureReqCallback(const catch_robot_msgs::CaptureReqConstPtr &req);
		void recognizeReqCallback(const catch_robot_msgs::RecognizeReqConstPtr &req);
		void occupancyReqCallback(const t2_msgs::OccupancyReqConstPtr &req);
		void cancelCaptureReqCallback(const T2_robot_vision::CancelCaptureReqConstPtr &req);

		// Subscriber and Publisher
		ros::Subscriber capture_req_sub_;
		ros::Subscriber recognize_req_sub_;
		ros::Subscriber occupancy_req_sub_;
		ros::Subscriber cancel_capture_req_sub_;
		ros::Publisher capture_res_pub_;
		//ros::Publisher recog_target_pub_;
		ros::Publisher cancel_capture_res_pub_;
		std::vector<ros::Publisher> recog_target_pubs_;
		std::vector<ros::Publisher> occupancy_target_pubs_;
		ros::Publisher occupancy_target_pub_;
		ros::Publisher test_pub_;
		
		std::vector<ros::ServiceClient> clients_;

		//T2_robot_vision::RecognitionTargetPtr recog_target_;
		std::map<uint32_t, T2_robot_vision::RecognitionTargetPtr> recognize_targets_;

		static const int NUM_REALSENSE_DEFAULT;
	    static const int NUM_ENSENSO_DEFAULT;
		static const int NUM_RECOGNITION_DEFAULT;
        static const int NUM_OCCUPANCY_DEFAULT;

		int num_realsense_;
		int num_ensenso_;
		int num_recognition_;
        int num_occupancy_;

		//static const int SENSOR_NUM = 5;
	};
}	// namespace T2_robot_vision

#endif //ROBOT_VISION_MANAGER

