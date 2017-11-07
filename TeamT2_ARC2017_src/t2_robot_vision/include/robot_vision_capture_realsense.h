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
#ifndef ROBOT_VISION_CAPTURE_REALSENSE
#define ROBOT_VISION_CAPTURE_REALSENSE

#include <sensor_msgs/CameraInfo.h>

// include base nodelet
#include "robot_vision_nodelet.h"

// include message type
#include <T2_robot_vision/Capture.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace T2_robot_vision
{

	class robot_vision_capture_realsense: public T2_robot_vision::robot_vision_nodelet
	{
	public:
		robot_vision_capture_realsense();
		~robot_vision_capture_realsense();

	private:
		virtual void onInit();
		void colorCallback(const sensor_msgs::ImageConstPtr &image);
		void depthCallback(const sensor_msgs::ImageConstPtr &image);
		void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &points);
		void imageColor2DepthCallback(const sensor_msgs::ImageConstPtr &image);
		void imageDepth2ColorCallback(const sensor_msgs::ImageConstPtr &image);
		void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr &info);

		bool capture(T2_robot_vision::CaptureRequest &req, T2_robot_vision::CaptureResponse &res);

		// Subscriber
		ros::Subscriber rgb_sub_;
		ros::Subscriber depth_sub_;
		ros::Subscriber points_sub_;
		ros::Subscriber rgb_c2d_sub_;
		ros::Subscriber depth_d2c_sub_;
		ros::Subscriber depth_info_sub_;

		ros::Publisher rgb_pub_;
		ros::Publisher depth_pub_;
		ros::Publisher points_pub_;
		ros::Publisher rgb_c2d_pub_;
		ros::Publisher depth_d2c_pub_;

		// Service Client
		ros::ServiceServer server_;
		ros::ServiceClient client_;

		sensor_msgs::ImageConstPtr rgb_;
		sensor_msgs::ImageConstPtr depth_;
		sensor_msgs::PointCloud2ConstPtr points_;
		sensor_msgs::ImageConstPtr rgb_c2d_;
		sensor_msgs::ImageConstPtr depth_d2c_;
		sensor_msgs::CameraInfoConstPtr depth_info_;
		
		std::map<std::string, cv::Mat> rgb_camera_matrix;
		std::map<std::string, cv::Mat> rgb_distCoeffs;
		std::map<std::string, cv::Mat> depth_camera_matrix;
		std::map<std::string, cv::Mat> depth_distCoeffs;
		std::map<std::string, cv::Mat> camera_rotation;
		std::map<std::string, cv::Mat> camera_translation;
		std::map<std::string, cv::Mat> least_squares;

		bool rgb_captured_;
		bool depth_captured_;
		bool points_captured_;
		bool rgb_c2d_captured_;
		bool depth_d2c_captured_;
		bool depth_info_recv_;
	};

}   // namespace T2_robot_vision

#endif  // ROBOT_VISION_CAPTURE_REALSENSE
