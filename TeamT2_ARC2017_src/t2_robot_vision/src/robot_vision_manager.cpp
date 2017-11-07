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
#include "robot_vision_manager.h"

// Include message type
#include <T2_robot_vision/CancelCaptureRes.h>
#include <catch_robot_msgs/CaptureRes.h>
#include <T2_robot_vision/Capture.h>

// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_manager, nodelet::Nodelet);

namespace T2_robot_vision
{
const int robot_vision_manager::NUM_REALSENSE_DEFAULT = 20;
const int robot_vision_manager::NUM_ENSENSO_DEFAULT = 1;
const int robot_vision_manager::NUM_RECOGNITION_DEFAULT = 6;
const int robot_vision_manager::NUM_OCCUPANCY_DEFAULT = 8;

robot_vision_manager::robot_vision_manager() :
    num_realsense_(0), num_ensenso_(0), num_recognition_(0), num_occupancy_(0)
{
}

/*
 * Nodelet Destructor.
 */
robot_vision_manager::~robot_vision_manager()
{
}

/*
 * Initialize the nodelet.
 */
void robot_vision_manager::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  capture_req_sub_ = nh.subscribe("capture_req", 20, &robot_vision_manager::captureReqCallback, this);
  capture_res_pub_ = nh.advertise<catch_robot_msgs::CaptureRes>("capture_res", 20);
  recognize_req_sub_ = nh.subscribe("recognize_req", 10, &robot_vision_manager::recognizeReqCallback, this);
  occupancy_req_sub_ = nh.subscribe("occupancy_req", 10, &robot_vision_manager::occupancyReqCallback, this);
  occupancy_target_pub_ = nh.advertise<T2_robot_vision::OccupancyTarget>("occupancy_target", 10);

  // get parameters
  pnh.param("num_realsense_capture_unit", num_realsense_, NUM_REALSENSE_DEFAULT);
  if (num_realsense_ < 1)
  {
    NODELET_ERROR_STREAM("Invalid value num_realsense_capture_unit: " << num_realsense_);
  }

  pnh.param("num_recognition_unit", num_recognition_, NUM_RECOGNITION_DEFAULT);
  if (num_recognition_ < 1)
  {
    NODELET_ERROR_STREAM("Invalid value num_recognition_unit: " << num_recognition_);
  }
  pnh.param("num_ensenso_capture_unit", num_ensenso_, NUM_ENSENSO_DEFAULT);
  if (num_ensenso_ < 1)
  {
    NODELET_ERROR_STREAM("Invalid value num_ensenso_capture_unit: " << num_ensenso_);
  }
  pnh.param("num_occupancy_unit", num_occupancy_, NUM_OCCUPANCY_DEFAULT);
  if (num_occupancy_ < 1)
  {
    NODELET_ERROR_STREAM("Invalid value num_occupancy_unit: " << num_occupancy_);
  }

  NODELET_INFO_STREAM("num_realsense_capture_unit: " << num_realsense_);
  NODELET_INFO_STREAM("num_ensenso_capture_unit: " << num_ensenso_);
  NODELET_INFO_STREAM("num_recognition_unit: " << num_recognition_);
  NODELET_INFO_STREAM("num_occupancy_unit: " << num_occupancy_);

  clients_.clear();
  for (int i = 0; i < num_ensenso_; ++i)
  {
    std::ostringstream oss;
    oss << i;
    std::string name = "capture_srv_ensenso";
    NODELET_INFO_STREAM("service name: " << name);
    ros::ServiceClient sc = nh.serviceClient<T2_robot_vision::Capture>(name);
    clients_.push_back(sc);
  }
  for (int i = 1; i < num_realsense_; ++i)
  {
    std::ostringstream oss;
    oss << i;
    std::string name = "capture_srv" + oss.str();
    NODELET_INFO_STREAM("service name: " << name);
    ros::ServiceClient sc = nh.serviceClient<T2_robot_vision::Capture>(name);
    clients_.push_back(sc);
  }

  recog_target_pubs_.clear();
  for (int i = 0; i < num_recognition_; ++i)
  {
    std::ostringstream oss;
    oss << i;
    std::string name = "recognition_target" + oss.str();
    NODELET_INFO_STREAM("[robot_vision_manager] publisher name: " << name);
    ros::Publisher pub = nh.advertise<T2_robot_vision::RecognitionTarget>(name, 10);
    recog_target_pubs_.push_back(pub);
  }

  occupancy_target_pubs_.clear();
  for (int i = 0; i < num_occupancy_; ++i)
  {
    std::ostringstream oss;
    oss << i;
    std::string name = "occupancy_target" + oss.str();
    // std::string name = "occupancy_target";
    NODELET_INFO_STREAM("[robot_vision_manager] publisher name: " << name);
    ros::Publisher pub = nh.advertise<T2_robot_vision::OccupancyTarget>(name, 10);
    occupancy_target_pubs_.push_back(pub);
  }
}

void robot_vision_manager::captureReqCallback(const catch_robot_msgs::CaptureReqConstPtr& req)
{
  T2_robot_vision::CaptureRequest img_req;
  T2_robot_vision::CaptureResponse img_res;

  img_req.job_no = req->job_no;
  img_req.cam_id = req->cam_id;

  if (req->job_no == 1 && req->seq_no == 1){
    recognize_targets_.clear();
  }

  int cam_id;
  std::ostringstream oss;
  oss << req->cam_id;
  std::string name = "calibration" + oss.str() + "/cam_id";
  getNodeHandle().getParam(name, cam_id);
  NODELET_INFO_STREAM("[robot_vision_manager] camera id: " << cam_id);
  if (cam_id < clients_.size())
  {
    if (clients_[cam_id].call(img_req, img_res))
    {
      std::map<uint32_t, T2_robot_vision::RecognitionTargetPtr>::iterator it = recognize_targets_.find(req->job_no);
      if (it != recognize_targets_.end()){
        it->second->data.push_back(img_res.data);		
      }
      else{
        T2_robot_vision::RecognitionTargetPtr recog_target(new T2_robot_vision::RecognitionTarget);
        recog_target->data.push_back(img_res.data);
        recognize_targets_[req->job_no] = recog_target;
      }
    }
    else
    {
      NODELET_ERROR("Failed to call Capture service!");
    }

    /* Capture Response (Topic) */
    catch_robot_msgs::CaptureResPtr res(new catch_robot_msgs::CaptureRes);
    res->seq_no = req->seq_no;
    res->job_no = req->job_no;
    res->cam_id = req->cam_id;
    res->result = 0; /* Set result */
    capture_res_pub_.publish(res);
  }
}

void robot_vision_manager::recognizeReqCallback(const catch_robot_msgs::RecognizeReqConstPtr& req)
{
  std::map<uint32_t, T2_robot_vision::RecognitionTargetPtr>::iterator it = recognize_targets_.find(req->job_no);
  if (it != recognize_targets_.end()){
    T2_robot_vision::RecognitionTargetPtr data = it->second;
    data->job_no = req->job_no;
    data->cad_id = req->cad_id;
    data->base_cam_id = data->data[0].cam_id; /* Set base camera id */
	int tmp_n = 1;
	if(data->data[0].cam_id!=0) tmp_n = data->data[0].cam_id;
	recog_target_pubs_[tmp_n % num_recognition_].publish(data);
    recognize_targets_.erase(it);
  }
}

void robot_vision_manager::occupancyReqCallback(const t2_msgs::OccupancyReqConstPtr& req)
{
  ROS_INFO("Receive Occupancy_req");
  std::map<uint32_t, T2_robot_vision::RecognitionTargetPtr>::iterator it = recognize_targets_.find(req->job_no);
  if (it != recognize_targets_.end()){
    T2_robot_vision::OccupancyTarget data;
    data.job_no = req->job_no;
    data.seq_no = req->seq_no;
	  data.base_cam_id = it->second->data[0].cam_id;
    data.points.push_back(it->second->data[0].points);
	int tmp_n = 1;
	if(it->second->data[0].cam_id!=0) tmp_n = it->second->data[0].cam_id;
  ROS_INFO("wait subscriber ...");
  ros::Rate rate(10); /* 10 Hz */
  while(ros::ok() && occupancy_target_pubs_[tmp_n % num_occupancy_].getNumSubscribers() == 0) {
    rate.sleep();
  } 
    occupancy_target_pubs_[tmp_n % num_occupancy_].publish(data);
    recognize_targets_.erase(it);
  }
  ROS_INFO_STREAM("Send Occupancy_target no:" << req->job_no % num_occupancy_);

}

void robot_vision_manager::cancelCaptureReqCallback(const T2_robot_vision::CancelCaptureReqConstPtr& req)
{
  std::map<uint32_t, T2_robot_vision::RecognitionTargetPtr>::iterator it = recognize_targets_.find(req->job_no);
  if (it != recognize_targets_.end()){
    recognize_targets_.erase(it);
  }

  T2_robot_vision::CancelCaptureResPtr res(new T2_robot_vision::CancelCaptureRes);
  res->seq_no = req->seq_no;
  res->job_no = req->job_no;
  res->result = 0;
  cancel_capture_res_pub_.publish(res);
}

} // namespace T2_robot_vision
