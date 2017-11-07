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

#include <T2_robot_vision/CalibratedData.h>
#include <catch_robot_msgs/RecognizeRes.h>
#include <T2_robot_vision/CadItem.h>
#include <robot_vision_calibration.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

#define USE_QUATERNION 1
#if USE_QUATERNION
#include <t2_msgs/RecognizeRes.h>
#endif

// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_calibration, nodelet::Nodelet);

namespace T2_robot_vision
{

#if USE_QUATERNION
typedef t2_msgs::RecognizeRes RecognizeRes;
#else
typedef catch_robot_msgs::RecognizeRes RecognizeRes;
#endif

robot_vision_calibration::robot_vision_calibration() :
    base_cam_id_(0)
{
}

robot_vision_calibration::~robot_vision_calibration()
{
}

void robot_vision_calibration::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  recog_target_sub_ = nh.subscribe("recognition_target", 10, &robot_vision_calibration::recognitionTargetCallback,
      this);
  recog_result_sub_ = nh.subscribe("recognized_result", 10, &robot_vision_calibration::recognizedResultCallback, this);

  calib_data_pub_ = nh.advertise<T2_robot_vision::CalibratedData>("calibrated_data", 10);
  recog_res_pub_ = nh.advertise<RecognizeRes>("recognize_res", 10);
  client_CI = nh.serviceClient <T2_robot_vision::CadItem>("caditem_srv");

  for (int i = 0; i <= 10; ++i)
  {
    std::ostringstream oss;
    oss << i;
    std::string calib_name = "/calibration" + oss.str();
    std::string ttw_name = calib_name + "/transform_to_world";
    if (nh.hasParam(ttw_name))
    {
      std::vector<double> vec;
      nh.getParam(ttw_name, vec);
      cv::Mat mat(4, 4, CV_64FC1, vec.data());
      matBin[i] = mat.clone();
    }
    else
    {
      NODELET_ERROR_STREAM("No parameter: " << ttw_name);
    }
  }
}

void robot_vision_calibration::recognitionTargetCallback(const T2_robot_vision::RecognitionTargetConstPtr &target)
{
  NODELET_INFO("Recognition Target Callback...");

  // ---------- Unify Data ---------- //
  T2_robot_vision::CalibratedDataPtr data(new T2_robot_vision::CalibratedData);

  seq_no_ = target->seq_no;
  job_no_ = target->job_no;
  base_cam_id_ = target->base_cam_id;
  point_cloud_ts_ = target->data[0].points.header.stamp;
  if (point_cloud_ts_.isZero())
  {
    point_cloud_ts_ = ros::Time::now();
  }

  data->recog_target = *target;

  T2_robot_vision::CadItemRequest cireq;
  T2_robot_vision::CadItemResponse cires;
  data->recog_target.category_id.clear();
  for( int i=0; i<data->recog_target.cad_id.size(); i++  ){
    cireq.cad_id = data->recog_target.cad_id[i];
    if( client_CI.call(cireq, cires) ){
      data->recog_target.category_id.push_back( cires.category_id );
      ROS_INFO(" OK: cad: %d -> item: %d", cireq.cad_id, cires.category_id );
    } else {
      data->recog_target.category_id.push_back( -2 );
      ROS_INFO(" NG: cad: %d -> item: %d", cireq.cad_id, cires.category_id );
    }
  }

  // 点群をベースカメラ座標に変換
  data->calibrated_points.push_back(target->data[0].points);
  calib_data_pub_.publish(data);
}

void robot_vision_calibration::recognizedResultCallback(const T2_robot_vision::RecognizedResultConstPtr &result)
{
  NODELET_INFO("Recognition Result Callback...");

  RecognizeRes::Ptr res(new RecognizeRes);

  res->seq_no = seq_no_;
  res->job_no = job_no_;

  std::map<uint32_t, cv::Mat>::iterator it = matBin.find(base_cam_id_);
  if (it == matBin.end())
  {
    NODELET_ERROR_STREAM("Not found BIN Cmaera pose matrix. cam_id:" << base_cam_id_);
  }
  else
  {
    cv::Mat pose(4, 4, CV_64FC1);

    for (int i = 0, sz = result->items.size(); i < sz; ++i)
    {
      catch_robot_msgs::ItemData item = result->items[i];

      double data[4][4] = { { item.pose.m[0], item.pose.m[1], item.pose.m[2], item.pose.m[3] },
                            { item.pose.m[4], item.pose.m[5], item.pose.m[6], item.pose.m[7] },
                            { item.pose.m[8], item.pose.m[9], item.pose.m[10], item.pose.m[11] },
                            { item.pose.m[12], item.pose.m[13], item.pose.m[14], item.pose.m[15] } };

      cv::Mat matIm(4, 4, CV_64FC1, data);
      pose = it->second * matIm;

      ROS_INFO_STREAM("calibration_ans:\n" << pose);
      Eigen::Affine3d affine;
      affine.matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(pose.ptr<double>(), pose.rows, pose.cols);
      geometry_msgs::Pose pose_msg;
      tf::poseEigenToMsg(affine, pose_msg);
      ROS_INFO_STREAM("pose_msg:\n" << pose_msg);
      t2_msgs::ItemData item_data;
      item_data.pose = pose_msg;
      item_data.cad_id = item.cad_id;
      item_data.category_id = item.category_id;
      item_data.probability = item.probability;
      item_data.scale = item.scale;
      item_data.single_gp = item.single_gp;
      res->items.push_back(item_data);
    }

    // ros message convert to point cloud.
    pcl::PointCloud<pcl::PointXYZRGBA> cloudXYZRGBA;
    pcl::fromROSMsg(result->obstacle_points, cloudXYZRGBA);

    // transeform to world coord of point cloud. 
    Eigen::Matrix4f matW;
    for (int row = 0; row < 4; row++)
      for (int col = 0; col < 4; col++)
        matW(col, row) = static_cast<float>((it->second).at<double>(col, row));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(cloudXYZRGBA, *transformed_cloud, matW);

    // point cloud convert to ros message
    pcl::toROSMsg(*transformed_cloud, res->obstacle_points);

    res->obstacle_points.header.stamp = point_cloud_ts_;
    res->obstacle_points.header.frame_id = "world";

  }

  recog_res_pub_.publish(res);
}

}	// namespace T2_robot_vision
