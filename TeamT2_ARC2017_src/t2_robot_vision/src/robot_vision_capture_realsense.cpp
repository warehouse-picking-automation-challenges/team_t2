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

/*
 * 撮影部 (RealSense)
 *  ⇔タスクプランナー	：(2)撮影データ取得要求・完了 (Capture.srv)
 */

#include "robot_vision_capture_realsense.h"

// Include message type
//#include <realsense_camera/SetPower.h>
#include <realsense_camera/ForcePower.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <sstream>
#include <fstream>

#define CAPTURE_WAIT_COUNT 5 

// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_capture_realsense, nodelet::Nodelet);

namespace T2_robot_vision
{
robot_vision_capture_realsense::robot_vision_capture_realsense() :
    rgb_captured_(false), depth_captured_(false), points_captured_(false), rgb_c2d_captured_(false), depth_d2c_captured_(false)
{
}

robot_vision_capture_realsense::~robot_vision_capture_realsense()
{
}

void robot_vision_capture_realsense::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  server_ = nh.advertiseService("capture_srv", &robot_vision_capture_realsense::capture, this);
  client_ = nh.serviceClient<realsense_camera::ForcePower>("camera/driver/force_power");

  rgb_pub_ = pnh.advertise<sensor_msgs::Image>("realsense_color", 10);
  depth_pub_ = pnh.advertise<sensor_msgs::Image>("realsense_depth", 10);
  points_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("realsense_points", 10);
  rgb_c2d_pub_ = pnh.advertise<sensor_msgs::Image>("realsense_color_c2d", 10);
  depth_d2c_pub_ = pnh.advertise<sensor_msgs::Image>("realsense_depth_d2c", 10);

  realsense_camera::ForcePower srv;
  srv.request.power_on = false;
  if (!client_.call(srv))
  {
    NODELET_ERROR("Failed to call SetPower on service!");
  }

  for (int i = 0; i <= 40; i++)
  {
    std::ostringstream oss;
    oss << i;
    std::string base_name = "/calibration" + oss.str();
    std::string serial_number_ = base_name + "/serial_number";
    std::string rgb_camera_matrix_ = base_name + "/rgb_camera_matrix";
    std::string rgb_distCoeffs_ = base_name + "/rgb_distCoeffs";
    std::string depth_camera_matrix_ = base_name + "/depth_camera_matrix";
    std::string depth_distCoeffs_ = base_name + "/depth_distCoeffs";
    std::string camera_rotation_ = base_name + "/camera_rotation";
    std::string camera_translation_ = base_name + "/camera_translation";
    std::string serial;
    std::string cam_serial;
    ros::NodeHandle &nh = getNodeHandle();
    std::vector<double> vec;
    std::vector<int> invec;
    cv::Mat matrix;
    nh.getParam("driver/serial_no", cam_serial);
    nh.getParam(serial_number_, serial);
    if(serial == cam_serial){
      nh.getParam(rgb_camera_matrix_, vec);
      matrix = cv::Mat(3, 3, CV_64FC1, vec.data());

      rgb_camera_matrix[serial] = matrix.clone();
      vec.clear();

      nh.getParam(rgb_distCoeffs_, vec);
      matrix = cv::Mat(5, 1, CV_64FC1, vec.data());
      rgb_distCoeffs[serial] = matrix.clone();
      vec.clear();

      nh.getParam(depth_camera_matrix_, vec);
      matrix = cv::Mat(3, 3, CV_64FC1, vec.data());
      depth_camera_matrix[serial] = matrix.clone();
      vec.clear();

      nh.getParam(depth_distCoeffs_, vec);
      matrix = cv::Mat(5, 1, CV_64FC1, vec.data());
      depth_distCoeffs[serial] = matrix.clone();
      vec.clear();

      nh.getParam(camera_rotation_, vec);
      matrix = cv::Mat(3, 3, CV_64FC1, vec.data());
      camera_rotation[serial] = matrix.clone();
      vec.clear();

      nh.getParam(camera_translation_, vec);
      matrix = cv::Mat(3, 1, CV_64FC1, vec.data());
      camera_translation[serial] = matrix.clone();
      vec.clear();

      std::string src_root_path = ros::package::getPath("T2_robot_vision");
      std::string file_name = src_root_path + "/data/" + "least_squares_plane/" + serial + "_least_squares_plane.csv";
      std::ifstream ifs(file_name);
      std::string str;
      double num;
      if (!ifs) {
        std::cout << "Error:Input data file not found" << std::endl;
        return;
      }

      while (std::getline(ifs, str)){
        std::string token;

        std::istringstream stream(str);

        while (std::getline(stream, token, ',')) {
          if (!stream.eof()){
            num = std::stod(token);
            int temp = (int)(num * std::pow(2,24));
            invec.push_back(temp);
          } //if
        } //while
      } //while

      ifs.close();

      matrix = cv::Mat(640, 480, CV_32SC1, invec.data());
      least_squares[serial] = matrix.clone();
    } //if
    
  } //for

}

void robot_vision_capture_realsense::colorCallback(const sensor_msgs::ImageConstPtr &image)
{
  static int capture_color_wait_count = 1;
  NODELET_INFO("Capture Color Image");
  if(capture_color_wait_count < CAPTURE_WAIT_COUNT){
	capture_color_wait_count++;
  }
  else{
	capture_color_wait_count = 1;
	rgb_ = image;
	rgb_captured_ = true;
	rgb_sub_.shutdown();
  }
}

void robot_vision_capture_realsense::depthCallback(const sensor_msgs::ImageConstPtr &image)
{
  static int capture_depth_wait_count = 1;
  NODELET_INFO("Capture Depth Image");
  if(capture_depth_wait_count < CAPTURE_WAIT_COUNT){
	capture_depth_wait_count++;
  }else{
	capture_depth_wait_count = 1;
	depth_ = image;
	depth_captured_ = true;
	depth_sub_.shutdown();
  }
}

void robot_vision_capture_realsense::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &points)
{
  static int capture_points_wait_count = 1;
  NODELET_INFO("Capture Point Cloud");
  if(capture_points_wait_count < CAPTURE_WAIT_COUNT){
	capture_points_wait_count++;
  }else{
	capture_points_wait_count = 1;
	points_ = points;
	points_captured_ = true;
	points_sub_.shutdown();
  }
}

void robot_vision_capture_realsense::imageColor2DepthCallback(const sensor_msgs::ImageConstPtr &image)
{
  static int capture_c2d_wait_count = 1;
  NODELET_INFO("Capture Color2Depth Image");
  if(capture_c2d_wait_count < CAPTURE_WAIT_COUNT){
	capture_c2d_wait_count++;
  }else{
	capture_c2d_wait_count = 1;
	rgb_c2d_ = image;
	rgb_c2d_captured_ = true;
	rgb_c2d_sub_.shutdown();
  }
}

void robot_vision_capture_realsense::imageDepth2ColorCallback(const sensor_msgs::ImageConstPtr &image)
{
  static int capture_d2c_wait_count = 1;
  NODELET_INFO("Capture Depth2Color Image");
  if(capture_d2c_wait_count < CAPTURE_WAIT_COUNT){
	capture_d2c_wait_count++;
  }else{
	capture_d2c_wait_count = 1;
	depth_d2c_ = image;
	depth_d2c_captured_ = true;
	depth_d2c_sub_.shutdown();
  }
}

void robot_vision_capture_realsense::depthInfoCallback(const sensor_msgs::CameraInfoConstPtr &info)
{
  NODELET_INFO("Depth Camera Info");
  depth_info_ = info;
  depth_info_recv_ = true;
  depth_info_sub_.shutdown();
}


void undistort_nearest(cv::Mat &src, cv::Mat &dst, 
    cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
  dst = cv::Mat(src.size(), src.type());

  int stripe_size0 = std::min(std::max(1, (1 << 12) / std::max(src.cols, 1)), src.rows);
  cv::Mat map1(stripe_size0, src.cols, CV_16SC2), map2(stripe_size0, src.cols, CV_16UC1);

  cv::Mat_<double> A, Ar, I = cv::Mat_<double>::eye(3, 3);

  cameraMatrix.convertTo(A, CV_64F);
  if (distCoeffs.data)
    distCoeffs = cv::Mat_<double>(distCoeffs);
  else
  {
    distCoeffs.create(5, 1, CV_64F);
    distCoeffs = 0.;
  }

  cv::InputArray _newCameraMatrix = cv::noArray();
  cv::Mat newCameraMatrix = _newCameraMatrix.getMat();;
  if (newCameraMatrix.data)
    newCameraMatrix.convertTo(Ar, CV_64F);
  else
    A.copyTo(Ar);

  double v0 = Ar(1, 2);

  for (int y = 0; y < src.rows; y += stripe_size0)
  {
    int stripe_size = std::min(stripe_size0, src.rows - y);
    Ar(1, 2) = v0 - y;
    cv::Mat map1_part = map1.rowRange(0, stripe_size),
      map2_part = map2.rowRange(0, stripe_size),
      dst_part = dst.rowRange(y, y + stripe_size);

    initUndistortRectifyMap(A, distCoeffs, I, Ar, cv::Size(src.cols, stripe_size),
      map1_part.type(), map1_part, map2_part);
    remap(src, dst_part, map1_part, map2_part, cv::INTER_NEAREST, cv::BORDER_CONSTANT);
  }
}


bool robot_vision_capture_realsense::capture(T2_robot_vision::CaptureRequest &req,
    T2_robot_vision::CaptureResponse &res)
{
  NODELET_INFO("Receive capture service call");

  ros::NodeHandle nh = getMTNodeHandle();
  ros::NodeHandle pnh = getMTPrivateNodeHandle();

  res.data.cam_id = req.cam_id;

  realsense_camera::ForcePower srv;
  srv.request.power_on = true;
  if (!client_.call(srv))
  {
    NODELET_ERROR("Failed to call SetPower on service!");
    return false;
  }
  depth_info_recv_ = false;
  depth_info_sub_ = nh.subscribe("camera/depth/camera_info", 10, &robot_vision_capture_realsense::depthInfoCallback, this);
  while (ros::ok() && !(depth_info_recv_)){
    ros::spinOnce();
  }


  rgb_captured_ = false;
  depth_captured_ = false;
  points_captured_ = false;
  rgb_c2d_captured_ = false;
  depth_d2c_captured_ = false;

  rgb_sub_ = nh.subscribe("camera/rgb/image_rect_color", 10, &robot_vision_capture_realsense::colorCallback, this);
  depth_sub_ = nh.subscribe("camera/depth/image_rect_raw", 10, &robot_vision_capture_realsense::depthCallback, this);
  points_sub_ = nh.subscribe("camera/depth/points", 10, &robot_vision_capture_realsense::pointsCallback, this);
  rgb_c2d_sub_ = nh.subscribe("camera/fusion/image_c2d", 10, &robot_vision_capture_realsense::imageColor2DepthCallback, this);
  depth_d2c_sub_ = nh.subscribe("camera/fusion/image_d2c", 10, &robot_vision_capture_realsense::imageDepth2ColorCallback, this);

  NODELET_INFO("Wait for captured image...");

  while (ros::ok() && !(rgb_captured_ && depth_captured_ && points_captured_ && rgb_c2d_captured_ && depth_d2c_captured_))
  {
    ros::spinOnce();
  }

  std::string serial;
  nh.getParam("driver/serial_no", serial);
  cv::Mat c_cameraM = rgb_camera_matrix[serial].clone();
  cv::Mat c_distCoeffs = rgb_distCoeffs[serial].clone();
  cv::Mat d_cameraM = depth_camera_matrix[serial].clone();
  cv::Mat d_distCoeffs = depth_distCoeffs[serial].clone();
  cv::Mat R = camera_rotation[serial].clone();
  cv::Mat T = camera_translation[serial].clone();
  cv::Mat lq = least_squares[serial].clone();
  cv::Mat rgb_undist;
  cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(*rgb_, sensor_msgs::image_encodings::RGB8);
  cv::undistort(cv_rgb_ptr->image, rgb_undist, c_cameraM, c_distCoeffs);
  sensor_msgs::ImagePtr msg_rgb_ptr = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb_undist).toImageMsg();
  res.data.rgb = *msg_rgb_ptr;
  cv::Mat depth_undist;
  sensor_msgs::Image depth_2 = *depth_;
  depth_2.encoding = "mono16";
  cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(depth_2, sensor_msgs::image_encodings::MONO16);
  cv_depth_ptr->image.copyTo(depth_undist);
  res.data.depth = *depth_;
  cv::Mat d2c(cv::Size(rgb_undist.cols, rgb_undist.rows), CV_16UC1, cv::Scalar(0));
  cv::Mat c2d(cv::Size(depth_undist.cols, depth_undist.rows), CV_8UC3, cv::Scalar(0, 0, 0));
  double div = 1000.0;
  for (int j = 0; j < depth_undist.rows; j++){
    for (int i = 0; i < depth_undist.cols; i++){
      short dp_s = depth_undist.at<short>(j, i);
      if (dp_s != 0){
        cv::Mat point(cv::Size(1, 3), CV_64FC1);
        point.at<double>(0, 0) = i;
        point.at<double>(1, 0) = j;
        point.at<double>(2, 0) = 1;

        point = d_cameraM.inv() * point;

      	double p_d = (double)dp_s / div;

        point.at<double>(2, 0) *= p_d;
        point.at<double>(0, 0) *= p_d;
        point.at<double>(1, 0) *= p_d;
      
        point = R * (point + T);
      
        point.at<double>(2, 0) /= p_d;
        point.at<double>(0, 0) /= p_d;
        point.at<double>(1, 0) /= p_d;

        point = c_cameraM * point;

        int x = (int)(point.at<double>(0, 0) + 0.5);
        int y = (int)(point.at<double>(1, 0) + 0.5);
        if ((x > 0) && (y > 0) && (x < rgb_undist.cols) && (y < rgb_undist.rows)){
          
          c2d.at<cv::Vec3b>(j, i)[0] = rgb_undist.at<cv::Vec3b>(y, x)[0];
          c2d.at<cv::Vec3b>(j, i)[1] = rgb_undist.at<cv::Vec3b>(y, x)[1];
          c2d.at<cv::Vec3b>(j, i)[2] = rgb_undist.at<cv::Vec3b>(y, x)[2];

          d2c.at<short>(y, x) = depth_undist.at<short>(j, i);

        }
      }
    }
  }
  cv::Mat mor_d2c;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
  cv::morphologyEx(d2c, mor_d2c, cv::MORPH_CLOSE, kernel);

  sensor_msgs::ImagePtr msg_c2d_ptr = cv_bridge::CvImage(std_msgs::Header(), "rgb8", c2d).toImageMsg();
  res.data.rgb_1 = *msg_c2d_ptr;

  sensor_msgs::ImagePtr msg_d2c_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono16", mor_d2c).toImageMsg();
  res.data.depth_1 = *msg_d2c_ptr;
  res.data.points = *points_;

  if (res.data.depth.encoding == "16UC1"){
    sensor_msgs::Image depth = res.data.depth;
    depth.encoding = "mono16";
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;
    xyzrgba_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(cv_ptr->image.cols, cv_ptr->image.rows));
    xyzrgba_cloud->is_dense = false;

    double fx = d_cameraM.at<double>(0, 0);
    double fy = d_cameraM.at<double>(1, 1);
    double cx = d_cameraM.at<double>(0, 2);
    double cy = d_cameraM.at<double>(1, 2);

    NODELET_INFO_STREAM("Depth Camera Info:");
    NODELET_INFO_STREAM(" fx: " << fx);
    NODELET_INFO_STREAM(" fy: " << fy);
    NODELET_INFO_STREAM(" cx: " << cx);
    NODELET_INFO_STREAM(" cy: " << cy);

    for (int v = 0; v < cv_ptr->image.rows; v++) {
      pcl::PointXYZRGBA* cloud_row = &xyzrgba_cloud->points[v * cv_ptr->image.cols];

      for (int u = 0; u < cv_ptr->image.cols; u++) {
        double d = (double)(cv_ptr->image.at<uint16_t>(v,u)) / 1000.0;

        cloud_row[u].x = d * (u - cx) / fx;
        cloud_row[u].y = d * (v - cy) / fy;
        cloud_row[u].z = d;
        cloud_row[u].r = 0;
        cloud_row[u].g = 255;
        cloud_row[u].b = 0;
      }
    }

    pcl::toROSMsg(*xyzrgba_cloud, res.data.points);
  }
  else{
  }
  rgb_pub_.publish(rgb_);
  depth_pub_.publish(depth_);
  points_pub_.publish(points_);

  rgb_c2d_pub_.publish(rgb_c2d_);
  depth_d2c_pub_.publish(depth_d2c_);

  srv.request.power_on = false;
  if (!client_.call(srv))
  {
    NODELET_ERROR("Failed to call SetPower off service!");
    return false;
  }

  NODELET_INFO(">> T2_robot_vision:captures service return.");
  return true;
}

} // namespace T2_robot_vision
