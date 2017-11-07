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
 * セグメント部
 * 	キャリブレーション部→ ：(9)セグメント要求/キャリブレーション済みデータ (CalibratedData.msg)
 *	→アイテム認識部       ：(10)アイテム認識要求/セグメント済みデータ (RecognizedSegment.msg)
 */

#include "robot_vision_segment.h"

// include publish message type
#include <T2_robot_vision/RecognizedSegment.h>
#include <T2_robot_vision/RecognizedItem.h>

#include <iostream>
#include <fstream>

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_segment, nodelet::Nodelet);

namespace T2_robot_vision
{

robot_vision_segment::robot_vision_segment()
{
}

/*
 * Nodelet Destructor.
 */
robot_vision_segment::~robot_vision_segment()
{
}

/*
 * Initialize the nodelet.
 */
void robot_vision_segment::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();
  
  try{
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string file_name = src_root_path + "/data/area_valid.png";
    areaMat = cv::imread(file_name, 0);
  }
  catch(cv::Exception& e){
    ROS_ERROR_STREAM(__FILE__<< ":" << __LINE__ << ":" << e.what());
  }

  segment_python_sub_ = nh.subscribe("calibrated_data", 10, &robot_vision_segment::seg_python_Callback, this);
  segment_python_pub_ = nh.advertise<T2_robot_vision::CalibratedData>("calibrated_data_sub", 10);
  
  segment_sub_ = nh.subscribe("recognized_segment_sub", 10, &robot_vision_segment::calibratedDataCallback, this);
  segment_pub_ = nh.advertise<T2_robot_vision::RecognizedSegment>("recognized_segment", 10);
  
  segment_unknown_pub_ = nh.advertise<T2_robot_vision::RecognizedItem>("recognized_unknown_segment", 10);
}

static void InputDataCheck(const T2_robot_vision::RecognitionTarget recog_target){
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " start");
  std::string src_root_path = ros::package::getPath("T2_robot_vision");
  std::string debug_path = src_root_path + "/data/debug/InputCheck_";
  for(std::vector<T2_robot_vision::CapturedData>::const_iterator itr = recog_target.data.begin();
      itr != recog_target.data.end();
      itr++){
    std::stringstream ss;
    ss << debug_path << "jobNo_" << recog_target.job_no << "_camID_" << itr->cam_id;
    /**2D_Data -> cv::Mat*/
    cv_bridge::CvImagePtr cv_color_ptr;
    cv_bridge::CvImagePtr cv_color_o_ptr;
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv_bridge::CvImagePtr cv_depth_c_ptr;
    try{
      if(itr->rgb_1.encoding == "rgb8"){
	cv_color_ptr = cv_bridge::toCvCopy(itr->rgb_1, sensor_msgs::image_encodings::RGB8);
      }else{
	ROS_ERROR_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id << " not compatible color encodings:" << itr->rgb_1.encoding);
	break;
      }
      if(itr->rgb.encoding == "rgb8"){
	cv_color_o_ptr = cv_bridge::toCvCopy(itr->rgb, sensor_msgs::image_encodings::RGB8);
      }else{
	ROS_ERROR_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id << " not compatible color encodings:" << itr->rgb.encoding);
	break;
      }
      if(itr->depth.encoding == "mono16"){
	cv_depth_ptr = cv_bridge::toCvCopy(itr->depth, sensor_msgs::image_encodings::MONO16);
      }else if(itr->depth.encoding == "16UC1"){
        sensor_msgs::Image depth;
        depth.header = itr->depth.header;
        depth.height = itr->depth.height;
        depth.width = itr->depth.width;
        depth.is_bigendian = itr->depth.is_bigendian;
        depth.step = itr->depth.step;
        depth.data = itr->depth.data;
        depth.encoding = "mono16";
        cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
      }else{
	ROS_ERROR_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id << " not compatible depth encodings:" << itr->depth.encoding);
	break;
      }
      if(itr->depth_1.encoding == "mono16"){
	cv_depth_c_ptr = cv_bridge::toCvCopy(itr->depth_1, sensor_msgs::image_encodings::MONO16);
      }else if(itr->depth_1.encoding == "16UC1"){
        sensor_msgs::Image depth;
        depth.header = itr->depth_1.header;
        depth.height = itr->depth_1.height;
        depth.width = itr->depth_1.width;
        depth.is_bigendian = itr->depth_1.is_bigendian;
        depth.step = itr->depth_1.step;
        depth.data = itr->depth_1.data;
        depth.encoding = "mono16";
        cv_depth_c_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
      }else{
	ROS_ERROR_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id << " not compatible depth encodings:" << itr->depth.encoding);
	break;
      }
    }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("T2_robot_vision:segment cam_id:%d, cv_bridge exception:%s", itr->cam_id,e.what());
      break;
    }

	cv::cvtColor(cv_color_o_ptr->image,cv_color_o_ptr->image,CV_RGB2BGR);
	cv::cvtColor(cv_color_ptr->image,cv_color_ptr->image,CV_RGB2BGR);

    cv::imwrite(ss.str()+"_color.bmp", cv_color_o_ptr->image);
    cv::imwrite(ss.str()+"_color_1.bmp", cv_color_ptr->image);
    cv::imwrite(ss.str()+"_depth.png", cv_depth_ptr->image);
    cv::imwrite(ss.str()+"_depth_1.png", cv_depth_c_ptr->image);
    /**PointCloud -> cv::Mat*/
    const sensor_msgs::PointCloud2 pc2 = itr->points;
    uint32_t h = pc2.height;
    uint32_t w = pc2.width;
    uint32_t row_step = pc2.row_step;
    ROS_INFO_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id
			 << " PointCloud Size Error" << std::endl
			 << " pc2.w:" << pc2.width << " pc2.h:" << pc2.height
			 << " depth_image w:" << w << " depth_image h" << h << std::endl
             << "row_step:" << pc2.row_step << " point_step:" << pc2.point_step << std::endl);
          // << "fileds0 size:" << pc2.fields[0].name << " fileds3 size:" << pc2.fields[3].name); // 落ちるのでコメントアウト
    
    if(w*h == 0){
      ROS_ERROR("T2_robot_vision:segment cam_id:%d, PointCloud size is 0", itr->cam_id);
      break;
    }else if(h == 1){
      h = cv_depth_ptr->image.rows;
      w = cv_depth_ptr->image.cols;
      if(w*h != pc2.width){
	ROS_ERROR_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id <<
			 " PointCloud Size Error" << std::endl
			 << " pc2.w:" << pc2.width << " pc2.h:" << pc2.height
			 << " depth_image w:" << w << " depth_image h" << h << std::endl);
	break;
      }
      row_step = w*pc2.point_step;
    }
    cv::Mat pMat(h, w, CV_32FC3);
    for(size_t v = 0; v < h; v++){
      for(size_t u = 0; u < w; u++){
	for(uint32_t channel = 0; channel < 3; channel++){
	  memcpy(&pMat.at<cv::Vec3f>(v, u)[channel],
		 &pc2.data[u*pc2.point_step + v*row_step + pc2.fields[channel].offset], sizeof(float));
	} //channel
      } //w
    } //h
    std::string pcd_name = ss.str()+"_pointcloud2.pcd";
    std::ofstream pcd_file(pcd_name.c_str(), std::ios::out);
    pcd_file << "#.PCD v.7 - Point Cloud Data file format" << std::endl;
    pcd_file << "VERSION .7" << std::endl;
    pcd_file << "FIELDS x y z" << std::endl;
    pcd_file << "SIZE 4 4 4" << std::endl;
    pcd_file << "TYPE F F F" << std::endl;
    pcd_file << "COUNT 1 1 1" << std::endl;
    pcd_file << "WIDTH " << pMat.cols << std::endl;
    pcd_file << "HEIGHT " << pMat.rows << std::endl;
    pcd_file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    pcd_file << "POINTS " << pMat.cols*pMat.rows << std::endl;
    pcd_file << "DATA ascii" << std::endl;
    for(unsigned int j=0; j<pMat.rows; j++){
      for(unsigned int i=0; i<pMat.cols; i++){
	pcd_file << pMat.at<cv::Vec3f>(j,i)[0] << " " << pMat.at<cv::Vec3f>(j,i)[1] << " " << pMat.at<cv::Vec3f>(j,i)[2] << std::endl;
      }
    }
    pcd_file.close();
  }
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " end");
}

static
int PosInverseMatrix(cv::Vec3f vec_O, cv::Vec3f vec_A,
		     cv::Vec3f vec_B, cv::Vec3f vec_C,
		     cv::Mat& invMat){
  cv::Mat posMat(3,3,CV_64FC1);
  posMat.at<double>(0,0) = vec_A[0] - vec_O[0];
  posMat.at<double>(1,0) = vec_A[1] - vec_O[1];
  posMat.at<double>(2,0) = vec_A[2] - vec_O[2];
  posMat.at<double>(0,1) = vec_B[0] - vec_O[0];
  posMat.at<double>(1,1) = vec_B[1] - vec_O[1];
  posMat.at<double>(2,1) = vec_B[2] - vec_O[2];
  posMat.at<double>(0,2) = vec_C[0] - vec_O[0];
  posMat.at<double>(1,2) = vec_C[1] - vec_O[1];
  posMat.at<double>(2,2) = vec_C[2] - vec_O[2];
  invMat = cv::Mat::zeros(3,3,CV_64FC1);
  int status = 0;
  status = cv::invert(posMat, invMat, 0);
  if(status == 0){
    return -1;
  }

  return 0;
}

static
void DetectForegroundSub(cv::Mat pMat, cv::Mat invMat, cv::Vec3f vec_O,
		      cv::Mat& flgMat){
  int h = pMat.rows;
  int w = pMat.cols;
  flgMat = cv::Mat::ones(h,w,CV_8UC1) * 1;

  for(int j = 0; j < h; j++){
    for(int i = 0; i < w; i++){
      if(pMat.at<cv::Vec3f>(j,i) == cv::Vec3f(0,0,0)){
	flgMat.at<unsigned char>(j,i) = 0;
	continue;
      }
      cv::Mat vecMat = cv::Mat(3,1,CV_64FC1);
      for(int k = 0; k< 3; k++){
	vecMat.at<double>(k,0) = pMat.at<cv::Vec3f>(j,i)[k] - vec_O[k];
      }
      bool outer_flg = false;
      cv::Mat coeffMat = invMat * vecMat;
      for(int k = 0; k < 3; k++){
	if(coeffMat.at<double>(k,0) < 0
	   || coeffMat.at<double>(k,0) > 1){
	  outer_flg = true;
	  break;
	}
      }
      if(outer_flg){
	flgMat.at<unsigned char>(j,i) = 0;
      }
    }
  }
  
}

int robot_vision_segment::DetectForegroundPoint(int cam_id, cv::Mat pMat, cv::Mat& flgMat){

  int status = 0;
  cv::Vec3f vec_O = cv::Vec3f(0,0,0);
  cv::Vec3f vec_A = cv::Vec3f(0,0,0);
  cv::Vec3f vec_B = cv::Vec3f(0,0,0);
  cv::Vec3f vec_C = cv::Vec3f(0,0,0);
  cv::Mat invMat;
  status = PosInverseMatrix(vec_O, vec_A, vec_B, vec_C, invMat);

  if(status < 0){
    int h = pMat.rows;
    int w = pMat.cols;
    flgMat = cv::Mat::ones(h,w,CV_8UC1) * 1;
    return 1;
  }

  DetectForegroundSub(pMat, invMat, vec_O, flgMat);
  return 0;
}

void robot_vision_segment::seg_python_Callback(const T2_robot_vision::CalibratedDataConstPtr &notice)
{
  ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:segment segment python_dummy nodelet start");
  
  const T2_robot_vision::RecognitionTarget recog_target = notice->recog_target;
  log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  //if(logptr->getLevel() == log4cxx::Level::getDebug()){
    InputDataCheck(recog_target);
  //}//DEBUG_MODE_ONLY
  //OutPut

  segment_python_pub_.publish(notice);
  ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:segment segment python_dummy nodelet end");
  
}

void robot_vision_segment::calibratedDataCallback(const T2_robot_vision::RecognizedSegmentConstPtr &notice)
{
  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:segment nodelet start");

  /**Capture Data Array -> cv::Mat array*/
  std::vector<uint32_t> cam_id_v;
  std::vector<cv::Mat> cMat_v;
  std::vector<cv::Mat> dMat_v;
  std::vector<cv::Mat> pMat_v;

  const T2_robot_vision::RecognitionTarget recog_target = notice->recog_target;
  
  for(std::vector<T2_robot_vision::CapturedData>::const_iterator itr = recog_target.data.begin();
       itr != recog_target.data.end();
       itr++){
    cam_id_v.push_back(itr->cam_id);
    /**2D_Data -> cv::Mat*/
    cv_bridge::CvImagePtr cv_color_ptr;
    cv_bridge::CvImagePtr cv_depth_ptr;
    try{
      if(itr->rgb_1.encoding == "rgb8"){
	cv_color_ptr = cv_bridge::toCvCopy(itr->rgb_1, sensor_msgs::image_encodings::RGB8);
      }else{
	ROS_ERROR_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id << " not compatible color encodings:" << itr->rgb_1.encoding);
	break;
      }
      if(itr->depth.encoding == "mono16"){
	 cv_depth_ptr = cv_bridge::toCvCopy(itr->depth, sensor_msgs::image_encodings::MONO16);
      }else if(itr->depth.encoding == "16UC1"){
        sensor_msgs::Image depth;
        depth.header = itr->depth.header;
        depth.height = itr->depth.height;
        depth.width = itr->depth.width;
        depth.is_bigendian = itr->depth.is_bigendian;
        depth.step = itr->depth.step;
        depth.data = itr->depth.data;
        depth.encoding = "mono16";
        cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
      }else{
	ROS_ERROR_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id << " not compatible depth encodings:" << itr->depth.encoding);
	break;
      }
    }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("T2_robot_vision:segment cam_id:%d, cv_bridge exception:%s", itr->cam_id,e.what());
      break;
    }



    cMat_v.push_back(cv_color_ptr->image);
    dMat_v.push_back(cv_depth_ptr->image);

          for(int i=0; i<cMat_v.size(); i++)
        {
            cvtColor(cMat_v[i], cMat_v[i], CV_RGB2BGR);
        }
  
    /**PointCloud -> cv::Mat*/
    const sensor_msgs::PointCloud2 pc2 = itr->points;
    uint32_t h = pc2.height;
    uint32_t w = pc2.width;
    uint32_t row_step = pc2.row_step;

    if(w*h == 0){
      ROS_ERROR("T2_robot_vision:segment cam_id:%d, PointCloud size is 0", itr->cam_id);
      break;
    }else if(h == 1){
      h = cv_depth_ptr->image.rows;
      w = cv_depth_ptr->image.cols;
      if(w*h != pc2.width){
	ROS_ERROR_STREAM("T2_robot_vision:segment cam_id:" << itr->cam_id <<
			 " PointCloud Size Error" << std::endl
			 << " pc2.w:" << pc2.width << " pc2.h:" << pc2.height
			 << " depth_image w:" << w << " depth_image h" << h << std::endl);
	break;
      }
      row_step = w*pc2.point_step;
    }
    cv::Mat pMat(h, w, CV_32FC3);
    for(size_t v = 0; v < h; v++){
      for(size_t u = 0; u < w; u++){
	for(uint32_t channel = 0; channel < 3; channel++){
	  memcpy(&pMat.at<cv::Vec3f>(v, u)[channel],
		 &pc2.data[u*pc2.point_step + v*row_step + pc2.fields[channel].offset], sizeof(float));
	} //channel
      } //w
    } //h
    pMat_v.push_back(pMat);
  } //captured data
  
  /**Segment algorithm*/
  std::vector<uint32_t> seg_sx;
  std::vector<uint32_t> seg_ex;
  std::vector<uint32_t> seg_sy;
  std::vector<uint32_t> seg_ey;
  std::vector<uint32_t> seg_imgidx;
  std::vector<cv::Mat> mMat_v;
  std::vector<cv::Mat> fMat_v;

  //OutPut
  T2_robot_vision::RecognizedSegmentPtr data(new T2_robot_vision::RecognizedSegment);
  
  data->recog_target = notice->recog_target;
  data->calibrated_points = notice->calibrated_points;

  //foreground segment
  SegmentedData foreground_seg_data;
  {
    cv::Mat foregroundMat;
    DetectForegroundPoint(cam_id_v[0], pMat_v[0], foregroundMat);
    uint32_t h = data->recog_target.data[0].points.height;
    uint32_t w = data->recog_target.data[0].points.width;
    uint32_t point_step = data->recog_target.data[0].points.point_step;
    uint32_t row_step = data->recog_target.data[0].points.row_step;
    uint32_t offset[3];
    offset[0] = data->recog_target.data[0].points.fields[0].offset;
    offset[1] = data->recog_target.data[0].points.fields[1].offset;
    offset[2] = data->recog_target.data[0].points.fields[2].offset;
    int fw = foregroundMat.cols;
    for(size_t v = 0; v < h; v++){
      for(size_t u = 0; u < w; u++){
	if(foregroundMat.at<unsigned char>(v,u%fw)!=0){
	  continue;
	}
	for(uint32_t channel = 0; channel < 3; channel++){
	  memset(&data->recog_target.data[0].points.data[u*point_step + v*row_step + offset[channel]], 0, sizeof(float));
	} //channel
      } //w
    } //h

#if 1
    {//save foreground point cloud
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string debug_path = src_root_path + "/data/debug/InputCheck_";
    std::stringstream ss;
    ss << debug_path << "jobNo_" << notice->recog_target.job_no << "_camID_" << cam_id_v[0];
    const sensor_msgs::PointCloud2 pc2 = data->recog_target.data[0].points;
    uint32_t h = pc2.height;
    uint32_t w = pc2.width;
    uint32_t row_step = pc2.row_step;
    cv::Mat pMat(h, w, CV_32FC3);
    for(size_t v = 0; v < h; v++){
      for(size_t u = 0; u < w; u++){
	for(uint32_t channel = 0; channel < 3; channel++){
	  memcpy(&pMat.at<cv::Vec3f>(v, u)[channel],
		 &pc2.data[u*pc2.point_step + v*row_step + pc2.fields[channel].offset], sizeof(float));
	} //channel
      } //w
    } //h
    std::string pcd_name = ss.str()+"_pointcloud2_foreground.pcd";
    std::ofstream pcd_file(pcd_name.c_str(), std::ios::out);
    pcd_file << "#.PCD v.7 - Point Cloud Data file format" << std::endl;
    pcd_file << "VERSION .7" << std::endl;
    pcd_file << "FIELDS x y z" << std::endl;
    pcd_file << "SIZE 4 4 4" << std::endl;
    pcd_file << "TYPE F F F" << std::endl;
    pcd_file << "COUNT 1 1 1" << std::endl;
    pcd_file << "WIDTH " << pMat.cols << std::endl;
    pcd_file << "HEIGHT " << pMat.rows << std::endl;
    pcd_file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    pcd_file << "POINTS " << pMat.cols*pMat.rows << std::endl;
    pcd_file << "DATA ascii" << std::endl;
    for(unsigned int j=0; j<pMat.rows; j++){
      for(unsigned int i=0; i<pMat.cols; i++){
	pcd_file << pMat.at<cv::Vec3f>(j,i)[0] << " " << pMat.at<cv::Vec3f>(j,i)[1] << " " << pMat.at<cv::Vec3f>(j,i)[2] << std::endl;
      }
    }
    pcd_file.close();
    }//save foreground point cloud end
#endif    

    sensor_msgs::ImagePtr ros_image_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", foregroundMat).toImageMsg();
    foreground_seg_data.mask = *ros_image_ptr;
    foreground_seg_data.sx = 0;
    foreground_seg_data.ex = pMat_v[0].cols-1;
    foreground_seg_data.sy = 0;
    foreground_seg_data.ey = pMat_v[0].rows-1;
    foreground_seg_data.img_idx = 0;
    foreground_seg_data.module_index = 1;
    foreground_seg_data.img_type = 0;
    data->segmented_data.push_back(foreground_seg_data);
  }

  for(int i = 0; i <  notice->segmented_data.size(); i++){
    SegmentedData seg_data;
    //sensor_msgs::ImagePtr ros_image_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", mMat_v[i]).toImageMsg();
    //seg_data.mask = *ros_image_ptr;
    seg_data.sx = notice->segmented_data[i].sx;
    seg_data.ex = notice->segmented_data[i].ex;
    seg_data.sy = notice->segmented_data[i].sy;
    seg_data.ey = notice->segmented_data[i].ey;
    seg_data.img_idx = 0;
    seg_data.module_index = 1;
    seg_data.img_type = 0;
    ROS_INFO_STREAM("job_no:" << jn << 
		    " T2_robot_vision:segment SegmentedResult " << i <<
		    " sx:" << seg_data.sx <<
		    " ex:" << seg_data.ex <<
		    " sy:" << seg_data.sy <<
		    " ey:" << seg_data.ey <<
		    " idx:" << seg_data.img_idx
		    );
    data->segmented_data.push_back(seg_data);
  }

  segment_pub_.publish(data);

  
  //OutPut
  T2_robot_vision::RecognizedItemPtr r_data(new T2_robot_vision::RecognizedItem);
  r_data->recog_target = notice->recog_target;
  for(size_t n = 0; n < notice->calibrated_points.size(); n++){
    r_data->calibrated_points.push_back(notice->calibrated_points[n]);
  }
  //r_data->calibrated_points = notice->calibrated_points;
  r_data->rcg_module_index = 9;
  r_data->segmented_data.push_back(foreground_seg_data);
  for(unsigned int i = 0; i <  notice->segmented_data.size(); i++){
    SegmentedData seg_data;
    seg_data.sx = notice->segmented_data[i].sx;
    seg_data.ex = notice->segmented_data[i].ex;
    seg_data.sy = notice->segmented_data[i].sy;
    seg_data.ey = notice->segmented_data[i].ey;
    seg_data.img_idx = 0;
    seg_data.module_index = 1;
    seg_data.img_type = 0;
    r_data->segmented_data.push_back(seg_data);

    ItemizedData item;
    item.category = 0;
    item.seg_id = i+1;
    item.score = 0;
    item.module_index = 9;
    
    item.posx = 0;
    item.posy = 0;
    item.posz = 0;
    item.pitch = 0;
    item.yaw = 0;
    item.roll = 0;
    r_data->itemized_data.push_back(item);
  }
  segment_unknown_pub_.publish(r_data);

  
  ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:segment segment nodelet end");
  return;
}
} //T2_robot_vision
