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
 * 平面
 */

#include "robot_vision_findplane.h"

// include publish message type
#include <T2_robot_vision/RecognizedItem.h>
#include <T2_robot_vision/Conv.h>

#include <iostream>
#include <fstream>

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_findplane, nodelet::Nodelet);

typedef struct PlaneData PlaneData;
struct PlaneData{
  cv::Mat planeMat;
  cv::Vec3d normal;
  int cx;
  int cy;
  unsigned int area;
  double score;
  
  bool operator < (const PlaneData& right) const {
    return score >= right.score;
  }
};

static
double PlaneScore(PlaneData plane_data){
  cv::Vec3d bv(0,cos(M_PI*40/180), sin(M_PI*40/180));
  double score = 0;

  cv::Vec3d nv = plane_data.normal;
  double denom = sqrt(nv[0]*nv[0]+nv[1]*nv[1]+nv[2]*nv[2])*sqrt(bv[0]*bv[0]+bv[1]*bv[1]+bv[2]*bv[2]);
  int w = plane_data.planeMat.cols;
  int h = plane_data.planeMat.rows;

  std::vector< std::vector<cv::Point> > contours;
  cv::Rect max_rect;
  unsigned int max_area = 0;
  cv::findContours(plane_data.planeMat, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  for(size_t i = 0; i < contours.size(); i++){
    cv::Rect rect = cv::boundingRect(contours[i]);
    unsigned int c_area = rect.width * rect.height;
    if(max_area < c_area){
      max_area = c_area;
      max_rect = rect;
    }
  }

  double cos_score = 0;
  if(denom != 0){
    cos_score = fabs((nv[0]*bv[0]+nv[1]*bv[1]+nv[2]*bv[2])/denom);
  }

  double pos_score = 0;
  denom = w/2+h/2;
  if(denom != 0){
    pos_score = (fabs(max_rect.x+max_rect.width/2-w/2)+fabs(max_rect.y+max_rect.height/2-h/2))/denom;
  }

  double area_score = 0;
  denom = w*h;
  if(denom != 0){
    area_score = max_area/denom;
  }
  
  score = cos_score - pos_score + area_score;
  return score;
  
}


static
int PlaneDetect(cv::Mat pCloud,
		std::vector<PlaneData>& plane_data_v,
		double invalid_depth,
		int p_num_threshold,
		unsigned int point_threshold,
		double dist_threshold){
  ROS_INFO_STREAM("robot_vision_findplane::PlaneDetect start");
  int width = pCloud.cols;
  int height = pCloud.rows;
  
  cv::Mat pMat = pCloud.clone();
  plane_data_v.clear();
  
  while(1){
    unsigned int valid_point_num = 0;
    for(int j=0; j<height; j++){
      for(int i=0; i<width; i++){
	if(pMat.at<cv::Vec3f>(j,i)[2] <= invalid_depth){
	  continue;
	}
	valid_point_num++;
      }
    }
    if(valid_point_num < point_threshold){
      if(plane_data_v.size() == 0){
        ROS_INFO_STREAM("robot_vision_findplane:: PlaneDetect end (NoPlane)");
	return -1;
      }
      break;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = valid_point_num;
    cloud->height = 1;
    cloud->points.resize(cloud->width*cloud->height);
    valid_point_num = 0;
    std::vector<cv::Vec2i> link_id;
    for(int j=0; j<height; j++){
      for(int i=0; i<width; i++){
	if(pMat.at<cv::Vec3f>(j,i)[2] <= invalid_depth){
	  continue;
	}
	cloud->points[valid_point_num].x = pMat.at<cv::Vec3f>(j,i)[0];
	cloud->points[valid_point_num].y = pMat.at<cv::Vec3f>(j,i)[1];
	cloud->points[valid_point_num].z = pMat.at<cv::Vec3f>(j,i)[2];
	cv::Vec2i v2i(j,i);
	link_id.push_back(v2i);
	valid_point_num++;
      }
    }
    if(valid_point_num < 100){
       ROS_INFO_STREAM("robot_vision_findplane:: valid points are too small");
       break;
    }
/**/
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_threshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size()<=0){
      break;
    }
    
    PlaneData p_data;
    p_data.planeMat = cv::Mat::zeros(height, width, CV_8UC1);
    unsigned int x_sum = 0;
    unsigned int y_sum = 0;
    valid_point_num = 0;
    for(size_t i=0; i<inliers->indices.size(); i++){
      size_t id = inliers->indices[i];
      int x = link_id[id][1];
      int y = link_id[id][0];
      p_data.planeMat.at<unsigned char>(y,x) = 1;
      x_sum += x;
      y_sum += y;
      valid_point_num++;
      pMat.at<cv::Vec3f>(y,x)[2] = invalid_depth;
    }
    if(valid_point_num == 0){
      break;
    }
    p_data.normal[0] = coefficients->values[0];
    p_data.normal[1] = coefficients->values[1];
    p_data.normal[2] = coefficients->values[2];
    p_data.cx = x_sum/valid_point_num;
    p_data.cy = y_sum/valid_point_num;
    p_data.area = valid_point_num;
    p_data.score = PlaneScore(p_data);
    plane_data_v.push_back(p_data);
    
    if(plane_data_v.size() >= p_num_threshold){
      break;
    }
/**/
  }
  std::sort(plane_data_v.begin(), plane_data_v.end());
  
  ROS_INFO_STREAM("robot_vision_findplane::PlaneDetect end");
  
  return 0;
}


namespace T2_robot_vision
{

robot_vision_findplane::robot_vision_findplane()
{
}

/*
 * Nodelet Destructor.
 */
robot_vision_findplane::~robot_vision_findplane()
{
}

/*
 * Initialize the nodelet.
 */
void robot_vision_findplane::onInit()
{
  ROS_INFO_STREAM("robot_vision_findplane::onInit() start");
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  yolo_findplane_sub_ = nh.subscribe("yolov2_results", 10, &robot_vision_findplane::PlaneCallback, this);
  linemod_findplane_sub_ = nh.subscribe("LineMOD_results", 10, &robot_vision_findplane::PlaneCallback, this);
  akaze_findplane_sub_ = nh.subscribe("AKAZE_results", 10, &robot_vision_findplane::PlaneCallback, this);
  seg_findplane_sub_ = nh.subscribe("recognized_unknown_segment", 10, &robot_vision_findplane::PlaneCallback, this);

  findplane_pub_ = nh.advertise<T2_robot_vision::RecognizedItem>("recognized_item", 10);
}
  
void robot_vision_findplane::PlaneCallback(const T2_robot_vision::RecognizedItemConstPtr &notice)
{
  
  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("job_no:" << jn << ", module_index:" << notice->rcg_module_index << ", T2_robot_vision:findplane nodelet start");
  int module_index = 9;
  if(notice->rcg_module_index == 3){
    module_index = 6;
  }else if(notice->rcg_module_index == 1){
    module_index = 2;
  }else if(notice->rcg_module_index == 7){
    module_index = 8;
  }else if(notice->rcg_module_index == 9){
    module_index = 9;
  }
  
  if(notice->itemized_data.size() == 0){
    T2_robot_vision::RecognizedItemPtr data(new T2_robot_vision::RecognizedItem);
    data->recog_target = notice->recog_target;
    for(size_t n = 0; n < notice->calibrated_points.size(); n++){
      data->calibrated_points.push_back(notice->calibrated_points[n]);
    }
    for(size_t n = 0; n < notice->segmented_data.size(); n++){
      data->segmented_data.push_back(notice->segmented_data[n]);
    } 
    data->rcg_module_index = module_index;
    findplane_pub_.publish(data);
    ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:findplane no itemized data");
    ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:findplane nodelet end");
    return;
  } 

  /**Capture Data Array -> cv::Mat array*/
  std::vector<uint32_t> cam_id_v;
  std::vector<cv::Mat> cMat_v;
  std::vector<cv::Mat> dMat_v;
  std::vector<cv::Mat> pMat_v;

  const T2_robot_vision::RecognitionTarget recog_target = notice->recog_target;
  
  log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  
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
	ROS_ERROR_STREAM("T2_robot_vision:findplane cam_id:" << itr->cam_id << " not compatible color encodings:" << itr->rgb_1.encoding);
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
	ROS_ERROR_STREAM("T2_robot_vision:findplane cam_id:" << itr->cam_id << " not compatible depth encodings:" << itr->depth.encoding);
	break;
      }
    }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("T2_robot_vision:findplane cam_id:%d, cv_bridge exception:%s", itr->cam_id,e.what());
      break;
    }
    cMat_v.push_back(cv_color_ptr->image);
    dMat_v.push_back(cv_depth_ptr->image);
    
    /**PointCloud -> cv::Mat*/
    const sensor_msgs::PointCloud2 pc2 = itr->points;
    uint32_t h = pc2.height;
    uint32_t w = pc2.width;
    uint32_t row_step = pc2.row_step;

    if(w*h == 0){
      ROS_ERROR("T2_robot_vision:findplane cam_id:%d, PointCloud size is 0", itr->cam_id);
      break;
    }else if(h == 1){
      h = cv_depth_ptr->image.rows;
      w = cv_depth_ptr->image.cols;
      if(w*h != pc2.width){
	ROS_ERROR_STREAM("T2_robot_vision:findplane cam_id:" << itr->cam_id <<
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
  
  /**Find Plane*/
  std::vector<uint32_t> seg_sx;
  std::vector<uint32_t> seg_ex;
  std::vector<uint32_t> seg_sy;
  std::vector<uint32_t> seg_ey;
  std::vector<uint32_t> seg_imgidx;
  std::vector<cv::Mat> mMat_v;

  for(size_t n = 0; n < notice->segmented_data.size(); n++){
    uint32_t sx = notice->segmented_data[n].sx;
    uint32_t ex = notice->segmented_data[n].ex;
    uint32_t sy = notice->segmented_data[n].sy;
    uint32_t ey = notice->segmented_data[n].ey;
    uint32_t imgidx = notice->segmented_data[n].img_idx;
    if(notice->segmented_data[n].img_type == 1){
      ros::NodeHandle &nh = getNodeHandle();
      ros::ServiceClient client_conv = nh.serviceClient <T2_robot_vision::Conv>("conv_srv");
      T2_robot_vision::ConvRequest req;
      T2_robot_vision::ConvResponse res;
      req.job_no = notice->recog_target.job_no;;
      req.cam_id = cam_id_v[imgidx];
      req.x0=sx;
      req.y0=sy;
      req.flg=0; // 0:color->depth
      ROS_INFO_STREAM("req: camid:" << req.cam_id << ", sx:" << req.x0 << ", sy:" << req.y0 );
      if( client_conv.call(req, res) ){
	int w = pMat_v[imgidx].cols;
	int h = pMat_v[imgidx].rows;
	if( res.x1 < 0 ){ res.x1=0; }
	if( res.x1 >=w ){ res.x1=w-1; }
	if( res.y1 < 0 ){ res.y1=0; }
	if( res.y1 >=h ){ res.y1=h-1; }
	ROS_INFO_STREAM("res: sx:" << res.x1 << ", sy:" << res.y1 );
	seg_sx.push_back(res.x1);
	seg_sy.push_back(res.y1);
	sx = res.x1;
	sy = res.y1;
      } else {
	ROS_INFO("T2_robot_vision:findplane Conv NG");
	break;
      }
      req.job_no = notice->recog_target.job_no;;
      req.cam_id = cam_id_v[imgidx];
      req.x0=ex;
      req.y0=ey;
      req.flg=0; // 0:color->depth
      ROS_INFO_STREAM("req: camid:" << req.cam_id << ", ex:" << req.x0 << ", ey:" << req.y0 );
      if( client_conv.call(req, res) ){
	int w = pMat_v[imgidx].cols;
	int h = pMat_v[imgidx].rows;
	if( res.x1 < 0 ){ res.x1=0; }
	if( res.x1 >=w ){ res.x1=w-1; }
	if( res.y1 < 0 ){ res.y1=0; }
	if( res.y1 >=h ){ res.y1=h-1; }
	ROS_INFO_STREAM("res: ex:" << res.x1 << ", ey:" << res.y1 );
	seg_ex.push_back(res.x1);
	seg_ey.push_back(res.y1);
	ex = res.x1;
	ey = res.y1;
      } else {
	ROS_INFO("T2_robot_vision:findplane Conv NG");
	break;
      }
    }else{
      seg_sx.push_back(sx);
      seg_sy.push_back(sy);
      seg_ex.push_back(ex);
      seg_ey.push_back(ey);
    }
    seg_imgidx.push_back(imgidx);

    int w_sub = ex-sx+1;
    int h_sub = ey-sy+1;
    cv::Mat pCloud_sub = cv::Mat(h_sub, w_sub, CV_32FC3);
    for(uint32_t y=sy; y<=ey; y++){
      for(uint32_t x=sx; x<=ex; x++){
	pCloud_sub.at<cv::Vec3f>(y-sy,x-sx) = pMat_v[imgidx].at<cv::Vec3f>(y,x);
      }
    }
    std::vector<PlaneData> plane_data_v;
    double invalid_depth = 0;
    //double dist_threshold = 0.05;
    double dist_threshold = 0.005;
    PlaneDetect(pCloud_sub, plane_data_v,
		invalid_depth,
		10, 20*20,  dist_threshold);
    if(plane_data_v.size() == 0){
      ROS_INFO_STREAM("findplane::No Plane");
      mMat_v.push_back(cv::Mat::zeros(h_sub, w_sub, CV_8UC1));
      continue;
    }
    mMat_v.push_back(plane_data_v[0].planeMat);
/********************************/
    cv::Mat dpMat(h_sub, w_sub, CV_8UC3);
    for(int j = sy; j <= ey; j++){
      for(int i = sx; i <= ex; i++){
        dpMat.at<cv::Vec3b>(j-sy,i-sx) = cMat_v[imgidx].at<cv::Vec3b>(j,i);
      }
    }
    char file_name_d[255];
    sprintf(file_name_d, "/home/dl-box/sekiya/sample_%d_p.png", (int)n);
    sprintf(file_name_d, "/home/dl-box/sekiya/sample_%d_d.png", (int)n);
/********************************/	
  }

  //OutPut
  T2_robot_vision::RecognizedItemPtr data(new T2_robot_vision::RecognizedItem);
  data->recog_target = notice->recog_target;
  for(size_t n = 0; n < notice->calibrated_points.size(); n++){
    data->calibrated_points.push_back(notice->calibrated_points[n]);
  }

  for(int i = 0; i < seg_imgidx.size(); i++){
    SegmentedData seg_data;
    sensor_msgs::ImagePtr ros_image_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", mMat_v[i]).toImageMsg();
    seg_data.module_index = module_index;
    seg_data.mask = *ros_image_ptr;
    seg_data.sx = seg_sx[i];
    seg_data.ex = seg_ex[i];
    seg_data.sy = seg_sy[i];
    seg_data.ey = seg_ey[i];
    seg_data.img_idx = seg_imgidx[i];
    data->segmented_data.push_back(seg_data);
  }
  for(size_t n = 0; n < notice->itemized_data.size(); n++){
    T2_robot_vision::ItemizedData item;
    item.category = notice->itemized_data[n].category;
    item.score = notice->itemized_data[n].score;
    item.seg_id = notice->itemized_data[n].seg_id;
    
    item.posx = 0;
    item.posy = 0;
    item.posz = 0;
    item.pitch = 0;
    item.yaw = 0;
    item.roll = 0;
    item.module_index = module_index;
    data->itemized_data.push_back(item);
  }
  data->rcg_module_index = module_index;
  findplane_pub_.publish(data);

  ROS_INFO_STREAM("job_no:" << jn << ", module_index:" << notice->rcg_module_index << ", T2_robot_vision:findplane nodelet end");
  return;
}
  
} //T2_robot_vision
