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

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <unistd.h>
#include <math.h>

#include "seg_common.hpp"
#include "cluster.hpp"
#include "normal_vector.hpp"

#include "robot_vision_segment.h"
#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#else
#include <time.h>
#include "seg_single.hpp"
#endif

#ifdef LIB_CLUSTER
#include "cluster_sub.hpp"
#endif //LIB_CLUSTER

std::ostream& operator<<(std::ostream& os, SegPrm const &seg_prm){
  return os << "seg_mode:" << seg_prm.segment_mode << std::endl
	    << "depth_min:" << seg_prm.depth_min << std::endl
	    << "depth_max:" << seg_prm.depth_max << std::endl
	    << "sub_window_sx[0]:" << seg_prm.sub_window_sx[0] << std::endl
	    << "sub_window_sy[0]:" << seg_prm.sub_window_sy[0] << std::endl
	    << "sub_window_w[0]:" << seg_prm.sub_window_w[0] << std::endl
	    << "sub_window_h[0]:" << seg_prm.sub_window_h[0] << std::endl
	    << "norm_min_interval:" << seg_prm.norm_min_interval << std::endl
	    << "norm_max_interval:" << seg_prm.norm_max_interval << std::endl
	    << "clst_init:" << seg_prm.clst_init << std::endl
	    << "merge_len_th:" << seg_prm.merge_len_th << std::endl
	    << "merge_nv_cos[0]:" << seg_prm.merge_nv_cos[0] << std::endl
	    << "merge_nv_cos[1]:" << seg_prm.merge_nv_cos[1] << std::endl
	    << "merge_plane_sin[0]:" << seg_prm.merge_plane_sin[0] << std::endl
	    << "merge_plane_sin[1]:" << seg_prm.merge_plane_sin[1] << std::endl
            << "color_dist:" << seg_prm.color_dist << std::endl
            << "color_nvdiff:" << seg_prm.color_nvdiff << std::endl
            << "plane_dist_th:" << seg_prm.plane_dist_th << std::endl
            << "plane_area_th:" << seg_prm.plane_area_th << std::endl
	    << "seg_max:" << seg_prm.seg_max << std::endl
	    << "area_min_th:" << seg_prm.area_min_th << std::endl
	    << "area_max_th:" << seg_prm.area_max_th << std::endl
	    << "recycle_len_th:" << seg_prm.recycle_len_th << std::endl
	    << "recycle_area_th:" << seg_prm.recycle_area_th << std::endl
	    << "rand_seed:" << seg_prm.rand_seed << std::endl
    ;
}

static
void SegPrmFileRead(std::string file_path,
		    SegPrm& seg_prm){
  
  std::ifstream f_st;
  char *endptr = NULL;

  f_st.open(file_path.c_str(), std::ios::in);
  if(!f_st){
    seg_prm.segment_mode = -1;
    return;
  }
  while(!f_st.eof()){
    std::string line_buf;
    std::getline(f_st, line_buf);
    const char comment_mark = '#';
    const char delim_mark = ':';
    if(line_buf.find(comment_mark) == 0){
      continue;
    }
    std::string::size_type p = line_buf.find(delim_mark);
    if(p==0 || p==std::string::basic_string::npos){
      continue;
    }
    std::string title = line_buf.substr(0, p);
    std::string t_value = line_buf.substr(p+1, std::string::basic_string::npos);

    if(title == "segment_mode"){
      seg_prm.segment_mode = atoi(t_value.c_str());
      continue;
    }
    if(title == "depth_min"){
      seg_prm.depth_min = atoi(t_value.c_str());
      continue;
    }
    if(title == "depth_max"){
      seg_prm.depth_max = atoi(t_value.c_str());
      continue;
    }
    if(title == "sub_window_sx"){
      std::string::size_type q = t_value.find(delim_mark);
      std::string cam_id = t_value.substr(0, q);
      std::string c_value = t_value.substr(q+1, std::string::basic_string::npos);
      int n = atoi(cam_id.c_str());
      if(n<0 || n>=MaxCameraNum){continue;}
      seg_prm.sub_window_sx[n] = atoi(c_value.c_str());
      continue;
    }
    if(title == "sub_window_sy"){
      std::string::size_type q = t_value.find(delim_mark);
      std::string cam_id = t_value.substr(0, q);
      std::string c_value = t_value.substr(q+1, std::string::basic_string::npos);
      int n = atoi(cam_id.c_str());
      if(n<0 || n>=MaxCameraNum){continue;}
      seg_prm.sub_window_sy[n] = atoi(c_value.c_str());
      continue;
    }
    if(title == "sub_window_w"){
      std::string::size_type q = t_value.find(delim_mark);
      std::string cam_id = t_value.substr(0, q);
      std::string c_value = t_value.substr(q+1, std::string::basic_string::npos);
      int n = atoi(cam_id.c_str());
      if(n<0 || n>=MaxCameraNum){continue;}
      seg_prm.sub_window_w[n] = atoi(c_value.c_str());
      continue;
    }
    if(title == "sub_window_h"){
      std::string::size_type q = t_value.find(delim_mark);
      std::string cam_id = t_value.substr(0, q);
      std::string c_value = t_value.substr(q+1, std::string::basic_string::npos);
      int n = atoi(cam_id.c_str());
      if(n<0 || n>=MaxCameraNum){continue;}
      seg_prm.sub_window_h[n] = atoi(c_value.c_str());
      continue;
    }
    if(title == "norm_min_interval"){
      seg_prm.norm_min_interval = atoi(t_value.c_str());
      continue;
    }
    if(title == "norm_max_interval"){
      seg_prm.norm_max_interval = atoi(t_value.c_str());
      continue;
    }
    if(title == "clst_init"){
      seg_prm.clst_init = atoi(t_value.c_str());
      continue;
    }
    if(title == "merge_len_th"){
      seg_prm.merge_len_th = atof(t_value.c_str());
      continue;
    }
    if(title == "merge_nv_cos_0"){
      seg_prm.merge_nv_cos[0] = atoi(t_value.c_str());
      continue;
    }
    if(title == "merge_nv_cos_1"){
      seg_prm.merge_nv_cos[1] = atoi(t_value.c_str());
      continue;
    }
    if(title == "merge_plane_sin_0"){
      seg_prm.merge_plane_sin[0] = atoi(t_value.c_str());
      continue;
    }
    if(title == "merge_plane_sin_1"){
      seg_prm.merge_plane_sin[1] = atoi(t_value.c_str());
      continue;
    }
    if(title == "color_dist"){
      seg_prm.color_dist = atoi(t_value.c_str());
      continue;
    }
    if(title == "plane_dist_th"){
      seg_prm.plane_dist_th = atof(t_value.c_str());
      continue;
    }
    if(title == "plane_area_th"){
      seg_prm.plane_area_th = strtol(t_value.c_str(), &endptr, 10);
      continue;
    }
    if(title == "color_nvdiff"){
      seg_prm.color_nvdiff = atof(t_value.c_str());
      continue;
    }
    if(title == "seg_max"){
      seg_prm.seg_max = atoi(t_value.c_str());
      continue;
    }
    if(title == "area_min_th"){
      seg_prm.area_min_th = strtol(t_value.c_str(), &endptr, 10);
      continue;
    }
    if(title == "area_max_th"){
      seg_prm.area_max_th = strtol(t_value.c_str(), &endptr, 10);
      continue;
    }
    if(title == "recycle_len_th"){
      seg_prm.recycle_len_th = atoi(t_value.c_str());
      continue;
    }
    if(title == "recycle_area_th"){
      seg_prm.recycle_area_th = atoi(t_value.c_str());
      continue;
    }
    if(title == "rand_seed"){
      seg_prm.rand_seed = atoi(t_value.c_str());
      continue;
    }
  }
  
  if(!seg_prm.Checker()){
    seg_prm.segment_mode = -1;
  }
}

#ifndef LIB_CLUSTER
static
void ROIMAT(cv::Mat& pCloud, cv::Mat& iColor, cv::Mat& iC3,
	    int sx, int sy, int window_w, int window_h){
  int w = pCloud.cols;
  int h = pCloud.rows;
  if(sx + window_w > w
     || sy + window_h > h){
    return;
  }
  cv::Mat pM = cv::Mat(window_h, window_w, CV_32FC3);
  cv::Mat iM = cv::Mat::ones(window_h, window_w, CV_8UC4);
  iC3 = cv::Mat(window_h, window_w, CV_8UC3);
  
  for(int j=0; j<window_h; j++){
    for(int i=0; i<window_w; i++){
      for(int k=0; k<3; k++){
	pM.at<cv::Vec3f>(j,i)[k] = pCloud.at<cv::Vec3f>(j+sy, i+sx)[k];
	iC3.at<cv::Vec3b>(j,i)[k] = iColor.at<cv::Vec4b>(j+sy, i+sx)[k];
      }
      for(int k=0; k<4; k++){
	iM.at<cv::Vec4b>(j,i)[k] = iColor.at<cv::Vec4b>(j+sy, i+sx)[k];
      }
    }
  }
  pCloud = pM;
  iColor = iM;
}


typedef struct LabelData LabelData;
struct LabelData{
  unsigned short int id;
  std::vector<unsigned short int> nbh_v;
  unsigned int area;
  int sx;
  int ex;
  int sy;
  int ey;
  /*
  bool operator<(const LabelData &right) const {
    if(this->area > right.area){
      return true;
    }
    return false;
  }
  */
  double len;
  bool operator<(const LabelData &right) const {
    if(this->len < right.len){
      return true;
    }
    return false;
  }
};

static
void LabelingSubFunc(cv::Mat iMat, cv::Mat &lbMat,
		     int x, int y, unsigned char pix,
		     unsigned short int &min, unsigned short int &max){

  unsigned short int id = lbMat.at<unsigned short int>(y,x);
  if(pix!=iMat.at<unsigned char>(y,x) || id==USHRT_MAX){
    return;
  }
  min = (id<min)?(id):(min);
  max = (id>max)?(id):(max);
}

static
void LabelingFunc(cv::Mat iMat, cv::Mat &labelMat,
		  std::vector<LabelData>& lb_v){
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " start");
  int w = iMat.cols;
  int h = iMat.rows;
  cv::Mat lbMat = cv::Mat::ones(h,w,CV_16UC1)*USHRT_MAX;
  std::vector<unsigned short int> link_id;
  unsigned short int id_counter = 0;
  for(int j=1; j<h-1; j++){
    for(int i=1; i<w-1; i++){
      unsigned char pix = iMat.at<unsigned char>(j,i);
      if(pix == 0){
	continue;
      }
      unsigned short int min = USHRT_MAX;
      unsigned short int max = 0;
      //LabelingSubFunc(iMat, lbMat, i-1, j-1, pix, min, max);
      LabelingSubFunc(iMat, lbMat, i-0, j-1, pix, min, max);
      //LabelingSubFunc(iMat, lbMat, i+1, j-1, pix, min, max);
      LabelingSubFunc(iMat, lbMat, i-1, j-0, pix, min, max);
      
      if(min==USHRT_MAX){
	lbMat.at<unsigned short int>(j,i) = id_counter;
	link_id.push_back(id_counter);
	id_counter++;
	if(id_counter >= USHRT_MAX-1){
	  i=w;
	  j=h;
	}
	continue;
      }
      lbMat.at<unsigned short int>(j,i) = min;
      unsigned short int id_buf = link_id[min];
      while(id_buf!=max && id_buf!=min){
	id_buf = link_id[id_buf];
      }
      if(id_buf==min){
	id_buf = link_id[min];
	link_id[min] = link_id[max];
	link_id[max] = id_buf;
      }
    }//i
  }//j
  
  std::vector<unsigned short int> table_id = link_id;
  for(unsigned short int i=0; i<link_id.size(); i++){
    table_id[i] = USHRT_MAX;
  }
  lb_v.clear();
  id_counter = 0;
  for(unsigned short int i=0; i<link_id.size(); i++){
    if(table_id[i] != USHRT_MAX){
      continue;
    }
    table_id[i] = id_counter;
    unsigned short int id_buf = link_id[i];
    while(id_buf != i){
      table_id[id_buf] = id_counter;
      id_buf = link_id[id_buf];
    }
    LabelData lb;
    lb.id = id_counter;
    lb.nbh_v.push_back(id_counter);
    lb.area = 0;
    lb.sx = w;
    lb.ex = 0;
    lb.sy = h;
    lb.ey = 0;
    lb_v.push_back(lb);
    id_counter++;
  }
  labelMat = cv::Mat::zeros(h,w,CV_16SC1);
  for(int j=1; j<h-1; j++){
    for(int i=1; i<w-1; i++){
      unsigned short int id_buf = lbMat.at<unsigned short int>(j,i);
      if(id_buf == USHRT_MAX){
	continue;
      }
      unsigned short int id = table_id[id_buf];
      if(id == USHRT_MAX){
	continue;
      }
      labelMat.at<unsigned short int>(j,i) = id+1;
      lb_v[id].area += 1;
      lb_v[id].sx = (i<lb_v[id].sx)?(i):(lb_v[id].sx);
      lb_v[id].ex = (i>lb_v[id].ex)?(i):(lb_v[id].ex);
      lb_v[id].sy = (j<lb_v[id].sy)?(j):(lb_v[id].sy);
      lb_v[id].ey = (j>lb_v[id].ey)?(j):(lb_v[id].ey);
    }//i
  }//j
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " end");
}
 
void CreateMask(cv::Mat maskMat,
		unsigned int min_area, unsigned int max_area,
		std::vector<cv::Mat>& mask_v,
		std::vector<uint32_t>& sx_v,
		std::vector<uint32_t>& ex_v,
		std::vector<uint32_t>& sy_v,
		std::vector<uint32_t>& ey_v){
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " start");
  cv::Mat labelMat;
  std::vector<LabelData> lb_v;
  LabelingFunc(maskMat, labelMat, lb_v);
  int w = maskMat.cols;
  int h = maskMat.rows;
  
  log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  if(logptr->getLevel() == log4cxx::Level::getDebug()){
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string debug_path = src_root_path + "/data/debug/segment_";
    cv::Mat I_lb(h, w, CV_8UC3);
    //srand(5);
    unsigned char color[1024*1000][3] = {{0}};
    for(int i=0; i<lb_v.size()+1; i++){
      color[i][0] = rand()%196 + 60;
      color[i][1] = rand()%196 + 60;
      color[i][2] = rand()%196 + 60;
    }
    for(int j=0; j<labelMat.rows; j++){
      for(int i=0; i<labelMat.cols; i++){
	unsigned short int id = labelMat.at<unsigned short int>(j,i);
	if(id == 0){
	  I_lb.at<cv::Vec3b>(j,i) = cv::Vec3b(0, 0, 0);
	  continue;
	}
	if(lb_v[id-1].area < min_area){
	  I_lb.at<cv::Vec3b>(j,i) = cv::Vec3b(128, 128, 128);
	  continue;
	}
	I_lb.at<cv::Vec3b>(j,i) =
	  cv::Vec3b(color[id][0], color[id][1], color[id][2]);
      }
    }
    cv::imwrite(debug_path+"maskLabel.jpg", I_lb);
  }//DEBUG_MODE_ONLY
   
  
  std::vector<LabelData> msk_lb_v;
  unsigned short int merge_counter = 0;
  for(unsigned int i=0; i<lb_v.size(); i++){
    if(lb_v[i].area > max_area || lb_v[i].area < min_area){
      continue;
    }
    msk_lb_v.push_back(lb_v[i]);
    
    for(unsigned int j=i+1; j<lb_v.size(); j++){
      if(lb_v[i].sx > lb_v[j].ex
	 || lb_v[i].ex < lb_v[j].sx
	 || lb_v[i].sy > lb_v[j].ey
	 || lb_v[i].ey < lb_v[j].sy){
	continue;
      }
      if(lb_v[j].area > max_area || lb_v[j].area < min_area){
	continue;
      }
      LabelData lb;
      lb.area = lb_v[i].area+lb_v[j].area;
      if(lb.area > max_area){
	continue;
      }
      lb.id = lb_v.size()+merge_counter;
      lb.nbh_v.push_back(i);
      lb.nbh_v.push_back(j);
      lb.sx = (lb_v[i].sx < lb_v[j].sx)?(lb_v[i].sx):(lb_v[j].sx);
      lb.ex = (lb_v[i].ex > lb_v[j].ex)?(lb_v[i].ex):(lb_v[j].ex);
      lb.sy = (lb_v[i].sy < lb_v[j].sy)?(lb_v[i].sy):(lb_v[j].sy);
      lb.ey = (lb_v[i].ey > lb_v[j].ey)?(lb_v[i].ey):(lb_v[j].ey);
      merge_counter++;
      msk_lb_v.push_back(lb);
    }//merge check
  }//lb_v

  //merge_check_again
  unsigned int merged_num = msk_lb_v.size();
  std::vector<unsigned int> merge_checker(lb_v.size());
  for(unsigned int i=0; i<merged_num; i++){
    if(msk_lb_v[i].nbh_v.size()==1){
      continue;
    }
    for(unsigned int j=i+1; j<merged_num; j++){
      if(msk_lb_v[i].sx > msk_lb_v[j].ex
	 || msk_lb_v[i].ex < msk_lb_v[j].sx
	 || msk_lb_v[i].sy > msk_lb_v[j].ey
	 || msk_lb_v[i].ey < msk_lb_v[j].sy){
	continue;
      }
      bool merged_flg = false;
      for(unsigned int k=0; k<merged_num; k++){
	merge_checker[k] = 0;
      }
      for(unsigned int k=0; k<msk_lb_v[i].nbh_v.size(); k++){
	merge_checker[msk_lb_v[i].nbh_v[k]]++;
      }
      for(unsigned int k=0; k<msk_lb_v[j].nbh_v.size(); k++){
	if(merge_checker[msk_lb_v[j].nbh_v[k]]){
	  merged_flg = true;
	  break;
	}
      }
      if(merged_flg){
	continue;
      }
      
      LabelData lb;
      lb.area = msk_lb_v[i].area+msk_lb_v[j].area;
      if(lb.area > max_area){
	continue;
      }
      lb.id = lb_v.size()+merge_counter;
      lb.nbh_v = msk_lb_v[i].nbh_v;
      lb.nbh_v.insert(lb.nbh_v.end(),
		      msk_lb_v[j].nbh_v.begin(),
		      msk_lb_v[j].nbh_v.end());
      lb.sx = (msk_lb_v[i].sx < msk_lb_v[j].sx)?(msk_lb_v[i].sx):(msk_lb_v[j].sx);
      lb.ex = (msk_lb_v[i].ex > msk_lb_v[j].ex)?(msk_lb_v[i].ex):(msk_lb_v[j].ex);
      lb.sy = (msk_lb_v[i].sy < msk_lb_v[j].sy)?(msk_lb_v[i].sy):(msk_lb_v[j].sy);
      lb.ey = (msk_lb_v[i].ey > msk_lb_v[j].ey)?(msk_lb_v[i].ey):(msk_lb_v[j].ey);
      merge_counter++;
      msk_lb_v.push_back(lb);
    }//merge check
  }//lb_v

  {
    for(unsigned int i=0; i<msk_lb_v.size(); i++){
      int sx = msk_lb_v[i].sx;
      int ex = msk_lb_v[i].ex;
      int sy = msk_lb_v[i].sy;
      int ey = msk_lb_v[i].ey;
      double len = sqrt((sx-w/2)*(sx-w/2)+(ex-w/2)*(ex-w/2)+(sy-h/2)*(sy-h/2)+(ey-h/2)*(ey-h/2));
      msk_lb_v[i].len = len;
    }
  }
  std::sort(msk_lb_v.begin(), msk_lb_v.end());
  for(unsigned int i=0; i<msk_lb_v.size(); i++){
    int sx = msk_lb_v[i].sx;
    int ex = msk_lb_v[i].ex;
    int sy = msk_lb_v[i].sy;
    int ey = msk_lb_v[i].ey;
    int msk_w = ex-sx+1;
    int msk_h = ey-sy+1;
    cv::Mat mMat = cv::Mat::zeros(msk_h,msk_w,CV_8UC1);
    for(int q=0; q<msk_h; q++){
      for(int p=0; p<msk_w; p++){
	unsigned short int id
	  = labelMat.at<unsigned short int>(q+sy,p+sx);
	if(id == 0){
	  mMat.at<unsigned char>(q,p) = 0;
	  continue;
	}
	id--;
	bool flg = false;
	for(unsigned int j=0; j<msk_lb_v[i].nbh_v.size(); j++){
	  if(id == msk_lb_v[i].nbh_v[j]){
	    flg = true;
	    break;
	  }
	}
	if(flg){
	  mMat.at<unsigned char>(q,p) = 1;
	}
      }//p
    }//q
    mask_v.push_back(mMat);
    sx_v.push_back(sx);
    ex_v.push_back(ex);
    sy_v.push_back(sy);
    ey_v.push_back(ey); 
  }//msk_lb
  
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " end");
}
 
static
int PointCludRangeChange(int cam_id, 
                         cv::Mat& pCloud, cv::Mat& iColor, cv::Mat &iC3,
			 cv::Mat fMat,
			 SegPrm seg_prm){
  int w = pCloud.cols;
  int h = pCloud.rows;
  if(w!=iColor.cols || h!=iColor.rows || iColor.depth()!=CV_8U){
    return -1;
  }

  std::vector<cv::Mat> chMat_v;
  cv::split(iColor, chMat_v);
  cv::Mat flgMat = cv::Mat::ones(h, w, CV_8UC1)*255;
  if(iColor.channels()==4){
    flgMat = chMat_v[3];
  }else if(iColor.channels()==3){
    chMat_v.push_back(flgMat);
    cv::merge(chMat_v, iColor);
  }

  float depth_min = 100000;
  float depth_max = -100000;
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      if(flgMat.at<unsigned char>(j,i)==0
	 || fMat.at<unsigned char>(j,i)<128){
	continue;
      }
      if(pCloud.at<cv::Vec3f>(j,i)[2] >= seg_prm.depth_max
	 || pCloud.at<cv::Vec3f>(j,i)[2] <= seg_prm.depth_min){
	continue;
      }
      if(pCloud.at<cv::Vec3f>(j,i)[2] > depth_max){
        depth_max = pCloud.at<cv::Vec3f>(j,i)[2];
      }
      if(pCloud.at<cv::Vec3f>(j,i)[2] < depth_min){
        depth_min = pCloud.at<cv::Vec3f>(j,i)[2];
      }
    }
  }
  
  float depth_margin = ((40<depth_max/10)?40:(depth_max/10));
  if(depth_min < 0){
    depth_margin += fabs(depth_min);
  }
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      if(flgMat.at<unsigned char>(j,i)==0
	 || fMat.at<unsigned char>(j,i)<128){
	pCloud.at<cv::Vec3f>(j,i) = cv::Vec3f(0, 0, 0);
	continue;
      }
      if(pCloud.at<cv::Vec3f>(j,i)[2] >= seg_prm.depth_max
	 || pCloud.at<cv::Vec3f>(j,i)[2] <= seg_prm.depth_min){
	pCloud.at<cv::Vec3f>(j,i) = cv::Vec3f(0, 0, 0);
	continue;
      }
      pCloud.at<cv::Vec3f>(j,i)[2] += depth_margin;
    }
  }

  int window_w = seg_prm.sub_window_w[cam_id];
  int window_h = seg_prm.sub_window_h[cam_id];
  if(window_w <= 0){
    window_w = w-seg_prm.sub_window_sx[cam_id];
  }
  if(window_h <= 0){
    window_h = h-seg_prm.sub_window_sy[cam_id];
  }

  ROIMAT(pCloud, iColor, iC3,
	 seg_prm.sub_window_sx[cam_id], seg_prm.sub_window_sy[cam_id], window_w, window_h);
    
  return 0;
}
#endif //LIB_CLUSTER
 
int T2_robot_vision::robot_vision_segment::SegmentAlgMain(int job_num,
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
							     ){
  
  
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " start");

  //reset
  seg_sx.clear();
  seg_ex.clear();
  seg_sy.clear();
  seg_ey.clear();
  mMat_v.clear();

  if(pMat_v.size()==0 || cMat_v.size()==0){
    return 1;
  }
  
  std::string src_root_path = ros::package::getPath("T2_robot_vision");
  std::string prm_file_path = src_root_path + "/data/seg_prm.txt";
  SegPrmFileRead(prm_file_path, seg_prm);
  ROS_INFO_STREAM("T2_robot_vision:segment_prm" << std::endl << seg_prm);

  if(seg_prm.segment_mode < 0){
    return -1;
  }  
  if(seg_prm.segment_mode == 0){
    ROS_INFO("T2_robot_vision:segment Dummy Output Mode");
    std::string dummy_mask_path = src_root_path + "/data/mask.bmp";
    cv::Mat mask = cv::imread(dummy_mask_path, -1);
    if(!mask.data){
      ROS_ERROR_STREAM("T2_robot_vision:segment dummy mask read ERROR " << dummy_mask_path);
      ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " end");
      return -1;
    }
    std::vector<cv::Mat> mask_v;
    std::vector<uint32_t> sx_v;
    std::vector<uint32_t> ex_v;
    std::vector<uint32_t> sy_v;
    std::vector<uint32_t> ey_v;
    CreateMask(mask,
	       seg_prm.area_min_th, seg_prm.area_max_th,
	       mask_v,sx_v, ex_v, sy_v, ey_v);
    for(unsigned int j=0; j<mask_v.size(); j++){
      seg_sx.push_back(sx_v[j]);
      seg_ex.push_back(ex_v[j]);
      seg_sy.push_back(sy_v[j]);
      seg_ey.push_back(ey_v[j]);
      seg_imgidx.push_back(0);
      mMat_v.push_back(mask_v[j]);
    }
    ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " end");
    return 0;
  }

  for(size_t n=0; n<pMat_v.size(); n++){
    if(pMat_v[n].data==NULL || cMat_v[n].data==NULL
      || cam_id_v[n] < 0 || cam_id_v[n] >= MaxCameraNum){
      continue;
    }
    cv::Mat pCloud = pMat_v[n];
    cv::Mat iC4 = cMat_v[n];
    cv::Mat iC3;
    cv::Mat fMat = cv::Mat::ones(pCloud.rows, pCloud.cols, CV_8UC1)*255;
    if(fMat_v.size()>n){
      if(fMat_v[n].rows == pCloud.rows
	 && fMat_v[n].cols == pCloud.cols){
	fMat = fMat_v[n];
      }
    }
    PointCludRangeChange(cam_id_v[n], pCloud, iC4, iC3, fMat, seg_prm);

    if(seg_prm.segment_mode == 2){
      int h = pCloud.rows;
      int w = pCloud.cols;
      cv::Mat medMat(h, w, CV_32FC3);
      cv::medianBlur(pCloud, medMat, 5);
      pCloud = medMat*1000;
      double alpha = 1.0f;
      double beta = 0.01f;
      double gamma = 0.01f;
      int min_size = seg_prm.norm_min_interval;
      int max_size = seg_prm.norm_max_interval;
      cv::Mat nvMat(h, w, CV_32FC1);
      NormalVectorMap(pCloud, nvMat, alpha, beta, gamma,
	              max_size, min_size);
      std::vector<SegmentMask> seg_mask_v;
      ClustSegment(pCloud, nvMat, iC3,
		 seg_prm, seg_mask_v);
      for(unsigned int i=0; i<seg_mask_v.size(); i++){
	seg_sx.push_back(seg_mask_v[i].sx+seg_prm.sub_window_sx[cam_id_v[n]]);
	seg_ex.push_back(seg_mask_v[i].ex+seg_prm.sub_window_sx[cam_id_v[n]]);
	seg_sy.push_back(seg_mask_v[i].sy+seg_prm.sub_window_sy[cam_id_v[n]]);
	seg_ey.push_back(seg_mask_v[i].ey+seg_prm.sub_window_sy[cam_id_v[n]]);
	mMat_v.push_back(seg_mask_v[i].mask);
	seg_imgidx.push_back(n);
      }
    }//segment mode
    else if(seg_prm.segment_mode == 1){
      int h = pCloud.rows;
      int w = pCloud.cols;
      double alpha = 1.0;
      double beta = 0.01;
      double gamma = 0.01;
      cv::Mat nvMat(h, w, CV_32FC1);
      NormalVectorMap(pCloud, nvMat,  alpha, beta, gamma,seg_prm.norm_max_interval, seg_prm.norm_min_interval);
    
      cv::Mat varMono(h, w, CV_8UC1);
      cv::Mat maskMat(h, w, CV_8UC1);
      VarNvMap(pCloud, nvMat, varMono, 30);
      std::vector<cv::Mat> mask_v;
      std::vector<uint32_t> sx_v;
      std::vector<uint32_t> ex_v;
      std::vector<uint32_t> sy_v;
      std::vector<uint32_t> ey_v;
      T2_robot_vision::robot_vision_segment::GuessPlane(pCloud, nvMat, varMono, maskMat, 30, 160, seg_prm.plane_dist_th, seg_prm.plane_area_th);
      
      CreateMask(maskMat, seg_prm.area_min_th, seg_prm.area_max_th,
                 mask_v,sx_v, ex_v, sy_v, ey_v);
      for(unsigned int i=0; i<mask_v.size(); i++){
	seg_sx.push_back(sx_v[i]+seg_prm.sub_window_sx[cam_id_v[n]]);
	seg_ex.push_back(ex_v[i]+seg_prm.sub_window_sx[cam_id_v[n]]);
	seg_sy.push_back(sy_v[i]+seg_prm.sub_window_sy[cam_id_v[n]]);
	seg_ey.push_back(ey_v[i]+seg_prm.sub_window_sy[cam_id_v[n]]);
	mMat_v.push_back(mask_v[i]);
	seg_imgidx.push_back(n);
      }
    }//segment mode
  }//pMat_v.size
  
  
  return 0;
}

