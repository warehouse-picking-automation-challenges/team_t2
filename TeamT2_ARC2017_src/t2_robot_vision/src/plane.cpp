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
#include <memory>
#include <list>
#include <iterator>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <limits.h>
#include <math.h>

#include "robot_vision_segment.h"

#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

typedef struct ElemPos ElemPos;
struct ElemPos{
  unsigned int e;
  int x;
  int y;
  bool operator<(const ElemPos &right) const {
    if(this->e > right.e){
      return true;
    }else if(this->e < right.e){
      return false;
    }else{
      return (this->x+this->y) > (right.x+right.y);
    }
  }
};

typedef struct ElemPlane ElemPlane;
struct ElemPlane{
  unsigned int id;
  int x;
  int y;
  unsigned int area;
  double pos[3];
  double nv[3];
};

static
void CheckPointCloudRange(cv::Mat pCloud,
			  double& min_x, double& max_x,
			  double& min_y, double& max_y,
			  double& min_z, double& max_z,
			  unsigned int& invalid_num){
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " start");
  min_x = DBL_MAX;
  max_x = DBL_MIN;
  min_y = DBL_MAX;
  max_y = DBL_MIN;
  min_z = DBL_MAX;
  max_z = DBL_MIN;
  invalid_num = 0;
  for(int j=0; j<pCloud.rows; j++){
    for(int i=0; i<pCloud.cols; i++){
      double x= pCloud.at<cv::Vec3f>(j,i)[0];
      double y= pCloud.at<cv::Vec3f>(j,i)[1];
      double z= pCloud.at<cv::Vec3f>(j,i)[2];
      if(min_x > x){min_x=x;}
      if(max_x < x){max_x=x;}
      if(min_y > y){min_y=y;}
      if(max_y < y){max_y=y;}
      if(min_z > z){min_z=z;}
      if(max_z < z){max_z=z;}
      if(x==0 && y==0 && z==0){invalid_num++;}
    }
  }
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " end");
}

static
double PointLength(cv::Mat pCloud, int x, int y, int p, int q){
  double a = pCloud.at<cv::Vec3f>(y,x)[0];
  double b = pCloud.at<cv::Vec3f>(y,x)[1];
  double c = pCloud.at<cv::Vec3f>(y,x)[2];
  double d = pCloud.at<cv::Vec3f>(q,p)[0];
  double e = pCloud.at<cv::Vec3f>(q,p)[1];
  double f = pCloud.at<cv::Vec3f>(q,p)[2];
  return sqrt((a-d)*(a-d)+(b-e)*(b-e)+(c-f)*(c-f));
}

void T2_robot_vision::robot_vision_segment::GuessPlane(cv::Mat pCloud, cv::Mat nvMat, cv::Mat varMono,
		cv::Mat& maskMat, 
		int t_size,
		int var_th, double len_th, int area_th){
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " start");
  
  int w = nvMat.cols;
  int h = nvMat.rows;
  if(w < t_size*2+1 || h < t_size*2+1 || t_size < 0){
    maskMat = cv::Mat::zeros(h,w,CV_8UC1);
    return;
  }
  unsigned int valid_area = w*h;
  {
    double min_x = DBL_MAX;
    double max_x = DBL_MIN;
    double min_y = DBL_MAX;
    double max_y = DBL_MIN;
    double min_z = DBL_MAX;
    double max_z = DBL_MIN;
    unsigned int invalid_num = 0;
    CheckPointCloudRange(pCloud, min_x, max_x,
			 min_y, max_y, min_z, max_z,
			 invalid_num);
    double max_range = max_x-min_x;
    if(max_range < max_y-min_y){max_range = max_y-min_y;}
    if(max_range < max_z-min_z){max_range = max_z-min_z;}
    double len_th_buf = max_range/(20);
    ROS_INFO_STREAM(min_x << "," << max_x << ":" << min_y << "," << max_y << ":" << min_z << "," << max_z);
    if(len_th_buf > len_th*20){
      ROS_WARN("T2_robot_vision:segment plane_dist_th changed %f -> %f",
	       len_th, len_th_buf);
      len_th = len_th_buf;
    }else if(len_th > len_th_buf*10){
      ROS_WARN("T2_robot_vision:segment plane_dist_th changed %f -> %f",
	       len_th, len_th_buf);
      len_th = len_th_buf;
    }
    valid_area -= invalid_num;
  }
  
  std::vector<ElemPos> base_line;
  base_line.clear();
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      ElemPos elem;
      elem.e = varMono.at<unsigned char>(j,i);
      if(elem.e == 0){
	continue;
      }
      elem.x = i;
      elem.y = j;
      base_line.push_back(elem);
    }
  }
  
  std::sort(base_line.begin(), base_line.end());
  std::vector<ElemPlane> plane_v;
  {
    ElemPlane ep;
    ep.id = 0;
    ep.x = 0;
    ep.y = 0;
    ep.area = 0;
    for(int k=0; k<3; k++){
      ep.pos[k] = 0;
      ep.nv[k] = 0;
    }
    plane_v.push_back(ep);
  }
  unsigned short int id_counter = 1;
  unsigned long int plane_area = 0;
  cv::Mat idMat = cv::Mat::zeros(h,w,CV_16UC1);
  for(int i=0; i<base_line.size(); i++){
    int x = base_line[i].x;
    int y = base_line[i].y;
    if(idMat.at<unsigned short int>(y,x) != 0){
      continue;
    }
    double pos[3] = {0};
    double nv[3] = {0};
    unsigned int area = 0;
    for(int q=y-t_size; q<=y+t_size; q++){
      for(int p=x-t_size; p<=x+t_size; p++){
	if(p<0 || q<0 || p>=w || q>=h){
	  continue;
	}
	if(varMono.at<unsigned char>(q,p) < var_th){
	  continue;
	}
	pos[0] += pCloud.at<cv::Vec3f>(q,p)[0];
	pos[1] += pCloud.at<cv::Vec3f>(q,p)[1];
	pos[2] += pCloud.at<cv::Vec3f>(q,p)[2];
	nv[0] += nvMat.at<cv::Vec3f>(q,p)[0];
	nv[1] += nvMat.at<cv::Vec3f>(q,p)[1];
	nv[2] += nvMat.at<cv::Vec3f>(q,p)[2];
	area++;
      }
    }
    if(area < t_size*t_size*4/2){
      continue;
    }
    for(int k=0; k<3; k++){
      pos[k]/=area;
      nv[k]/=area;
    }
    double denom = sqrt(nv[0]*nv[0]+nv[1]*nv[1]+nv[2]*nv[2]);
    unsigned int area_counter = 0;
    for(int q=0; q<h; q++){
      for(int p=0; p<w; p++){
	if(idMat.at<unsigned short int>(q,p) != 0){
	  continue;
	}
	double plane_len = fabs(nv[0]*(pos[0]-pCloud.at<cv::Vec3f>(q,p)[0])
				+nv[1]*(pos[1]-pCloud.at<cv::Vec3f>(q,p)[1])
				+nv[2]*(pos[2]-pCloud.at<cv::Vec3f>(q,p)[2]))/denom;
	if(plane_len < len_th){
	  idMat.at<unsigned short int>(q,p) = id_counter;
	  area_counter++;
	}
      }
    }
    if(area_counter == 0){
      continue;
    }
    ElemPlane ep;
    ep.id = id_counter;
    ep.x = x;
    ep.y = y;
    ep.area = area_counter;
    for(int k=0; k<3; k++){
      ep.pos[k] = pos[k];
      ep.nv[k] = nv[k];
    }
    plane_v.push_back(ep);
    id_counter++;
    plane_area += area_counter;

    if(plane_area > valid_area*2/3){
      break;
    }
    if(plane_v.size() > 10){
      break;
    }
  }//base_line

  maskMat = cv::Mat::zeros(h,w,CV_8UC1);
  for(int j=1; j<h-1; j++){
    for(int i=1; i<w-1; i++){
      if(pCloud.at<cv::Vec3f>(j,i)[2] == 0){
	continue;
      }
      unsigned short int id = idMat.at<unsigned short int>(j,i);
      if(plane_v[id].area > area_th){
	continue;
      }
      if(PointLength(pCloud, i, j, i-0, j-1) > len_th*10){
	continue;
      }
      if(PointLength(pCloud, i, j, i-1, j-0) > len_th*10){
	continue;
      }
	
      maskMat.at<unsigned char>(j,i) = 255;
    }
  }
  
  
  log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  if(logptr->getLevel() == log4cxx::Level::getDebug()){
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string debug_path = src_root_path + "/data/debug/segment_";
    cv::imwrite(debug_path+"mask.jpg", maskMat);
    cv::Mat I_lb(h, w, CV_8UC3);
    //srand(5);
    unsigned char color[1024*1000][3] = {{0}};
    for(int i=0; i<id_counter+1; i++){
      color[i][0] = rand()%196 + 60;
      color[i][1] = rand()%196 + 60;
      color[i][2] = rand()%196 + 60;
    }
    for(int j=0; j<idMat.rows; j++){
      for(int i=0; i<idMat.cols; i++){
	unsigned short int id = idMat.at<unsigned short int>(j,i);
	if(id == 0){
	  I_lb.at<cv::Vec3b>(j,i) = cv::Vec3b(0, 0, 0);
	  continue;
	}
	I_lb.at<cv::Vec3b>(j,i) =
	  cv::Vec3b(color[id][0], color[id][1], color[id][2]);
      }
    }
    cv::imwrite(debug_path+"plane.jpg", I_lb);
  }//DEBUG_MODE_ONLY
  
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " end");
}

void T2_robot_vision::robot_vision_segment::VarNvMap(cv::Mat pCloud, cv::Mat nvMat, cv::Mat& varMono, int t_size){
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " start");
  int w = nvMat.cols;
  int h = nvMat.rows;
  if(w < t_size*2+1 || h < t_size*2+1 || t_size < 0){
    varMono = cv::Mat::zeros(h,w, CV_8UC1);
    return;
  }
  
  std::vector<cv::Mat> planes;
  std::vector<cv::Mat> mat_v;
  std::vector<cv::Mat> mat2_v;
  cv::split(nvMat, planes);
  for(std::vector<cv::Mat>::const_iterator itr = planes.begin();
      itr != planes.end();
      itr++){
    cv::Mat sMat(h+1,w+1,CV_64FC1);
    cv::Mat s2Mat(h+1,w+1,CV_64FC1);
    cv::integral(*itr, sMat, s2Mat);
    mat_v.push_back(sMat);
    mat2_v.push_back(s2Mat);
  }
  cv::Mat m_merge;
  cv::Mat m2_merge;
  cv::merge(mat_v, m_merge);
  cv::merge(mat2_v, m2_merge);

  cv::Mat meanMat(h,w,CV_32FC3);
  cv::Mat varMat(h,w,CV_32FC3);

  for(int j=t_size; j<h-t_size; j++){
    for(int i=t_size; i<w-t_size; i++){
      for(int c=0; c<3; c++){
	double p =
	  m_merge.at<cv::Vec3d>(j+t_size+1,i+t_size+1)[c]
	  +m_merge.at<cv::Vec3d>(j-t_size,i-t_size)[c]
	  -m_merge.at<cv::Vec3d>(j+t_size+1,i-t_size)[c]
	  -m_merge.at<cv::Vec3d>(j-t_size,i+t_size+1)[c];
	p/=(t_size*t_size*4);
	meanMat.at<cv::Vec3f>(j,i)[c] = p;

	double q = 
	  m2_merge.at<cv::Vec3d>(j+t_size+1,i+t_size+1)[c]
	  +m2_merge.at<cv::Vec3d>(j-t_size,i-t_size)[c]
	  -m2_merge.at<cv::Vec3d>(j+t_size+1,i-t_size)[c]
	  -m2_merge.at<cv::Vec3d>(j-t_size,i+t_size+1)[c];
	varMat.at<cv::Vec3f>(j,i)[c] = q/(t_size*t_size*4)-p*p;
      }
    }
  }
  
  cv::Mat imColor = cv::Mat::zeros(h,w, CV_8UC3);
  cv::Mat ivColor = cv::Mat::zeros(h,w, CV_8UC3);
  cv::Mat ivMono = cv::Mat::zeros(h,w, CV_8UC1);
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      float x = meanMat.at<cv::Vec3f>(j,i)[0];
      float y = meanMat.at<cv::Vec3f>(j,i)[1];
      float z = meanMat.at<cv::Vec3f>(j,i)[2];
      imColor.at<cv::Vec3b>(j,i)[2] = x*64+127;
      imColor.at<cv::Vec3b>(j,i)[1] = y*64+127;
      imColor.at<cv::Vec3b>(j,i)[0] = z*64+127;
      
      x = varMat.at<cv::Vec3f>(j,i)[0];
      y = varMat.at<cv::Vec3f>(j,i)[1];
      z = varMat.at<cv::Vec3f>(j,i)[2];
      ivColor.at<cv::Vec3b>(j,i)[2] = x*64+127;
      ivColor.at<cv::Vec3b>(j,i)[1] = y*64+127;
      ivColor.at<cv::Vec3b>(j,i)[0] = z*64+127;
      double max_v = 0;
      max_v = (x>y)?x:y;
      max_v = (max_v>z)?max_v:z;
      max_v*=512;
      if(max_v>255){
	max_v = 255;
      }
      ivMono.at<unsigned char>(j,i) = 255 - max_v;
    }
  }
  cv::GaussianBlur(ivMono, ivMono, cv::Size(15,15), 0);
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      if(pCloud.at<cv::Vec3f>(j,i)[2] == 0
	 || i<t_size || i>=w-t_size
	 || j<t_size || j>=h-t_size){
	ivMono.at<unsigned char>(j,i) = 0;
      }
    }
  }
  varMono = ivMono;
  
  log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  if(logptr->getLevel() == log4cxx::Level::getDebug()){
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string debug_path = src_root_path + "/data/debug/segment_";
    cv::imwrite(debug_path+"varMono.jpg", ivMono);
  }//DEBUG_MODE_ONLY
  
  ROS_DEBUG_STREAM("T2_robot_vision:segment " << __FUNCTION__ << " end");
  
}
