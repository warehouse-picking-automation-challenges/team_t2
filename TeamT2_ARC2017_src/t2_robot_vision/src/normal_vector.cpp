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

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <math.h>

#include "seg_common.hpp" 
#include "robot_vision_segment.h"
#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#endif

static
void DepthCorrection(cv::Mat pCloud, cv::Mat& dMat, double alpha){
  int w = pCloud.cols;
  int h = pCloud.rows;
  if(w!=dMat.cols || h!=dMat.rows
     || dMat.depth()!=CV_32F || dMat.channels()!=1){
    dMat = cv::Mat(h,w,CV_32FC1);
  }
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      double d = pCloud.at<cv::Vec3f>(j,i)[2];
      dMat.at<float>(j,i) = static_cast<float>(alpha*d);
    }
  }
  
}

static
void DepthChangeIndicationMap(cv::Mat pCloud, cv::Mat dMat,
			      cv::Mat& cMat, double gamma){
  int w = dMat.cols;
  int h = dMat.rows;
  if(pCloud.cols != w || pCloud.rows != h){
    cMat = cv::Mat::ones(h, w, CV_8UC1)*255;
    return;
  }
  cv::Mat thMat(h, w, CV_32FC1);
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      double d = dMat.at<float>(j,i);
      thMat.at<float>(j,i) = static_cast<float>(gamma*d);
    }
  }
  cMat = cv::Mat::ones(h, w, CV_8UC1)*255;
  for(int j=1; j<h-1; j++){
    for(int i=1; i<w-1; i++){
      double dx =
	fabs(pCloud.at<cv::Vec3f>(j,i+1)[2]-pCloud.at<cv::Vec3f>(j,i-1)[2]);
      double dy =
	fabs(pCloud.at<cv::Vec3f>(j+1,i)[2]-pCloud.at<cv::Vec3f>(j-1,i)[2]);
      double th = thMat.at<float>(j,i);
      if(dx >= th || dy>= th){
	cMat.at<unsigned char>(j,i) = 0;
      }
    }
  }
}

static
void DistanceTransformMap(cv::Mat cMat, cv::Mat& tMat){
  int w = cMat.cols;
  int h = cMat.rows;
  if(w!=tMat.cols || h!=tMat.rows
     || tMat.depth()!=CV_32F || tMat.channels()!=1){
    tMat = cv::Mat(h,w,CV_32FC1);
  }
  cv::distanceTransform(cMat, tMat, CV_DIST_L1, 3);
}

static
void PreSmoothingArea(cv::Mat dMat, cv::Mat& bMat, double beta){
  int w = dMat.cols;
  int h = dMat.rows;
  if(w!=bMat.cols || h!=bMat.rows
     || bMat.depth()!=CV_32F || bMat.channels()!=1){
    bMat = cv::Mat(h,w,CV_32FC1);
  }
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      double d = dMat.at<float>(j,i);
      bMat.at<float>(j,i) = static_cast<float>(beta*d);
    }
  }
}

static
void SmoothingAreaMap(cv::Mat bMat, cv::Mat tMat, cv::Mat& rMat,
		      int max_size, int min_size){
  int w = bMat.cols;
  int h = bMat.rows;
  if(tMat.cols != w || tMat.rows != h){
    rMat = cv::Mat::zeros(h,w,CV_8UC1);
    return;
  }
  if(w!=rMat.cols || h!=rMat.rows
     || rMat.depth()!=CV_8U || rMat.channels()!=1){
    rMat = cv::Mat(h,w,CV_8UC1);
  }
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      double b = bMat.at<float>(j,i);
      double t = tMat.at<float>(j,i)/1.4142;
      double r = 0;
      if(b<t && b<max_size){
	r = b;
      }else if(t<max_size){
	r = t;
      }else{
	r = max_size;
      }
      if(r < min_size){
	r = min_size;
      }
      rMat.at<unsigned char>(j,i) = static_cast<unsigned char>(r);	
    }
  }
}

static
void DepthIntegralMap(cv::Mat pCloud, cv::Mat& diMat, cv::Mat& zcMat){
  int w = pCloud.cols;
  int h = pCloud.rows;
  if(w+1!=diMat.cols || h+1!=diMat.rows
     || diMat.depth()!=CV_64F || diMat.channels()!=1){
    diMat = cv::Mat(h+1,w+1,CV_64FC1);
  }

  cv::Mat zMat(h,w,CV_32FC1);
  cv::Mat cMat(h,w,CV_32FC1);
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      zMat.at<float>(j,i) = pCloud.at<cv::Vec3f>(j,i)[2];
      cMat.at<float>(j,i) = (pCloud.at<cv::Vec3f>(j,i)[2]!=0)?(1):(0);
    }
  }
  cv::integral(zMat, diMat, -1);
  cv::integral(cMat, zcMat, -1);
}

static
float IntegralAreaResult(cv::Mat diMat, cv::Mat zcMat, int x, int y, int r){
  double numer =
    diMat.at<double>(y+r+1,x+r+1)
    -diMat.at<double>(y-r,x+r+1)
    -diMat.at<double>(y+r+1,x-r)
    +diMat.at<double>(y-r,x-r);
  double denom = 
    zcMat.at<double>(y+r+1,x+r+1)
    -zcMat.at<double>(y-r,x+r+1)
    -zcMat.at<double>(y+r+1,x-r)
    +zcMat.at<double>(y-r,x-r);
  if(denom == 0){
    return 0;
  }
  return static_cast<float>(numer/denom);
}

void NormalVectorMap(cv::Mat pCloud, cv::Mat& nvMat,
		     double alpha, double beta, double gamma,
		     int max_size, int min_size){

  
  int w = pCloud.cols;
  int h = pCloud.rows;
  
  cv::Mat dMat(h,w,CV_32FC1);
  DepthCorrection(pCloud, dMat, alpha);
  
  cv::Mat cMat = cv::Mat::ones(h, w, CV_8UC1);
  DepthChangeIndicationMap(pCloud, dMat, cMat, gamma);
  
  cv::Mat tMat(h,w,CV_32FC1);
  DistanceTransformMap(cMat, tMat);

  cv::Mat bMat(h,w,CV_32FC1);
  PreSmoothingArea(dMat, bMat, beta);
  
  cv::Mat rMat(h,w,CV_8UC1);
  SmoothingAreaMap(bMat, tMat, rMat, max_size, min_size);

  cv::Mat diMat(h+1,w+1,CV_64FC1);
  cv::Mat zcMat(h+1,w+1,CV_64FC1);
  DepthIntegralMap(pCloud, diMat, zcMat);

  nvMat = cv::Mat::zeros(h,w,CV_32FC3);
  cv::Mat vhMat(h,w,CV_32FC3);
  cv::Mat vvMat(h,w,CV_32FC3);
  for(int j=1; j<h-1; j++){
    for(int i=1; i<w-1; i++){
      if(pCloud.at<cv::Vec3f>(j,i)[2] == 0){
	continue;
      }
      int r = rMat.at<unsigned char>(j,i);
      if(i-r<0 || j-r<0 || i+r>=w || j+r>=h){
	int buf_a = (i<j)?i:j;
	int buf_b = (w-1-i<h-1-j)?(w-1-i):(h-1-j);
	int buf = (buf_a<buf_b)?buf_a:buf_b;
	r = buf;
      }
      vhMat.at<cv::Vec3f>(j,i)[0] =
	(pCloud.at<cv::Vec3f>(j,i+r)[0]-pCloud.at<cv::Vec3f>(j,i-r)[0])/2;
      vhMat.at<cv::Vec3f>(j,i)[1] =
	(pCloud.at<cv::Vec3f>(j,i+r)[1]-pCloud.at<cv::Vec3f>(j,i-r)[1])/2;
      /*
      vhMat.at<cv::Vec3f>(j,i)[2] =
	(pCloud.at<cv::Vec3f>(j,i+r)[2]-pCloud.at<cv::Vec3f>(j,i-r)[2])/2;
      */
      
      vhMat.at<cv::Vec3f>(j,i)[2] =
	(IntegralAreaResult(diMat, zcMat, i+1, j, r-1)
	 -IntegralAreaResult(diMat, zcMat, i-1, j, r-1))/2;
      
      vvMat.at<cv::Vec3f>(j,i)[0] =
	(pCloud.at<cv::Vec3f>(j+r,i)[0]-pCloud.at<cv::Vec3f>(j-r,i)[0])/2;
      vvMat.at<cv::Vec3f>(j,i)[1] =
	(pCloud.at<cv::Vec3f>(j+r,i)[1]-pCloud.at<cv::Vec3f>(j-r,i)[1])/2;
      /*
      vvMat.at<cv::Vec3f>(j,i)[2] =
	(pCloud.at<cv::Vec3f>(j+r,i)[2]-pCloud.at<cv::Vec3f>(j-r,i)[2])/2;
      */
      
      vvMat.at<cv::Vec3f>(j,i)[2] =
	(IntegralAreaResult(diMat, zcMat, i, j+1, r-1)
	 -IntegralAreaResult(diMat, zcMat, i, j-1, r-1))/2;
      
      
      cv::Vec3f vh = vhMat.at<cv::Vec3f>(j,i);
      cv::Vec3f vv = vvMat.at<cv::Vec3f>(j,i);
      cv::Vec3f vn = vh.cross(vv);
      nvMat.at<cv::Vec3f>(j,i) = vn;
    }
  }
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      float x = nvMat.at<cv::Vec3f>(j,i)[0];
      float y = nvMat.at<cv::Vec3f>(j,i)[1];
      float z = nvMat.at<cv::Vec3f>(j,i)[2];
      double len = sqrt(x*x+y*y+z*z);
      if(len == 0){
	continue;
      }
      nvMat.at<cv::Vec3f>(j,i)[0] = x/len;
      nvMat.at<cv::Vec3f>(j,i)[1] = y/len;
      nvMat.at<cv::Vec3f>(j,i)[2] = z/len;
    }
  }
    
  if(logptr->getLevel() == log4cxx::Level::getDebug()){
    int w = nvMat.cols;
    int h = nvMat.rows;
    cv::Mat inColor = cv::Mat::zeros(h,w, CV_8UC3);
    for(int j=0; j<h; j++){
      for(int i=0; i<w; i++){
	float x = nvMat.at<cv::Vec3f>(j,i)[0];
	float y = nvMat.at<cv::Vec3f>(j,i)[1];
	float z = nvMat.at<cv::Vec3f>(j,i)[2];
	inColor.at<cv::Vec3b>(j,i)[2] = x*120+127;
	inColor.at<cv::Vec3b>(j,i)[1] = y*120+127;
	inColor.at<cv::Vec3b>(j,i)[0] = z*120+127;
      }
    }
    std::string debug_file_path = src_root_path + "/data/debug/segment_norm.jpg";
    cv::imwrite(debug_file_path, inColor);
  }
}

