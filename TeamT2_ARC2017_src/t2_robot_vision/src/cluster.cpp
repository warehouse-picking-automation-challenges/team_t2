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

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <limits.h>
#include <time.h>

#include "cluster.hpp"

#ifdef LIB_CLUSTER
#include "cluster_sub.hpp"
#endif //LIB_CLUSTER

#include "robot_vision_segment.h"
#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#endif



static
void CalcDensityMap(cv::Mat pCloud, cv::Mat& iDens, int radius){

  int w = pCloud.cols;
  int h = pCloud.rows;
  
  cv::Mat iIntvl(h, w, CV_8UC1);
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      iIntvl.at<unsigned char>(j,i) = 1;
    }
  }
  
  cv::Mat iGrad(pCloud.rows, pCloud.cols, CV_32FC1);
  for(int j=0; j<pCloud.rows; j++){
    for(int i=0; i<pCloud.cols; i++){
      int interval = iIntvl.at<unsigned char>(j,i);
      int sx = i-interval;
      int ex = i+interval;
      int sy = j-interval;
      int ey = j+interval;
      if(sx<0){sx=0;}
      if(ex>=w){ex=w-1;}
      if(sy<0){sy=0;}
      if(ey>=h){ey=h-1;}
      double dx = pCloud.at<cv::Vec3f>(j,sx)[2]-pCloud.at<cv::Vec3f>(j,ex)[2];
      double dy = pCloud.at<cv::Vec3f>(sy,i)[2]-pCloud.at<cv::Vec3f>(ey,i)[2];
      iGrad.at<float>(j,i) = sqrt(dx*dx+dy*dy + 1);
    }
  }
  
  if(iDens.rows!=h || iDens.cols!=w){
    iDens = cv::Mat(h,w,CV_32FC1);
  }
  double max = 0;
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      double z = pCloud.at<cv::Vec3f>(j,i)[2];
      if(z == 0){
	iDens.at<float>(j,i) = 0;
	continue;
      }
      double denom = z*z*iGrad.at<float>(j,i);
      iDens.at<float>(j,i) = (radius*radius)/denom;
      if(max < iDens.at<float>(j,i)){
	max = iDens.at<float>(j,i);
      }
    }
  }
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      iDens.at<float>(j,i) = 1 - iDens.at<float>(j,i)*1/max;
    }
  }
}

static
void UpdateClusterList(std::unique_ptr<CLUSTER[]>& clst, unsigned short int clst_num){
  for(unsigned short int id = 0; id < clst_num; id++){
    int cx = clst[id].cx;
    int cy = clst[id].cy;
    double r = clst[id].radius*2+1;
    //erase
    for(std::list<unsigned short int>::iterator itr = clst[id].id_list.begin();
	itr != clst[id].id_list.end();
	itr++){
      unsigned short int id_n = *itr;
      int x = clst[id_n].cx;
      int y = clst[id_n].cy;
      if(abs(x-cx)>=r || abs(y-cy)>=r){
	clst[id].id_list.erase(itr);
      }
    }//erase end
    //add
    for(unsigned short int i=0; i<clst_num; i++){
      int x = clst[i].cx;
      int y = clst[i].cy;
      if(abs(x-cx)<r && abs(y-cy)<r){
	clst[id].id_list.push_back(i);
      }
    }//add end
    clst[id].id_list.unique();
  }
}

static
void SetClstMat(cv::Mat pCloud,
		std::unique_ptr<CLUSTER[]>& clst,
		unsigned short int& clst_num,
		cv::Mat& clstMat){
  int w = clstMat.cols;
  int h = clstMat.rows;
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      if(clstMat.at<unsigned short int>(j,i)!=USHRT_MAX){
	continue;
      }
      if(pCloud.at<cv::Vec3f>(j,i)[2] == 0){
	clstMat.at<unsigned short int>(j,i) = USHRT_MAX;
	continue;
      }
      int min_id = USHRT_MAX;
      double min = w+h;
      for(unsigned short int id=0; id<clst_num; id++){
	int cx = clst[id].cx;
	int cy = clst[id].cy;
	double r = clst[id].radius*8;
	if(abs(cx-i)>r || abs(cy-j)>r){
	  continue;
	}
	double len = (cx-i)*(cx-i)+(cy-j)*(cy-j);
	if(min > len){
	  min = len;
	  min_id = id;
	}
      }
      if(min_id == USHRT_MAX){
	for(unsigned short int id=0; id<clst_num; id++){
	  int cx = clst[id].cx;
	  int cy = clst[id].cy;
	  double len = (cx-i)*(cx-i)+(cy-j)*(cy-j);
	  if(min > len){
	    min = len;
	    min_id = id;
	  }
	}
      }
      clstMat.at<unsigned short int>(j,i) = min_id;
    }//w
  }//h
}

static
void SetClusterCenter(cv::Mat& iDens, int dens_th,
		      cv::Mat pCloud, cv::Mat nvMat, cv::Mat iColor,
		      std::unique_ptr<CLUSTER[]>& clst,
		      unsigned short int& clst_num,
		      cv::Mat& clstMat, double& mean_r){
  int w = iDens.cols;
  int h = iDens.rows;

  typedef struct CLST_P CLST_P;
  struct CLST_P{
    int cx;
    int cy;
    double radius;
  };
  std::vector<CLST_P> c_vector;
  
  cv::Mat diMat(h+1, w+1, CV_64FC1);
  cv::Mat flgMat = cv::Mat::zeros(h,w,CV_8UC1);
  cv::integral(iDens, diMat, -1);
  for(int j=2; j<h-2; j++){
    for(int i=2; i<w-2; i++){
      if(flgMat.at<unsigned char>(j,i)!=0
	 || pCloud.at<cv::Vec3f>(j,i)[2] == 0){
	continue;
      }
  
      if(c_vector.size() > USHRT_MAX/2){
	break;
      }
      CLST_P clst_p;
      clst_p.cx = i;
      clst_p.cy = j;
      int r;
      for(r=2; r<20; r++){//3
	if(i-r<0 || j-r<0 || i+r+1>=w || j+r+1>=h){
	  r--;
	  break;
	}
	double score =
	  diMat.at<double>(j+r+1,i+r+1)
	  -diMat.at<double>(j-r,i+r+1)
	  -diMat.at<double>(j+r+1,i-r)
	  +diMat.at<double>(j-r,i-r);
	if(score > dens_th){
	  break;
	}
      }
      clst_p.radius = r;
      c_vector.push_back(clst_p);
      for(int q=j-r*2-1; q<j+r*2; q++){
	if(q<0 || q>=h){
	  continue;
	}
	for(int p=i-r*2-1; p<i+r*2; p++){
	  if(p<0 || p>=w){
	    continue;
	  }
	  flgMat.at<unsigned char>(q,p) = 1;
	}
      }
    }//i
  }//j
  
  try{
    std::unique_ptr<CLUSTER[]> buf(new CLUSTER[c_vector.size()]);
    clst = std::move(buf);
    clst_num = c_vector.size();
  }catch(...){
    clst_num = 0;
    return;
  }
  double total_radius = 0;
  clstMat = cv::Mat::ones(h, w, CV_16UC1)*USHRT_MAX;
  cv::Mat distMat = cv::Mat::ones(h,w,CV_16UC1)*USHRT_MAX;
  {
    unsigned short int i = 0;
    int c_flg = 0;
    for(std::vector<CLST_P>::iterator itr = c_vector.begin();
	itr!=c_vector.end();
	itr++){
      if(i>= clst_num){
	c_flg = 1;
	break;
      }
      CLST_P clst_p = *itr;
      int x = clst_p.cx;
      int y = clst_p.cy;
      double r = clst_p.radius;
      total_radius += r;
      clst[i].id = i;
      clst[i].cx = x;
      clst[i].cy = y;
      clst[i].radius = r;
      clst[i].position[0] = pCloud.at<cv::Vec3f>(y,x)[0];
      clst[i].position[1] = pCloud.at<cv::Vec3f>(y,x)[1];
      clst[i].position[2] = pCloud.at<cv::Vec3f>(y,x)[2];
      clst[i].norm_v[0] = nvMat.at<cv::Vec3f>(y,x)[0];
      clst[i].norm_v[1] = nvMat.at<cv::Vec3f>(y,x)[1];
      clst[i].norm_v[2] = nvMat.at<cv::Vec3f>(y,x)[2];
      clst[i].color[0] = iColor.at<cv::Vec3b>(y,x)[0];
      clst[i].color[1] = iColor.at<cv::Vec3b>(y,x)[1];
      clst[i].color[2] = iColor.at<cv::Vec3b>(y,x)[2];
      clst[i].counter = 0;
      for(int q=y-r*4; q<=y+r*4; q++){
	if(q<0 || q>=h){
	  continue;
	}
	for(int p=x-r*4; p<=x+r*4; p++){
	  if(p<0 || p>=w){
	    continue;
	  }
	  if(pCloud.at<cv::Vec3f>(q,p)[2] == 0){
	    continue;
	  }
	  unsigned short int d = abs(q-y)+abs(p-x);
	  if(d < distMat.at<unsigned short int>(q,p)){
	    clstMat.at<unsigned short int>(q,p) = i;
	    distMat.at<unsigned short int>(q,p) = d;
	  }
	}//p
      }//q
      i++;
    }
    if(c_flg == 1){
      clst_num = 0;
      return;
    }
  }
  if(clst_num>0){
    mean_r = total_radius/clst_num;
  }else{
    mean_r = 0;
  }
  SetClstMat(pCloud, clst, clst_num, clstMat);
  UpdateClusterList(clst, clst_num);
}


#ifndef LIB_CLUSTER
static
double LengthCluster(std::unique_ptr<CLUSTER[]>& clst,
		     unsigned short int id,
		     float p[3], float nv[3], float c[3]){
  if(clst[id].position[2] == 0 && p[2]!=0){
    return USHRT_MAX;
  }
  double p_diff = sqrt((clst[id].position[0]-p[0])*(clst[id].position[0]-p[0])+(clst[id].position[1]-p[1])*(clst[id].position[1]-p[1])+(clst[id].position[2]-p[2])*(clst[id].position[2]-p[2]));
  double nv_diff = 1-(clst[id].norm_v[0]*nv[0]+clst[id].norm_v[1]*nv[1]+clst[id].norm_v[2]*nv[2]);
  double c_diff = sqrt((clst[id].color[0]-c[0])*(clst[id].color[0]-c[0])+(clst[id].color[1]-c[1])*(clst[id].color[1]-c[1])+(clst[id].color[2]-c[2])*(clst[id].color[2]-c[2]));
  
  //return p_diff*50+nv_diff*2+c_diff/100;
  return p_diff*40+nv_diff*10+c_diff/10;
}
#endif //LIB_CLUSTER

static
bool LengthClusterMerge(std::unique_ptr<CLUSTER[]>& clst,
			unsigned short int id1, unsigned short int id2,
			double len_th, SegPrm seg_prm){
  if(clst[id1].position[2] == 0
     || clst[id2].position[2] == 0){
    return false;
  }
  double nv_diff = (clst[id1].norm_v[0]*clst[id2].norm_v[0]+clst[id1].norm_v[1]*clst[id2].norm_v[1]+clst[id1].norm_v[2]*clst[id2].norm_v[2]);
  double p_diff = sqrt((clst[id1].position[0]-clst[id2].position[0])*(clst[id1].position[0]-clst[id2].position[0])+(clst[id1].position[1]-clst[id2].position[1])*(clst[id1].position[1]-clst[id2].position[1])+(clst[id1].position[2]-clst[id2].position[2])*(clst[id1].position[2]-clst[id2].position[2]));
  double plane_diff1 = (clst[id1].norm_v[0]*(clst[id2].position[0]-clst[id1].position[0])+clst[id1].norm_v[1]*(clst[id2].position[1]-clst[id1].position[1])+clst[id1].norm_v[2]*(clst[id2].position[2]-clst[id1].position[2]));
  double plane_diff2 = (clst[id2].norm_v[0]*(clst[id1].position[0]-clst[id2].position[0])+clst[id2].norm_v[1]*(clst[id1].position[1]-clst[id2].position[1])+clst[id2].norm_v[2]*(clst[id1].position[2]-clst[id2].position[2]));
  double c_diff = sqrt((clst[id1].color[0]-clst[id2].color[0])*(clst[id1].color[0]-clst[id2].color[0])+(clst[id1].color[1]-clst[id2].color[1])*(clst[id1].color[1]-clst[id2].color[1])+(clst[id1].color[2]-clst[id2].color[2])*(clst[id1].color[2]-clst[id2].color[2]));

  
  if(p_diff == 0){
    return true;
  }
  if(p_diff > len_th){
    return false;
  }

  double denom = 1;
  if(clst[id1].counter < 100){
    denom = seg_prm.merge_nv_cos[0];
  }else{
    denom = seg_prm.merge_nv_cos[1];
  }
  if(nv_diff < cos(M_PI/denom)){
    return false;
  }
  
  double ph_ang = cos(M_PI/4);
  double c_ang = cos(M_PI/6);
  if(fabs(clst[id1].norm_v[2] < ph_ang)
     && fabs(clst[id2].norm_v[2]) < ph_ang){
    return true;
  }else if(fabs(clst[id1].norm_v[2] < c_ang)
     && fabs(clst[id2].norm_v[2]) < c_ang){
    denom = seg_prm.merge_plane_sin[0];
  }else{
    denom = seg_prm.merge_plane_sin[1];
  }
  
  if((fabs(plane_diff1)/p_diff > sin(M_PI/denom) || fabs(plane_diff2)/p_diff > sin(M_PI/denom))){
    return false;
  }
  
  if(c_diff > seg_prm.color_dist
     && nv_diff < seg_prm.color_nvdiff){
    return false;
  }
  
  return true;
}

#ifndef LIB_CLUSTER
static
double MoveClstCenter(cv::Mat pCloud, cv::Mat nvMat, cv::Mat iColor,
		      std::unique_ptr<CLUSTER[]>& clst,
		      unsigned short int clst_num, cv::Mat& clstMat){


  int w = pCloud.cols;
  int h = pCloud.rows;

  for(unsigned short int i=0; i<clst_num; i++){
    clst[i].x_sum = 0;
    clst[i].y_sum = 0;
    clst[i].radius = w+h;
    for(int j=0; j<3; j++){
      clst[i].p_sum[j] = 0;
      clst[i].n_sum[j] = 0;
      clst[i].c_sum[j] = 0;
    }
    clst[i].counter = 0;
  }
  
  double max_diff = 0;
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      unsigned short int id = clstMat.at<unsigned short int>(j,i);
      if(id == USHRT_MAX){
	continue;
      }
      float p[3];
      float nv[3];
      float c[3];
      for(int k=0; k<3; k++){
	p[k] = pCloud.at<cv::Vec3f>(j,i)[k];
	nv[k] = nvMat.at<cv::Vec3f>(j,i)[k];
	c[k] = iColor.at<cv::Vec3b>(j,i)[k];
      }
      double min_len = USHRT_MAX;
      int min_id = id;
      for(std::list<unsigned short int>::iterator itr = clst[id].id_list.begin();
	  itr != clst[id].id_list.end();
	  itr++){
	unsigned short int id_n = *itr;
	double len = LengthCluster(clst, id_n, p, nv, c);
	//double p_len = sqrt((p[0]-clst[id_n].position[0])*(p[0]-clst[id_n].position[0])+(p[1]-clst[id_n].position[1])*(p[1]-clst[id_n].position[1])+(p[2]-clst[id_n].position[2])*(p[2]-clst[id_n].position[2]));
	if(len < min_len){
	  if(min_len != USHRT_MAX
	     && clst[min_id].radius > min_len
	     && min_len > 0.005){
	    clst[min_id].radius = min_len;
	  }
	  min_len = len;
	  min_id = id_n;
	}else if(id_n!=min_id){
	double p_len = sqrt((p[0]-clst[id_n].position[0])*(p[0]-clst[id_n].position[0])+(p[1]-clst[id_n].position[1])*(p[1]-clst[id_n].position[1])+(p[2]-clst[id_n].position[2])*(p[2]-clst[id_n].position[2]));
	  if(clst[id_n].radius > p_len && p_len > 0.005){
	    clst[id_n].radius = p_len;
	  }
	}
      }
      
      clstMat.at<unsigned short int>(j,i) = min_id;
      clst[min_id].x_sum += i;
      clst[min_id].y_sum += j;
      for(int k=0; k<3; k++){
	clst[min_id].p_sum[k] += p[k];
	clst[min_id].n_sum[k] += nv[k];
	clst[min_id].c_sum[k] += c[k];
      }
      clst[min_id].counter++;
      
      if(min_len > max_diff){
	max_diff = min_len;
      }
    }//i
  }//j

  for(unsigned short int i=0; i<clst_num; i++){
    unsigned int denom = clst[i].counter;
    if(denom == 0){
      continue;
    }
    clst[i].cx = clst[i].x_sum/denom;
    clst[i].cy = clst[i].y_sum/denom;
    for(int j=0; j<3; j++){
      clst[i].position[j] = clst[i].p_sum[j]/denom;
      clst[i].norm_v[j] = clst[i].n_sum[j]/denom;
      clst[i].color[j] = clst[i].c_sum[j]/denom;
    }
  }

  return max_diff;
}
#endif //LIB_CLUSTER

static
void MergeCluster(std::unique_ptr<CLUSTER[]>& clst, unsigned short int clst_num,
		  std::vector<CLST_SUB>& clst_mv,
		  double mean_r,
		  SegPrm seg_prm){

  double len_th = 12;
  if(seg_prm.merge_len_th > 0){
    len_th = mean_r * seg_prm.merge_len_th;
  }else{
    len_th = mean_r * fabs(seg_prm.merge_len_th);
  }
    
  std::unique_ptr<unsigned short int[]> link;
  try{
    std::unique_ptr<unsigned short int[]> buf(new unsigned short int[clst_num]);
    link = std::move(buf);
  }catch(...){
    return;
  }
  for(unsigned short int i=0; i<clst_num; i++){
    link[i] = i;
  }
  for(unsigned short int i=0; i<clst_num; i++){
    double r_th = clst[i].radius*3;
    if(len_th < r_th){
      r_th = len_th;
    }
    for(std::list<unsigned short int>::iterator itr = clst[i].id_list.begin();
	itr != clst[i].id_list.end();
	itr++){
      unsigned short int j = *itr;
      bool flg = LengthClusterMerge(clst, i, j, r_th, seg_prm);
      if(flg){
	unsigned short int buf = link[i];
	while(buf!=i && buf!=j){
	  buf = link[buf];
	}
	if(buf == i){
	  buf = link[i];
	  link[i] = link[j];
	  link[j] = buf;
	}
      }// len < th
    }// list
  }// clst_num
  
  std::unique_ptr<unsigned short int[]> table;
  try{
    std::unique_ptr<unsigned short int[]> buf(new unsigned short int[clst_num]);
    table = std::move(buf);
  }catch(...){
    return;
  }
  clst_mv.clear();
  for(unsigned short int i=0; i<clst_num; i++){
    table[i] = 0;
  }
  unsigned short int clst_num_new = 1;
  for(unsigned short int i=0; i<clst_num; i++){
    if(table[i]!=0){
      continue;
    }
    CLST_SUB clst_sub;
    clst_sub.id = clst_num_new;
    clst_sub.size = clst[i].counter;
    table[i] = clst_num_new;
    clst[i].id = table[i];
    unsigned short int j = link[i];
    while(j!=i){
      table[j] = clst_num_new;
      clst[j].id = clst_num_new;
      clst_sub.size += clst[j].counter;
      j = link[j];
    }
    clst_mv.push_back(clst_sub);
    clst_num_new++;
  }
  
  for(unsigned short int i=0; i<clst_num; i++){
    if(link[i]!=i){
      continue;
    }
    unsigned short int area_max = 0;
    unsigned short int id_max = USHRT_MAX;
    unsigned short int id_count = 0;
    unsigned short int n_count = 0;
    for(std::list<unsigned short int>::iterator itr = clst[i].id_list.begin();
	itr != clst[i].id_list.end();
	itr++){
      unsigned short int j = *itr;
      n_count++;
      if(id_max == table[j]){
	id_count++;
	continue;
      }
      if(area_max < clst_mv[table[j]-1].size){
	area_max = clst_mv[table[j]-1].size;
	id_max = table[j];
	id_count = 1;
      }
    }
    if(id_count*3 >= n_count){
      clst[i].id = id_max;
      clst_mv[id_max-1].size += clst[i].counter;
    }
  }
}

void ClustSegment(cv::Mat pCloud, cv::Mat nvMat, cv::Mat iColor,
		  SegPrm seg_prm, std::vector<SegmentMask>& seg_mask_v){  
  int w = pCloud.cols;
  int h = pCloud.rows;
  
  int radius = 80;
  cv::Mat iDens(h,w, CV_32FC1);
  CalcDensityMap(pCloud, iDens, radius);

  std::unique_ptr<CLUSTER[]> clst;
  unsigned short int clst_num;
  double mean_r;
  int dens_th = seg_prm.clst_init;
  cv::Mat clstMat(h, w, CV_16UC1);
  SetClusterCenter(iDens, dens_th, pCloud, nvMat, iColor,
		   clst, clst_num, clstMat, mean_r);

#ifndef LIB_CLUSTER
  double old_diff = 0;
  for(int i=0; i<5; i++){
    double max_diff = MoveClstCenter(pCloud, nvMat, iColor, clst, clst_num, clstMat);
    if(max_diff < 5){
      break;
    }
    if(max_diff == old_diff){
      break;
    }
    old_diff = max_diff;
  }
#else
  
  MoveClstLoop(pCloud, nvMat, iColor,
	       clst,
	       clst_num, clstMat,
	       5);
#endif //LIB_CLUSTER

  std::vector<CLST_SUB> clst_mv;
  MergeCluster(clst, clst_num, clst_mv, mean_r, seg_prm);
  
  for(int j=0; j<h; j++){
    for(int i=0; i<w; i++){
      unsigned short int id_i = clstMat.at<unsigned short int>(j,i);
      if(id_i == USHRT_MAX){
	continue;
      }
      if(clst[id_i].id==0){
	continue;
      }
      unsigned short int id = clst[id_i].id-1;
      if(clst_mv[id].sx > i){
	clst_mv[id].sx = i;
      }
      if(clst_mv[id].ex < i){
	clst_mv[id].ex = i;
      }
      if(clst_mv[id].sy > j){
	clst_mv[id].sy = j;
      }
      if(clst_mv[id].ey < j){
	clst_mv[id].ey = j;
      }
    }
  }
  
  std::sort(clst_mv.begin(), clst_mv.end());
  seg_mask_v.clear();
  int counter = 0;
  for(unsigned int i=0; i<clst_mv.size(); i++){
    if(clst_mv[i].size > seg_prm.area_max_th){
      continue;
    }
    if(clst_mv[i].size < seg_prm.area_min_th){
      break;
    }
    if(counter >= seg_prm.seg_max){
      break;
    }
    counter++;
    SegmentMask seg_mask;
    int w_w = clst_mv[i].ex-clst_mv[i].sx+1;
    int h_w = clst_mv[i].ey-clst_mv[i].sy+1;
    int sx = clst_mv[i].sx;
    int sy = clst_mv[i].sy;

    seg_mask.sx = sx;
    seg_mask.sy = sy;
    seg_mask.ex = clst_mv[i].ex;
    seg_mask.ey = clst_mv[i].ey;
    seg_mask.mask = cv::Mat::zeros(h_w, w_w, CV_8UC1);
    for(int q=0; q<h_w; q++){
      for(int p=0; p<w_w; p++){
	unsigned short int id_i = clstMat.at<unsigned short int>(q+sy,p+sx);
	if(id_i == USHRT_MAX){
	  continue;
	}
	if(clst[id_i].id == 0){
	  continue;
	}
	unsigned short int id = clst[id_i].id;
	if(id == clst_mv[i].id){
	  seg_mask.mask.at<unsigned char>(q, p) = 255;
	}
      }//p
    }//q
    seg_mask_v.push_back(seg_mask);
  }//i seg_max

  {
    srand(seg_prm.rand_seed);
    unsigned char color[1024*1000][3] = {{0}};
    for(int i=0; i<clst_num; i++){
      color[i][0] = rand()%196 + 60;
      color[i][1] = rand()%196 + 60;
      color[i][2] = rand()%196 + 60;
    }
    cv::Mat I_cl = cv::Mat::zeros(clstMat.rows, clstMat.cols, CV_8UC3);
    cv::Mat I_cn = cv::Mat::zeros(clstMat.rows, clstMat.cols, CV_8UC3);
    cv::Mat I_cc = cv::Mat::zeros(clstMat.rows, clstMat.cols, CV_8UC3);
    cv::Mat I_lb = cv::Mat::zeros(clstMat.rows, clstMat.cols, CV_8UC3);
    for(int j=0; j<clstMat.rows; j++){
      for(int i=0; i<clstMat.cols; i++){
	unsigned short int id_i = clstMat.at<unsigned short int>(j,i);
	if(id_i == USHRT_MAX){
	  continue;
	}
	
#ifndef DEMO_OUTPUT
	I_cl.at<cv::Vec3b>(j,i) =
	  cv::Vec3b(color[id_i][0], color[id_i][1], color[id_i][2]);
	I_cn.at<cv::Vec3b>(j,i) =
	  cv::Vec3b(clst[id_i].norm_v[2]*127+128,clst[id_i].norm_v[1]*127+128,clst[id_i].norm_v[0]*127+128);
	I_cc.at<cv::Vec3b>(j,i) =
	  cv::Vec3b(clst[id_i].color[0],clst[id_i].color[1],clst[id_i].color[2]);
	int id = clst[id_i].id;
	if(id == 0){
	  continue;
	}
	I_lb.at<cv::Vec3b>(j,i) =
	  cv::Vec3b(color[id][0], color[id][1], color[id][2]);
#else //DEMO_OUTPUT
	int id = clst[id_i].id;
	if(id == 0){
	  continue;
	}
	I_lb.at<cv::Vec3b>(j,w-i-1) =
	  cv::Vec3b(color[id][0], color[id][1], color[id][2]);
#endif //DEMO_OUTPUT
      }
    }
    cv::imwrite("./debug/sample_clst.jpg", I_cl);
    cv::imwrite("./debug/sample_clst_result_norm.jpg", I_cn);
    cv::imwrite("./debug/sample_clst_result_color.jpg", I_cc);
    cv::imwrite("./debug/sample_clst_merge.jpg", I_lb);
  }
  
  return;
}
