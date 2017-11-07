

#ifndef SEG_COMMON_H
#define SEG_COMMON_H

#include <iostream>
#include <list>

#include <math.h>
#include <limits.h>

#define ROS_SEKIYA
//#define DEMO_OUTPUT
//#define LIB_CLUSTER
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


#define MaxCameraNum 20

class CLUSTER {
public:
  std::list<unsigned short int> id_list;
  unsigned short int id;
  int cx;
  int cy;
  double radius;
  float position[3];
  float norm_v[3];
  float color[3];
  double x_sum;
  double y_sum;
  double p_sum[3];
  double n_sum[3];
  double c_sum[3];
  unsigned int counter;
};

class CLST_SUB{
public:
  unsigned short int id;
  unsigned int size;
  int sx;
  int sy;
  int ex;
  int ey;
  CLST_SUB(){
    id = 0;
    size = 0;
    sx = INT_MAX;
    ex = 0;
    sy = INT_MAX;
    ey = 0;
  }
  bool operator<(const CLST_SUB &right) const {
    return (this->size > right.size);
  }
};

class CLST_POS{
public:
  int sx;
  int sy;
  int ex;
  int ey;
};
  
typedef struct SegPrm SegPrm;
struct SegPrm{
  int segment_mode;
  int depth_min;
  int depth_max;
  int sub_window_sx[MaxCameraNum];
  int sub_window_sy[MaxCameraNum];
  int sub_window_w[MaxCameraNum];
  int sub_window_h[MaxCameraNum];
  int norm_min_interval;
  int norm_max_interval;
  int clst_init;
  double merge_len_th;
  int merge_nv_cos[2];
  int merge_plane_sin[2];
  int color_dist;
  double color_nvdiff;
  double plane_dist_th;
  unsigned int plane_area_th;
  int seg_max;
  unsigned int area_min_th;
  unsigned int area_max_th;
  int recycle_len_th;
  int recycle_area_th;
  int rand_seed;
  SegPrm(){
    segment_mode = -1;
    norm_min_interval = 1;
    norm_max_interval = 2;
    depth_min = -2000;
    depth_max = 3000;
    for(int i=0; i<MaxCameraNum; i++){
      sub_window_sx[i]=0;
      sub_window_sy[i]=0;
      sub_window_w[i]=0;
      sub_window_h[i]=0;
    }
    clst_init = 100;
    merge_len_th = 3;
    merge_nv_cos[0] = 3;
    merge_nv_cos[1] = 5;
    merge_plane_sin[0] = 9;
    merge_plane_sin[1] = 9;
    color_dist = 100;
    color_nvdiff = 0;
    plane_dist_th = 10;
    plane_area_th = 6400;
    seg_max = 5;
    area_min_th = 6400;
    area_max_th = 160000;
    recycle_len_th = 10;
    recycle_area_th = 30;
    rand_seed = 0;
  }
  bool Checker(void){
    if(norm_min_interval <= 0){return false;}
    if(norm_max_interval < norm_min_interval){return false;}
    if(depth_max < depth_min){return false;}
    for(int i=0; i<MaxCameraNum; i++){
      if(sub_window_sx[i] < 0 || sub_window_sy[i] < 0){return false;}
    }
    if(clst_init < 50 || clst_init > 400){return false;}
    if(merge_nv_cos[0] < 1){return false;}
    if(merge_nv_cos[1] < 1){return false;}
    if(merge_plane_sin[0] < 1){return false;}
    if(merge_plane_sin[1] < 1){return false;}
    if(color_dist < 0){return false;}
    if(color_nvdiff > 1){return false;}
    if(plane_dist_th < 0){return false;}
    if(plane_area_th < 0){return false;}
    if(seg_max < 1){return false;}
    if(area_min_th > area_max_th){return false;}
    if(recycle_area_th < 0){return false;}
    if(rand_seed < 0){return false;}
    return true;
  }
};

class SegmentMask {
public:
  unsigned short int id;
  int sx;
  int sy;
  int ex;
  int ey;
  cv::Mat mask;
};


#endif //SEG_COMMON_H
