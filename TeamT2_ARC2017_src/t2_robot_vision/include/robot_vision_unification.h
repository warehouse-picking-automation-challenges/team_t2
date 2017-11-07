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

#pragma once
#ifndef ROBOT_VISION_UNIFICATION
#define ROBOT_VISION_UNIFICATION

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

// include base nodelet
#include "robot_vision_nodelet.h"

// include subscribe message type
#include <T2_robot_vision/RecognizedItem.h>
#include <T2_robot_vision/Conv.h>
#include <T2_robot_vision/ItemCad.h>
#include <T2_robot_vision/CadItem.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#define MAX_PATH 15
#define MAX_CAM 15 
#define MAX_RES 100
#define MAX_MCNT 150
#define MAX_MVCNT 15000

#define ICP_MODE 1
#define DUMP_UNILOG

namespace T2_robot_vision
{
class robot_vision_unification: public T2_robot_vision::robot_vision_nodelet
{
public:
  robot_vision_unification();
  virtual ~robot_vision_unification();

protected:
  virtual void onInit();
  void recognizedItemCallback(const T2_robot_vision::RecognizedItemConstPtr &notice);
  void resultYOLOv2Callback(const T2_robot_vision::RecognizedItemConstPtr &notice);
  void resultLineMODCallback(const T2_robot_vision::RecognizedItemConstPtr &notice);
  void resultAKAZECallback(const T2_robot_vision::RecognizedItemConstPtr &notice);
  bool conv(T2_robot_vision::ConvRequest &req, T2_robot_vision::ConvResponse &res);
  bool Cad2Item(T2_robot_vision::CadItemRequest &req, T2_robot_vision::CadItemResponse &res);
  bool Item2Cad(T2_robot_vision::ItemCadRequest &req, T2_robot_vision::ItemCadResponse &res);
  int convert( int camid, float x0, float y0, float d0, float *x1, float *y1, int flg );
  int storedata(const T2_robot_vision::RecognizedItemConstPtr &notice);
  int integrate(const T2_robot_vision::RecognizedItemConstPtr &notice);
  int resetproc(); 
 
  int rcgpath[MAX_PATH];
  int job_no_proc;
  int rcgpath_proc[MAX_PATH];
  
  int cnt_msg, cnt_res;
  int res_recog_module[MAX_RES];
  int res_category[MAX_RES];
  int res_pitch[MAX_RES];
  int res_yaw[MAX_RES];
  int res_roll[MAX_RES];
  float res_posx[MAX_RES];
  float res_posy[MAX_RES];
  float res_posz[MAX_RES];
  float res_homo[MAX_RES][9];
  int res_score[MAX_RES];
  int res_seg_module[MAX_RES];
  int res_seg_sx[MAX_RES];
  int res_seg_ex[MAX_RES];
  int res_seg_sy[MAX_RES];
  int res_seg_ey[MAX_RES];
  int res_seg_imgidx[MAX_RES];
  int res_seg_imgtype[MAX_RES];
  sensor_msgs::Image res_seg_mask[MAX_RES];

  int table_cadid[MAX_MCNT]; // recognized category -> cad id
  int table_procpriority[MAX_MCNT][MAX_PATH]; // recognized category -> proiority
  double table_matrix[MAX_MCNT][9]; // recognized category -> matrix
  std::string table_name[MAX_MCNT]; // recognized category -> item name
  int table_type[MAX_MCNT]; // 0: regular items, 1: deformable, 2: transparent

  double cp_cfx[MAX_CAM], cp_cfy[MAX_CAM], cp_ccx[MAX_CAM], cp_ccy[MAX_CAM];
  double cp_dfx[MAX_CAM], cp_dfy[MAX_CAM], cp_dcx[MAX_CAM], cp_dcy[MAX_CAM];
  double cp_cdmat[MAX_CAM][16], cp_dcmat[MAX_CAM][16];
  
#ifdef DUMP_UNILOG
  std::ofstream exportlog;
#endif

#if ICP_MODE == 2
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model[MAX_MCNT];
  pcl::PointCloud<pcl::Normal>::Ptr normal_model[MAX_MCNT];
#endif
  int gp_type[MAX_MCNT];
  double gp_mat[MAX_MCNT][16];

  // Subscriber and Publisher
  ros::Subscriber unification_sub_lmod_;
  ros::Subscriber unification_sub_yolo_;
  ros::Subscriber unification_sub_akaze_;
  ros::Subscriber unification_sub_;
  ros::Publisher unification_pub_;

  // Service Client
  ros::ServiceServer server_;
  ros::ServiceServer server_IC_;
  ros::ServiceServer server_CI_;
  //ros::ServiceClient client_;
};

}

#endif //ROBOT_VISION_UNIFICATION

