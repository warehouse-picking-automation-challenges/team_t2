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

#ifdef T2_TASK_STRATEGY_COMPONENT_SRC
  #define T2_TASK_STRATEGY_COMPONENT_EXT
#else
  #define T2_TASK_STRATEGY_COMPONENT_EXT extern
#endif

/* システムヘッダ参照 */
#include <mutex>

/* 外部ヘッダ参照 */
#include "t2_task_planner/t2_task_strategy_def_internal.h"

/* 外部定数定義 */
/* 外部型定義 */

/* 外部変数定義 */
T2_TASK_STRATEGY_COMPONENT_EXT std::mutex mtx_component;

/* 外部関数定義 */
T2_TASK_STRATEGY_COMPONENT_EXT bool updateOctomapProc(
  uint place_id,
  geometry_msgs::Pose pose,
  octomap_msgs::Octomap octomap
);
T2_TASK_STRATEGY_COMPONENT_EXT void printPlaceStProc(uint place_id);
T2_TASK_STRATEGY_COMPONENT_EXT void printPlaceStAllProc(void);
T2_TASK_STRATEGY_COMPONENT_EXT void setVoxelForceProc(
  uint place_id,
  int coordRow, int coordCol, int coordHigh,
  int sizeRow, int sizeCol, int sizeHigh,
  bool exist
);
T2_TASK_STRATEGY_COMPONENT_EXT void plotOutProc(uint place_id);

T2_TASK_STRATEGY_COMPONENT_EXT void setRobotEasyPose(Eigen::Affine3d *pose);

T2_TASK_STRATEGY_COMPONENT_EXT bool findRp(uint place_id, double item_row, double item_col, double item_high, bool protrude_flg, Eigen::Vector3d *vecRpP);
T2_TASK_STRATEGY_COMPONENT_EXT bool getPlaceLength(uint place_id, double *place_row, double *place_col, double *place_high);
T2_TASK_STRATEGY_COMPONENT_EXT Eigen::Affine3d selectPreRp(uint place_id, bool xy_rot);
T2_TASK_STRATEGY_COMPONENT_EXT double calcProtrude(uint place_id, Eigen::Vector3d vecRcf, double item_high);
T2_TASK_STRATEGY_COMPONENT_EXT double calcProtrudeCoord(uint place_id, double coordZ);
T2_TASK_STRATEGY_COMPONENT_EXT bool getPlaceUpsidePose(uint place_id, Eigen::Affine3d *pose);
T2_TASK_STRATEGY_COMPONENT_EXT bool findFreeLayerCoordZ(uint place_id, double *coordZ);
T2_TASK_STRATEGY_COMPONENT_EXT int calcFreeVolume(uint place_id);
T2_TASK_STRATEGY_COMPONENT_EXT int calcFreeBottomArea(uint place_id);

T2_TASK_STRATEGY_COMPONENT_EXT bool getParamGraspInfoList(uint cad_id, ros::NodeHandle *nhP, XmlRpc::XmlRpcValue *graspParamP);
T2_TASK_STRATEGY_COMPONENT_EXT uint findGraspInfoIdx(XmlRpc::XmlRpcValue *infoParamP, uint32_t gp_number);
T2_TASK_STRATEGY_COMPONENT_EXT bool getParamFixRp(uint cad_id, ros::NodeHandle *nhP, XmlRpc::XmlRpcValue *fixRpParamP);
T2_TASK_STRATEGY_COMPONENT_EXT bool getParamTaskStrategy(ros::NodeHandle *nhP, XmlRpc::XmlRpcValue *strategyParamP);
T2_TASK_STRATEGY_COMPONENT_EXT bool getParamContainerInfo(ros::NodeHandle *nhP, XmlRpc::XmlRpcValue *containerParamP);
T2_TASK_STRATEGY_COMPONENT_EXT uint findContainerIdx(XmlRpc::XmlRpcValue *containerParamP, uint place_id);

T2_TASK_STRATEGY_COMPONENT_EXT void printReqInfo(GRASP_INPUT_REQ_T *reqP, std::string str);
T2_TASK_STRATEGY_COMPONENT_EXT void printRspInfo(GRASP_OUTPUT_RSP_T *rspP, std::string str);
T2_TASK_STRATEGY_COMPONENT_EXT void printRspDebug(GRASP_OUTPUT_RSP_T *rspP, std::string str);

