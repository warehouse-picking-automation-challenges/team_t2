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

#define T2_TASK_STRATEGY_COMMON_SRC
#include "t2_task_planner/t2_task_strategy_common.h"

/* システムヘッダ参照 */

/* 外部ヘッダ参照 */
#include "t2_task_planner/t2_task_strategy_component.h"

/* 内部定数定義 */
/* 内部型定義 */
/* 内部変数定義 */
/* 内部関数定義 */

/****************/
/* 外部関数処理 */
/****************/
/* Octomap更新関数 */
bool updateOctomap(
  uint place_id,
  octomap_msgs::OctomapWithPose octomapwithpose
){
  std::lock_guard<std::mutex> lock(mtx_component);
  
  bool ret;
  
  ROS_INFO("updateOctomap(input place_id=%d)", (int)place_id);
  ret = updateOctomapProc(place_id, octomapwithpose.origin, octomapwithpose.octomap);
  ROS_INFO("updateOctomap(output place_id=%d, return=%d)", (int)place_id, (int)ret);
  
  return ret;
}

/* component内部変数place表示機能 */
void printPlaceSt(uint place_id)
{
  std::lock_guard<std::mutex> lock(mtx_component);
  printPlaceStProc(place_id);
}

/* component内部変数place全表示機能 */
void printPlaceStAll(void)
{
  std::lock_guard<std::mutex> lock(mtx_component);
  printPlaceStAllProc();
}

/* 強制Voxel操作 */
void setVoxelForce(
  uint place_id,
  int coordRow, int coordCol, int coordHigh,
  int sizeRow, int sizeCol, int sizeHigh,
  bool exist
){
  std::lock_guard<std::mutex> lock(mtx_component);
  setVoxelForceProc(place_id, coordRow, coordCol, coordHigh, sizeRow, sizeCol, sizeHigh, exist);
}

/* 現在のVoxel状況(プロット)をファイル出力する */
void plotOut(uint place_id)
{
  std::lock_guard<std::mutex> lock(mtx_component);
  plotOutProc(place_id);
}

/****************/
/* 内部関数処理 */
/****************/

