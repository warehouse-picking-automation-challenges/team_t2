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

#ifdef T2_TASK_STRATEGY_GRASP_PACK_SRC
  #define T2_TASK_STRATEGY_GRASP_PACK_EXT
#else
  #define T2_TASK_STRATEGY_GRASP_PACK_EXT extern
#endif

/* システムヘッダ参照 */

/* 外部ヘッダ参照 */
#include "t2_task_planner/t2_task_strategy_def.h"

/* 外部定数定義 */
/* 外部型定義 */
/* 外部変数定義 */

/* 外部関数定義 */
T2_TASK_STRATEGY_GRASP_PACK_EXT bool getGpRpFix(
  uint cad_id,                                                  /* アイテム識別子 */
  uint single_gp,                                               /* 平面検知フラグ */
  geometry_msgs::Pose center_pose,                              /* 図心座標 */
  std::vector<uint> place_id_list,                              /* 検索対象の場所 */
  std::vector<GpRp_t> *gprp                                     /* 結果ポインタ(※出力) */
);

T2_TASK_STRATEGY_GRASP_PACK_EXT bool getGpRpWithPrediction(
  uint cad_id,                                                  /* アイテム識別子 */
  uint single_gp,                                               /* 平面検知フラグ */
  geometry_msgs::Pose center_pose,                              /* 図心座標 */
  std::vector<Location_t> grasp_location_list,                  /* 移動元アイテムリスト */
  std::vector<Location_t> release_location_list,                /* 移動先アイテムリスト */
  std::vector<uint> place_id_list,                              /* 検索対象の場所 */
  std::vector<GpRp_t> *gprp                                     /* 結果ポインタ(※出力) */
);

T2_TASK_STRATEGY_GRASP_PACK_EXT bool getGpRpWithoutPrediction(
  uint cad_id,                                                  /* アイテム識別子 */
  uint single_gp,                                               /* 平面検知フラグ */
  geometry_msgs::Pose center_pose,                              /* 図心座標 */
  std::vector<uint> place_id_list,                              /* 検索対象の場所 */
  std::vector<GpRp_t> *gprp                                     /* 結果ポインタ(※出力) */
);

