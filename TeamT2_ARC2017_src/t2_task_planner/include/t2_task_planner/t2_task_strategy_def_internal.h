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

#ifndef T2_TASK_STRATEGY_DEF_INTERNAL_H
#define T2_TASK_STRATEGY_DEF_INTERNAL_H

/* システムヘッダ参照 */
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

/* 外部ヘッダ参照 */
#include "t2_task_planner/t2_task_strategy_def.h"

/* 内部専用定数定義 */
/* GRASP_OUTPUT_RSP_T result用 */
#define GRASP_RESULT_SUCCESS            ((int32_t)0)
#define GRASP_RESULT_FAILED             ((int32_t)1)
#define GRASP_RESULT_CAD_ID_ERR         ((int32_t)2)

/* 内部専用型定義 */
typedef struct{
  uint32_t cad_id;
  uint32_t single_gp;
  Eigen::Affine3d g_center_pos;
  std::vector<Location_t> grasp_location_list;
  std::vector<Location_t> release_location_list;
  std::vector<uint> place_id_list;
} GRASP_INPUT_REQ_T;

typedef struct{
  uint32_t gp_number;
  double score;
  std::string grasp_pattern;
  uint release_place_id;
  double protrude_length;
  
  double length_of_pushing_for_suction;
  double threshold_of_vacuum_for_suction;
  int32_t suction_strength;
  double carry_speed;
  double width_between_finger_for_pinch;
  double width_between_finger_for_release;
  double finger_intrusion_for_pinch;
  double max_effort_for_pinch;
  
  Eigen::Affine3d grasp_point_item;
  Eigen::Affine3d approach_point_item;
  Eigen::Affine3d rp;
  Eigen::Affine3d rap;
} GRASP_POINT_T;

typedef struct{
  int32_t result;
  double total_score;
  uint32_t total_point;
  std::vector<GRASP_POINT_T> grasp_point;
} GRASP_OUTPUT_RSP_T;

#endif /* T2_TASK_STRATEGY_DEF_INTERNAL_H */
