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

#ifndef T2_TASK_STRATEGY_DEF_H
#define T2_TASK_STRATEGY_DEF_H

/* システムヘッダ参照 */
#include <ros/ros.h>
#include <octomap_msgs/OctomapWithPose.h>

/* 内外共通定数定義 */

/* 内外共通型定義 */
typedef struct{
  uint place_id;
  std::vector<uint> cad_id_list;
} Location_t;

typedef struct{
  uint32_t gp_number;
  double score;
  std::string grasp_pattern;
  uint release_place_id;
  double protrude_length;
  
  int32_t suction_strength;
  double threshold_of_vacuum_for_suction;
  double carry_speed;
  double width_between_finger_for_pinch;
  double width_between_finger_for_release;
  double finger_intrusion_for_pinch;
  double max_effort_for_pinch;
  
  geometry_msgs::Pose gp;
  geometry_msgs::Pose gap;
  geometry_msgs::Pose rp;
  geometry_msgs::Pose rap;
} GpRp_t;

#endif /* T2_TASK_STRATEGY_DEF_H */
