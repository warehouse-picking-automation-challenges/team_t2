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

#ifndef T2_MOTION_PLANNER_CAPABILITY_NAMES_H
#define T2_MOTION_PLANNER_CAPABILITY_NAMES_H

#include <string>

namespace t2_motion_planner
{
const std::string NODE_NAME = "t2_motion_planner";

const std::string ARM_EXECUTE_ACTION_NAME = "arm_execute";
const std::string ARM_MOVE_ACTION_NAME = "arm_move";
const std::string ARM_PLAN_ACTION_NAME = "arm_plan";

const std::string ARM_PICK_PLAN_ACTION_NAME = "arm_pick_plan";
const std::string ARM_PICK_EXECUTE_ACTION_NAME = "arm_pick_execute";

const std::string ARM_PLACE_MOVE_ACTION_NAME = "arm_place_move";

const std::string CLEAR_ARM_PLAN_SERVICE_NAME = "clear_arm_plan";

const std::string GET_ARM_POSE_SERVICE_NAME = "get_arm_pose";
const std::string GET_ARM_GROUP_STATE_POSE_SERVICE_NAME = "get_arm_group_state_pose";
const std::string SAVE_ARM_PLAN_SERVICE_NAME = "save_arm_plan";
const std::string SET_HAND_SUCTION_SERVICE_NAME = "set_hand_suction";
const std::string GET_PLANNED_ARM_TRAJECTRY_SERVICE_NAME = "get_planned_arm_trajectory";

const std::string FEEDBACK_EVENT_LEAVE_CONTAINER = "leave_container";

}  // namespace t2_motion_planner

#endif  // T2_MOTION_PLANNER_CAPABILITY_NAMES_H
