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

#ifndef T2_PLANNING_SCENE_UPDATER_CAPABILITY_NAMES_H
#define T2_PLANNING_SCENE_UPDATER_CAPABILITY_NAMES_H

#include <string>

namespace t2_planning_scene_updater
{

const std::string NODE_NAME = "t2_planning_scene_updater";

const std::string ADD_COLLISION_OBJECT_SERVICE_NAME = "add_collision_object";
const std::string REMOVE_COLLISION_OBJECT_SERVICE_NAME = "remove_collision_object";
const std::string ATTACH_OBJECT_SERVICE_NAME = "attach_object";
const std::string DETACH_OBJECT_SERVICE_NAME = "detach_object";
const std::string SET_ACM_SERVICE_NAME = "set_acm";
const std::string ADD_ITEMS_SERVICE_NAME = "add_items";
const std::string SET_PICK_ITEM_SERVICE_NAME = "set_pick_item";
const std::string SET_CONTAINER_OBJECT_SERVICE_NAME = "set_container_object";
const std::string CLEAR_ATTACHED_ITEM_SERVICE_NAME = "clear_attached_item";
const std::string GET_ATTACHED_ITEM_SERVICE_NAME = "get_attached_item";

}  // namespace t2_planning_scene_updater

#endif  // T2_PLANNING_SCENE_UPDATER_CAPABILITY_NAMES_H
