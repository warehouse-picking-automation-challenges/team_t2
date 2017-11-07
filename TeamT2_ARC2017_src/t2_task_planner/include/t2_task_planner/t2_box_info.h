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

#ifndef T2_BOX_INFO_H
#define T2_BOX_INFO_H

#include <ros/ros.h>
#include <ros/package.h>
#include <t2_msgs/ItemData.h>
#include <t2_msgs/GraspPoint.h>
#include <t2_task_planner/t2_task_planner_def.h>
#include <t2_task_planner/t2_task_strategy_def.h>
#include <map>
#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include <sys/stat.h>
#include <json/json.h>

typedef enum
{
  ITEM_RECOG_NOTYET = 0,
  ITEM_RECOG_OK,
  ITEM_PICK_NG,
  ITEM_PLACE_NG,
  ITEM_PICKED,
  ITEM_ORDER_DONE
} ItemStatus;

typedef enum
{
  ITEM_NORMAL,
  ITEM_MISRECOG,
  ITEM_UNKNOWN
} ItemRecogStatus;

typedef enum
{
  ORDER_NONE = 0,
  ORDER_DONE,
  ORDER_FAIL,
  ORDER_EXIST,
  ORDER_LATER,
  ORDER_MOVE
} OrderStatus;

typedef enum
{
  RECOG_NOTYET = 0,
  RECOG_OK
} RecogStatus;

typedef enum
{
  GRASP_NOTYET = 0,
  GRASP_OK,
  GRASP_NG
} GraspStatus;

enum PlanStatus
{
  PLAN_NOTYET,
  PLAN_OK,
  PLAN_NG
};

typedef std::vector<t2_msgs::GraspPoint> GraspPointArray;

typedef std::vector<GpRp_t> GpRpArray;
typedef std::vector<uint> GpNumberArray;

namespace BoxTypes
{
  enum BoxType
  {
    Bin,
    Box,
    Tote,
    AmnestyTote
  };
}

namespace MoveTypes
{
  enum MoveType
  {
    Between,
    Inside
  };
}

typedef struct
{
  bool enabled;
  uint box_id;
  uint camera_id;
  BoxTypes::BoxType type;
  double boxvolume;
  std::string box_name;
  double volume_of_items;
  RecogStatus recog_status;
  RecogStatus octomap_status;
  bool capture_item_allowed;
  bool capture_octomap_allowed;
  uint item_recog_jobno;
  uint octomap_recog_jobno;
  uint pre_item_recog_jobno;
  uint pre_octomap_recog_jobno;
  std::vector<OrderStatus> order_status;  // size = orderinfo.size()
  std::vector<std::string> item_name;
  std::vector<uint> cad_id;
  std::vector<double> volume;
  std::vector<uint> order_no;  // 0:no order, 1-3:order1-3,
  std::vector<ItemStatus> status;
  std::vector<ItemRecogStatus> item_recog_status;
  std::vector<t2_msgs::ItemData> item_data;
  std::vector<OrderStatus> item_order;
  std::vector<uint> recog_index;
  std::vector<GpRpArray> grasp_release_points;
  std::vector<uint> grasp_point_index;
  std::vector<GpNumberArray> failed_gp_numbers;
  PlanStatus plan_status;
  std::vector<PlanStatus> item_plan_status;
} Boxinfo_t;

typedef struct
{
  uint order_no;
  OrderStatus order_status;
  //std::vector<uint> laterbox_index;  // for move between bins
  std::vector<uint> from_box_index;
  std::vector<uint> item_index;
  std::vector<std::string> item_name;
  uint to_box_index;
  //	std::vector<uint>			to_box_index;
  std::vector<ItemStatus> status;
  std::vector<OrderStatus> item_order;
} Orderinfo_t;

typedef struct
{
  double x;
  double y;
  double z;
} Dimensions;

typedef struct
{
  std::vector<std::string> size_id;
  std::vector<Dimensions> dimensions;
  std::vector<double> volume;
} BoxSize_t;

struct PickOrderInfo
{
  uint order_no;
  uint content_index;
  int move_item_index;    // Bin間移動時の対象アイテム
  int to_box_index;       // Bin間移動時の移動先
  std::vector<uint> to_place_id;
  bool permit_protrusion; // Bin間/内移動時のはみ出し許可
  uint start_place_id;
  OrderStatus status;
  PlanStatus plan_status;
};

void initializeParam();
bool initializeBoxInfo(TaskType type);
void initializePickInfo();
void initializeStowInfo();

bool getItemInfoFromDatabase(const std::string &item_name, uint &cad_id, double &volume);
bool setItemInfo();
bool setBoxInfo();
bool setBoxSize();
bool setBoxSize(uint box_index, uint box_size_index);
void setBoxInfoToDatabase();
bool setPickInfo();
bool setItemOrderinfo(uint order_index, const std::string &item_name);

void resetRecognize(uint box_index);
void setRecogStatus(uint box_index, RecogStatus recog_status);
void setOctomapStatus(uint box_index, RecogStatus octomap_status);
void setGraspPointIndex(uint box_index, uint item_index, uint grasp_point_index);
void setPickPlanStatus(uint pick_index, PlanStatus plan_status);
void setPickMoveIndex(uint pick_index, uint move_item_index, uint to_box_index);
void setPickMoveInfo(uint pick_index, uint move_item_index, const std::vector<uint>& to_place_id);
void setStowPlanStatus(PlanStatus status);
void setStowItemIndex(uint item_index);
void setPickStartPlaceID(uint pick_index, uint start_place_id);
void setPickGoalPlaceID(uint start_place_id, uint goal_place_id);
void setStowStartPlaceID(uint start_place_id);
void setStowPreviousPlaceID();
void setStowPlaceMoveStatus(bool while_place_move);
bool whileStowPlaceMove();

uint getBinCount();
uint getCameraID(uint place_id);
std::string getItemName(uint cad_id);
double getItemVolume(uint cad_id);
double getGraspEasiness(uint cad_id);

std::vector<uint> getItemIndexList(uint box_index, ItemRecogStatus status, double threshold);
std::vector<uint> getRecognizedBinID();
std::vector<Location_t> getLocationList(const std::vector<uint>& place_id_list);
uint getPickItemIndex(uint place_id);
uint getStowItemIndex();
uint getPickStartPlaceID(uint pick_index);
uint getPickGoalPlaceID(uint place_id);
const std::vector<uint>& getPickGoalPlaceIDList(uint place_id);
uint getStowStartPlaceID();
uint getCurrentItemCount(uint box_index);
uint getTargetItemCount(uint place_id);
uint getNonTargetItemCount(uint place_id);
uint getOrderItemCount(uint place_id);
uint getPickDestinationPlaceID(uint from_place_id, uint item_index);
double getTotalTargetGraspEasiness(uint place_id);
void clearFailedGPList();
std::vector<geometry_msgs::Point> getFailedGPList();
bool permitProtrusion();
bool hasOrderItem(uint place_id);

void updateRecognize(uint box_index, const std::vector<t2_msgs::ItemData> &item_data);
void updateOctomapStatus(uint box_index);
void updateGraspAndRelease(uint box_index, uint item_index, const GpRpArray& array);

OrderStatus selectPickBox(uint pickorder_no, uint *box_index);
bool selectPickItem(uint pickorder_no, uint box_index, uint *item_index, uint *to_box_index);
bool selectMoveItem(Boxinfo_t& box_info, MoveTypes::MoveType move_type, uint& item_index);

// 2次試作用
bool getBoxIndex(uint place_id, uint* box_index);
bool getBoxInfo(uint place_id, Boxinfo_t** box_info);
bool getOrderInfo(uint order_no, Orderinfo_t** order_info);
bool setPickOrderInfo(uint pick_index = 0);
bool selectMovePlace(uint from_place_id, uint target_item_count_limit, uint& to_place_id);
OrderStatus selectPickInfo(uint& from_place_id, std::vector<uint>& to_place_id, uint& item_index);
OrderStatus selectMoveInfo(uint& from_place_id, std::vector<uint>& to_place_id, uint& item_index);
void resetTargetItemStatus(BoxTypes::BoxType type);
void resetNontargetItemStatus(BoxTypes::BoxType type);
void resetMovedItemInfo();

void updateBoxInfoByPick(uint box_index, uint item_index);
void updateBoxInfoByMove(uint from_index, uint item_index, uint to_index, TaskType type);
void updateOrderInfoByRecogError(uint place_id, uint item_index);
void updateOrderInfoByPlanError(uint place_id, uint item_index);
void updateStowInfoByPlanError(uint box_index, uint item_index, const geometry_msgs::Point& failed_gp_pos);

bool retryPickSameItem(uint place_id, uint item_index, uint pick_execute_attmepts);

bool isLater();

bool waitForRecognition(uint box_index, uint timeout);
bool waitForPickPlan(uint place_id, uint item_index);
bool waitForStowPlan();

void printBoxinfo(uint box_index);
void printAllBoxinfo();
void printOrderinfo(uint order_index);
void printAllOrderinfo();
void printPickOrder(int index);
void printAllPickOrder();

void writeItemLocationJson(const std::string &filename = "item_location_file.json");
std::string replaceAllString(std::string str, const std::string &from, const std::string &to);

#endif // T2_BOX_INFO_H
