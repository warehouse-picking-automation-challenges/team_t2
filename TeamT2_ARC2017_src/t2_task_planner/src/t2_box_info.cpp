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

#include <t2_task_planner/t2_box_info.h>

std::vector<Boxinfo_t> boxinfo;

std::vector<Orderinfo_t> orderinfo;
// 0-2:  pick workorder1, workorder2, workorder3

BoxSize_t boxsizeinfo;

int g_bin_count;
int g_box_count;

std::map<uint, uint> place_id_to_index_;

std::vector<PickOrderInfo> pick_order_;
static bool is_later_ = false;

// Move phase [0: between(without protrusion), 1: inside, 2: between(with protrusion)]
static int move_phase_ = 0;
static int target_item_count_limit_ = 0;

static std::mutex box_info_mutex_;
static std::mutex failed_gp_mutex_;

struct PickInfo
{
  uint item_index;
  uint to_place_id;
  std::vector<uint> to_place_id_list;
};
std::map<uint, PickInfo> pick_info_;

struct StowInfo
{
  uint item_index;
  uint start_place_id;
  uint previous_place_id;
  bool while_place_move;
  std::vector<geometry_msgs::Point> failed_gp_list;
};
StowInfo stow_info_;

// Bin間移動済みアイテム情報
struct BoxItemInfo
{
  uint place_id;
  uint item_index;
};
std::vector<BoxItemInfo> moved_between_item_info_;
std::vector<BoxItemInfo> moved_inside_item_info_;

// Bin間移動先およびBin内移動元の対象外place_idリスト
std::vector<int> move_excluded_place_id_list_;

void initializeParam()
{
  move_phase_ = 0;
  target_item_count_limit_ = 0;
}

bool initializeBoxInfo(TaskType type)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  boxinfo.clear();
  g_bin_count = 0;
  g_box_count = 0;

  const std::string item_location_path = ORDER_INPUT_PATH + "/item_location";
  XmlRpc::XmlRpcValue item_location_value;
  if (!pnh.getParam(item_location_path, item_location_value))
  {
    ROS_ERROR("Failed to get item_location param.");
    return false;
  }

  int boxinfo_size = 0;
  XmlRpc::XmlRpcValue bins_value;
  for (auto it = item_location_value.begin(); it != item_location_value.end(); ++it)
  {
    std::string name = static_cast<std::string>(it->first);
    if (name == "bins")
    {
      bins_value = it->second;
      g_bin_count = it->second.size();
      boxinfo_size += g_bin_count;
    }
    else if (name == "boxes")
    {
      g_box_count = it->second.size();
      boxinfo_size += g_box_count;
    }
    else
    { /* tote */
      boxinfo_size += 2;
    }
  }

  // PickTaskでitem_location_file.jsonのboxが空だったとき、box_settingsからboxの個数を取得する
  if (type == TaskType::Pick && g_box_count == 0)
  {
    XmlRpc::XmlRpcValue box_settings;
    if (!pnh.getParam(BOX_SETTINGS_PATH, box_settings))
    {
      ROS_ERROR("Failed to get box_settings param.");
      return false;
    }
    g_box_count = box_settings.size();
    boxinfo_size += g_box_count;
  }

  ROS_ASSERT(bins_value.getType() == XmlRpc::XmlRpcValue::TypeArray);

  int box_count = 0;
  // Set id and box_name
  for (int i = 0; i < boxinfo_size; i++)
  {
    Boxinfo_t boxinfo_item { };
    boxinfo_item.enabled = false;
    boxinfo_item.box_id = i + 1;
    boxinfo_item.capture_item_allowed = true;
    boxinfo_item.capture_octomap_allowed = true;

    std::string box_name;
    if (i < g_bin_count)
    {
      box_name = static_cast<std::string>(bins_value[i]["bin_id"]);
      boxinfo_item.box_name = box_name;
      boxinfo_item.type = BoxTypes::Bin;
    }
    else if (i == g_bin_count)
    {
      boxinfo_item.box_name = "TOTE1";
      boxinfo_item.type = BoxTypes::Tote;
    }
    else if (i == g_bin_count + 1)
    {
      boxinfo_item.box_name = "TOTE2";
      boxinfo_item.type = BoxTypes::AmnestyTote;
    }
    else
    {
      std::ostringstream oss;
      oss << ++box_count;
      std::string name = "BOX" + oss.str();
      boxinfo_item.box_name = name;
      boxinfo_item.type = BoxTypes::Box;
    }

    boxinfo.push_back(boxinfo_item);
  }

  std::string containter_info_path = "container_info";
  XmlRpc::XmlRpcValue container_info_array;
  if (!nh.getParam(CONTAINER_INFO_PATH, container_info_array))
  {
    ROS_ERROR("getParam() error: %s", containter_info_path.c_str());
    return false;
  }
  ROS_ASSERT(container_info_array.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int i = 0; i < boxinfo.size(); ++i)
  {
    for (int j = 0; j < container_info_array.size(); ++j)
    {
      ROS_ASSERT(container_info_array[j]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
      std::string container_name = static_cast<std::string>(container_info_array[j]["name"]);
      if (container_name == boxinfo[i].box_name)
      {
        boxinfo[i].enabled = true;
        boxinfo[i].box_id = static_cast<int>(container_info_array[j]["place_id"]);
        boxinfo[i].camera_id = static_cast<int>(container_info_array[j]["camera_id"]);
        boxinfo[i].boxvolume = static_cast<double>(container_info_array[j]["volume"]);

        std::string box_type = static_cast<std::string>(container_info_array[j]["type"]);
        if (box_type == "bin")
        {
          boxinfo[i].type = BoxTypes::Bin;
        }
        else if (box_type == "box")
        {
          boxinfo[i].type = BoxTypes::Box;
        }
        else if (box_type == "tote")
        {
          boxinfo[i].type = BoxTypes::Tote;
        }
        else if (box_type == "amnesty_tote")
        {
          boxinfo[i].type = BoxTypes::AmnestyTote;
        }
        ROS_INFO("boxinfo[%d]: name=%s, place_id=%u, volume=%lf", i, boxinfo[i].box_name.c_str(), boxinfo[i].box_id, boxinfo[i].boxvolume);
        break;
      }
    }
    if (boxinfo[i].enabled)
    {
      place_id_to_index_[boxinfo[i].box_id] = i;
    }
  }
  return true;
}

void initializePickInfo()
{
  pick_info_.clear();
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    const Boxinfo_t& box_info = boxinfo[i];
    if (!box_info.enabled || BoxTypes::Bin != box_info.type)
    {
      continue;
    }

    PickInfo pick_info = {};
    pick_info_[box_info.box_id] = pick_info;
  }
}

void initializeStowInfo()
{
  stow_info_.item_index = 0;
  stow_info_.start_place_id = 0;
  stow_info_.previous_place_id = 0;
  stow_info_.while_place_move = false;

  // Toteの全アイテムをオーダーありに設定
  Boxinfo_t *tote;
  getBoxInfo(D_tote_1_ID, &tote);
  for (std::size_t i = 0; i < tote->item_order.size(); ++i)
  {
    tote->item_order[i] = ORDER_EXIST;
  }
}

bool getItemInfoFromDatabase(const std::string &item_name, uint &cad_id, double &volume)
{
  ros::NodeHandle nh;

  XmlRpc::XmlRpcValue grasp_info;

  if (!nh.getParam("/t2_database/grasp_info_list", grasp_info))
  {
    ROS_ERROR("Failed to get grasp_info");
    return false;
  }

  ROS_ASSERT(grasp_info.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  for (auto it = grasp_info.begin(); it != grasp_info.end(); ++it)
  {
    if (item_name == static_cast<std::string>(grasp_info[it->first]["item_name"]))
    {
      cad_id = (uint)std::stoi((static_cast<std::string>(it->first)).substr(std::string("cad").size()));
      volume = static_cast<double>(grasp_info[it->first]["volume_of_item"]);
      return true;
    }
  }

  ROS_ERROR("Can't find item_name in database. item_name = %s", item_name.c_str());

  return false;
}

bool setItemsToBox(Boxinfo_t* box, const std::vector<std::string>& item_names)
{
  int item_count = item_names.size();
  if (item_count > 0)
  {
    box->cad_id.resize(item_count);
    box->volume.resize(item_count);
    box->order_no.assign(item_count, 0);
    box->status.assign(item_count, ITEM_RECOG_NOTYET);
    box->item_recog_status.assign(item_count, ITEM_NORMAL);
    box->item_data.resize(item_count);
    box->recog_index.resize(item_count);
    box->grasp_release_points.resize(item_count);
    box->grasp_point_index.resize(item_count);
    box->failed_gp_numbers.resize(item_count);
    box->item_name.resize(item_count);
    box->item_order.assign(item_count, ORDER_NONE);
    box->item_plan_status.assign(item_count, PLAN_NOTYET);

    for (int i = 0; i < item_count; ++i)
    {
      std::string item_name = item_names[i];
      box->item_name[i] = item_name;

      uint cad_id;
      double volume;
      if (getItemInfoFromDatabase(item_name, cad_id, volume))
      {
        box->cad_id[i] = cad_id;
        box->volume[i] = volume;
        box->volume_of_items += volume;
      }
      else
      {
        ROS_ERROR("Error getItemInfoFromDatabase()");
        return false;
      }
    }
  }
  return true;
}

// Set itemlist to boxinfo
bool setItemInfo()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  uint i, j, k, m, itemcount;
  XmlRpc::XmlRpcValue contentslist;
  XmlRpc::XmlRpcValue boxsize;
  uint box_index = 0, to_box_index = 0;

  ROS_INFO("setItemInfo() start");

  const std::string bins_path = ORDER_INPUT_PATH + "/item_location/bins";

  // item_location

  if (!private_nh.getParam(bins_path, contentslist))
  {
    ROS_ERROR("Failed to get param. %s", (bins_path).c_str());
    return false;
  }

  ROS_ASSERT(contentslist.size() == g_bin_count);

  for (j = 0; j < g_bin_count; j++)
  {
    ROS_ASSERT(contentslist[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue& contents_array = contentslist[j]["contents"];
    std::vector<std::string> item_names;
    for (int i = 0; i < contents_array.size(); ++i)
    {
      item_names.push_back(static_cast<std::string>(contents_array[i]));
    }
    setItemsToBox(&boxinfo[j], item_names);
  }

  const std::string tote_path = ORDER_INPUT_PATH + "/item_location/tote";
  if (!private_nh.getParam(tote_path, contentslist))
  {
    ROS_ERROR("Failed to get param. %s", tote_path.c_str());
    return false;
  }
  std::vector<std::string> item_names;
  for (int i = 0; i < contentslist["contents"].size(); ++i)
  {
    item_names.push_back(static_cast<std::string>(contentslist["contents"][i]));
  }
  setItemsToBox(&boxinfo[g_bin_count], item_names);

  printAllBoxinfo();
  return true;
}

bool setBoxInfo()
{
  ros::NodeHandle private_nh("~");
  uint i, j, k, m, itemcount;
  XmlRpc::XmlRpcValue contentslist;
  XmlRpc::XmlRpcValue boxsize;
  // uint bincount = 0;
  uint box_index = 0, to_box_index = 0;
  XmlRpc::XmlRpcValue::ValueStruct::const_iterator it;

  ROS_INFO("setBoxInfo() start");

  const std::string box_sizes_path = ORDER_INPUT_PATH + "/box_sizes/boxes";

  // boxsize
  if (!private_nh.getParam(box_sizes_path, boxsize))
  {
    ROS_ERROR("Failed to get param. box_sizes");
    return false;
  }
  ROS_ASSERT(boxsize.getType() == XmlRpc::XmlRpcValue::TypeArray);

  boxsizeinfo.size_id.clear();
  boxsizeinfo.volume.clear();
  for (i = 0; i < boxsize.size(); i++)
  {
    ROS_ASSERT(boxsize[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (it = boxsize[i].begin(); it != boxsize[i].end(); ++it)
    {
      if (it->first == "dimensions")
      {  // bins
        ROS_ASSERT(boxsize[i][it->first].getType() == XmlRpc::XmlRpcValue::TypeArray);
        XmlRpc::XmlRpcValue &dimensions_value = boxsize[i][it->first];

        Dimensions dimensions;
        dimensions.x = static_cast<double>(dimensions_value[0]);
        dimensions.y = static_cast<double>(dimensions_value[1]);
        dimensions.z = static_cast<double>(dimensions_value[2]);

        boxsizeinfo.dimensions.push_back(dimensions);
        boxsizeinfo.volume.push_back(dimensions.x * dimensions.y * dimensions.z);
      }
      else if (it->first == "size_id")
      {  // bins
        ROS_ASSERT(boxsize[i][it->first].getType() == XmlRpc::XmlRpcValue::TypeString);
        boxsizeinfo.size_id.push_back((std::string)boxsize[i][it->first]);
      }
    }
  }
  return true;
}

bool setBoxSize()
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue box_settings_array;
  if (!nh.getParam(BOX_SETTINGS_PATH, box_settings_array))
  {
    ROS_ERROR("getParam() error. %s", BOX_SETTINGS_PATH.c_str());
    return false;
  }

  for (int i = 0; i < box_settings_array.size(); ++i)
  {
    bool exists = false;
    std::string name = static_cast<std::string>(box_settings_array[i]["name"]);
    std::string size_id = static_cast<std::string>(box_settings_array[i]["size_id"]);
    for (int j = 0; j < boxinfo.size(); ++j)
    {
      if (name == boxinfo[j].box_name)
      {
        for (int k = 0; k < boxsizeinfo.size_id.size(); ++k)
        {
          if (size_id == boxsizeinfo.size_id[k])
          {
            boxinfo[j].box_name = size_id;
            boxinfo[j].boxvolume = boxsizeinfo.volume[k];
            boxinfo[j].enabled = true;
            exists = true;
            break;
          }
        }
        break;
      }
    }
    if (!exists)
    {
      ROS_ERROR("setBoxSize() error. name=%s, size_id=%s", name.c_str(), size_id.c_str());
      return false;
    }
  }
  return true;
}

bool setBoxSize(uint box_index, uint box_size_index)
{
  if (box_size_index >= boxsizeinfo.size_id.size())
  {
    return false;
  }

  int box_count = 0;
  for (int i = 0; i < boxinfo.size(); ++i)
  {
    if (BoxTypes::Box == boxinfo[i].type)
    {
      if (box_index == box_count)
      {
        boxinfo[i].boxvolume = boxsizeinfo.volume[box_size_index];
        boxinfo[i].box_name = boxsizeinfo.size_id[box_size_index];
        boxinfo[i].enabled = true;
      }
      box_count++;
    }
  }

  if (box_index >= box_count)
  {
    return false;
  }
  return true;
}

void setBoxInfoToDatabase()
{
  ROS_INFO("setBoxInfoToDatabase() start");
  ros::NodeHandle nh;

  int id = 1;

  XmlRpc::XmlRpcValue container_info_array;
  if (!nh.getParam(CONTAINER_INFO_PATH, container_info_array))
  {
    ROS_ERROR("getParam() error %s", CONTAINER_INFO_PATH.c_str());
    return;
  }

  XmlRpc::XmlRpcValue box_settings_array;
  if (!nh.getParam(BOX_SETTINGS_PATH, box_settings_array))
  {
    ROS_ERROR("getParam() error %s", BOX_SETTINGS_PATH.c_str());
    return;
  }

  for (int i = 0; i < boxinfo.size(); ++i)
  {
    if (BoxTypes::Box != boxinfo[i].type)
    {
      continue;
    }
    std::string name = "BOX";
    std::ostringstream oss;
    oss << id;
    name.append(oss.str());

    for (int j = 0; j < container_info_array.size(); ++j)
    {
      XmlRpc::XmlRpcValue& value = container_info_array[j];
      std::string container_name = static_cast<std::string>(value["name"]);
      if (container_name == name)
      {
        Dimensions dim;
        double volume;
        for (int k = 0; k < boxsizeinfo.size_id.size(); ++k)
        {
          if (boxinfo[i].box_name == boxsizeinfo.size_id[k])
          {
            dim = boxsizeinfo.dimensions[k];
            volume = boxsizeinfo.volume[k];
            break;
          }
        }

        std::string direction;
        bool fold = true;
        for (int k = 0; k < box_settings_array.size(); ++k)
        {
          std::string box_name = static_cast<std::string>(box_settings_array[k]["name"]);
          if (box_name == name)
          {
            direction = static_cast<std::string>(box_settings_array[k]["direction"]);
            fold = static_cast<bool>(box_settings_array[k]["fold"]);
          }
        }
        if (direction == "y")
        {
          double tmp = dim.x;
          dim.x = dim.y;
          dim.y = tmp;
        }

        if (fold)
        {
          double pos_x = static_cast<double>(value["position"][0]);
          double pos_z = static_cast<double>(value["position"][2]);
          double inside_pos_x = static_cast<double>(value["inside_position"][0]);
          double inside_pos_z = static_cast<double>(value["inside_position"][2]);
          double box_bottom_thickness = std::abs(pos_z - inside_pos_z);
          double box_side_thickness = std::abs(pos_x - inside_pos_x);
          double box_height = dim.z + box_bottom_thickness;
          double box_fold_up = ((direction == "y") ? dim.x : dim.y) / 2.0 + box_side_thickness - box_height;

          if (box_fold_up > 0)
          {
            value["inside_position"][2] = XmlRpc::XmlRpcValue(inside_pos_z + box_fold_up);
          }
        }

        XmlRpc::XmlRpcValue dim_param;
        dim_param.setSize(3);
        dim_param[0] = XmlRpc::XmlRpcValue(dim.x);
        dim_param[1] = XmlRpc::XmlRpcValue(dim.y);
        dim_param[2] = XmlRpc::XmlRpcValue(dim.z);

        value["inside_dimensions"] = dim_param;
        value["volume"] = volume;
      }
    }
    id++;
  }
  nh.setParam(CONTAINER_INFO_PATH, container_info_array);
  ROS_INFO("setBoxInfoToDatabase() end");
}

// set pickorder info
bool setPickInfo()
{
  ros::NodeHandle private_nh("~");
  uint size, order_no;
  XmlRpc::XmlRpcValue picklist;
  uint ordercount = 0;
  uint box_index = 0, to_box_index = 0;
  XmlRpc::XmlRpcValue::ValueStruct::const_iterator it;

  ROS_INFO("setPickInfo() start");

  memset(&picklist, 0, sizeof(picklist));
  orderinfo.clear();

  const std::string order_pick_path = ORDER_INPUT_PATH + "/order_pick/orders";

  if (!private_nh.getParam(order_pick_path, picklist))
  {
    ROS_ERROR("Failed to get param. %s", (order_pick_path).c_str());
    return false;
  }

  ordercount = picklist.size();
  orderinfo.resize(ordercount);

  // initialize boxinfo[i].order_status
  for (int i = 0; i < boxinfo.size(); i++)
  {
    boxinfo[i].order_status.resize(ordercount, ORDER_NONE);
  }
  ROS_INFO("boxinfo order_status resize.");

  for (int i = 0; i < ordercount; i++)
  {
    orderinfo[i].order_no = (uint)(i + 1);
    orderinfo[i].order_status = ORDER_NONE;

    orderinfo[i].from_box_index.clear();
    orderinfo[i].item_index.clear();
    orderinfo[i].item_name.clear();
    orderinfo[i].status.clear();
    orderinfo[i].item_order.clear();

    ROS_ASSERT(picklist[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto it = picklist[i].begin(); it != picklist[i].end(); ++it)
    {
      if (it->first == "contents")
      {
        XmlRpc::XmlRpcValue &contents = picklist[i][it->first];
        for (int m = 0; m < contents.size(); m++)
        {
          ROS_ASSERT(contents[m].getType() == XmlRpc::XmlRpcValue::TypeString);
          std::string item_name = static_cast<std::string>(contents[m]);
          if (!setItemOrderinfo(i, item_name))
          {
            ROS_WARN("setItemOrderInfo(order_index=%d, item_name=%s) failed.", i, item_name.c_str());
          }
        }
      }
      else if (it->first == "size_id")
      {
        bool size_id_exists = false;
        for (int j = 0; j < boxinfo.size(); j++)
        {
          Boxinfo_t& box_info = boxinfo[j];
          if (!box_info.enabled || BoxTypes::Box != box_info.type)
          {
            continue;
          }
          if (box_info.box_name == (std::string)picklist[i][it->first])
          {
            bool order_exists = false;
            for (int k = 0; k < i; ++k)
            {
              if (j == orderinfo[k].to_box_index)
              {
                order_exists = true;
                break;
              }
            }
            if (!order_exists)
            {
              orderinfo[i].to_box_index = j;
              size_id_exists = true;
              break;
            }
          }
        }
        if (!size_id_exists)
        {
          return false;
        }
      }
    }
  }

  printAllBoxinfo();
  //printAllPickOrder();
  return true;
}

bool setItemOrderinfo(uint order_index, const std::string &item_name)
{
  ROS_ASSERT(order_index < orderinfo.size());

  Orderinfo_t& order_info = orderinfo[order_index];

  bool order_exist = false;
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    Boxinfo_t& box_info = boxinfo[i];
    if (!box_info.enabled || BoxTypes::Bin != box_info.type)
    {
      continue;
    }

    for (std::size_t j = 0; j < box_info.item_name.size(); ++j)
    {
      if (item_name == box_info.item_name[j])
      {
        order_exist = true;
        box_info.item_order[j] = ORDER_EXIST;
      }
    }
  }
  if (order_exist)
  {
      order_info.order_status = ORDER_EXIST;
      order_info.item_name.push_back(item_name);
      order_info.status.push_back(ITEM_RECOG_NOTYET);
      order_info.item_order.push_back(ORDER_EXIST);
      return true;
  }
  return false;
}

bool getBoxInfo(uint place_id, Boxinfo_t** box_info)
{
  for (int i = 0; i < boxinfo.size(); i++)
  {
    if (boxinfo[i].box_id == place_id)
    {
      *box_info = &boxinfo[i];
      return true;
    }
  }
  return false;
}

bool getBoxIndex(uint place_id, uint* box_index)
{
  for (uint i = 0; i < boxinfo.size(); i++)
  {
    if (boxinfo[i].box_id == place_id)
    {
      *box_index = i;
      return true;
    }
  }
  return false;
}

bool getBoxIndex(const std::string& box_name, uint* box_index)
{
  for (uint i = 0; i < boxinfo.size(); i++)
  {
    if (boxinfo[i].box_name == box_name)
    {
      *box_index = i;
      return true;
    }
  }
  return false;
}

bool getOrderInfo(uint order_no, Orderinfo_t** order_info)
{
  for (int i = 0; i < orderinfo.size(); i++)
  {
    if (orderinfo[i].order_no == order_no)
    {
      *order_info = &orderinfo[i];
      return true;
    }
  }
  return false;
}

bool initPickOrderInfo()
{
  // Connect to Parameter Server
  ros::NodeHandle nh;
  std::string order_pick_path = ORDER_INPUT_PATH + "/order_pick/orders";
  XmlRpc::XmlRpcValue orders_value;
  if (!nh.getParam(order_pick_path, orders_value))
  {
    ROS_ERROR("Failed to get param. %s", (order_pick_path).c_str());
    return false;
  }

  int order_count = orders_value.size();
  ROS_ASSERT(orders_value.getType() == XmlRpc::XmlRpcValue::TypeArray);

  orderinfo.clear();

  for (int i = 0; i < order_count; ++i)
  {
    XmlRpc::XmlRpcValue &order_value = orders_value[i];

    ROS_ASSERT(order_value.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if (order_value.hasMember("size_id") && order_value.hasMember("contents"))
    {
      Orderinfo_t order_info;
      order_info.order_no = i + 1;

      std::string size_id = static_cast<std::string>(order_value["size_id"]);
      uint box_index;
      if (!getBoxIndex(size_id, &box_index))
      {
        ROS_ERROR("No size_id %s", size_id.c_str());
        return false;
      }
      order_info.to_box_index = box_index;

      XmlRpc::XmlRpcValue &contents_value = order_value["contents"];
      int contents_size = contents_value.size();

      if (contents_size > 0)
      {
        order_info.order_status = ORDER_EXIST;
        order_info.item_index.assign(contents_size, 0);
        order_info.from_box_index.assign(contents_size, 0);
        order_info.status.assign(contents_size, ITEM_RECOG_NOTYET);
        order_info.item_order.assign(contents_size, ORDER_NONE);
        for (int j = 0; j < contents_size; ++j)
        {
          order_info.item_name.push_back(static_cast<std::string>(contents_value[j]));
        }
      }
      else
      {
        order_info.order_status = ORDER_NONE;
      }
      orderinfo.push_back(order_info);
    }
    else
    {
      ROS_ERROR("Invaild param %s", order_pick_path.c_str());
      orderinfo.clear();
      return false;
    }
  }
  return true;
}

// 有効なターゲットアイテムを含むアイテム数が最小のBinを見つける
uint countValidTargetItem(const std::string& item_name, uint* place_id, uint* item_index)
{
  int min_total_count = -1;
  int min_place_id = -1;
  int min_item_index = -1;
  for (int i = 0; i < boxinfo.size(); i++)
  {
    // Bin only
    if (!boxinfo[i].enabled || BoxTypes::Bin != boxinfo[i].type)
    {
      continue;
    }
    int total_count = 0;
    int target_count = 0;
    int target_item_index = -1;
    for (int j = 0; j < boxinfo[i].item_name.size(); j++)
    {
      if (ITEM_PICKED == boxinfo[i].status[j])
      {
        continue;   // アイテムがPick済の場合は除外する
      }
      total_count++;
      if ((boxinfo[i].item_name[j] == item_name) && (ORDER_NONE == boxinfo[i].item_order[j]))
      {
        // オーダーが決まっていないターゲットアイテムがある場合
        target_count++;
        target_item_index = j;
      }
    }

    // 有効なターゲットアイテムが存在する場合
    if (target_count > 0)
    {
      // アイテム総数が最小のBinを選択する
      if (min_total_count < 0)
      {
        min_total_count = total_count;
        min_place_id = boxinfo[i].box_id;
        min_item_index = target_item_index;
      }
      else if (total_count < min_total_count)
      {
        min_total_count = total_count;
        min_place_id = boxinfo[i].box_id;
        min_item_index = target_item_index;
      }
    }
  }

  if (min_total_count < 0)
  {
    return 0;
  }
  *place_id = min_place_id;
  *item_index = min_item_index;
  return min_total_count;
}

bool setPickBoxItemInfo(uint pick_index, const std::vector<int>& max_order_index, uint* order_index)
{
  ROS_INFO("setPickBoxItemInfo(pick_index=%u max_order_index.size()=%lu", pick_index, max_order_index.size());

  Orderinfo_t* prev_order_info = NULL;
  if (pick_index > 0)
  {
    uint prev_order_no = pick_order_[pick_index - 1].order_no;
    if (!getOrderInfo(prev_order_no, &prev_order_info))
    {
      ROS_ERROR("OrderInfo have no order_no %u", prev_order_no);
      return false;
    }
  }

  // ターゲットアイテムのうちどのアイテムをオーダーするかを決定する
  int min_count = -1;
  int min_place_id = -1;
  int min_item_index = -1;
  int min_order_index = -1;
  int min_order_item_index = -1;

  int tmp_count = -1;
  int tmp_place_id = -1;
  int tmp_item_index = -1;
  int tmp_order_index = -1;
  int tmp_order_item_index = -1;

  for (int i : max_order_index)
  {
    Orderinfo_t* order_info = &orderinfo[i];

    for (int j = 0; j < order_info->item_name.size(); j++)
    {
      if (ORDER_NONE != order_info->item_order[j])
      {
        continue;   // オーダー済み
      }
      std::string item_name = order_info->item_name[j];

      uint place_id = 0;
      uint item_index = 0;
      uint count = countValidTargetItem(item_name, &place_id, &item_index);

      if (count > 0)
      {
        if (pick_index > 0)   // 2回目以降
        {
          uint prev_order_item_index = pick_order_[pick_index - 1].content_index;
          uint prev_box_index = prev_order_info->from_box_index[prev_order_item_index];
          uint prev_place_id = boxinfo[prev_box_index].box_id;

          // 前回と同じBinの場合、一時保存
          if (prev_place_id == place_id)
          {
            // 最小のアイテム数を保存
            if (tmp_count < 0 || count < tmp_count)
            {
              tmp_count = count;
              tmp_place_id = place_id;
              tmp_item_index = item_index;
              tmp_order_index = i;
              tmp_order_item_index = j;
            }
            continue;
          }
        }

        if (min_count < 0 || count < min_count)
        {
          min_count = count;
          min_place_id = place_id;
          min_item_index = item_index;
          min_order_index = i;
          min_order_item_index = j;
        }
      }
      else
      {
        // オーダーがあってもBinに有効なアイテムがない場合はオーダー失敗にする
        order_info->item_order[j] = ORDER_FAIL;
      }
    }
  }

  if (min_count < 0)
  {
    if (tmp_count > 0)
    {
      ROS_INFO("Debug comment");
      // 前回と同じBinにアイテムがある場合
      // min_count = tmp_count;
      min_place_id = tmp_place_id;
      min_item_index = tmp_item_index;
      min_order_index = tmp_order_index;
      min_order_item_index = tmp_order_item_index;
    }
    else
    {
      // Binにアイテムがない場合
      return false;
    }
  }

  // return
  *order_index = min_order_index;

  // Add PickOrderInfo
  PickOrderInfo pick_order_info;
  pick_order_info.order_no = orderinfo[min_order_index].order_no;
  pick_order_info.content_index = min_order_item_index;
  pick_order_info.start_place_id = 0;
  pick_order_info.status = ORDER_EXIST;
  pick_order_info.plan_status = PLAN_NOTYET;
  pick_order_info.move_item_index = min_item_index;
  pick_order_.push_back(pick_order_info);

  // Set OrderInfo
  uint box_index = boxinfo.size();
  for (int i = 0; i < boxinfo.size(); ++i)
  {
    if (boxinfo[i].box_id == min_place_id)
    {
      box_index = i;
      break;
    }
  }
  if (box_index == boxinfo.size())
  {
    ROS_ERROR("No place_id=%d", min_place_id);
  }
  orderinfo[min_order_index].item_index[min_order_item_index] = min_item_index;
  orderinfo[min_order_index].from_box_index[min_order_item_index] = box_index;
  orderinfo[min_order_index].item_order[min_order_item_index] = ORDER_EXIST;

  // Set BoxInfo
  Boxinfo_t* box_info;
  if (!getBoxInfo(min_place_id, &box_info))
  {
    ROS_ERROR("getBoxInfo(place_id=%u) error", min_place_id);
    return false;
  }
  box_info->item_order[min_item_index] = ORDER_EXIST;

  return true;
}

bool setPickOrderInfo(uint pick_index)
{
  ROS_INFO("setPickOrderInfo(pick_index=%u) start", pick_index);

  if (pick_index == 0)
  {
    if (!initPickOrderInfo())
    {
      ROS_ERROR("initPickOrderInfo() failed");
      return false;
    }
    is_later_ = false;
    pick_order_.clear();
  }
  else
  {
    for (int i = pick_index; i < pick_order_.size(); ++i)
    {
      for (Orderinfo_t& order : orderinfo)
      {
        if (order.order_no == pick_order_[i].order_no)
        {
          uint box_index = order.from_box_index[pick_order_[i].content_index];
          uint item_index = order.item_index[pick_order_[i].content_index];
          boxinfo[box_index].item_order[item_index] = ORDER_NONE;
          order.item_order[pick_order_[i].content_index] = ORDER_NONE;
        }
      }
    }

    pick_order_.resize(pick_index);
  }

  std::vector<int> valid_count;     // valid item count for each box
  valid_count.assign(orderinfo.size(), 0);
  for (int i = 0; i < orderinfo.size(); i++)
  {
    for (int j = 0; j < orderinfo[i].item_name.size(); j++)
    {
      if (orderinfo[i].item_order[j] == ORDER_NONE)
      {
        valid_count[i]++;
      }
    }
  }

  int prev_order_index = -1;
  if (pick_index > 0)
  {
    for (int i = 0; i < orderinfo.size(); ++i)
    {
      if (pick_order_[pick_index - 1].order_no == orderinfo[i].order_no)
      {
        prev_order_index = i;
      }
    }
  }

  bool contents_exist = true;
  while (contents_exist)
  {
    int pick_index = pick_order_.size();
    contents_exist = false;

    // オーダーの残りアイテム数が最大のBoxを選ぶ
    std::vector<int> max_order_index;

    int max_index = -1;
    int max_contents_size = 0;
    for (int i = 0; i < orderinfo.size(); i++)
    {
      if (valid_count[i] > 0)
      {
        contents_exist = true;
      }
      else
      {
        continue;
      }
      if (i == prev_order_index)
      {
        continue;
      }
      if (valid_count[i] > max_contents_size)
      {
        max_order_index.assign(1, i);
        max_contents_size = valid_count[i];
      }
      else if (valid_count[i] == max_contents_size)
      {
        max_order_index.push_back(i);
      }
    }
    if (contents_exist)
    {
      if (max_order_index.empty())
      {
        max_order_index.assign(1, prev_order_index);
      }

      uint target_order_index;
      if (!setPickBoxItemInfo(pick_index, max_order_index, &target_order_index))
      {
        ROS_WARN("No valid order item");

        // Update valid item count for each box
        valid_count.assign(orderinfo.size(), 0);
        for (int j = 0; j < orderinfo.size(); j++)
        {
          for (int k = 0; k < orderinfo[j].item_name.size(); k++)
          {
            if (orderinfo[j].item_order[k] == ORDER_NONE)
            {
              valid_count[j]++;
            }
          }
        }
      }
      else
      {
        valid_count[target_order_index]--;
        prev_order_index = target_order_index;
      }
    }
  }
  ROS_INFO("Pick order size is %lu", pick_order_.size());

  ROS_INFO("setPickOrderInfo() end");
  return true;
}

double getBoxDensity(uint place_id)
{
  for (int i = 0; i < boxinfo.size(); i++)
  {
    if (place_id == boxinfo[i].box_id)
    {
      double total_volume = 0;
      for (int j = 0; j < boxinfo[i].item_name.size(); j++)
      {
        if (boxinfo[i].status[j] != ITEM_PICKED)
        {
          total_volume += boxinfo[i].volume[j];
        }
      }
      return total_volume / boxinfo[i].boxvolume;
    }
  }
  return -1;
}

bool selectMoveItemInfo(uint pick_order_index, uint from_place_id, uint* move_place_id, uint* move_item_index)
{
  ROS_INFO("selectMoveItemInfo(pick_order_index=%u) start", pick_order_index);
  Boxinfo_t* box_info;
  getBoxInfo(from_place_id, &box_info);

  // Select item to move
  int target_item_index = -1;
  double target_probability = -1;
  for (int i = 0; i < box_info->item_name.size(); i++)
  {
    // 非ターゲットアイテムのみ
    if (box_info->item_order[i] != ORDER_NONE)
    {
      continue;
    }

    // misrecognition/unknown以外のアイテム
    if (box_info->status[i] != ITEM_RECOG_OK)
    {
      continue;
    }

    // 認識スコア最大
    double probability = box_info->item_data[i].probability;
    if (probability > target_probability)
    {
      target_item_index = i;
      target_probability = probability;
    }
    else if (probability == target_probability)
    {
      // 図心のz座標が大きい
      double position_z = box_info->item_data[i].pose.position.z;
      double target_position_z = box_info->item_data[target_item_index].pose.position.z;
      if (position_z > target_position_z)
      {
        target_item_index = i;
      }
      else if (position_z == target_position_z)
      {
        // 体積が小さい
        double volume = box_info->volume[i];
        double target_volume = box_info->volume[target_item_index];
        if (volume < target_volume)
        {
          target_item_index = i;
        }
      }
    }
  }

  // 対象アイテムが存在しない
  if (target_item_index < 0)
  {
    ROS_INFO("cannot find move item (pick_order[%d])", pick_order_index);
    return false;
  }

  // Select target bin
  int target_place_id = -1;
  int min_target_item_count = -1;
  for (int i = 0; i < boxinfo.size(); i++)
  {
    if (BoxTypes::Bin != boxinfo[i].type || !boxinfo[i].enabled)
    {
      continue;
    }

    // 移動元のBin以外
    if (boxinfo[i].box_id == from_place_id)
    {
      continue;
    }

    // Occupancy認識済み
    if (RECOG_OK != boxinfo[i].octomap_status)
    {
      continue;
    }

    uint place_id = boxinfo[i].box_id;
    uint target_item_count = getTargetItemCount(place_id);

    if (target_place_id < 0)
    {
      target_place_id = place_id;
      min_target_item_count = target_item_count;
    }
    else if (target_item_count < min_target_item_count)
    {
      target_place_id = place_id;
      min_target_item_count = target_item_count;
    }
  }

  if (target_place_id < 0)
  {
    ROS_INFO("cannot find place to move (pick_order[%u])", pick_order_index);
    return false;
  }

  ROS_INFO("move item[%d] in place_id=%u to place_id=%u (pick_order[%u])",
        target_item_index, from_place_id, target_place_id, pick_order_index);
  *move_place_id = target_place_id;
  *move_item_index = target_item_index;
  return true;
}

bool selectMoveItem(Boxinfo_t& box_info, MoveTypes::MoveType move_type, uint& item_index)
{
  int target_item_index = -1;
  double target_probability = -1;
  for (std::size_t i = 0; i < box_info.item_name.size(); ++i)
  {
    // 非orderアイテム
    if (box_info.item_order[i] != ORDER_NONE)
    {
      continue;
    }

    ItemStatus item_status = box_info.status[i];
    // 認識済みアイテム
    if (ITEM_RECOG_OK != item_status)
    {
      continue;
    }

    ItemRecogStatus item_recog_status = box_info.item_recog_status[i];
    // Bin間移動: misrecognition/unknown以外のアイテム
    if (MoveTypes::Between == move_type && ITEM_NORMAL != item_recog_status)
    {
      continue;
    }

    // Plan失敗済みアイテムは除外
    if (PLAN_NG == box_info.item_plan_status[i])
    {
      continue;
    }

    // 移動済みアイテムは除外
    if (MoveTypes::Between == move_type)
    {
      // Bin間移動
      bool moved = false;
      for (std::size_t j = 0; j < moved_between_item_info_.size(); ++j)
      {
        const BoxItemInfo& moved_item = moved_between_item_info_[j];
        if (moved_item.place_id == box_info.box_id && moved_item.item_index == i)
        {
          moved = true;
          break;
        }
      }
      if (moved)
      {
        ROS_INFO("Reject moved between item (place_id=%u, item_index=%lu)", box_info.box_id, i);
        box_info.item_plan_status[i] = PLAN_NG;
        continue;
      }
    }
    else if (MoveTypes::Inside == move_type)
    {
      // Bin内移動
      bool moved = false;
      for (std::size_t j = 0; j < moved_inside_item_info_.size(); ++j)
      {
        const BoxItemInfo& moved_item = moved_inside_item_info_[j];
        if (moved_item.place_id == box_info.box_id && moved_item.item_index == i)
        {
          moved = true;
          break;
        }
      }
      if (moved)
      {
        ROS_INFO("Reject moved inside item (place_id=%u, item_index=%lu)", box_info.box_id, i);
        box_info.item_plan_status[i] = PLAN_NG;
        continue;
      }
    }

    // 認識スコア最大
    double probability = box_info.item_data[i].probability;
    if (probability > target_probability)
    {
      target_item_index = i;
      target_probability = probability;
    }
    else if (probability == target_probability)
    {
      // 図心のz座標が大きい
      double position_z = box_info.item_data[i].pose.position.z;
      double target_position_z = box_info.item_data[target_item_index].pose.position.z;
      if (position_z > target_position_z)
      {
        target_item_index = i;
      }
      else if (position_z == target_position_z)
      {
        // 体積が小さい
        double volume = box_info.volume[i];
        double target_volume = box_info.volume[target_item_index];
        if (volume < target_volume)
        {
          target_item_index = i;
        }
      }
    }
  }

  // 対象アイテムが存在しない
  if (target_item_index < 0)
  {
    ROS_INFO("Cannot select item to move (place_id=%u)", box_info.box_id);
    return false;
  }

  ROS_INFO("Move item[%d] from place_id=%u", target_item_index, box_info.box_id);

  // 出力
  item_index = target_item_index;
  return true;
}

bool selectPlaceMoveFrom(MoveTypes::MoveType move_type, uint& from_place_id)
{
  double max_total_grasp_easiness = -1;
  uint target_place_id = 0;
  uint min_non_order_item_count = 0;

  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    const Boxinfo_t& box_info = boxinfo[i];
    uint place_id = box_info.box_id;

    if (!box_info.enabled || BoxTypes::Bin != box_info.type)
    {
      continue;
    }

    // Bin内移動: 対象外place_idリスト
    if (MoveTypes::Inside == move_type)
    {
      bool move_excluded = false;
      for (int place_id : move_excluded_place_id_list_)
      {
        if (place_id == box_info.box_id)
        {
          move_excluded = true;
          break;
        }
      }
      if (move_excluded)
      {
        ROS_INFO("selectPlaceMoveFrom() place_id=%u is excluded", box_info.box_id);
        continue;
      }
    }

    // アイテム認識済みBin
    if (RECOG_OK != box_info.recog_status)
    {
      continue;
    }

    // Bin内移動：箱詰認識済みbin
    if (MoveTypes::Inside == move_type && RECOG_OK != box_info.octomap_status)
    {
      continue;
    }

    // 非orderアイテムが存在する
    bool has_non_order_item = false;
    int non_order_item_count = 0;
    for (std::size_t j = 0; j < box_info.item_order.size(); ++j)
    {
      ItemStatus item_status = box_info.status[j];
      if (ORDER_NONE == box_info.item_order[j] && ITEM_RECOG_OK == item_status)
      {
        non_order_item_count++;

        // Bin間移動：misrecognition/unknown以外
        if (MoveTypes::Between == move_type && ITEM_NORMAL != box_info.item_recog_status[j])
        {
          continue;
        }

        // Plan失敗済みのアイテムは除外
        if (PLAN_NG == box_info.item_plan_status[j])
        {
          continue;
        }

        has_non_order_item = true;
      }
    }
    if (!has_non_order_item)
    {
      continue;
    }

    // orderアイテムの把持容易度の合計値が大きい
    double total_grasp_easiness = getTotalTargetGraspEasiness(place_id);
    ROS_INFO("[move] grasp_easiness=%f, non_order_item_count=%u (box_index=%lu)",
        total_grasp_easiness, non_order_item_count, i);
    if (total_grasp_easiness > max_total_grasp_easiness)
    {
      max_total_grasp_easiness = total_grasp_easiness;
      target_place_id = place_id;
      min_non_order_item_count = non_order_item_count;
    }
    else if (total_grasp_easiness == max_total_grasp_easiness)
    {
      // 非orderアイテム数が少ない
      if (non_order_item_count < min_non_order_item_count)
      {
        target_place_id = place_id;
        min_non_order_item_count = non_order_item_count;
      }
    }
  }

  if (max_total_grasp_easiness <= 0)
  {
    ROS_INFO("Cannot select place_id to move from");
    return false;
  }

  // 出力
  from_place_id = target_place_id;
  return true;
}

bool selectPlaceMoveFrom(MoveTypes::MoveType move_type, uint& pick_index, uint& from_place_id)
{
  double max_total_grasp_easiness = -1;
  uint target_pick_index = 0;
  uint target_place_id = 0;
  uint non_target_item_count = 0;

  // 調査済みのBinリスト
  std::vector<uint> place_id_list;

  for (std::size_t i = 0; i < pick_order_.size(); ++i)
  {
    // 後回しのBinが対象
    if (ORDER_LATER == pick_order_[i].status)
    {
      uint order_no = pick_order_[i].order_no;
      Orderinfo_t* order_info;
      getOrderInfo(order_no, &order_info);

      uint content_index = pick_order_[i].content_index;
      uint box_index = order_info->from_box_index[content_index];
      uint place_id = boxinfo[box_index].box_id;

      // 調査済みのBinは除外
      std::vector<uint>::iterator it = std::find(place_id_list.begin(), place_id_list.end(), place_id);
      if (it != place_id_list.end())
      {
        continue;
      }

      // アイテム認識済みBin
      if (RECOG_OK != boxinfo[box_index].recog_status)
      {
        continue;
      }

      // Bin内移動時はOccupancy認識済みBin
      if (MoveTypes::Inside == move_type && RECOG_OK != boxinfo[box_index].octomap_status)
      {
        continue;
      }

      // 非ターゲットアイテムが存在する
      bool has_non_target_item = false;
      for (int j = 0; j < boxinfo[box_index].item_order.size(); ++j)
      {
        ItemStatus item_status = boxinfo[box_index].status[j];
        if (ORDER_NONE == boxinfo[box_index].item_order[j] && ITEM_PICKED != item_status)
        {
          // Bin間移動時はmisrecognition/unknown以外
          if (MoveTypes::Between == move_type && ITEM_NORMAL != boxinfo[box_index].item_recog_status[j])
          {
            continue;
          }

          // すでにPlan失敗の場合は除外する
          if (ITEM_PICK_NG == item_status)
          {
            continue;
          }

          has_non_target_item = true;
          break;
        }
      }
      if (!has_non_target_item)
      {
        continue;
      }

      // ターゲットアイテムの把持容易度合計
      double total_grasp_easiness = getTotalTargetGraspEasiness(place_id);
      if (total_grasp_easiness > max_total_grasp_easiness)
      {
        max_total_grasp_easiness = total_grasp_easiness;
        target_pick_index = i;
        target_place_id = place_id;
        non_target_item_count = getNonTargetItemCount(place_id);
      }
      else if (total_grasp_easiness == max_total_grasp_easiness)
      {
        // 非ターゲットアイテム数が少ない
        uint count = getNonTargetItemCount(place_id);
        if (count < non_target_item_count)
        {
          target_pick_index = i;
          target_place_id = place_id;
          non_target_item_count = count;
        }
      }

      // 調査済みBinリストに追加
      place_id_list.push_back(place_id);
    }
  }

  if (max_total_grasp_easiness < 0)
  {
    ROS_INFO("Cannot select place_id to move from");
    return false;
  }

  // 出力
  from_place_id = target_place_id;
  pick_index = target_pick_index;
  return true;
}

bool selectPlaceMoveTo(uint from_place_id, uint order_item_count_limit, std::vector<uint>& to_place_id)
{
  ROS_INFO("selectPlaceMoveTo(from_place_id=%u, target_item_count_limit=%d", from_place_id, order_item_count_limit);

  to_place_id.clear();
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    if (BoxTypes::Bin != boxinfo[i].type || !boxinfo[i].enabled)
    {
      continue;
    }

    // 移動元のBin以外
    if (boxinfo[i].box_id == from_place_id)
    {
      continue;
    }

    // 対象外place_idリスト
    bool move_excluded = false;
    for (int place_id : move_excluded_place_id_list_)
    {
      if (place_id == boxinfo[i].box_id)
      {
        move_excluded = true;
        break;
      }
    }
    if (move_excluded)
    {
      ROS_INFO("selectPlaceMoveTo() place_id=%u is excluded", boxinfo[i].box_id);
      continue;
    }

    // 箱詰認識済み
    if (RECOG_OK != boxinfo[i].octomap_status)
    {
      continue;
    }

    uint place_id = boxinfo[i].box_id;
    uint order_item_count = getOrderItemCount(place_id);

    // orderアイテム数制約
    if (order_item_count != order_item_count_limit)
    {
      continue;
    }

    to_place_id.push_back(boxinfo[i].box_id);
  }

  if (to_place_id.size() <= 0)
  {
    ROS_INFO("Cannot select place_id to move (order item count limit = %d)", order_item_count_limit);
    return false;
  }

  std::string list = "[ ";
  for (std::size_t i = 0; i < to_place_id.size(); ++i)
  {
    std::ostringstream oss;
    oss << to_place_id[i];
    list.append(oss.str() + " ");
  }
  list.append("]");
  ROS_INFO("[move] Move item from place_id=%u to place_id=%s", from_place_id, list.c_str());

  return true;
}

OrderStatus selectPickInfo(uint& from_place_id, std::vector<uint>& to_place_id, uint& item_index)
{
  ROS_INFO("selectPickInfo() start");

  ros::Duration duration(0.1);

  to_place_id.clear();

  // order終了判定
  bool all_order_done = true;
  for (std::size_t i = 0; i < orderinfo.size(); ++i)
  {
    const Orderinfo_t& order_info = orderinfo[i];
    for (std::size_t j = 0; j < order_info.item_order.size(); ++j)
    {
      if (ORDER_EXIST == order_info.item_order[j])
      {
        all_order_done = false;
        break;
      }
    }
    if (!all_order_done)
    {
      break;
    }
  }
  if (all_order_done)
  {
    ROS_INFO("All order is done");
    return ORDER_NONE;
  }

  while (true)
  {
    int max_order_item_count = -1;
    int target_box_index = -1;
    int target_item_index = -1;
    int target_order_index = -1;
    for (std::size_t i = 0; i < boxinfo.size(); ++i)
    {
      const Boxinfo_t& box_info = boxinfo[i];
      if (!box_info.enabled || BoxTypes::Bin != box_info.type)
      {
        continue;
      }

      for (std::size_t j = 0; j < box_info.item_plan_status.size(); ++j)
      {
        // アイテム状態が計画成功
        if (PLAN_OK == box_info.item_plan_status[j] && ORDER_EXIST == box_info.item_order[j])
        {
          // 格納元bin内のorderアイテム数が最大
          int order_item_count = getOrderItemCount(box_info.box_id);
          //ROS_INFO("order_item_count = %d", order_item_count);
          //ROS_INFO("max_order_item_count(0) = %d", max_order_item_count);
          if (order_item_count > max_order_item_count)
          {
            //ROS_INFO("test");
            max_order_item_count = order_item_count;
            target_box_index = i;
            target_item_index = j;
            std::string item_name = boxinfo[target_box_index].item_name[target_item_index];
            for (std::size_t k = 0; k < orderinfo.size(); ++k)
            {
              bool order_exists = false;
              const Orderinfo_t& order_info = orderinfo[k];
              for (std::size_t l = 0; l < order_info.item_name.size(); ++l)
              {
                if (item_name == order_info.item_name[l] && ORDER_EXIST == order_info.item_order[l])
                {
                  order_exists = true;
                  target_order_index = k;
                  break;
                }
              }
              if (order_exists)
              {
                break;
              }
            }
            //ROS_INFO("target_order_index = %d", target_order_index);
            if (target_order_index < 0)
            {
              ROS_ERROR("Box(place_id=%d) item(item_index=%d) is not order exist",
                  boxinfo[target_box_index].box_id, target_item_index);
            }
          }
        }
      }
    }

    //ROS_INFO("max_order_item_count = %d", max_order_item_count);

    if (max_order_item_count < 0)
    {
      bool has_plan_notyet = false;
      for (std::size_t i = 0; i < boxinfo.size(); ++i)
      {
        const Boxinfo_t& box_info = boxinfo[i];
        if (!box_info.enabled || BoxTypes::Bin != box_info.type)
        {
          continue;
        }

        // bin状態が未のbinが存在する
        if (PLAN_NOTYET == box_info.plan_status)
        {
          has_plan_notyet = true;
          break;
        }
      }
      if (has_plan_notyet)
      {
        duration.sleep();
        continue;
      }
      else
      {
        // bin状態が未のbinが存在しない
        ROS_INFO("All bin plan_status is OK");
        break;
      }
    }

    // 計画成功アイテムあり(出力)
    from_place_id = boxinfo[target_box_index].box_id;
    to_place_id.push_back(boxinfo[orderinfo[target_order_index].to_box_index].box_id);
    item_index = target_item_index;
    return ORDER_EXIST;
  }

  return ORDER_FAIL;
}

OrderStatus selectMoveInfo(uint& from_place_id, std::vector<uint>& to_place_id, uint& item_index)
{
  ROS_INFO("selectMoveInfo() start");

  int max_order_item_count = 0;
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    const Boxinfo_t& box_info = boxinfo[i];
    if (!box_info.enabled || BoxTypes::Bin != box_info.type)
    {
      continue;
    }

    uint order_item_count = getOrderItemCount(box_info.box_id);
    if (order_item_count > max_order_item_count)
    {
      max_order_item_count = order_item_count;
    }
  }

  ROS_INFO("selectMoveInfo() max_target_item_count=%d", max_order_item_count);

  // Moving between bins
  if (move_phase_ == 0)
  {
    ROS_INFO("Move phase = %d", move_phase_);
    while (target_item_count_limit_ <= max_order_item_count)
    {
      if (selectPlaceMoveFrom(MoveTypes::Between, from_place_id))
      {
        Boxinfo_t& box_info = boxinfo[place_id_to_index_[from_place_id]];

        while (true)
        {
          if (selectMoveItem(box_info, MoveTypes::Between, item_index))
          {
            if (selectPlaceMoveTo(from_place_id, target_item_count_limit_, to_place_id))
            {
              return ORDER_LATER;
            }
            else
            {
              // 次のアイテムへ
              box_info.item_plan_status[item_index] = PLAN_NG;
              continue;
            }
          }
          break;
        }
        continue;
      }
      // アイテムの計画NGをリセット
      resetNontargetItemStatus(BoxTypes::Bin);

      // ターゲットアイテム数制約を一段緩和
      target_item_count_limit_++;
    }

    // Bin間移動の候補なしでBin内移動へ
    move_phase_ = 1;

    // ターゲットアイテム数制約をリセット
    target_item_count_limit_ = 0;
  }

  // Moveing inside bin
  if (move_phase_ == 1)
  {
    ROS_INFO("Move phase = %d", move_phase_);
    if (selectPlaceMoveFrom(MoveTypes::Inside, from_place_id))
    {
      Boxinfo_t& box_info = boxinfo[place_id_to_index_[from_place_id]];
      if (selectMoveItem(box_info, MoveTypes::Inside, item_index))
      {
        ROS_INFO("Move item[%d] to place_id=%u", item_index, from_place_id);
        to_place_id.clear();
        to_place_id.push_back(from_place_id);
        return ORDER_LATER;
      }
    }

    // Bin内移動の候補なし
    move_phase_ = 2;

    // アイテムの計画NGをリセット
    resetNontargetItemStatus(BoxTypes::Bin);
  }

  // Moving between bins 2
  if (move_phase_ == 2)
  {
    ROS_INFO("Move phase = %d", move_phase_);
    while (target_item_count_limit_ <= max_order_item_count)
    {
      if (selectPlaceMoveFrom(MoveTypes::Between, from_place_id))
      {
        Boxinfo_t& box_info = boxinfo[place_id_to_index_[from_place_id]];

        while (true)
        {
          if (selectMoveItem(box_info, MoveTypes::Between, item_index))
          {
            if (selectPlaceMoveTo(from_place_id, target_item_count_limit_, to_place_id))
            {
              return ORDER_LATER;
            }
            else
            {
              // 次のアイテムへ
              box_info.item_plan_status[item_index] = PLAN_NG;
              continue;
            }
          }
          break;
        }
        continue;
      }
      // アイテムの計画NGをリセット
      resetNontargetItemStatus(BoxTypes::Bin);

      // ターゲットアイテム数制約を一段緩和
      target_item_count_limit_++;
    }

    // Bin間移動の候補なしでBin内移動へ
    move_phase_ = 0;

    // ターゲットアイテム数制約をリセット
    target_item_count_limit_ = 0;
  }

  return ORDER_FAIL;
}

/*
OrderStatus selectPickInfo(uint& pick_index, uint& from_place_id, std::vector<uint>& to_place_id, uint& item_index)
{
  ROS_INFO("selectPickInfo() start");

  to_place_id.clear();

  ros::Duration duration(0.1);

  // Moving from bin to box
  bool has_order_exist = true;
  while (has_order_exist)
  {
    has_order_exist = false;
    for (std::size_t i = 0; i < pick_order_.size(); ++i)
    {
      if (ORDER_EXIST == pick_order_[i].status)
      {
        //ROS_INFO("pick_index=%lu is order_exist", i);
        has_order_exist = true;
        if (PLAN_OK == pick_order_[i].plan_status)
        {
          uint order_no = pick_order_[i].order_no;
          Orderinfo_t* order_info;
          getOrderInfo(order_no, &order_info);

          uint content_index = pick_order_[i].content_index;

          // 出力
          pick_index = i;
          from_place_id = boxinfo[order_info->from_box_index[content_index]].box_id;
          to_place_id.push_back(boxinfo[order_info->to_box_index].box_id);
          item_index = order_info->item_index[content_index];
          return ORDER_EXIST;
        }
      }
    }
    duration.sleep();
  }




  bool all_order_done = true;
  for (std::size_t i = 0; i < pick_order_.size(); ++i)
  {
    if (ORDER_DONE != pick_order_[i].status)
    {
      all_order_done = false;
      break;
    }
  }
  if (all_order_done)
  {
    return ORDER_NONE;
  }

  is_later_ = true;
  ROS_INFO("Change pick order later");

  Boxinfo_t *box_info;

  int max_target_item_count = 0;
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    if (!boxinfo[i].enabled || BoxTypes::Bin != boxinfo[i].type)
    {
      continue;
    }

    uint target_item_count = getTargetItemCount(boxinfo[i].box_id);
    if (target_item_count > max_target_item_count)
    {
      max_target_item_count = target_item_count;
    }
  }

  ROS_INFO("selectPickInfo() max_target_item_count=%d", max_target_item_count);

  // Moving between bins
  if (move_phase_ == 0)
  {
    ROS_INFO("Move phase = %d", move_phase_);
    while (target_item_count_limit_ <= max_target_item_count)
    {
      if (selectPlaceMoveFrom(MoveTypes::Between, pick_index, from_place_id))
      {
        getBoxInfo(from_place_id, &box_info);

        while (true)
        {
          if (selectMoveItem(*box_info, MoveTypes::Between, item_index))
          {
            if (selectPlaceMoveTo(from_place_id, target_item_count_limit_, to_place_id))
            {
              return ORDER_LATER;
            }
            else
            {
              // 次のアイテムへ
              box_info->status[item_index] = ITEM_PICK_NG;
              continue;
            }
          }
          break;
        }
      }
      // アイテムの把持NGをリセット
      resetNontargetItemStatus(BoxTypes::Bin);

      // ターゲットアイテム数制約を一段緩和
      target_item_count_limit_++;
    }

    // Bin間移動の候補なしでBin内移動へ
    move_phase_ = 1;

    // ターゲットアイテム数制約をリセット
    target_item_count_limit_ = 0;
  }

  // Moveing inside bin
  if (move_phase_ == 1)
  {
    ROS_INFO("Move phase = %d", move_phase_);
    if (selectPlaceMoveFrom(MoveTypes::Inside, pick_index, from_place_id))
    {
      getBoxInfo(from_place_id, &box_info);
      if (selectMoveItem(*box_info, MoveTypes::Inside, item_index))
      {
        ROS_INFO("Move item[%d] to place_id=%u", item_index, from_place_id);
        to_place_id.clear();
        to_place_id.push_back(from_place_id);
        return ORDER_LATER;
      }
    }

    // Bin内移動の候補なし
    move_phase_ = 2;

    // PlanNGをリセット
    resetNontargetItemStatus(BoxTypes::Bin);
  }

  // Moving between bins 2
  if (move_phase_ == 2)
  {
    ROS_INFO("Move phase = %d", move_phase_);
    while (target_item_count_limit_ <= max_target_item_count)
    {
      if (selectPlaceMoveFrom(MoveTypes::Between, pick_index, from_place_id))
      {
        getBoxInfo(from_place_id, &box_info);

        while (true)
        {
          if (selectMoveItem(*box_info, MoveTypes::Between, item_index))
          {
            if (selectPlaceMoveTo(from_place_id, target_item_count_limit_, to_place_id))
            {
              return ORDER_LATER;
            }
            else
            {
              // 次のアイテムへ
              box_info->status[item_index] = ITEM_PICK_NG;
              continue;
            }
          }
          break;
        }
      }
      // アイテムの把持NGをリセット
      resetNontargetItemStatus(BoxTypes::Bin);

      // ターゲットアイテム数制約を一段緩和
      target_item_count_limit_++;
    }

    // Bin間移動の候補なしでBin内移動へ
    move_phase_ = 0;

    // ターゲットアイテム数制約をリセット
    target_item_count_limit_ = 0;
  }

  return ORDER_FAIL;  // やり直し
}
*/

void resetItemStatus(BoxTypes::BoxType type, OrderStatus item_order_status)
{
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    Boxinfo_t& box_info = boxinfo[i];

    if (!box_info.enabled || type != box_info.type)
    {
      continue;
    }

    for (std::size_t j = 0; j < box_info.item_order.size(); ++j)
    {
      // アイテムの把持失敗を認識済みへ変更
      if (item_order_status == box_info.item_order[j] && PLAN_NG == box_info.item_plan_status[j])
      {
        if (ITEM_RECOG_NOTYET != box_info.status[j])
        {
          box_info.status[j] = ITEM_RECOG_OK;
        }
        box_info.item_plan_status[j] = PLAN_NOTYET;
        box_info.failed_gp_numbers[j].clear();
        ROS_INFO("resetItemStatus(%d) [i, j] = [%lu, %lu]", item_order_status, i, j);
      }
    }
  }
}

void resetTargetItemStatus(BoxTypes::BoxType type)
{
  ROS_INFO("resetTargetItemStatus()");
  // ターゲットアイテムの計画失敗を認識済みへ変更
  resetItemStatus(type, ORDER_EXIST);
}

void resetNontargetItemStatus(BoxTypes::BoxType type)
{
  ROS_INFO("resetNontargetItemStatus()");
  // 非ターゲットアイテムの計画失敗を認識済みへ変更
  resetItemStatus(type, ORDER_NONE);
}

void resetMovedItemInfo()
{
  ROS_INFO("resetMovedItemInfo()");
  moved_between_item_info_.clear();
  moved_inside_item_info_.clear();
}

void resetRecognize(uint box_index)
{
  std::lock_guard<std::mutex> lock(box_info_mutex_);

  uint i, itemcount;
  ROS_INFO("resetRecognize(box_index=%d) start", box_index);

  Boxinfo_t& box_info = boxinfo[box_index];

  box_info.recog_status = RECOG_NOTYET;
  box_info.octomap_status = RECOG_NOTYET;
  itemcount = box_info.item_name.size();
  ROS_INFO("size=%d\n", itemcount);
  if (itemcount > 0)
  {
    for (i = 0; i < itemcount; i++)
    {
      // ITEM_RECOG_NOTYET, ITEM_RECOG_OK, ITEM_PICK_NG, ITEM_PLACE_NG,
      if (box_info.status[i] < ITEM_PICKED)
      {
        box_info.status[i] = ITEM_RECOG_NOTYET;
      }

      box_info.grasp_point_index[i] = 0;
      if (box_info.grasp_release_points[i].size() > 0)
      {
        box_info.grasp_release_points[i].clear();
      }
      if (box_info.failed_gp_numbers[i].size() > 0)
      {
        box_info.failed_gp_numbers[i].clear();
      }
    }
  }
  return;
}

void setRecogStatus(uint box_index, RecogStatus recog_status)
{
  ROS_ASSERT(box_index < boxinfo.size());
  ROS_INFO("setRecogStatus(box_index=%u, recog_status=%d)", box_index, recog_status);
  boxinfo[box_index].recog_status = recog_status;
}

void setOctomapStatus(uint box_index, RecogStatus octomap_status)
{
  ROS_ASSERT(box_index < boxinfo.size());
  ROS_INFO("setOctomapStatus(box_index=%u, octomap_status=%d)", box_index, octomap_status);
  boxinfo[box_index].octomap_status = octomap_status;
}

void setGraspPointIndex(uint box_index, uint item_index, uint grasp_point_index)
{
  ROS_ASSERT(box_index < boxinfo.size());
  ROS_ASSERT(item_index < boxinfo[box_index].grasp_point_index.size());
  ROS_INFO("setGraspPointIndex(box_index=%u, item_index=%d, grasp_point_index=%d)", box_index, item_index, grasp_point_index);
  boxinfo[box_index].grasp_point_index[item_index] = grasp_point_index;
}

void setPickPlanStatus(uint pick_index, PlanStatus plan_status)
{
  ROS_ASSERT(pick_index < pick_order_.size());
  ROS_INFO("setPickPlanStatus(pick_index=%u, plan_status=%d)", pick_index, plan_status);
  pick_order_[pick_index].plan_status = plan_status;
}

void setPickMoveIndex(uint pick_index, uint move_item_index, uint to_box_index)
{
  ROS_ASSERT(pick_index < pick_order_.size());
  ROS_INFO("setPickMoveIndex(pick_index=%u, move_item_index=%d)", pick_index, move_item_index);
  pick_order_[pick_index].move_item_index = move_item_index;
  pick_order_[pick_index].to_box_index = to_box_index;
}

void setPickMoveInfo(uint place_id, uint item_index, const std::vector<uint>& to_place_id)
{
  uint box_index = place_id_to_index_[place_id];
  ROS_INFO("setPickMoveInfo(place_id=%u, item_index=%u, to_place_id.size()=%lu)",
      place_id, item_index, to_place_id.size());

  // 非orderアイテムのorder情報をORDER_MOVEに変更
  boxinfo[box_index].item_order[item_index] = ORDER_MOVE;
  pick_info_[place_id].item_index = item_index;
  pick_info_[place_id].to_place_id_list = to_place_id;
}

/*
void setPickMoveInfo(uint pick_index, uint move_item_index, const std::vector<uint>& to_place_id)
{
  ROS_ASSERT(pick_index < pick_order_.size());
  ROS_INFO("setPickMoveInfo(pick_index=%u, move_item_index=%d)", pick_index, move_item_index);
  pick_order_[pick_index].move_item_index = move_item_index;
  pick_order_[pick_index].to_place_id = to_place_id;
}
*/

void setStowPlanStatus(PlanStatus status)
{
  ROS_INFO("setStowPlanStatus(status=%d)", status);
  uint box_index = place_id_to_index_[D_tote_1_ID];
  boxinfo[box_index].plan_status = status;
}

void setStowItemIndex(uint item_index)
{
  ROS_INFO("setStowItemIndex(item_index=%d)", item_index);
  stow_info_.item_index = item_index;
}

void setPickStartPlaceID(uint pick_index, uint start_place_id)
{
  ROS_ASSERT(pick_index < pick_order_.size());
  pick_order_[pick_index].start_place_id = start_place_id;
}

void setPickGoalPlaceID(uint start_place_id, uint goal_place_id)
{
  pick_info_[start_place_id].to_place_id = goal_place_id;
}

void setStowStartPlaceID(uint start_place_id)
{
  stow_info_.previous_place_id = stow_info_.start_place_id;
  stow_info_.start_place_id = start_place_id;
}

void setStowPreviousPlaceID()
{
  stow_info_.start_place_id = stow_info_.previous_place_id;
}

void setStowPlaceMoveStatus(bool while_place_move)
{
  stow_info_.while_place_move = while_place_move;
}

bool whileStowPlaceMove()
{
  return stow_info_.while_place_move;
}

uint getBinCount()
{
  uint bin_count = 0;
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    if (boxinfo[i].enabled && BoxTypes::Bin == boxinfo[i].type)
    {
      bin_count++;
    }
  }
  return bin_count;
}

uint getCameraID(uint place_id)
{
  for (auto it = boxinfo.begin(); it != boxinfo.end(); ++it)
  {
    if (place_id == it->box_id)
    {
      return it->camera_id;
    }
  }
  return 0;
}

std::string getItemName(uint cad_id)
{
  std::string item_name;
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue grasp_info;
  if (!nh.getParam("/t2_database/grasp_info_list", grasp_info))
  {
    ROS_ERROR("Failed to get grasp_info");
    return item_name;
  }
  ROS_ASSERT(grasp_info.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  std::ostringstream oss;
  oss << cad_id;
  std::string cad_id_str = "cad" + oss.str();

  for (auto it = grasp_info.begin(); it != grasp_info.end(); ++it)
  {
    if (it->first == cad_id_str)
    {
      item_name = static_cast<std::string>(grasp_info[it->first]["item_name"]);
      return item_name;
    }
  }
  return item_name;
}

double getItemVolume(uint cad_id)
{
  double volume = -1;
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue grasp_info;
  if (!nh.getParam("/t2_database/grasp_info_list", grasp_info))
  {
    ROS_ERROR("Failed to get grasp_info");
    return volume;
  }
  ROS_ASSERT(grasp_info.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  std::ostringstream oss;
  oss << cad_id;
  std::string cad_id_str = "cad" + oss.str();

  for (auto it = grasp_info.begin(); it != grasp_info.end(); ++it)
  {
    if (it->first == cad_id_str)
    {
      volume = static_cast<double>(grasp_info[it->first]["volume_of_item"]);
      return volume;
    }
  }
  return volume;
}

double getGraspEasiness(uint cad_id)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue grasp_info;
  if (!nh.getParam("/t2_database/grasp_info_list", grasp_info))
  {
    ROS_ERROR("Failed to get grasp_info");
    return -1;
  }
  ROS_ASSERT(grasp_info.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  std::ostringstream oss;
  oss << cad_id;
  std::string cad_id_str = "cad" + oss.str();

  for (auto it = grasp_info.begin(); it != grasp_info.end(); ++it)
  {
    if (it->first == cad_id_str)
    {
      return static_cast<double>(grasp_info[it->first]["grasp_easiness"]);
    }
  }
  return -1;
}

std::vector<uint> getItemIndexList(uint box_index, ItemRecogStatus status, double threshold)
{
  ROS_INFO("getItemIndexList(box_index=%u, status=%d, threshold=%f)", box_index, status, threshold);
  const Boxinfo_t& box_info = boxinfo[box_index];
  std::vector<uint> array;
  for (int i = 0; i < box_info.cad_id.size(); ++i)
  {
    if (box_info.item_recog_status[i] == status &&
        (box_info.status[i] == ITEM_RECOG_OK || box_info.status[i] == ITEM_PICK_NG) &&
        box_info.item_data[i].probability >= threshold)
    {
      array.push_back(i);
    }
  }

  // DEBUG
  std::ostringstream oss;
  for (auto it = array.begin(); it != array.end(); ++it)
  {
    oss << *it << " ";
  }
  std::string output = "item_index: [ " + oss.str() + "]";
  ROS_INFO_STREAM(output);

  return array;
}

std::vector<uint> getRecognizedBinID()
{
  ROS_INFO("getRecognizedBinID()");
  std::vector<uint> array;
  for (int i = 0; i < boxinfo.size(); ++i)
  {
    if (BoxTypes::Bin != boxinfo[i].type)
    {
      continue;
    }
    if (RECOG_OK == boxinfo[i].octomap_status)
    {
      array.push_back(boxinfo[i].box_id);
    }
  }

  // DEBUG
  std::ostringstream oss;
  for (auto it = array.begin(); it != array.end(); ++it)
  {
    oss << *it << " ";
  }
  std::string output = "place_id: [ " + oss.str() + "]";
  ROS_INFO_STREAM(output);

  return array;
}

std::vector<Location_t> getLocationList(const std::vector<uint>& place_id_list)
{
  std::vector<Location_t> list;

  for (std::size_t i = 0; i < place_id_list.size(); ++i)
  {
    uint place_id = place_id_list[i];
    Boxinfo_t *box_info;
    if (!getBoxInfo(place_id, &box_info))
    {
      ROS_WARN("Invalid place_id=%u", place_id);
    }
    std::vector<uint> cad_id_list;
    for (std::size_t j = 0; j < box_info->cad_id.size(); ++j)
    {
      if (ITEM_PICKED != box_info->status[j])
      {
        cad_id_list.push_back(box_info->cad_id[j]);
      }
    }

    Location_t location = { place_id, cad_id_list };
    list.push_back(location);
  }
  return list;
}

uint getPickItemIndex(uint place_id)
{
  return pick_info_[place_id].item_index;
}

const std::vector<uint>& getPickGoalPlaceIDList(uint place_id)
{
  return pick_info_[place_id].to_place_id_list;
}

uint getStowItemIndex()
{
  return stow_info_.item_index;
}

uint getPickStartPlaceID(uint pick_index)
{
  ROS_ASSERT(pick_index < pick_order_.size());
  return pick_order_[pick_index].start_place_id;
}

uint getPickGoalPlaceID(uint place_id)
{
  return pick_info_[place_id].to_place_id;
}

uint getStowStartPlaceID()
{
  return stow_info_.start_place_id;
}

uint getCurrentItemCount(uint box_index)
{
  ROS_ASSERT(box_index < boxinfo.size());
  const Boxinfo_t& box_info = boxinfo[box_index];
  int item_count = 0;
  for (int i = 0; i < box_info.status.size(); ++i)
  {
    if (ITEM_PICKED != box_info.status[i])
    {
      item_count++;
    }
  }
  return item_count;
}

uint getTargetItemCount(uint place_id)
{
  Boxinfo_t *box_info;
  getBoxInfo(place_id, &box_info);

  uint count = 0;
  for (size_t i = 0; i < box_info->item_order.size(); ++i)
  {
    // ターゲットアイテムかつ移動済みでない
    if (ORDER_EXIST == box_info->item_order[i] && ITEM_PICKED != box_info->status[i])
    {
      count++;
    }
  }
  return count;
}

uint getNonTargetItemCount(uint place_id)
{
  Boxinfo_t *box_info;
  getBoxInfo(place_id, &box_info);

  uint count = 0;
  for (size_t i = 0; i < box_info->item_order.size(); ++i)
  {
    // 非ターゲットアイテムかつ移動済みでない
    if (ORDER_NONE == box_info->item_order[i] && ITEM_PICKED != box_info->status[i])
    {
      count++;
    }
  }
  return count;
}

uint getOrderItemCount(uint place_id)
{
  const Boxinfo_t& box_info = boxinfo[place_id_to_index_[place_id]];
  uint count = 0;
  for (std::size_t i = 0; i < box_info.item_name.size(); ++i)
  {
    bool order_exists = false;
    for (std::size_t j = 0; j < orderinfo.size(); ++j)
    {
      const Orderinfo_t order_info = orderinfo[j];
      for (std::size_t k = 0; k < order_info.item_name.size(); ++k)
      {
        if (box_info.item_name[i] == order_info.item_name[k] && ORDER_EXIST == order_info.item_order[k])
        {
          order_exists = true;
          count++;
          break;
        }
      }
      if (order_exists)
      {
        break;
      }
    }
  }
  return count;
}

uint getPickDestinationPlaceID(uint from_place_id, uint item_index)
{
  for (std::size_t i = 0; i < orderinfo.size(); ++i)
  {
    const std::vector<std::string>& item_names = orderinfo[i].item_name;
    for (std::size_t j = 0; j < item_names.size(); ++j)
    {
      uint index = place_id_to_index_[from_place_id];
      if (item_names[j] == boxinfo[index].item_name[item_index] && ORDER_DONE != orderinfo[i].item_order[j])
      {
        return boxinfo[orderinfo[i].to_box_index].box_id;
      }
    }
  }
  return 0;
}

double getTotalTargetGraspEasiness(uint place_id)
{
  double total = 0;

  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue grasp_info;
  if (!nh.getParam("/t2_database/grasp_info_list", grasp_info))
  {
    ROS_ERROR("Failed to get grasp_info");
    return -1;
  }
  ROS_ASSERT(grasp_info.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  Boxinfo_t *box_info;
  getBoxInfo(place_id, &box_info);

  for (int i = 0; i < box_info->item_name.size(); ++i)
  {
    OrderStatus item_order = box_info->item_order[i];
    if (ORDER_EXIST != item_order && ORDER_LATER != item_order)
    {
      continue;
    }
    const std::string& item_name = box_info->item_name[i];
    for (auto it = grasp_info.begin(); it != grasp_info.end(); ++it)
    {
      if (item_name == static_cast<std::string>(grasp_info[it->first]["item_name"]))
      {
        double grasp_easiness = static_cast<double>(grasp_info[it->first]["grasp_easiness"]);
        total += grasp_easiness;
        break;
      }
    }
  }

  return total;
}

void clearFailedGPList()
{
  std::lock_guard<std::mutex> lock(failed_gp_mutex_);
  ROS_INFO("clearFailedGPList()");
  stow_info_.failed_gp_list.clear();
}

std::vector<geometry_msgs::Point> getFailedGPList()
{
  std::lock_guard<std::mutex> lock(failed_gp_mutex_);
  return stow_info_.failed_gp_list;
}

bool permitProtrusion()
{
  if (move_phase_ == 2)
  {
    return true;
  }
  return false;
}

bool hasOrderItem(uint place_id)
{
  uint box_index = place_id_to_index_[place_id];
  const Boxinfo_t& box_info = boxinfo[box_index];
  for (std::size_t i = 0; i < box_info.item_order.size(); ++i)
  {
    OrderStatus item_order = box_info.item_order[i];
    if (ORDER_EXIST == item_order || ORDER_LATER == item_order)
    {
      return true;
    }
  }
  return false;
}

void updateRecognize(uint box_index, const std::vector<t2_msgs::ItemData> &item_data)
{
  std::lock_guard<std::mutex> lock(box_info_mutex_);

  ROS_INFO("updateRecognize(box_index=%d item_data.size=%d) start", box_index, (int)item_data.size());

  Boxinfo_t& box_info = boxinfo[box_index];
  for (std::size_t i = 0; i < item_data.size(); ++i)
  {
    ROS_INFO("recognized item[%lu] cad_id=%u", i, item_data[i].cad_id);
    uint item_cad_id = item_data[i].cad_id;

    bool item_exists = false;
    for (std::size_t j = 0; j < box_info.cad_id.size(); ++j)
    {
      // cad_idが一致
      if (item_cad_id == box_info.cad_id[j])
      {
        if (ITEM_RECOG_OK != box_info.status[j] && ITEM_PICKED > box_info.status[j] &&
            (ITEM_NORMAL == box_info.item_recog_status[j] || ITEM_MISRECOG == box_info.item_recog_status[j]))
        {
          ROS_INFO("recognized item[%lu] exists in box[%u] item[%lu]", i, box_index, j);
          // 未認識なら認識済みへ変更
          item_exists = true;
          box_info.item_data[j] = item_data[i];
          box_info.recog_index[j] = i;
          box_info.status[j] = ITEM_RECOG_OK;
          break;
        }
        else if (ITEM_ORDER_DONE == box_info.status[j])
        {
          // オーダー済みの場合
          item_exists = true;
          box_info.recog_index[j] = i;
        }
      }
    }

    // 存在しないcad_idのとき
    if (!item_exists)
    {
      ROS_INFO("recognized item[%lu] not exist", i);
      box_info.status.push_back(ITEM_RECOG_OK);
      if (9999 == item_cad_id)
      {
        box_info.item_recog_status.push_back(ITEM_UNKNOWN);
        box_info.item_name.push_back("unknown");
      }
      else
      {
        box_info.item_recog_status.push_back(ITEM_MISRECOG);
        box_info.item_name.push_back(getItemName(item_cad_id));
      }
      box_info.cad_id.push_back(item_data[i].cad_id);
      box_info.volume.push_back(getItemVolume(item_cad_id));
      box_info.item_data.push_back(item_data[i]);
      box_info.item_order.push_back(ORDER_NONE);
      box_info.grasp_release_points.resize(box_info.grasp_release_points.size() + 1);
      box_info.grasp_point_index.push_back(0);
      box_info.failed_gp_numbers.resize(box_info.failed_gp_numbers.size() + 1);
      box_info.recog_index.push_back(i);
      box_info.order_no.push_back(0);
      box_info.item_plan_status.push_back(PLAN_NOTYET);
    }
  }
  box_info.recog_status = RECOG_OK;
}

void updateOctomapStatus(uint box_index)
{
  ROS_INFO("updateOctomapWithPose(box_index=%u start)", box_index);
  boxinfo[box_index].octomap_status = RECOG_OK;
}

void updateGraspAndRelease(uint box_index, uint item_index, const GpRpArray& array)
{
  std::lock_guard<std::mutex> lock(box_info_mutex_);

  ROS_INFO("updateGraspAndRelease(box_index=%d, item_index=%d, array.size()=%lu) start",
      box_index, item_index, array.size());

  GpRpArray& gprp_array = boxinfo[box_index].grasp_release_points[item_index];
  gprp_array.clear();
  for (int i = 0; i < array.size(); ++i)
  {
    if (array[i].score > 0)
    {
      gprp_array.push_back(array[i]);
    }
  }
}

void updateBoxInfoByPick(uint box_index, uint item_index)
{
  ROS_INFO("updateBoxInfoByPick(box_index=%u, item_index=%u) start", box_index, item_index);
  boxinfo[box_index].status[item_index] = ITEM_PICKED;
}

void updateBoxInfoByMove(uint from_index, uint item_index, uint to_index, TaskType type)
{
  ROS_INFO("updateBoxInfoByMove(from_index=%u, item_index=%u, to_index=%u, type=%d) start",
      from_index, item_index, to_index, type);

  ROS_INFO("Before update");
  if (from_index != to_index)
  {
    printBoxinfo(from_index);
  }
  printBoxinfo(to_index);
  if (TaskType::Pick == type)
  {
    //printAllPickOrder();
  }

  Boxinfo_t& from = boxinfo[from_index];
  Boxinfo_t& to = boxinfo[to_index];

  // Bin内移動
  if (from_index == to_index)
  {
    from.status[item_index] = ITEM_RECOG_NOTYET;
    from.item_order[item_index] = ORDER_NONE;

    // アイテム状態初期化
    for (std::size_t i = 0; i < from.item_plan_status.size(); ++i)
    {
      from.item_plan_status[i] = PLAN_NOTYET;
    }

    // Bin内移動済みリストに追加
    BoxItemInfo moved_item_info;
    moved_item_info.place_id = from.box_id;
    moved_item_info.item_index = item_index;
    moved_inside_item_info_.push_back(moved_item_info);

    // 移動元binのorderアイテムの後回し属性を解除
    for (std::size_t i = 0; i < from.item_order.size(); ++i)
    {
      if (ORDER_LATER == from.item_order[i])
      {
        from.item_order[i] = ORDER_EXIST;
      }
    }

    // bin状態をリセット
    from.plan_status = PLAN_NOTYET;
  }
  else
  {
    // Move item info
    to.volume_of_items += from.volume[item_index];
    to.item_name.push_back(from.item_name[item_index]);
    to.cad_id.push_back(from.cad_id[item_index]);
    to.volume.push_back(from.volume[item_index]);
    to.order_no.push_back(from.order_no[item_index]);
    to.item_recog_status.push_back(from.item_recog_status[item_index]);
    to.item_plan_status.push_back(PLAN_NOTYET);

    // empty data
    uint itemcount = to.item_name.size();
    to.item_data.resize(itemcount);
    to.recog_index.resize(itemcount);
    to.grasp_release_points.resize(itemcount);
    to.grasp_point_index.resize(itemcount);
    to.failed_gp_numbers.resize(itemcount);

    if (TaskType::Pick == type)
    {
      OrderStatus item_order = from.item_order[item_index];
      if (ORDER_EXIST == item_order)
      {
        // order情報の更新
        std::size_t order_index = 0;
        for (; orderinfo.size(); ++order_index)
        {
          if (to_index == orderinfo[order_index].to_box_index)
          {
            break;
          }
        }
        Orderinfo_t& order_info = orderinfo[order_index];
        std::string item_name = from.item_name[item_index];
        for (std::size_t i = 0; i < order_info.item_name.size(); ++i)
        {
          if (item_name == order_info.item_name[i] && ORDER_EXIST == order_info.item_order[i])
          {
            order_info.item_order[i] = ORDER_DONE;
            order_info.status[i] = ITEM_ORDER_DONE;   // TODO: temporary
            break;
          }
        }

        // box情報の更新
        from.item_order[item_index] = ORDER_DONE;
        to.status.push_back(ITEM_ORDER_DONE);
        to.item_order.push_back(ORDER_DONE);

        // 同じアイテムのオーダー残数が0以上
        bool item_order_exist = false;
        for (std::size_t i = 0; i < orderinfo.size(); ++i)
        {
          for (std::size_t j = 0; j < orderinfo[i].item_name.size(); ++j)
          {
            if (item_name == orderinfo[i].item_name[j] && ORDER_EXIST == orderinfo[i].item_order[j])
            {
              item_order_exist = true;
              break;
            }
          }
          if (item_order_exist)
          {
            break;
          }
        }

        // アイテム残数が0のとき、残りのアイテムを非orderアイテムとする
        for (std::size_t i = 0; i < boxinfo.size(); ++i)
        {
          if (!boxinfo[i].enabled || BoxTypes::Bin != boxinfo[i].type)
          {
            continue;
          }
          for (std::size_t j = 0; j < boxinfo[i].item_name.size(); ++j)
          {
            if (item_name == boxinfo[i].item_name[j] &&
                (ORDER_EXIST == boxinfo[i].item_order[j] || ORDER_LATER == boxinfo[i].item_order[j]))
            {
              boxinfo[i].item_order[j] = ORDER_NONE;
            }
          }
        }
      }
      else
      {
        // アイテム状態初期化
        for (std::size_t i = 0; i < from.item_plan_status.size(); ++i)
        {
          from.item_plan_status[i] = PLAN_NOTYET;
        }
        
        from.item_order[item_index] = ORDER_NONE;
        to.status.push_back(ITEM_RECOG_NOTYET);
        to.item_order.push_back(ORDER_NONE);

        // Bin間移動済みリストに追加
        BoxItemInfo moved_item_info;
        moved_item_info.place_id = to.box_id;
        moved_item_info.item_index = to.item_data.size() - 1;
        moved_between_item_info_.push_back(moved_item_info);

        // 移動元binのorderアイテムの後回し属性を解除
        for (std::size_t i = 0; i < from.item_order.size(); ++i)
        {
          if (ORDER_LATER == from.item_order[i])
          {
            from.item_order[i] = ORDER_EXIST;
          }
        }

        // bin状態をリセット
        from.plan_status = PLAN_NOTYET;
      }
    }
    else
    {
      setStowPlaceMoveStatus(false);
      from.item_order[item_index] = ORDER_DONE;
      to.status.push_back(ITEM_ORDER_DONE);
      to.item_order.push_back(ORDER_DONE);
    }
  }

  resetRecognize(to_index);

  ROS_INFO("After update");
  if (from_index != to_index)
  {
    printBoxinfo(from_index);
  }
  printBoxinfo(to_index);
  if (TaskType::Pick == type)
  {
    //printAllPickOrder();
  }
}

void updateOrderInfoByRecogError(uint place_id, uint item_index)
{
  ROS_INFO("updateOrderInfoByRecogError(place_id=%u, item_index=%u)", place_id, item_index);

  uint box_index = place_id_to_index_[place_id];

  // orderアイテムのとき、オーダー情報を後回しにする
  if (ORDER_EXIST == boxinfo[box_index].item_order[item_index])
  {
    boxinfo[box_index].item_order[item_index] = ORDER_LATER;
  }
}

void updateOrderInfoByPlanError(uint place_id, uint item_index)
{
  ROS_INFO("updateOrderInfoByPlanError(place_id=%u, item_index=%u)", place_id, item_index);

  uint box_index = place_id_to_index_[place_id];
  boxinfo[box_index].status[item_index] = ITEM_PICK_NG;
  boxinfo[box_index].item_plan_status[item_index] = PLAN_NG;

  // orderアイテムのとき、オーダー情報を後回しにする
  if (ORDER_EXIST == boxinfo[box_index].item_order[item_index])
  {
    if (PLAN_OK == boxinfo[box_index].plan_status)
    {
      boxinfo[box_index].plan_status = PLAN_NOTYET;
    }
    boxinfo[box_index].item_order[item_index] = ORDER_LATER;
  }
}

void updateStowInfoByPlanError(uint place_id, uint item_index, const geometry_msgs::Point& failed_gp_pos)
{
  ROS_INFO("updateStowInfoByPlanError(place_id=%u, item_index=%u)", place_id, item_index);

  uint box_index = place_id_to_index_[place_id];
  ROS_ASSERT(item_index < boxinfo[box_index].status.size());

  boxinfo[box_index].status[item_index] = ITEM_PICK_NG;
  if (PLAN_OK == boxinfo[box_index].plan_status)
  {
    boxinfo[box_index].plan_status = PLAN_NOTYET;
  }

  std::lock_guard<std::mutex> lock(failed_gp_mutex_);
  stow_info_.failed_gp_list.push_back(failed_gp_pos);
}

bool retryPickSameItem(uint place_id, uint item_index, uint pick_execute_attempts)
{
  ROS_INFO("retryPickSameItem(place_id=%u, item_index=%u, pick_execute_attempts=%u)",
      place_id, item_index, pick_execute_attempts);

  uint box_index = place_id_to_index_[place_id];
  Boxinfo_t& box_info = boxinfo[box_index];
  uint gprp_index = box_info.grasp_point_index[item_index];
  uint gp_number = box_info.grasp_release_points[item_index][gprp_index].gp_number;
  box_info.failed_gp_numbers[item_index].push_back(gp_number);

  if (box_info.failed_gp_numbers[item_index].size() < pick_execute_attempts)
  {
    return true;
  }
  return false;
}

// define which box to pick
/*
OrderStatus selectPickBox(uint pickorder_index, uint *box_index)
{
  uint i;
  uint temp = 0;
  OrderStatus status = ORDER_NONE;
  ROS_INFO("selectPickBox() start");

  if (orderinfo[pickorder_index].order_status == ORDER_EXIST)
  {
    for (i = 0; i < g_bin_count; i++)
    {
      if (boxinfo[i].order_status[pickorder_index] == ORDER_EXIST)
      {
        status = ORDER_EXIST;
        if (temp == 0)
        {
          temp = boxinfo[i].item_name.size();
          *box_index = i;
        }
        else if (temp > boxinfo[i].item_name.size())
        {
          temp = boxinfo[i].item_name.size();
          *box_index = i;
        }
      }
    }
    if (status == ORDER_EXIST)
    {
      ROS_INFO("ORDER_EXIST ... box[%d] (pickorder_%d)", *box_index, pickorder_index);
      return ORDER_EXIST;
    }
    else
    {
      orderinfo[pickorder_index].order_status = ORDER_NONE;
    }
  }

  if (orderinfo[pickorder_index].laterbox_index.size() > 0)
  {
    orderinfo[pickorder_index].order_status = ORDER_LATER;
    ROS_INFO("ORDER_LATER ... (pickorder_%d)", pickorder_index);
    return ORDER_LATER;
  }
  ROS_INFO("cannot select box (pickorder_%d)", pickorder_index);
  return ORDER_NONE;
}
*/

/*
bool selectPickItem(uint pickorder_index, uint box_index, uint *item_index, uint *to_box_index)
{
  OrderStatus status;
  ROS_INFO("selectPickItem (pickorder_index=%d, box_index_%d) start", pickorder_index, box_index);
  double probability = -1;
  uint i, j;

  //  printBoxinfo(box_index);

  status = boxinfo[box_index].order_status[pickorder_index];

  if (status == ORDER_EXIST)
  {
#if 1  // probabilityのチェックを外す（一時的に）
    for (i = 0; i < boxinfo[box_index].item_name.size(); i++)
    {
      if ((boxinfo[box_index].order_no[i] == orderinfo[pickorder_index].order_no) &&
          (boxinfo[box_index].status[i] == ITEM_RECOG_OK))
      {
        *item_index = i;
        *to_box_index = orderinfo[pickorder_index].to_box_index;
        ROS_INFO("selectPickItem() success,  item(pickorder_index:%d, box_index:%d, item_index=%d", pickorder_index,
                 box_index, *item_index);

        return true;
      }
    }

#else
    for (i = 0; i < boxinfo[box_index].item_name.size(); i++)
    {
      if ((boxinfo[box_index].order_no[i] == orderinfo[pickorder_index].order_no) &&
          (boxinfo[box_index].status[i] == ITEM_RECOG_OK))
      {
        if (probability < boxinfo[box_index].item_data[i].probability)
        {
          probability = boxinfo[box_index].item_data[i].probability;
          *item_index = i;
          *to_box_index = orderinfo[pickorder_index].to_box_index;
        }
        ROS_INFO("selectPickItem() success,  item(pickorder_index:%d, box_index:%d, item_index=%d", pickorder_index,
                 box_index, *item_index);

        return true;
      }
    }
#endif
    if (probability == -1)
    {
      orderinfo[pickorder_index].laterbox_index.push_back(box_index);
      boxinfo[box_index].order_status[pickorder_index] = ORDER_LATER;
      ROS_INFO("selectPickItem() failed, probability=%+lf", boxinfo[box_index].item_data[i].probability);
      return false;
    }
    else
    {
      ROS_INFO("selectPickItem() success,  item(pickorder_index:%d, box_index:%d, item_index=%d", pickorder_index,
               box_index, *item_index);
      return true;
    }
  }

  ROS_INFO("selectPickItem() failed");
  return false;
}
*/

bool isLater()
{
  return is_later_;
}

bool waitForRecognition(uint box_index, uint timeout)
{
  ROS_INFO("waitForRecognition(box_index=%u, timeout=%u) start", box_index, timeout);
  ros::Time start = ros::Time::now();
  while (RECOG_NOTYET == boxinfo[box_index].recog_status)
  {
    if (ros::Time::now() - start > ros::Duration(timeout))
    {
      ROS_ERROR("waitForRecognition(box_index=%u, timeout=%u) fail", box_index, timeout);
      return false;
    }
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("waitForRecognition(box_index=%u, timeout=%u) end", box_index, timeout);
  return true;
}

bool waitForPickPlan(uint place_id, uint item_index)
{
  ROS_INFO("waitForPickPlan(place_id=%u, item_index=%u) start", place_id, item_index);
  uint box_index = place_id_to_index_[place_id];
  const Boxinfo_t& box_info = boxinfo[box_index];
  while (PLAN_NOTYET == box_info.item_plan_status[item_index])
  {
    ros::Duration(0.1).sleep();
  }
  if (PLAN_NG == box_info.item_plan_status[item_index])
  {
    ROS_ERROR("waitForPickPlan(place_id=%u, item_index=%u) failed", place_id, item_index);
    return false;
  }
  ROS_INFO("waitForPickPlan(place_id=%u, item_index=%u) end", place_id, item_index);
  return true;
}

bool waitForStowPlan()
{
  ROS_INFO("waitForStowPlan() start");
  Boxinfo_t& tote = boxinfo[place_id_to_index_[D_tote_1_ID]];
  while (PLAN_OK != tote.plan_status)
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("waitForStowPlan() end");
  return true;
}

// print the designated boxinfo.
void printBoxinfo(uint box_index)
{
  uint i, j, itemcount;

  ROS_INFO("boxinfo[%d]***************", box_index);
  ROS_INFO("box_name=%s, box_id=%d, boxvolume=%lf, volume_of_items=%lf, recog_status=%d, recog_jobno=%d",
           boxinfo[box_index].box_name.c_str(), boxinfo[box_index].box_id, boxinfo[box_index].boxvolume,
           boxinfo[box_index].volume_of_items, boxinfo[box_index].recog_status, boxinfo[box_index].item_recog_jobno);

  itemcount = (uint)boxinfo[box_index].item_name.size();
  printf("itemcount=%d\n", itemcount);

  if (itemcount > 0)
  {
    for (i = 0; i < itemcount; i++)
    {
      ROS_INFO("item_name=%s, cad_id=%d, volume=%lf, item_order=%d,  status=%d, "
               "recog_index=%d,grasp_point_index=%d, probability=%+lf, item_recog_status=%d",
               boxinfo[box_index].item_name[i].c_str(), boxinfo[box_index].cad_id[i], boxinfo[box_index].volume[i],
               boxinfo[box_index].item_order[i], boxinfo[box_index].status[i], boxinfo[box_index].recog_index[i],
               boxinfo[box_index].grasp_point_index[i], boxinfo[box_index].item_data[i].probability, boxinfo[box_index].item_recog_status[i]);

      /*
      for (j = 0; j < boxinfo[box_index].grasp_release_points[i].size(); j++)
      {
        ROS_INFO("item_data[%d].grasp_release_points.score=%f", i, boxinfo[box_index].grasp_release_points[i][j].score);
      }
      */
    }
  }
  return;
}

void printAllBoxinfo()
{
  uint i;
  for (i = 0; i < boxinfo.size(); i++)
  {
    if (!boxinfo[i].enabled)
    {
      continue;
    }
    printBoxinfo(i);
  }
  return;
}

void printOrderinfo(uint order_index)
{
  printf("\n");
  uint i, temp;

  if (orderinfo[order_index].order_status == ORDER_DONE)
  {
    ROS_INFO("order_pick/work_order%d (orderinfo[%d] to box[size_id=%s])************* DONE! (%d)",
             orderinfo[order_index].order_no, order_index,
             boxinfo[orderinfo[order_index].to_box_index].box_name.c_str(), orderinfo[order_index].order_status);
  }
  else if (orderinfo[order_index].order_status == ORDER_NONE)
  {
    ROS_INFO("order_pick/work_order%d (orderinfo[%d] to box[size_id=%s])************* NO ORDER (%d)",
             orderinfo[order_index].order_no, order_index,
             boxinfo[orderinfo[order_index].to_box_index].box_name.c_str(), orderinfo[order_index].order_status);
  }
  else if (orderinfo[order_index].order_status == ORDER_EXIST)
  {
    ROS_INFO("order_pick/work_order%d (orderinfo[%d] to box[size_id=%s])************* pick on-going (%d)",
             orderinfo[order_index].order_no, order_index,
             boxinfo[orderinfo[order_index].to_box_index].box_name.c_str(), orderinfo[order_index].order_status);
  }
  else if (orderinfo[order_index].order_status == ORDER_LATER)
  {
    ROS_INFO("order_pick/work_order%d (orderinfo[%d] to box[size_id=%s])************* move between bins on-going (%d)",
             orderinfo[order_index].order_no, order_index,
             boxinfo[orderinfo[order_index].to_box_index].box_name.c_str(), orderinfo[order_index].order_status);
  }
  else if (orderinfo[order_index].order_status == ORDER_FAIL)
  {
    ROS_WARN("order_pick/work_order%d (orderinfo[%d] to box[size_id=%s])************* Fail!! (%d)",
             orderinfo[order_index].order_no, order_index,
             boxinfo[orderinfo[order_index].to_box_index].box_name.c_str(), orderinfo[order_index].order_status);
  }

  temp = orderinfo[order_index].item_name.size();
  for (i = 0; i < temp; i++)
  {
    ROS_INFO("%s (item_index:%d) from box_id:%d(box_index:%d)  status:%d", orderinfo[order_index].item_name[i].c_str(),
             orderinfo[order_index].item_index[i], boxinfo[orderinfo[order_index].from_box_index[i]].box_id,
             orderinfo[order_index].from_box_index[i], orderinfo[order_index].status[i]);
  }

  /*
  temp = orderinfo[order_index].laterbox_index.size();
  for (i = 0; i < temp; i++)
  {
    ROS_INFO("laterbox_index[%d]=%d", i, orderinfo[order_index].laterbox_index[i]);
  }
  */

  return;
}

void printAllOrderinfo()
{
  for (std::size_t i = 0; i < orderinfo.size(); i++)
  {
    printOrderinfo(i);
  }
  return;
}

void printPickOrder(int index)
{
  PickOrderInfo& pick_order = pick_order_[index];

  std::string order_status;
  switch (pick_order.status)
  {
    case ORDER_EXIST:
      order_status = "ORDER_EXIST";
      break;
    case ORDER_NONE:
      order_status = "ORDER_NONE";
      break;
    case ORDER_DONE:
      order_status = "ORDER_DONE";
      break;
    case ORDER_FAIL:
      order_status = "ORDER_FAIL";
      break;
    case ORDER_LATER:
      order_status = "ORDER_LATER";
      break;
    default:
      order_status = "UNKNOWN";
      break;
  }

  Orderinfo_t* order_info;
  getOrderInfo(pick_order.order_no, &order_info);

  uint from_box_index = order_info->from_box_index[pick_order.content_index];
  uint to_box_index = order_info->to_box_index;
  uint item_index = order_info->item_index[pick_order.content_index];
  std::string from_box_name = boxinfo[from_box_index].box_name;
  std::string to_box_name = boxinfo[to_box_index].box_name;

  ROS_INFO("pick_order[%d] (order_no=%u, contents[%u]) *** %s",
      index, pick_order.order_no, pick_order.content_index, order_status.c_str());
  ROS_INFO("box[%u]:(name=%s) -> box[%u]:(name=%s) %s (item_index=%u)",
      from_box_index, from_box_name.c_str(), to_box_index, to_box_name.c_str(),
      order_info->item_name[pick_order.content_index].c_str(), item_index);
}

void printAllPickOrder()
{
  for (int i = 0; i < pick_order_.size(); i++)
  {
    printPickOrder(i);
  }
}

void writeItemLocationJson(const std::string& filename)
{
  ROS_INFO("writeItemLocationJson(filename=%s) start", filename.c_str());

  std::string path = ros::package::getPath("t2_database");
  path.append("/order/output");

  struct stat stat_dir;
  if (-1 == stat(path.c_str(), &stat_dir))
  {
    mkdir(path.c_str(), 0775);
  }

  path.append("/" + filename);

  Json::Value root;

  // bins
  for (int i = 0; i < boxinfo.size(); i++)
  {
    const Boxinfo_t& bin = boxinfo[i];
    if (BoxTypes::Bin != bin.type)
    {
      continue;
    }
    Json::Value info_node;
    info_node["bin_id"] = bin.box_name;
    info_node["contents"] = Json::Value(Json::arrayValue);
    Json::Value &contents = info_node["contents"];
    for (int j = 0; j < bin.item_name.size(); j++)
    {
      if (bin.status[j] != ITEM_PICKED && bin.item_recog_status[j] == ITEM_NORMAL)
      {
        contents.append(bin.item_name[j]);
      }
    }
    root["bins"].append(info_node);
  }

  // boxes
  root["boxes"] = Json::Value(Json::arrayValue);
  for (int i = 0; i < boxinfo.size(); i++)
  {
    Boxinfo_t &box = boxinfo[i];
    if (BoxTypes::Box != box.type)
    {
      continue;
    }
    Json::Value info_node;
    info_node["*1*size_id"] = box.box_name;
    info_node["*2*contents"] = Json::Value(Json::arrayValue);
    Json::Value &contents = info_node["*2*contents"];
    for (auto it = box.item_name.begin(); it != box.item_name.end(); ++it)
    {
      contents.append(*it);
    }
    root["boxes"].append(info_node);
  }

  // tote
  for (int i = 0; i < boxinfo.size(); i++)
  {
    Boxinfo_t &tote = boxinfo[i];
    if (BoxTypes::Tote != tote.type)
    {
      continue;
    }
    root["tote"]["contents"] = Json::Value(Json::arrayValue);
    Json::Value &contents = root["tote"]["contents"];
    for (int j = 0; j < tote.item_name.size(); j++)
    {
      if (tote.status[j] != ITEM_PICKED && tote.item_recog_status[j] == ITEM_NORMAL)
      {
        contents.append(tote.item_name[j]);
      }
    }
  }

  Json::StyledWriter writer;
  std::string result_output = writer.write(root);

  result_output = replaceAllString(result_output, "*1*", "");
  result_output = replaceAllString(result_output, "*2*", "");

  std::ofstream ofstream;
  ofstream.open(path.c_str(), std::ios::out);
  ofstream << result_output;
  ofstream.close();

  ROS_INFO("writeItemLocationJson(filename=%s) end", filename.c_str());
}

std::string replaceAllString(std::string str, const std::string &from, const std::string &to)
{
  size_t start_pos = 0;
  while (true)
  {
    start_pos = str.find(from.c_str(), start_pos);
    if (start_pos != std::string::npos)
    {
      str.replace(start_pos, from.length(), to);
    }
    else
    {
      break;
    }
  }
  return str;
}
