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

#define T2_TASK_STRATEGY_VIEWER_SRC
#include "t2_task_planner/t2_task_strategy_viewer.h"

/* システムヘッダ参照 */
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <octomap_msgs/conversions.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>

/* 外部ヘッダ参照 */
#include "t2_task_planner/t2_task_strategy_common.h"
#include "t2_task_planner/t2_task_strategy_grasp_pack.h"

/* 内部定数定義 */

/* 内部変数定義 */
static ros::NodeHandle *nhP;
static std::string cur_item_id;
static int draw_target_idx;

static uint in_cad_id;
static uint single_gp;
static geometry_msgs::Pose in_center_pose;
static std::vector<uint> in_place_id_list;
static std::vector<GpRp_t> out_gprp;

static uint stow_in_cad_id;
static geometry_msgs::Pose stow_in_center_pose;
static std::vector<Location_t> stow_in_grasp_location_list;
static std::vector<Location_t> stow_in_release_location_list;
static std::vector<uint> stow_in_place_id_list;

static uint pick_in_cad_id;
static geometry_msgs::Pose pick_in_center_pose;
static std::vector<uint> pick_in_place_id_list;

static ros::ServiceClient gps_sc;
static moveit_msgs::GetPlanningScene gps_srv_msg;

static ros::ServiceClient aps_sc;
static moveit_msgs::ApplyPlanningScene aps_srv_msg;
static moveit_msgs::ObjectColor oc;

static ros::Publisher pub_center;
static ros::Publisher pub_gp;
static ros::Publisher pub_ap;
static geometry_msgs::PoseStamped tpc_msg;

static ros::Publisher pub_points, pub_axes, pub_texts;
static visualization_msgs::Marker points, axis, text;
static visualization_msgs::MarkerArray axes, texts;

/* 内部関数定義 */
static void setDummyBox(uint place_id, double dim_x, double dim_y, double dim_z);
static void defaultSetOctomap(std::vector<uint> place_id_list);
static void key_execute_stow_fix(void);
static void key_execute_stow_prediction(void);
static void key_execute_pick(void);
static void key_chg_set(void);
static void key_chg_msg_item(void);
static void key_chg_msg_mat(void);
static void key_chg_msg_qua(void);
static void key_print(void);
static void key_chg_msg_place(void);
static void key_chg_single_gp(void);
static void key_set_vox(void);
static void key_print_place(void);
static void key_plot_out(void);
static bool add_item_rviz(void);
static bool rm_item_rviz(void);
static bool add_item_rviz_rp(void);
static geometry_msgs::Pose calcRcf(void);
static bool rm_item_rviz_rp(void);
static void draw_pose(void);
static void draw_all_gp(void);


/****************/
/*     main     */
/****************/
int main(int argc, char **argv)
{
  int key;
  
  ros::init(argc, argv, "t2_task_strategy_viewer");
  
  /* ノードハンドル登録 */
  ros::NodeHandle nh;
  nhP = &nh;
  
  /* 各種設定値を初期化 */
  cur_item_id = "";                                             /* 表示中アイテムIDの初期化 */
  draw_target_idx = 0;                                          /* 描画対象のインデックス */
  
  /* defaultポジション */
  /* updateOctomap */
  std::vector<uint> usePlaceList;
  usePlaceList.push_back(1);
  usePlaceList.push_back(2);
  usePlaceList.push_back(3);
  usePlaceList.push_back(4);
  usePlaceList.push_back(5);
  usePlaceList.push_back(12);
  usePlaceList.push_back(14);
  setDummyBox(14, 0.343, 0.279, 0.121);
  defaultSetOctomap(usePlaceList);
  //printPlaceStAll(); //★debug
  /* Stow */
  stow_in_cad_id = 9002;
  stow_in_center_pose.position.x = 1.6560;
  stow_in_center_pose.position.y = 1.7435;
  stow_in_center_pose.position.z = 0.76135;
  stow_in_center_pose.orientation.x = 0.0;
  stow_in_center_pose.orientation.y = 0.0;
  stow_in_center_pose.orientation.z = 0.0;
  stow_in_center_pose.orientation.w = 1.0;
  stow_in_grasp_location_list.clear();
  for (int i=0; i<5; i++)
  {
    Location_t tmpLoc;
    tmpLoc.place_id = (uint)(1 + i);
    stow_in_release_location_list.push_back(tmpLoc);
  }
  stow_in_place_id_list.push_back(1);
  stow_in_place_id_list.push_back(2);
  stow_in_place_id_list.push_back(3);
  stow_in_place_id_list.push_back(4);
  stow_in_place_id_list.push_back(5);
  /* Pick */
  single_gp = 0;
  pick_in_cad_id = 9002;
  pick_in_center_pose.position.x = 0.7500;
  pick_in_center_pose.position.y = 1.6650;
  pick_in_center_pose.position.z = 0.4000;
  pick_in_center_pose.orientation.x = 0.0;
  pick_in_center_pose.orientation.y = 0.0;
  pick_in_center_pose.orientation.z = 0.0;
  pick_in_center_pose.orientation.w = 1.0;
  pick_in_place_id_list.push_back(14);                          /* BOX2 */
  //pick_in_place_id_list.push_back(1);                           /* BINA */
  
  /* サービス */
  /* GetPlanningScene */
  gps_sc = nhP->serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
  /* defaultメッセージ */
  gps_srv_msg.request.components.components = moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;
  
  /* ApplyPlanningScene */
  aps_sc = nhP->serviceClient<moveit_msgs::ApplyPlanningScene>(move_group::APPLY_PLANNING_SCENE_SERVICE_NAME);
  /* defaultメッセージ */
  aps_srv_msg.request.scene.is_diff = true;
  oc.color.r = 0.0;
  oc.color.g = 1.0;
  oc.color.b = 0.0;
  oc.color.a = 0.5;
  
  /* トピック */
  /* PoseStamped */
  pub_center = nhP->advertise<geometry_msgs::PoseStamped>(CENTER_TPC_MSG, 1);
  pub_gp = nhP->advertise<geometry_msgs::PoseStamped>(GP_TPC_MSG, 1);
  pub_ap = nhP->advertise<geometry_msgs::PoseStamped>(AP_TPC_MSG, 1);
  /* defaultメッセージ */
  tpc_msg.header.frame_id = "world";

  /* マーカー初期設定 */
  pub_points = nhP->advertise<visualization_msgs::Marker>("t2_grasp_planner/points", 1);
  points.header.frame_id = "world";
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.pose.orientation.w = 1.0;
  points.pose.position.x = 0.0;
  points.pose.position.y = 0.0;
  points.pose.position.z = 0.0;
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.scale.z = 0.01;

  pub_axes = nhP->advertise<visualization_msgs::MarkerArray>("t2_grasp_planner/axes", 1);
  axis.header.frame_id = "world";
  axis.type = visualization_msgs::Marker::CYLINDER;
  axis.pose.orientation.x = 0.0;
  axis.pose.orientation.y = 0.0;
  axis.pose.orientation.z = 0.0;
  axis.pose.orientation.w = 1.0;
  axis.pose.position.x = 0.0;
  axis.pose.position.y = 0.0;
  axis.pose.position.z = 0.0;
  axis.scale.x = 0.003;  
  axis.scale.y = 0.003;  
  axis.scale.z = 0.02;  

  pub_texts = nhP->advertise<visualization_msgs::MarkerArray>("t2_grasp_planner/texts", 1);
  text.header.frame_id = "world";
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.pose.orientation.x = 0.0;
  text.pose.orientation.y = 0.0;
  text.pose.orientation.z = 0.0;
  text.pose.orientation.w = 1.0;
  text.pose.position.x = 0.0;
  text.pose.position.y = 0.0;
  text.pose.position.z = 0.0;
  text.scale.z = 0.02;  
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.b = 1.0;
  text.color.a = 1.0;  
  
  ROS_INFO("Run t2_task_strategy_viewer Node.");
  
  
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    printf("(Stow(Fix):1, Pick:2, Stow(Prediction):3, GPTest:4, Quit:99)\n");
    printf("Input Key > ");
    scanf("%d", &key);
    if (key == 99){ break; }
    else if (key == 1)
    {
      /* Stow */
      in_cad_id = stow_in_cad_id;
      in_center_pose = stow_in_center_pose;
      in_place_id_list = stow_in_place_id_list;
      
      while (ros::ok())
      {
        printf("(Execute:1, ChgSet:2, ChgMsgItem:3, ChgMsgMat:4, ChgMsgQua:5, Print:6, Quit:99)\n");
        printf("Input Key > ");
        scanf("%d", &key);
        if (key == 99){ break; }
        else if (key == 1){ key_execute_stow_fix(); }
        else if (key == 2){ key_chg_set(); }
        else if (key == 3){ key_chg_msg_item(); }
        else if (key == 4){ key_chg_msg_mat(); }
        else if (key == 5){ key_chg_msg_qua(); }
        else if (key == 6){ key_print(); }
        else { ; /* 何もしない */ }
        
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    else if (key == 2)
    {
      /* Pick */
      in_cad_id = pick_in_cad_id;
      in_center_pose = pick_in_center_pose;
      in_place_id_list = pick_in_place_id_list;
      
      while (ros::ok())
      {
        printf("(Execute:1, ChgSet:2, ChgSingleGp:8, Print:6, Quit:99)\n");
        printf("(ChgMsgItem:3, ChgMsgMat:4, ChgMsgQua:5)\n");
        printf("(SetVox:80, PrintPlace:60, PlotOut:50)\n");
        printf("Input Key > ");
        scanf("%d", &key);
        if (key == 99){ break; }
        else if (key ==  1){ key_execute_pick(); }
        else if (key ==  2){ key_chg_set(); }
        else if (key ==  3){ key_chg_msg_item(); }
        else if (key ==  4){ key_chg_msg_mat(); }
        else if (key ==  5){ key_chg_msg_qua(); }
        else if (key ==  6){ key_print(); }
        else if (key ==  8){ key_chg_single_gp(); }
        else if (key == 80){ key_set_vox(); }
        else if (key == 60){ key_print_place(); }
        else if (key == 50){ key_plot_out(); }
        else { ; /* 何もしない */ }
        
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    else if (key == 3)
    {
      /* Stow */
      in_cad_id = stow_in_cad_id;
      in_center_pose = stow_in_center_pose;
      in_place_id_list = stow_in_place_id_list;
      
      while (ros::ok())
      {
        printf("(Execute:1, ChgSet:2, Print:6, Quit:99)\n");
        printf("(ChgMsgItem:3, ChgMsgMat:4, ChgMsgQua:5, ChgMsgPlace:7)\n");
        printf("(SetVox:80, PrintPlace:60, PlotOut:50)\n");
        printf("Input Key > ");
        scanf("%d", &key);
        if (key == 99){ break; }
        else if (key ==  1){ key_execute_stow_prediction(); }
        else if (key ==  2){ key_chg_set(); }
        else if (key ==  3){ key_chg_msg_item(); }
        else if (key ==  4){ key_chg_msg_mat(); }
        else if (key ==  5){ key_chg_msg_qua(); }
        else if (key ==  6){ key_print(); }
        else if (key ==  7){ key_chg_msg_place(); }
        else if (key == 80){ key_set_vox(); }
        else if (key == 60){ key_print_place(); }
        else if (key == 50){ key_plot_out(); }
        else { ; /* 何もしない */ }
        
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    else if (key == 4)
    {
      /* GPTest */
      in_cad_id = pick_in_cad_id;
      in_center_pose = pick_in_center_pose;
      in_place_id_list = pick_in_place_id_list;

      /* Place先は十分大きいBoxを想定 */
      setDummyBox(14, 1.000, 1.000, 1.000);
      defaultSetOctomap(usePlaceList);

      /* 把持点増加機能をOFFにする */
      XmlRpc::XmlRpcValue matParam;                      /* yaml読込用 */
      bool ret;  
      ret = nhP->getParam("t2_task_planner/task_strategy", matParam);
      matParam["gp_offset_z_suction"] = XmlRpc::XmlRpcValue(false);
      matParam["gp_offset_z_pinch"] = XmlRpc::XmlRpcValue(false);
      matParam["gp_increase_z"] = XmlRpc::XmlRpcValue(false);
      matParam["gp_increase_rot"] = XmlRpc::XmlRpcValue(false);
      matParam["delete_score_zero"] = XmlRpc::XmlRpcValue(false);
      nhP->setParam("t2_task_planner/task_strategy", matParam);

      while (ros::ok())
      {
        printf("(Execute:1, ChgSet:2, ChgMsgItem:3, ChgMsgMat:4, ChgMsgQua:5, Print:6, ChgScore:7, Quit:99)\n");
        printf("(SetVox:80, PrintPlace:60, PlotOut:50)\n");
        printf("Input Key > ");
        scanf("%d", &key);
        if (key == 99){ break; }
        else if (key ==  1)
        { 
          /* 全把持点表示 */
          system("roslaunch t2_database t2_database.launch");
          key_execute_pick();
          draw_all_gp();
        }
        else if (key ==  2){ key_chg_set(); }
        else if (key ==  3)
        { 
          key_chg_msg_item();

          /* 全把持点表示 */
          system("roslaunch t2_database t2_database.launch");
          key_execute_pick();
          draw_all_gp();
        }
        else if (key ==  4){ key_chg_msg_mat(); }
        else if (key ==  5){ key_chg_msg_qua(); }
        else if (key ==  6){ key_print(); }
        else if (key ==  7)
        {
          /* 現在アイテムの把持情報リストを読み込み */
          XmlRpc::XmlRpcValue infoParam;                           /* yaml読込用 */
          std::ostringstream oss;
          oss << (int)in_cad_id;
          std::string grasp_list = "t2_database/grasp_info_list/cad" + oss.str();
          ret = nhP->getParam(grasp_list.c_str(), infoParam);
          
          /* スコアを変更するgp_numberを指定 */
          std::cout << "Item CAD id = " << (int)in_cad_id << std::endl;
          std::cout << "Target GP number = ";
          unsigned int target_gp_number;
          scanf("%d", &target_gp_number);

          /* gp_numberからGPのインデックスを検索 */
          unsigned int target_gp_i;
          unsigned int total_point = (uint32_t)((int)infoParam["gp_num_for_suction"]) + (uint32_t)((int)infoParam["gp_num_for_pinch"]);
          for (target_gp_i = 0; target_gp_i < total_point; target_gp_i++)
          {
            if ((unsigned int)(int)infoParam["gp"][target_gp_i]["gp_number"] == target_gp_number) break;
          }
          if (target_gp_i >= total_point) 
          {
            ROS_ERROR("Can't find target GP number.");
            continue;
          }

          /* 新しいスコアを指定 */
          std::cout << "Current score of GP(" << target_gp_number << ") = " << infoParam["gp"][target_gp_i]["score"] << std::endl;
          std::cout << "New score of GP(" << target_gp_number << ") = ";
          double new_score;
          scanf("%lf", &new_score);         
          if (new_score < 0.0 || new_score > 1.0) 
          {
            ROS_ERROR("Invalid score value.");
            continue;
          }

          /* パラメータ書き換え */
          infoParam["gp"][target_gp_i]["score"] = XmlRpc::XmlRpcValue(new_score);
          nhP->setParam(grasp_list.c_str(), infoParam);

          /* 新しいパラメータをyamlに書き出す */
          std::ostringstream oss_grasp_info_path;
          oss_grasp_info_path << ros::package::getPath("t2_database") << "/data/grasp_info_list/" << in_cad_id << "_" << (std::string)infoParam["item_name"] << ".yaml";
          std::ostringstream oss_cmd;
          oss_cmd << "rosparam dump " << oss_grasp_info_path.str() << " " << grasp_list;
          system(oss_cmd.str().c_str());

          /* 全把持点表示 */
          system("roslaunch t2_database t2_database.launch");
          key_execute_pick();
          draw_all_gp();          
        }
        else if (key == 80){ key_set_vox(); }
        else if (key == 60){ key_print_place(); }
        else if (key == 50){ key_plot_out(); }
        else { ; /* 何もしない */ }
        
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    else { ; /* 何もしない */ }
  }
  
  return 0;
}

/****************/
/* 内部関数処理 */
/****************/
static void setDummyBox(uint place_id, double dim_x, double dim_y, double dim_z)
{
  uint containerIdx = UINT_MAX;
  
  XmlRpc::XmlRpcValue containerParam;                           /* yaml読込用 */
  bool ret = nhP->getParam("t2_database/container_info", containerParam);
  if (ret == false)
  {
    /* パラメータロードエラー */
    ROS_ERROR("ParameterServer(container_info) get error.");
  }
  else
  {
    /* 該当place_idが、container_infoのどのindexか検索する */
    for (std::size_t i=0; i<containerParam.size(); i++)
    {
      if ((int)containerParam[i]["place_id"] == (int)place_id)
      {
        containerIdx = (uint)i;
        break;
      }
    }
    if (containerIdx == UINT_MAX)
    {
      ROS_ERROR("ParameterServer(container_info) can't find place_id.");
    }
    else
    {
      XmlRpc::XmlRpcValue dim_param;
      dim_param.setSize(3);
      dim_param[0] = XmlRpc::XmlRpcValue(dim_x);
      dim_param[1] = XmlRpc::XmlRpcValue(dim_y);
      dim_param[2] = XmlRpc::XmlRpcValue(dim_z);
      containerParam[containerIdx]["inside_dimensions"] = dim_param;
      nhP->setParam("t2_database/container_info", containerParam);
    }
  }
}

static void defaultSetOctomap(std::vector<uint> place_id_list)
{
  octomap_msgs::OctomapWithPose octomapwithpose;
  geometry_msgs::Pose pose_m;
  octomap_msgs::Octomap octomap;
  
  Eigen::Affine3d pose_e = Eigen::Affine3d::Identity();
  
  /* 空のOctomap生成 */
  octomap::OcTree tree(0.03);
  //tree.updateNode(point3d(0.05, 0.05, 0.05), true);           /* 何か初期配置する場合に行う */
  
  /* メッセージ化 */
  octomap_msgs::binaryMapToMsg(tree, octomap);
  printf("\n");
  
  for (int i=0; i<(int)place_id_list.size(); i++)
  {
    if (place_id_list[i] == 1)
    {
      pose_e.linear() <<
        -1.0,  0.0,    0.0,
         0.0,  0.707, -0.707,
         0.0, -0.707, -0.707;
      pose_e.translation().x() = 0.850;
      pose_e.translation().y() = 2.300;
      pose_e.translation().z() = 0.660;
    }
    else if (place_id_list[i] == 2)
    {
      pose_e.linear() <<
         0.0, -0.707,  0.707,
        -1.0,  0.0,    0.0,
         0.0, -0.707, -0.707;
      pose_e.translation().x() = 0.390;
      pose_e.translation().y() = 1.650;
      pose_e.translation().z() = 0.660;
    }
    else if (place_id_list[i] == 3)
    {
      pose_e.linear() <<
         0.0, -0.707,  0.707,
        -1.0,  0.0,    0.0,
         0.0, -0.707, -0.707;
      pose_e.translation().x() = 0.390;
      pose_e.translation().y() = 1.320;
      pose_e.translation().z() = 0.660;
    }
    else if (place_id_list[i] == 4)
    {
      pose_e.linear() <<
         0.0,  0.707, -0.707,
         1.0,  0.0,    0.0,
         0.0, -0.707, -0.707;
      pose_e.translation().x() = 1.310;
      pose_e.translation().y() = 1.650;
      pose_e.translation().z() = 0.660;
    }
    else if (place_id_list[i] == 5)
    {
      pose_e.linear() <<
         0.0,  0.707, -0.707,
         1.0,  0.0,    0.0,
         0.0, -0.707, -0.707;
      pose_e.translation().x() = 1.310;
      pose_e.translation().y() = 1.320;
      pose_e.translation().z() = 0.660;
    }
    else if (place_id_list[i] == 12)
    {
      pose_e.linear() <<
         0.0,  0.707, -0.707,
         1.0,  0.0,    0.0,
         0.0, -0.707, -0.707;
      pose_e.translation().x() = 1.970;
      pose_e.translation().y() = 0.950;
      pose_e.translation().z() = 1.100;
    }
    else if (place_id_list[i] == 14)
    {
      pose_e.linear() <<
         0.0,  0.707, -0.707,
         1.0,  0.0,    0.0,
         0.0, -0.707, -0.707;
      pose_e.translation().x() = 1.970;
      pose_e.translation().y() = 0.950;
      pose_e.translation().z() = 1.100;
    }
    else
    {
      ROS_ERROR("(defaultSetOctomap) Unsupport place_id.");
      break;
    }
    
    tf::poseEigenToMsg(pose_e, pose_m);
    
    octomapwithpose.origin = pose_m;
    octomapwithpose.octomap = octomap;
    updateOctomap(place_id_list[i], octomapwithpose);
  }
}

static void key_execute_stow_fix(void)
{
  bool ret = true;
  
  ROS_INFO("(key_execute_stow_fix) Execute id = %d", (int)in_cad_id);
  
  /* GPRP算出実行 */
  if (ret == true)
  {
    out_gprp.clear();
    ret = getGpRpFix(in_cad_id, (uint)0, in_center_pose, in_place_id_list, &out_gprp);
    if (ret == true)
    {
      ROS_INFO("(getGpRpFix) return = true");
      if (out_gprp.size() <= 0)
      {
        ret = false;
        ROS_ERROR("(getGpRpFix) Response Data Error.");
      }
    }
    else
    {
      ROS_ERROR("(getGpRpFix) return = false");
    }
  }
  
  /* 古いアイテムをRvizから削除 */
  if (ret == true)
  {
    ret = rm_item_rviz_rp();
    if (ret == false)
    {
      ROS_FATAL("(rm_item_rviz_rp) Remove Item Error.");
    }
  }
  if (ret == true)
  {
    ret = rm_item_rviz();
    if (ret == false)
    {
      ROS_FATAL("(rm_item_rviz) Remove Item Error.");
    }
  }
  
  /* 新しいアイテムをRvizに追加 */
  if (ret == true)
  {
    ret = add_item_rviz();
    if (ret == false)
    {
      ROS_FATAL("(add_item_rviz) Add Item Error.");
    }
  }
  if (ret == true)
  {
    ret = add_item_rviz_rp();
    if (ret == false)
    {
      ROS_FATAL("(add_item_rviz_rp) Add Item Error.");
    }
  }
  
  /* Center/GP/RPをRvizに表示 */
  if (ret == true)
  {
    draw_pose();
  }
}

static void key_execute_stow_prediction(void)
{
  bool ret = true;
  
  ROS_INFO("(key_execute_stow_prediction) Execute id = %d", (int)in_cad_id);
  
  /* GPRP算出実行 */
  if (ret == true)
  {
    out_gprp.clear();
    ret = getGpRpWithPrediction(in_cad_id, (uint)0, in_center_pose, stow_in_grasp_location_list, stow_in_release_location_list, in_place_id_list, &out_gprp);
    if (ret == true)
    {
      ROS_INFO("(getGpRpWithPrediction) return = true");
      if (out_gprp.size() <= 0)
      {
        ret = false;
        ROS_ERROR("(getGpRpWithPrediction) Response Data Error.");
      }
    }
    else
    {
      ROS_ERROR("(getGpRpWithPrediction) return = false");
    }
  }
  
  /* 古いアイテムをRvizから削除 */
  if (ret == true)
  {
    ret = rm_item_rviz_rp();
    if (ret == false)
    {
      ROS_FATAL("(rm_item_rviz_rp) Remove Item Error.");
    }
  }
  if (ret == true)
  {
    ret = rm_item_rviz();
    if (ret == false)
    {
      ROS_FATAL("(rm_item_rviz) Remove Item Error.");
    }
  }
  
  /* 新しいアイテムをRvizに追加 */
  if (ret == true)
  {
    ret = add_item_rviz();
    if (ret == false)
    {
      ROS_FATAL("(add_item_rviz) Add Item Error.");
    }
  }
  if (ret == true)
  {
    ret = add_item_rviz_rp();
    if (ret == false)
    {
      ROS_FATAL("(add_item_rviz_rp) Add Item Error.");
    }
  }
  
  /* Center/GP/RPをRvizに表示 */
  if (ret == true)
  {
    draw_pose();
  }
}

static void key_execute_pick(void)
{
  bool ret = true;
  
  ROS_INFO("(key_execute_pick) Execute id = %d", (int)in_cad_id);
  
  /* GPRP算出実行 */
  if (ret == true)
  {
    out_gprp.clear();
    ret = getGpRpWithoutPrediction(in_cad_id, single_gp, in_center_pose, in_place_id_list, &out_gprp);
    if (ret == true)
    {
      ROS_INFO("(getGpRpWithoutPrediction) return = true");
      if (out_gprp.size() <= 0)
      {
        ret = false;
        ROS_ERROR("(getGpRpWithoutPrediction) Response Data Error.");
      }
    }
    else
    {
      ROS_ERROR("(getGpRpWithoutPrediction) return = false");
    }
  }
  
  /* 古いアイテムをRvizから削除 */
  if (ret == true)
  {
    ret = rm_item_rviz_rp();
    if (ret == false)
    {
      ROS_FATAL("(rm_item_rviz_rp) Remove Item Error.");
    }
  }
  if (ret == true)
  {
    ret = rm_item_rviz();
    if (ret == false)
    {
      ROS_FATAL("(rm_item_rviz) Remove Item Error.");
    }
  }
  
  /* 新しいアイテムをRvizに追加 */
  if (ret == true)
  {
    ret = add_item_rviz();
    if (ret == false)
    {
      ROS_FATAL("(add_item_rviz) Add Item Error.");
    }
  }
  if (ret == true)
  {
    ret = add_item_rviz_rp();
    if (ret == false)
    {
      ROS_FATAL("(add_item_rviz_rp) Add Item Error.");
    }
  }
  
  /* Center/GP/RPをRvizに表示 */
  if (ret == true)
  {
    draw_pose();
  }
}

static void key_chg_set(void)
{
  printf("Input Draw Target Index (0:Score Max, [0]>[1]>[2]>...>[total_point-1]) > ");
  scanf("%d", &draw_target_idx);
}

static void key_chg_msg_item(void)
{
  std::cout << "Before cad_id = " << in_cad_id << std::endl;
  
  printf("Input cad_id (uint32) > ");
  scanf("%d", &in_cad_id);
  
  std::cout << "After cad_id = " << in_cad_id << std::endl;
}

static void key_chg_msg_mat(void)
{
  int key;
  
  Eigen::Affine3d center_pose;
  tf::poseMsgToEigen(in_center_pose, center_pose);
  
  std::cout << "Before" << std::endl;
  std::cout << center_pose.matrix() << std::endl;
  
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    printf("Input Change Part Type (linear:1, translation:2, Quit:99) > ");
    scanf("%d", &key);
    if (key == 99)
    {
      break;
    }
    else if (key == 1)
    {
      Eigen::Affine3d tmp_pose = Eigen::Affine3d::Identity();
      double angle;
      
      printf("Input linear (default:0, manual:1\n");
      printf("              right:10, left:11, near:20, far:21, rotation:30) > ");
      scanf("%d", &key);
      if (key == 0)
      {
        /* default */
        center_pose.linear() = tmp_pose.linear();
      }
      else if (key == 1)
      {
        /* manual */
        printf("linear()(0,0) > ");
        scanf("%lf", &center_pose.linear()(0,0));
        printf("linear()(1,0) > ");
        scanf("%lf", &center_pose.linear()(1,0));
        printf("linear()(2,0) > ");
        scanf("%lf", &center_pose.linear()(2,0));
        printf("linear()(0,1) > ");
        scanf("%lf", &center_pose.linear()(0,1));
        printf("linear()(1,1) > ");
        scanf("%lf", &center_pose.linear()(1,1));
        printf("linear()(2,1) > ");
        scanf("%lf", &center_pose.linear()(2,1));
        printf("linear()(0,2) > ");
        scanf("%lf", &center_pose.linear()(0,2));
        printf("linear()(1,2) > ");
        scanf("%lf", &center_pose.linear()(1,2));
        printf("linear()(2,2) > ");
        scanf("%lf", &center_pose.linear()(2,2));
      }
      else if (key == 10)
      {
        /* right */
        printf("Input degree(0-90) > ");
        scanf("%lf", &angle);                                   /* [deg] */
        
        angle = (angle * M_PI) / 180;                           /* [rad]変換 */
        
        tmp_pose.linear() <<
          std::cos(angle), 0, std::sin(angle),
          0, 1, 0,
          -std::sin(angle), 0, std::cos(angle);
        
        center_pose.linear() = tmp_pose.linear() * center_pose.linear();
      }
      else if (key == 11)
      {
        /* left */
        printf("Input degree(0-90) > ");
        scanf("%lf", &angle);                                   /* [deg] */
        
        angle *= (-1);
        angle = (angle * M_PI) / 180;                           /* [rad]変換 */
        
        tmp_pose.linear() <<
          std::cos(angle), 0, std::sin(angle),
          0, 1, 0,
          -std::sin(angle), 0, std::cos(angle);
        
        center_pose.linear() = tmp_pose.linear() * center_pose.linear();
      }
      else if (key == 20)
      {
        /* near */
        printf("Input degree(0-90) > ");
        scanf("%lf", &angle);                                   /* [deg] */
        
        angle = (angle * M_PI) / 180;                           /* [rad]変換 */
        
        tmp_pose.linear() <<
          1, 0, 0,
          0, std::cos(angle), -std::sin(angle),
          0, std::sin(angle), std::cos(angle);
        
        center_pose.linear() = tmp_pose.linear() * center_pose.linear();
      }
      else if (key == 21)
      {
        /* far */
        printf("Input degree(0-90) > ");
        scanf("%lf", &angle);                                   /* [deg] */
        
        angle *= (-1);
        angle = (angle * M_PI) / 180;                           /* [rad]変換 */
        
        tmp_pose.linear() <<
          1, 0, 0,
          0, std::cos(angle), -std::sin(angle),
          0, std::sin(angle), std::cos(angle);
        
        center_pose.linear() = tmp_pose.linear() * center_pose.linear();
      }
      else if (key == 30)
      {
        /* rotation */
        printf("Input degree(0-360) > ");
        scanf("%lf", &angle);                                   /* [deg] */
        
        angle = (angle * M_PI) / 180;                           /* [rad]変換 */
        
        tmp_pose.linear() <<
          std::cos(angle), -std::sin(angle), 0,
          std::sin(angle), std::cos(angle), 0,
          0, 0, 1;
        
        center_pose.linear() = tmp_pose.linear() * center_pose.linear();
      }
      else
      {
        ; /* 何もしない */
      }
    }
    else if (key == 2)
    {
      printf("translation().x() > ");
      scanf("%lf", &center_pose.translation().x());
      printf("translation().y() > ");
      scanf("%lf", &center_pose.translation().y());
      printf("translation().z() > ");
      scanf("%lf", &center_pose.translation().z());
    }
    else
    {
      ; /* 何もしない */
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  std::cout << "After" << std::endl;
  std::cout << center_pose.matrix() << std::endl;
  
  tf::poseEigenToMsg(center_pose, in_center_pose);
}

static void key_chg_msg_qua(void)
{
  int key;
  
  std::cout << "Before" << std::endl;
  std::cout << in_center_pose << std::endl;
  
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    printf("Input Change Part Type (position:1, orientation:2, Quit:99) > ");
    scanf("%d", &key);
    if (key == 99)
    {
      break;
    }
    else if (key == 1)
    {
      printf("position.x > ");
      scanf("%lf", &in_center_pose.position.x);
      printf("position.y > ");
      scanf("%lf", &in_center_pose.position.y);
      printf("position.z > ");
      scanf("%lf", &in_center_pose.position.z);
    }
    else if (key == 2)
    {
      printf("orientation.x > ");
      scanf("%lf", &in_center_pose.orientation.x);
      printf("orientation.y > ");
      scanf("%lf", &in_center_pose.orientation.y);
      printf("orientation.z > ");
      scanf("%lf", &in_center_pose.orientation.z);
      printf("orientation.w > ");
      scanf("%lf", &in_center_pose.orientation.w);
    }
    else
    {
      ; /* 何もしない */
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  std::cout << "After" << std::endl;
  std::cout << in_center_pose << std::endl;
}

static void key_print(void)
{
  if ((int)out_gprp.size() > 0)
  {
    printf("cad_id = %d\n", (int)in_cad_id);
    printf("total_point = %d\n", (int)out_gprp.size());
    
    for (int i=0; i<(int)out_gprp.size(); i++)
    {
      printf("grasp_point[%d].score = %lf\n", i, out_gprp[i].score);
      printf("grasp_point[%d].gp_number = %d\n", i, (int)out_gprp[i].gp_number);
      printf("grasp_point[%d].grasp_pattern = %s\n", i, out_gprp[i].grasp_pattern.c_str());
    }
  }
  else
  {
    ROS_ERROR("(Print) May be not Execute yet.");
  }
}

static void key_chg_msg_place(void)
{
  int key;
  int place_id;
  
  printf("Before place_id_list = ");
  for (std::size_t i=0; i<in_place_id_list.size(); i++)
  {
    printf("%d, ", (int)in_place_id_list[i]);
  }
  printf("\n");
  
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    printf("Input Key (Default:0, Add:1, DelAll:2, Quit:99) > ");
    scanf("%d", &key);
    if (key == 99)
    {
      break;
    }
    else if (key == 0)
    {
      /* default */
      in_place_id_list = stow_in_place_id_list;
    }
    else if (key == 1)
    {
      /* add */
      printf("place_id (int) > ");
      scanf("%d", &place_id);
      in_place_id_list.push_back((uint)place_id);
    }
    else if (key == 2)
    {
      /* delete all */
      in_place_id_list.clear();
    }
    else
    {
      ; /* 何もしない */
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  printf("After place_id_list = ");
  for (std::size_t i=0; i<in_place_id_list.size(); i++)
  {
    printf("%d, ", (int)in_place_id_list[i]);
  }
  printf("\n");
}

static void key_chg_single_gp(void)
{
  printf("Input single_gp flag (0:normal, 1:single_gp) > ");
  scanf("%d", &single_gp);
}

static void key_set_vox(void)
{
  int key;
  
  int place_id;
  int coordRow, coordCol, coordHigh;                            /* 座標 */
  int sizeRow, sizeCol, sizeHigh;                               /* アイテムサイズ */
  
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    printf("Input Key (Add:1, Del:2, Quit:99) > ");
    scanf("%d", &key);
    if (key == 99)
    {
      break;
    }
    else if (key == 1)
    {
      /* add */
      printf("place_id (int) > ");
      scanf("%d", &place_id);
      
      printf("position.row (int) > ");
      scanf("%d", &coordRow);
      printf("position.col (int) > ");
      scanf("%d", &coordCol);
      printf("position.high (int) > ");
      scanf("%d", &coordHigh);
      
      printf("size.row (int) > ");
      scanf("%d", &sizeRow);
      printf("size.col (int) > ");
      scanf("%d", &sizeCol);
      printf("size.high (int) > ");
      scanf("%d", &sizeHigh);
      
      setVoxelForce((uint)place_id, coordRow, coordCol, coordHigh, sizeRow, sizeCol, sizeHigh, true);
    }
    else if (key == 2)
    {
      /* delete */
      printf("place_id (int) > ");
      scanf("%d", &place_id);
      
      printf("position.row (int) > ");
      scanf("%d", &coordRow);
      printf("position.col (int) > ");
      scanf("%d", &coordCol);
      printf("position.high (int) > ");
      scanf("%d", &coordHigh);
      
      printf("size.row (int) > ");
      scanf("%d", &sizeRow);
      printf("size.col (int) > ");
      scanf("%d", &sizeCol);
      printf("size.high (int) > ");
      scanf("%d", &sizeHigh);
      
      setVoxelForce((uint)place_id, coordRow, coordCol, coordHigh, sizeRow, sizeCol, sizeHigh, false);
    }
    else
    {
      ; /* 何もしない */
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}

static void key_print_place(void)
{
  int place_id;
  
  printf("place_id (int) > ");
  scanf("%d", &place_id);
  
  printPlaceSt((uint)place_id);
}

static void key_plot_out(void)
{
  int place_id;
  
  printf("place_id (int) > ");
  scanf("%d", &place_id);
  
  plotOut((uint)place_id);
}

#ifdef COMPILE_MARKER_TEST
static void key_add_marker(void)
{
  int key;
  
  marker.header.stamp = ros::Time();
  marker.action = visualization_msgs::Marker::ADD;
  
  printf("id (int) > ");
  scanf("%d", &marker.id);
  
  marker.points.clear();
  
  ros::Rate loop_rate(100);
  while (ros::ok()){
    printf("Input Key (Add:1, Quit:99) > ");
    scanf("%d", &key);
    if (key == 99)
    {
      break;
    }
    else if (key == 1)
    {
      /* add */
      geometry_msgs::Point point;
      printf("x (double) > ");
      scanf("%lf", &point.x);
      printf("y (double) > ");
      scanf("%lf", &point.y);
      printf("z (double) > ");
      scanf("%lf", &point.z);
      marker.points.push_back(point);
    }
    else
    {
      ; /* 何もしない */
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  pub_vis.publish(marker);
}

static void key_del_marker(void)
{
  marker.header.stamp = ros::Time();
  marker.action = visualization_msgs::Marker::DELETE;
  
  printf("id (int) > ");
  scanf("%d", &marker.id);
  
  marker.points.clear();
  
  pub_vis.publish(marker);
}
#endif /* COMPILE_MARKER_TEST */

static bool rm_item_rviz(void)
{
  moveit_msgs::CollisionObject co;
  bool ret;
  
  /* メッセージに削除要求を登録 */
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  co.id = cur_item_id;
  aps_srv_msg.request.scene.world.collision_objects.push_back(co);
  
  ret = aps_sc.call(aps_srv_msg);
  if (ret == true)
  {
    cur_item_id = "";
  }
  else
  {
    ROS_FATAL("(ApplyPlanningScene) Service Response Error.");
  }
  
  return ret;
}

static bool rm_item_rviz_rp(void)
{
  moveit_msgs::CollisionObject co;
  bool ret;
  
  /* メッセージに削除要求を登録 */
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  co.id = "rp_" + cur_item_id;
  aps_srv_msg.request.scene.world.collision_objects.push_back(co);
  
  ret = aps_sc.call(aps_srv_msg);
  if (ret == true)
  {
    //cur_item_id = "";                                         /* RPの方では触らない */
  }
  else
  {
    ROS_FATAL("(ApplyPlanningScene) Service Response Error.");
  }
  
  return ret;
}

static bool add_item_rviz(void)
{
  XmlRpc::XmlRpcValue infoParam;
  moveit_msgs::CollisionObject co;
  bool ret;
  
  /* 現在の状況を取得 */
  ret = gps_sc.call(gps_srv_msg);
  if (ret == true)
  {
    /* stlファイルを特定 */
    std::ostringstream oss;
    oss << (int)in_cad_id;
    std::string grasp_list = "t2_database/grasp_info_list/cad" + oss.str();
    ret = nhP->getParam(grasp_list.c_str(), infoParam);
    if (ret == true)
    {
      /* stlファイルを取り込み */
      std::string mesh_file = (std::string)infoParam["mesh_file"];
      mesh_file = "package://t2_database/meshes/items/" + mesh_file;
      Eigen::Vector3d eigen_scale(1.0, 1.0, 1.0);
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(shapes::createMeshFromResource(mesh_file, eigen_scale), shape_msg);
      
      /* アイテムのstlと姿勢をメッセージに登録 */
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.header.frame_id = "world";
      co.id = oss.str();
      co.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
      co.mesh_poses.push_back(in_center_pose);
      /* アイテムの色をメッセージに登録 */
      oc.id = oss.str();
      
      /* moveit経由でRvizに表示してもらう */
      aps_srv_msg.request.scene.world.collision_objects.push_back(co);
      aps_srv_msg.request.scene.object_colors = gps_srv_msg.response.scene.object_colors;
      aps_srv_msg.request.scene.object_colors.push_back(oc);
      
      ret = aps_sc.call(aps_srv_msg);
      if (ret == true)
      {
        cur_item_id = co.id;
      }
      else
      {
        ROS_FATAL("(ApplyPlanningScene) Service Response Error.");
      }
    }
    else
    {
      ROS_ERROR("(grasp_info_list) Parameter Load Error.");
    }
  }
  else
  {
    ROS_FATAL("(GetPlanningScene) Service Response Error.");
  }
  
  return ret;
}

static bool add_item_rviz_rp(void)
{
  XmlRpc::XmlRpcValue infoParam;
  moveit_msgs::CollisionObject co;
  geometry_msgs::Pose rcf;
  bool ret;
  
  /* 現在の状況を取得 */
  ret = gps_sc.call(gps_srv_msg);
  if (ret == true)
  {
    /* stlファイルを特定 */
    std::ostringstream oss;
    oss << (int)in_cad_id;
    std::string grasp_list = "t2_database/grasp_info_list/cad" + oss.str();
    ret = nhP->getParam(grasp_list.c_str(), infoParam);
    if (ret == true)
    {
      /* stlファイルを取り込み */
      std::string mesh_file = (std::string)infoParam["mesh_file"];
      mesh_file = "package://t2_database/meshes/items/" + mesh_file;
      Eigen::Vector3d eigen_scale(1.0, 1.0, 1.0);
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(shapes::createMeshFromResource(mesh_file, eigen_scale), shape_msg);
      
      /* アイテムのstlと姿勢をメッセージに登録 */
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.header.frame_id = "world";
      co.id = "rp_" + oss.str();
      co.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
      rcf = calcRcf();
      co.mesh_poses.push_back(rcf);
      /* アイテムの色をメッセージに登録 */
      oc.id = "rp_" + oss.str();
      
      /* moveit経由でRvizに表示してもらう */
      aps_srv_msg.request.scene.world.collision_objects.push_back(co);
      aps_srv_msg.request.scene.object_colors = gps_srv_msg.response.scene.object_colors;
      aps_srv_msg.request.scene.object_colors.push_back(oc);
      
      ret = aps_sc.call(aps_srv_msg);
      if (ret == true)
      {
        //cur_item_id = co.id;                                  /* RPの方では触らない */
      }
      else
      {
        ROS_FATAL("(ApplyPlanningScene) Service Response Error.");
      }
    }
    else
    {
      ROS_ERROR("(grasp_info_list) Parameter Load Error.");
    }
  }
  else
  {
    ROS_FATAL("(GetPlanningScene) Service Response Error.");
  }
  
  return ret;
}

static geometry_msgs::Pose calcRcf(void)
{
  geometry_msgs::Pose ret;
  
  Eigen::Affine3d gcf, gp, rcf, rp;
  
  tf::poseMsgToEigen(in_center_pose, gcf);
  tf::poseMsgToEigen(out_gprp[draw_target_idx].gp, gp);
  tf::poseMsgToEigen(out_gprp[draw_target_idx].rp, rp);
  
  rcf = rp * gp.inverse() * gcf;
  
  tf::poseEigenToMsg(rcf, ret);
  
  return ret;
}

static void draw_pose(void)
{
  if (draw_target_idx < (int)out_gprp.size())
  {
    /* 表示対象のスコアを表示 */
    ROS_INFO("(getGpRp) [%d].score = %lf", draw_target_idx, out_gprp[draw_target_idx].score);
    ROS_INFO("(getGpRp) [%d].gp_number = %d", draw_target_idx, (int)out_gprp[draw_target_idx].gp_number);
    ROS_INFO("(getGpRp) [%d].grasp_pattern = %s", draw_target_idx, out_gprp[draw_target_idx].grasp_pattern.c_str());
    ROS_INFO("(getGpRp) [%d].protrude_length = %lf", draw_target_idx, out_gprp[draw_target_idx].protrude_length);
    
    /* Center/GP/RPをRvizに表示 */
    tpc_msg.pose = in_center_pose;
    pub_center.publish(tpc_msg);
    
    tpc_msg.pose = out_gprp[draw_target_idx].gp;
    pub_gp.publish(tpc_msg);
    
    tpc_msg.pose = out_gprp[draw_target_idx].rp;           // ★簡易的にRPを表示させるため、rvizのGAPの口にRPを突っ込むことにした
    pub_ap.publish(tpc_msg);
  }
  else
  {
    ROS_ERROR("(draw_target_idx) Over total_point.");
  }
}

static void draw_all_gp(void)
{
  /* マーカー消去指令用 */
  visualization_msgs::Marker marker_delete;
  visualization_msgs::MarkerArray markerarray_delete;
  marker_delete.action = 3; /* DELETEALL */
  marker_delete.header.frame_id = "world"; /* WARNING対策 */
  markerarray_delete.markers.push_back(marker_delete);

  /* マーカー消去 */
  pub_points.publish(marker_delete);
  pub_axes.publish(markerarray_delete);
  pub_texts.publish(markerarray_delete);

  /* マーカー初期化 */
  points.action = visualization_msgs::Marker::ADD;
  points.points.clear();
  points.colors.clear();    
  axes.markers.clear();
  texts.markers.clear();

  for (unsigned int i = 0; i < out_gprp.size(); i++)
  {
    /* GP中心位置(+スコアを色表示) */
    points.points.push_back(out_gprp[i].gp.position);  
    std_msgs::ColorRGBA color;
    color.r = out_gprp[i].score;
    color.g = (out_gprp[i].grasp_pattern == "suction" ? 0.0 : 1.0);
    color.b = 1.0 - out_gprp[i].score;
    color.a = 1.0;
    points.colors.push_back(color);    

    /* GP姿勢 */
    Eigen::Affine3d gp;
    Eigen::Vector3d gp_pos, axis_x, axis_y, axis_z;
    geometry_msgs::Point msg_x, msg_y, msg_z;
    tf::poseMsgToEigen(out_gprp[i].gp, gp);
    gp_pos = gp.translation();
    axis_x = gp.linear().col(0);
    axis_y = gp.linear().col(1);
    axis_z = gp.linear().col(2);

    /* x軸 */
    gp.translation() = gp_pos + axis_x * 0.010;
    gp.linear().col(0) = -axis_z;
    gp.linear().col(1) = axis_y;
    gp.linear().col(2) = axis_x;
    tf::poseEigenToMsg(gp, axis.pose);
    axis.id = 3*i+0;
    axis.color.r = 1.0;
    axis.color.g = 0.0;
    axis.color.b = 0.0;
    axis.color.a = 1.0;
    axes.markers.push_back(axis);  

    /* y軸 */
    gp.translation() = gp_pos + axis_y * 0.010;
    gp.linear().col(0) = axis_x;
    gp.linear().col(1) = -axis_z;
    gp.linear().col(2) = axis_y;
    tf::poseEigenToMsg(gp, axis.pose);
    axis.id = 3*i+1;
    axis.color.r = 0.0;
    axis.color.g = 1.0;
    axis.color.b = 0.0;
    axis.color.a = 1.0;
    axes.markers.push_back(axis);  

    /* z軸 */
    gp.translation() = gp_pos + axis_z * 0.010;
    gp.linear().col(0) = axis_x;
    gp.linear().col(1) = axis_y;
    gp.linear().col(2) = axis_z;
    tf::poseEigenToMsg(gp, axis.pose);
    axis.id = 3*i+2;
    axis.color.r = 0.0;
    axis.color.g = 0.0;
    axis.color.b = 1.0;
    axis.color.a = 1.0;
    axes.markers.push_back(axis);  

    /* 番号とスコア */
    text.id = i;
    std::stringstream ss;
    ss << (int)(out_gprp[i].gp_number/100) << "(" << (int)(out_gprp[i].score*100) << ")";
    text.text = ss.str();
    gp.translation() = gp_pos - axis_z * 0.03;
    tf::poseEigenToMsg(gp, text.pose);
    texts.markers.push_back(text);
  }

  /* マーカーを配信 */
  pub_points.publish(points);
  pub_axes.publish(axes);
  pub_texts.publish(texts);
}
