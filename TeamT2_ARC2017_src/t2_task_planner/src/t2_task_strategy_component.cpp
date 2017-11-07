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

#define T2_TASK_STRATEGY_COMPONENT_SRC
#include "t2_task_planner/t2_task_strategy_component.h"

/* システムヘッダ参照 */
#include <eigen_conversions/eigen_msg.h>
#include <octomap_msgs/conversions.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

/* 外部ヘッダ参照 */
/* 内部定数定義 */

/* 内部型定義 */
typedef struct{
  uint place_id;                                                /* 場所 */
  std::string camera_pos;                                       /* 場所に対するカメラの設置位置 */
  
  double row_d;                                                 /* 行(縦の長さ)(リアル値) */
  double col_d;                                                 /* 列(横の長さ)(リアル値) */
  double high_d;                                                /* 高(高さの長さ)(リアル値) */
  
  int row_i;                                                    /* 行(縦の長さ)(かさ上げ値) */
  int col_i;                                                    /* 列(横の長さ)(かさ上げ値) */
  int high_i;                                                   /* 高(高さの長さ)(かさ上げ値) */
  
  Eigen::Vector3d org;                                          /* 原点(voxel[0][0][0])のワールド座標 */
  Eigen::Affine3d upside_pose;                                  /* 場所の上方姿勢 */
  
  Eigen::Affine3d pose;                                         /* カメラ姿勢(キャリブレーション値) */
  
  octomap_msgs::Octomap octomap;
  std::vector< std::vector< std::vector<bool> > > voxel;
} Place_t;

/* 内部変数定義 */
static std::vector<Place_t> place;
static ros::Publisher vis_pub;

/* 内部関数定義 */
static void storeOctomap2Voxel(Place_t *placeP);
static Eigen::Vector3d coordVox2World(int x, int y, int z, Eigen::Vector3d org, double resolution, std::string camera_pos);
static Eigen::Vector3d coordOfst(Eigen::Vector3d vecCoord, std::string camera_pos, double ofst_row, double ofst_col, double ofst_high);
static Eigen::Vector3d coordWorld2Camera(Eigen::Vector3d vecWorld, Eigen::Affine3d pose);
static bool checkExist(Eigen::Vector3d vecCamera, octomap::OcTree &ot);
static bool checkStore(uint placeIdx, int x, int y, int z, int row, int col, int high);
static bool checkStoreProtrude(uint placeIdx, int x, int y, int z, int row, int col, int high);
static uint findPlaceIdx(uint place_id);
static void plotOctoRviz(Place_t *placeP);

/****************/
/* 外部関数処理 */
/****************/
//------------------------------
// Octomap更新関数
//------------------------------
bool updateOctomapProc(
  uint place_id,
  geometry_msgs::Pose pose,
  octomap_msgs::Octomap octomap
){
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue containerParam, strategyParam;            /* yaml読込用 */
  uint containerIdx;
  
  Place_t tmp;
  
  /* 入力チェック */
  /* パラメータロード(container_info) */
  bool ret = getParamContainerInfo(&nh, &containerParam);
  if (ret == false)
  {
    /* パラメータロードエラー */
    return false;
  }
  else
  {
    /* 該当place_idが、container_infoのどのindexか検索する */
    containerIdx = findContainerIdx(&containerParam, place_id);
    if (containerIdx == UINT_MAX)
    {
      ROS_ERROR("ParameterServer(container_info) can't find place_id.");
      return false;
    }
  }
  
  /* パラメータロード(task_strategy) */
  ret = getParamTaskStrategy(&nh, &strategyParam);
  if (ret == false)
  {
    /* パラメータロードエラー */
    return false;
  }
  
  /* 要素の中身を作成 */
  tmp.place_id = place_id;
  tmp.camera_pos = (std::string)containerParam[containerIdx]["camera_pos"];
  /* 箱サイズ */
  if (tmp.camera_pos == "far")
  {
    /* "far" */
    tmp.row_d = (double)containerParam[containerIdx]["inside_dimensions"][1];
    tmp.col_d = (double)containerParam[containerIdx]["inside_dimensions"][0];
    tmp.high_d = (double)containerParam[containerIdx]["inside_dimensions"][2];
  }
  else if (tmp.camera_pos == "left")
  {
    /* "left" */
    tmp.row_d = (double)containerParam[containerIdx]["inside_dimensions"][0];
    tmp.col_d = (double)containerParam[containerIdx]["inside_dimensions"][1];
    tmp.high_d = (double)containerParam[containerIdx]["inside_dimensions"][2];
  }
  else
  {
    /* "right" */
    tmp.row_d = (double)containerParam[containerIdx]["inside_dimensions"][0];
    tmp.col_d = (double)containerParam[containerIdx]["inside_dimensions"][1];
    tmp.high_d = (double)containerParam[containerIdx]["inside_dimensions"][2];
  }
  /* 箱サイズは小さくなるように切り捨て */
  tmp.row_i = (int)(tmp.row_d / octomap.resolution);
  tmp.col_i = (int)(tmp.col_d / octomap.resolution);
  tmp.high_i = (int)((tmp.high_d + (double)strategyParam["voxel_high_offset"]) / octomap.resolution);
  /* voxel原点算出 */
  if (tmp.camera_pos == "far")
  {
    /* "far" */
    tmp.org.x() = (double)containerParam[containerIdx]["inside_position"][0];
    tmp.org.y() = (double)containerParam[containerIdx]["inside_position"][1] - tmp.row_d;
    tmp.org.z() = (double)containerParam[containerIdx]["inside_position"][2];
    /* 切り捨て部分を算出し、中央にアジャストしたところをvoxel原点とする */
    tmp.org.x() -= ((tmp.col_d - ((double)tmp.col_i * octomap.resolution)) / 2);
    tmp.org.y() += ((tmp.row_d - ((double)tmp.row_i * octomap.resolution)) / 2);
    tmp.org.z() += (((tmp.high_d + (double)strategyParam["voxel_high_offset"]) - ((double)tmp.high_i * octomap.resolution)) / 2);
  }
  else if (tmp.camera_pos == "left")
  {
    /* "left" */
    tmp.org.x() = (double)containerParam[containerIdx]["inside_position"][0];
    tmp.org.y() = (double)containerParam[containerIdx]["inside_position"][1];
    tmp.org.z() = (double)containerParam[containerIdx]["inside_position"][2];
    /* 切り捨て部分を算出し、中央にアジャストしたところをvoxel原点とする */
    tmp.org.x() -= ((tmp.row_d - ((double)tmp.row_i * octomap.resolution)) / 2);
    tmp.org.y() -= ((tmp.col_d - ((double)tmp.col_i * octomap.resolution)) / 2);
    tmp.org.z() += (((tmp.high_d + (double)strategyParam["voxel_high_offset"]) - ((double)tmp.high_i * octomap.resolution)) / 2);
  }
  else
  {
    /* "right" */
    tmp.org.x() = (double)containerParam[containerIdx]["inside_position"][0] - tmp.row_d;
    tmp.org.y() = (double)containerParam[containerIdx]["inside_position"][1] - tmp.col_d;
    tmp.org.z() = (double)containerParam[containerIdx]["inside_position"][2];
    /* 切り捨て部分を算出し、中央にアジャストしたところをvoxel原点とする */
    tmp.org.x() += ((tmp.row_d - ((double)tmp.row_i * octomap.resolution)) / 2);
    tmp.org.y() += ((tmp.col_d - ((double)tmp.col_i * octomap.resolution)) / 2);
    tmp.org.z() += (((tmp.high_d + (double)strategyParam["voxel_high_offset"]) - ((double)tmp.high_i * octomap.resolution)) / 2);
  }
  
  /* 上方姿勢 */
  tmp.upside_pose = Eigen::Affine3d::Identity();
  if (tmp.camera_pos == "far")
  {
    /* "far" */
    tmp.upside_pose.translation().x() = (double)containerParam[containerIdx]["inside_position"][0] - (tmp.col_d / 2);
    tmp.upside_pose.translation().y() = (double)containerParam[containerIdx]["inside_position"][1] - (tmp.row_d / 2);
    tmp.upside_pose.translation().z() = (double)containerParam[containerIdx]["inside_position"][2] + tmp.high_d;
  }
  else if (tmp.camera_pos == "left")
  {
    /* "left" */
    tmp.upside_pose.translation().x() = (double)containerParam[containerIdx]["inside_position"][0] - (tmp.row_d / 2);
    tmp.upside_pose.translation().y() = (double)containerParam[containerIdx]["inside_position"][1] - (tmp.col_d / 2);
    tmp.upside_pose.translation().z() = (double)containerParam[containerIdx]["inside_position"][2] + tmp.high_d;
  }
  else
  {
    /* "right" */
    tmp.upside_pose.translation().x() = (double)containerParam[containerIdx]["inside_position"][0] - (tmp.row_d / 2);
    tmp.upside_pose.translation().y() = (double)containerParam[containerIdx]["inside_position"][1] - (tmp.col_d / 2);
    tmp.upside_pose.translation().z() = (double)containerParam[containerIdx]["inside_position"][2] + tmp.high_d;
  }
  setRobotEasyPose(&tmp.upside_pose);                           /* ロボットにとって容易なハンド姿勢を算出 */
  
  /* Octomap関連 */
  tf::poseMsgToEigen(pose, tmp.pose);
  tmp.octomap = octomap;
  
  /* 要素の中身(voxel)を作成 */
  /* 空のvoxel領域を作成 */
  tmp.voxel.resize(tmp.row_i);
  for (std::size_t i=0; i<tmp.voxel.size(); i++)
  {
    tmp.voxel[i].resize(tmp.col_i);
    for (std::size_t j=0; j<tmp.voxel[i].size(); j++)
    {
      tmp.voxel[i][j].resize(tmp.high_i);
      for (std::size_t k=0; k<tmp.voxel[i][j].size(); k++)
      {
        tmp.voxel[i][j][k] = false;
      }
    }
  }
  /* octomapからvoxelにデータを格納する */
  storeOctomap2Voxel(&tmp);
  
  /* 更新対象の古い要素を削除 */
  uint placeIdx = findPlaceIdx(place_id);
  if(placeIdx != UINT_MAX)
  {
      place.erase(place.begin() + placeIdx);
  }
  
  /* 新しい要素を登録 */
  place.push_back(tmp);
  
  //plotOutProc(place_id); //★debug
  //printPlaceStProc(place_id); //★debug
  //std::cout << "pose_qua = " << std::endl << pose << std::endl; //★debug
  
  /* OctomapのRviz表示 */
  plotOctoRviz(&tmp);
  
  return true;
}

//------------------------------
// 内部変数place表示機能
//------------------------------
void printPlaceStProc(uint place_id)
{
  std::cout << "printPlaceStProc()" << std::endl;
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    std::cout << "place[" << placeIdx << "]." << std::endl;
    std::cout << "place_id = " << place[placeIdx].place_id << std::endl;
    std::cout << "camera_pos = " << place[placeIdx].camera_pos << std::endl;
    std::cout << "row = " << place[placeIdx].row_i << std::endl;
    std::cout << "col = " << place[placeIdx].col_i << std::endl;
    std::cout << "high = " << place[placeIdx].high_i << std::endl;
    std::cout << "org = " << std::endl << place[placeIdx].org << std::endl;
    std::cout << "pose = " << std::endl << place[placeIdx].pose.matrix() << std::endl;
  }
  else
  {
    std::cout << "place_id(" << place_id << ") Not Found." << std::endl;
  }
}

//------------------------------
// 内部変数place全表示機能
//------------------------------
void printPlaceStAllProc(void)
{
  std::cout << "printPlaceStAllProc()" << std::endl;
  
  uint place_id;
  
  for (std::size_t i=0; i<place.size(); i++)
  {
    place_id = place[i].place_id;
    printPlaceStProc(place_id);
  }
}

//------------------------------
// 強制Voxel操作
//------------------------------
void setVoxelForceProc(
  uint place_id,
  int coordRow, int coordCol, int coordHigh,
  int sizeRow, int sizeCol, int sizeHigh,
  bool exist
){
  std::cout << "setVoxelForceProc()" << std::endl;
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    for (int i=0; i<sizeRow; i++)
    {
      for (int j=0; j<sizeCol; j++)
      {
        for (int k=0; k<sizeHigh; k++)
        {
          place[placeIdx].voxel[coordRow+i][coordCol+j][coordHigh+k] = exist;
        }
      }
    }
  }
  else
  {
    std::cout << "place_id(" << place_id << ") Not Found." << std::endl;
  }
  
  plotOctoRviz(&place[placeIdx]);
}

//------------------------------
// Voxel状況(プロット)をファイル出力する
//------------------------------
void plotOutProc(uint place_id)
{
  std::cout << "plotOutProc()" << std::endl;
  
  /* 出力ファイル設定 */
  std::string packPath = ros::package::getPath("t2_task_planner");
#if 1
  std::string fileName = packPath + "/plotOut.csv";
#else
  //★debug
  std::ostringstream oss;
  oss << (int)place_id;
  std::string fileName = packPath + "/plotOut" + oss.str() + ".csv";
#endif
  std::ofstream ofs(fileName.c_str());
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    /* ファイル書き出し */
    for (std::size_t i=0; i<place[placeIdx].voxel.size(); i++)
    {
      for (std::size_t j=0; j<place[placeIdx].voxel[i].size(); j++)
      {
        for (std::size_t k=0; k<place[placeIdx].voxel[i][j].size(); k++)
        {
          ofs << i << ", ";
          ofs << j << ", ";
          ofs << k << ", ";
          if (place[placeIdx].voxel[i][j][k] == true)
          {
            ofs << "1" << ", ";                                   /* id */
          }
          else
          {
            ofs << "0" << ", ";                                   /* id */
          }
          ofs << "0";                                             /* color */
          ofs << std::endl;
        }
      }
    }
  }
  else
  {
    std::cout << "place_id(" << place_id << ") Not Found." << std::endl;
  }
}

//------------------------------
// ロボットにとって容易なハンド姿勢を算出
// Eigen::Affine3d *pose: (入出力)座標を入力し、姿勢を出力する
//                        使用制限：(並進(特にxy) != ロボット座標)とする
//------------------------------
void setRobotEasyPose(Eigen::Affine3d *pose)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue strategyParam;
  
  /* 表示オプションを読込 */
  (void)getParamTaskStrategy(&nh, &strategyParam);              /* 返戻値は確認しない */
  
  /* ロボット原点(ワールド座標) */
  Eigen::Vector3d robot_origin(
    (double)strategyParam["r_pos"]["x"],
    (double)strategyParam["r_pos"]["y"],
    (double)strategyParam["r_pos"]["z"]
  );
  
  /* 出力姿勢を算出 */
  /* z成分 */
  pose->linear().col(2).x() = 0.0;
  pose->linear().col(2).y() = 0.0;
  pose->linear().col(2).z() = -1.0;
  
  /* y成分 */
  pose->linear().col(1) = robot_origin - pose->translation();   /* 入力座標(point)からロボット(robot)までのベクトル算出 */
  pose->linear().col(1).z() = 0.0;                              /* xy成分だけにする */
  pose->linear().col(1) /= pose->linear().col(1).norm();        /* 単位化 */
  
  /* x成分 */
  pose->linear().col(0) = pose->linear().col(1).cross(pose->linear().col(2)); /* x = yとzの外積 */
}

//------------------------------
// itemが収まる場所を検索(収まる場所のワールド座標を算出)
// bool protrude_flg：ハミ出しフラグ(false:不許可、true:許可)
//------------------------------
bool findRp(uint place_id, double item_row, double item_col, double item_high, bool protrude_flg, Eigen::Vector3d *vecRpP)
{
  bool ret = true;
  
  int row, col, high;                                           /* itemのvoxelサイズ */
  int x, y, z;                                                  /* voxel座標 */
  uint placeIdx;
  
  if (ret == true)
  {
    /* placeを検索 */
    placeIdx = findPlaceIdx(place_id);
    if (placeIdx == UINT_MAX)
    {
      ret = false;
    }
  }
  
  if (ret == true)
  {
    /* 入力itemのワールドサイズをvoxelサイズに変換(切り上げ) */
    row = (int)(item_row / place[placeIdx].octomap.resolution) + 1;
    col = (int)(item_col / place[placeIdx].octomap.resolution) + 1;
    high = (int)(item_high / place[placeIdx].octomap.resolution) + 1;
  }
  
  if (ret == true)
  {
    /* itemが格納できるvoxel座標を検索 */
    for (z=0; z<place[placeIdx].high_i; z++)
    {
      for (x=0; x<place[placeIdx].row_i; x++)
      {
        for (y=0; y<place[placeIdx].col_i; y++)
        {
          if (protrude_flg == false)
          {
            ret = checkStore(placeIdx, x, y, z, row, col, high);
          }
          else
          {
            ret = checkStoreProtrude(placeIdx, x, y, z, row, col, high);
          }
          
          if (ret == true)
          {
            break;
          }
        }
        if (ret == true)
        {
          break;
        }
      }
      if (ret == true)
      {
        break;
      }
    }
  }
  
  if (ret == true)
  {
    /* voxel座標をworld座標に変換 */
    Eigen::Vector3d vecItem = coordVox2World(x, y, z, place[placeIdx].org, place[placeIdx].octomap.resolution, place[placeIdx].camera_pos);
    
    /* 座標は、voxel原点に一番近い頂点が格納されているので、アイテム図心位置になるように変換 */
    *vecRpP = coordOfst(vecItem, place[placeIdx].camera_pos, (item_row / 2), (item_col / 2), (item_high / 2));
  }
  
  return ret;
}

//------------------------------
// 指定place_idの寸法を取得
// uint place_id: (入力)place番号
// double *place_row/col/high: (出力)placeの寸法
//------------------------------
bool getPlaceLength(uint place_id, double *place_row, double *place_col, double *place_high)
{
  bool ret = true;
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    *place_row = place[placeIdx].row_d;
    *place_col = place[placeIdx].col_d;
    *place_high = place[placeIdx].high_d;
  }
  else
  {
    ret = false;
  }
  
  return ret;
}

//------------------------------
// カメラ位置を考慮した仮のRPを選択
// (注)place未発見時は単位行列を返す
//------------------------------
Eigen::Affine3d selectPreRp(uint place_id, bool xy_rot)
{
  Eigen::Affine3d ret = Eigen::Affine3d::Identity();
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    if (place[placeIdx].camera_pos == "far")
    {
      /* "far" */
      if (xy_rot == false)
      {
        ret.linear() <<
          -1,  0,  0,
           0,  1,  0,
           0,  0, -1;
      }
      else
      {
        ret.linear() <<
           0,  1,  0,
           1,  0,  0,
           0,  0, -1;
      }
    }
    else if (place[placeIdx].camera_pos == "left")
    {
      /* "left" */
      if (xy_rot == false)
      {
        ret.linear() <<
           0, -1,  0,
          -1,  0,  0,
           0,  0, -1;
      }
      else
      {
        ret.linear() <<
           1,  0,  0,
           0, -1,  0,
           0,  0, -1;
      }
    }
    else
    {
      /* "right" */
      if (xy_rot == false)
      {
        ret.linear() <<
           0,  1,  0,
           1,  0,  0,
           0,  0, -1;
      }
      else
      {
        ret.linear() <<
          -1,  0,  0,
           0,  1,  0,
           0,  0, -1;
      }
    }
  }
  
  return ret;
}

//------------------------------
// 図心座標にアイテムを置いたときのplaceからのはみ出し量を算出
//------------------------------
double calcProtrude(uint place_id, Eigen::Vector3d vecRcf, double item_high)
{
  double ret;
  double coordZ;
  
  coordZ = (vecRcf.z() + (item_high / 2));
  ret = calcProtrudeCoord(place_id, coordZ);
  
  return ret;
}

//------------------------------
// 入力z座標のplaceからのはみ出し量を算出
//
// (注)はみ出し量はvoxel空間からのはみ出し量でなく、実placeからのはみ出し量。
//     upside_poseは実placeの高さで、かさ上げオフセットを含んでいない。
// (注)place未発見時は0を返す
//------------------------------
double calcProtrudeCoord(uint place_id, double coordZ)
{
  double ret = 0;
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    ret = coordZ - place[placeIdx].upside_pose.translation().z();
  }
  
  return ret;
}

//------------------------------
// 指定place_idの上方姿勢を取得
//------------------------------
bool getPlaceUpsidePose(uint place_id, Eigen::Affine3d *pose)
{
  bool ret = false;
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    *pose = place[placeIdx].upside_pose;
    ret = true;
  }
  
  return ret;
}

//------------------------------
// 空率が閾値を超えた層を検索し、そのz座標を取得する
//------------------------------
bool findFreeLayerCoordZ(uint place_id, double *coordZ)
{
  bool ret = true;
  
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue strategyParam;
  
  double free_th;                                               /* 閾値 */
  int area;                                                     /* 面積 */
  int num;                                                      /* 空個数 */
  int findLayer = INT_MAX;                                      /* 見つけた層 */
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx == UINT_MAX)
  {
    ret = false;
  }
  
  if (ret == true)
  {
    /* 行列計算パラメータ展開 */
    ret = getParamTaskStrategy(&nh, &strategyParam);
    free_th = (double)strategyParam["place_upside_rp_free_th"];
  }
  
  if (ret == true)
  {
    /* 閾値越えの層を検索し、そのz座標を取得する */
    area = place[placeIdx].row_i * place[placeIdx].col_i;       /* 面積算出 */
    
    std::size_t i;
    for (i=0; i<place[placeIdx].high_i; i++)
    {
      /* このループの出力はi */
      
      num = 0;
      
      for (std::size_t j=0; j<place[placeIdx].row_i; j++)
      {
        for (std::size_t k=0; k<place[placeIdx].col_i; k++)
        {
          
          if (place[placeIdx].voxel[j][k][i] == false)
          {
            num++;
          }
        }
      }
      
      if (((double)num / (double)area) > free_th)
      {
        /* 閾値越えの層を発見 */
        /* 発見した時のiを保持して抜ける */
        break;
      }
      /* 最後まで見つからなかった場合、iはVoxel空間の1層外側で抜ける */
    }
    
    *coordZ = place[placeIdx].org.z() + (place[placeIdx].octomap.resolution * (double)i);
  }
  
  return ret;
}

//------------------------------
// placeの体積の空を算出
// (注)place未発見時は0個の結果を返す
//------------------------------
int calcFreeVolume(uint place_id)
{
  int ret = 0;
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    for (int i=0; i<place[placeIdx].row_i; i++)
    {
      for (int j=0; j<place[placeIdx].col_i; j++)
      {
        for (int k=0; k<place[placeIdx].high_i; k++)
        {
          if (place[placeIdx].voxel[i][j][k] == false)
          {
            ret++;
          }
        }
      }
    }
  }
  
  return ret;
}

//------------------------------
// placeの底面積の空を算出
// (注)place未発見時は0個の結果を返す
//------------------------------
int calcFreeBottomArea(uint place_id)
{
  int ret = 0;
  
  uint placeIdx = findPlaceIdx(place_id);
  if (placeIdx != UINT_MAX)
  {
    /* 空のVoxelの個数を数える */
    for (int i=0; i<place[placeIdx].row_i; i++)
    {
      for (int j=0; j<place[placeIdx].col_i; j++)
      {
        if (place[placeIdx].voxel[i][j][0] == false)
        {
          ret++;
        }
      }
    }
  }
  
  return ret;
}

//------------------------------
// 把持情報リスト展開
//------------------------------
bool getParamGraspInfoList(uint cad_id, ros::NodeHandle *nhP, XmlRpc::XmlRpcValue *graspParamP)
{
  bool ret;
  
  std::ostringstream oss;
  oss << (int)cad_id;
  std::string grasp_list = "t2_database/grasp_info_list/cad" + oss.str();
  ret = nhP->getParam(grasp_list.c_str(), *graspParamP);
  if (ret == false)
  {
    /* パラメータロードエラー */
    ROS_ERROR("ParameterServer(grasp_info_list) get error.");
  }
  
  return ret;
}

//------------------------------
// 把持情報リストの中でgp_numberが一致するindexを検索する
//------------------------------
uint findGraspInfoIdx(XmlRpc::XmlRpcValue *infoParamP, uint32_t gp_number)
{
  uint ret = UINT_MAX;
  
  /* 該当gp_numberが、把持情報リストのどのindexか検索する */
  for (std::size_t i=0; i<(*infoParamP)["gp"].size(); i++)
  {
    if ((int)(*infoParamP)["gp"][i]["gp_number"] == (int)gp_number)
    {
      ret = (uint)i;
      break;
    }
  }
  
  return ret;
}

//------------------------------
// 決め打ちパラメータ展開
//------------------------------
bool getParamFixRp(uint cad_id, ros::NodeHandle *nhP, XmlRpc::XmlRpcValue *fixRpParamP)
{
  bool ret;
  
  std::ostringstream oss;
  oss << (int)cad_id;
  std::string fix_rp = "t2_task_planner/fix_rp/rp" + oss.str();
  ret = nhP->getParam(fix_rp.c_str(), *fixRpParamP);
  if (ret == false)
  {
    /* パラメータロードエラー */
    ROS_ERROR("ParameterServer(fix_rp) get error.");
  }
  
  return ret;
}

//------------------------------
// task_strategy展開
//------------------------------
bool getParamTaskStrategy(ros::NodeHandle *nhP, XmlRpc::XmlRpcValue *strategyParamP)
{
  bool ret;
  
  ret = nhP->getParam("t2_task_planner/task_strategy", *strategyParamP);
  if (ret == false)
  {
    /* パラメータロードエラー */
    ROS_ERROR("ParameterServer(task_strategy) get error.");
  }
  
  return ret;
}

//------------------------------
// container_info展開
//------------------------------
bool getParamContainerInfo(ros::NodeHandle *nhP, XmlRpc::XmlRpcValue *containerParamP)
{
  bool ret;
  
  ret = nhP->getParam("t2_database/container_info", *containerParamP);
  if (ret == false)
  {
    /* パラメータロードエラー */
    ROS_ERROR("ParameterServer(container_info) get error.");
  }
  
  return ret;
}

//------------------------------
// container_infoパラメータの中でplace_idが一致するindexを検索する
//------------------------------
uint findContainerIdx(XmlRpc::XmlRpcValue *containerParamP, uint place_id)
{
  uint ret = UINT_MAX;
  
  /* 該当place_idが、container_infoのどのindexか検索する */
  for (std::size_t i=0; i<(*containerParamP).size(); i++)
  {
    if ((int)(*containerParamP)[i]["place_id"] == (int)place_id)
    {
      ret = (uint)i;
      break;
    }
  }
  
  return ret;
}

//------------------------------
// GRASP_INPUT_REQ_TのROS_INFO出力
//------------------------------
void printReqInfo(GRASP_INPUT_REQ_T *reqP, std::string str)
{
  ROS_INFO("printReqInfo(%s)", str.c_str());
  ROS_INFO("cad_id=%d", (int)reqP->cad_id);
  ROS_INFO("single_gp=%d", (int)reqP->single_gp);
  for (std::size_t i=0; i<reqP->place_id_list.size(); i++)
  {
    ROS_INFO("place_id_list[%d]=%d", (int)i, (int)reqP->place_id_list[i]);
  }
  ROS_INFO_STREAM("Center" << std::endl << reqP->g_center_pos.matrix());
  ROS_INFO_STREAM("");
}

//------------------------------
// GRASP_OUTPUT_RSP_TのROS_INFO出力
//------------------------------
void printRspInfo(GRASP_OUTPUT_RSP_T *rspP, std::string str)
{
  ROS_INFO("printRspInfo(%s)", str.c_str());
  //ROS_INFO("result=%d", rspP->result);
  //ROS_INFO("total_score=%lf", rspP->total_score);
  //ROS_INFO("total_point=%d", (int)rspP->total_point);
  //ROS_INFO_STREAM("");
  
  for (std::size_t i=0; i<rspP->grasp_point.size(); i++)
  {
    ROS_INFO("gp_number=%d", (int)rspP->grasp_point[i].gp_number);
    ROS_INFO("score=%lf", rspP->grasp_point[i].score);
    ROS_INFO("grasp_pattern=%s", rspP->grasp_point[i].grasp_pattern.c_str());
    ROS_INFO("release_place_id=%d", (int)rspP->grasp_point[i].release_place_id);
    ROS_INFO("protrude_length=%lf", rspP->grasp_point[i].protrude_length);
    
    ROS_INFO_STREAM("GP" << std::endl << rspP->grasp_point[i].grasp_point_item.matrix());
    //ROS_INFO_STREAM("GAP" << std::endl << rspP->grasp_point[i].approach_point_item.matrix());
    ROS_INFO_STREAM("RP" << std::endl << rspP->grasp_point[i].rp.matrix());
    //ROS_INFO_STREAM("RAP" << std::endl << rspP->grasp_point[i].rap.matrix());
    ROS_INFO_STREAM("");
  }
  
  ROS_INFO_STREAM("");
}

//------------------------------
// GRASP_OUTPUT_RSP_TのROS_DEBUG出力
//------------------------------
void printRspDebug(GRASP_OUTPUT_RSP_T *rspP, std::string str)
{
  ROS_DEBUG("printRspDebug(%s)", str.c_str());
  ROS_DEBUG("result=%d", rspP->result);
  ROS_DEBUG("total_score=%lf", rspP->total_score);
  ROS_DEBUG("total_point=%d", (int)rspP->total_point);
  ROS_DEBUG_STREAM("");
  
  for (std::size_t i=0; i<rspP->grasp_point.size(); i++)
  {
    ROS_DEBUG("gp_number=%d", (int)rspP->grasp_point[i].gp_number);
    ROS_DEBUG("score=%lf", rspP->grasp_point[i].score);
    ROS_DEBUG("grasp_pattern=%s", rspP->grasp_point[i].grasp_pattern.c_str());
    ROS_DEBUG("release_place_id=%d", (int)rspP->grasp_point[i].release_place_id);
    ROS_DEBUG("protrude_length=%lf", rspP->grasp_point[i].protrude_length);
    
    ROS_DEBUG_STREAM("GP" << std::endl << rspP->grasp_point[i].grasp_point_item.matrix());
    //ROS_DEBUG_STREAM("GAP" << std::endl << rspP->grasp_point[i].approach_point_item.matrix());
    ROS_DEBUG_STREAM("RP" << std::endl << rspP->grasp_point[i].rp.matrix());
    //ROS_DEBUG_STREAM("RAP" << std::endl << rspP->grasp_point[i].rap.matrix());
    ROS_DEBUG_STREAM("");
  }
  
  ROS_DEBUG_STREAM("");
}

/****************/
/* 内部関数処理 */
/****************/
//------------------------------
// octomapからvoxelにデータを格納する
//------------------------------
static void storeOctomap2Voxel(Place_t *placeP)
{
  Eigen::Vector3d worldPoint, cameraPoint;
  
  octomap::AbstractOcTree* aotP = octomap_msgs::msgToMap(placeP->octomap);
  octomap::OcTree* otP = (octomap::OcTree*)aotP;
  octomap::OcTree ot = *otP;
  
  for (int i=0; i<placeP->row_i; i++)
  {
    for (int j=0; j<placeP->col_i; j++)
    {
      for (int k=0; k<placeP->high_i; k++)
      {
        /* 該当ポイントのワールド座標を算出 */
        worldPoint = coordVox2World(i, j, k, placeP->org, placeP->octomap.resolution, placeP->camera_pos);
        
        /* 該当ポイントのカメラ座標を算出 */
        cameraPoint = coordWorld2Camera(worldPoint, placeP->pose);
        
        /* カメラ座標の位置に物体が存在するかOctomapから確認し、結果を格納 */
        placeP->voxel[i][j][k] = checkExist(cameraPoint, ot);
      }
    }
  }
  
  delete aotP;
}

//------------------------------
// voxel座標からワールド座標へ変換
//------------------------------
static Eigen::Vector3d coordVox2World(int x, int y, int z, Eigen::Vector3d org, double resolution, std::string camera_pos)
{
  Eigen::Vector3d ret;
  
  if (camera_pos == "far")
  {
    /* "far" */
    ret.x() = org.x() - (resolution * (double)y);
    ret.y() = org.y() + (resolution * (double)x);
  }
  else if (camera_pos == "left")
  {
    /* "left" */
    ret.x() = org.x() - (resolution * (double)x);
    ret.y() = org.y() - (resolution * (double)y);
  }
  else
  {
    /* "right" */
    ret.x() = org.x() + (resolution * (double)x);
    ret.y() = org.y() + (resolution * (double)y);
  }
  ret.z() = org.z() + (resolution * (double)z);
  
  return ret;
}

//------------------------------
// カメラ位置を考慮した座標のofstずらし
//------------------------------
static Eigen::Vector3d coordOfst(Eigen::Vector3d vecCoord, std::string camera_pos, double ofst_row, double ofst_col, double ofst_high)
{
  Eigen::Vector3d ret;
  
  if (camera_pos == "far")
  {
    /* "far" */
    ret.x() = vecCoord.x() - ofst_col;
    ret.y() = vecCoord.y() + ofst_row;
  }
  else if (camera_pos == "left")
  {
    /* "left" */
    ret.x() = vecCoord.x() - ofst_row;
    ret.y() = vecCoord.y() - ofst_col;
  }
  else
  {
    /* "right" */
    ret.x() = vecCoord.x() + ofst_row;
    ret.y() = vecCoord.y() + ofst_col;
  }
  ret.z() = vecCoord.z() + ofst_high;
  
  return ret;
}

//------------------------------
// Pointのワールド座標からカメラ座標へ変換
//------------------------------
static Eigen::Vector3d coordWorld2Camera(Eigen::Vector3d vecWorld, Eigen::Affine3d pose)
{
  Eigen::Vector3d ret;
  
  Eigen::Matrix<double, 4, 1> matWorld;
  Eigen::Matrix<double, 4, 1> matCamera;
  
  matWorld <<
    vecWorld.x(),
    vecWorld.y(),
    vecWorld.z(),
    1;
  
  matCamera = pose.inverse() * matWorld;
  
  ret.x() = matCamera(0, 0);
  ret.y() = matCamera(1, 0);
  ret.z() = matCamera(2, 0);
  
  return ret;
}

//------------------------------
// カメラ座標の位置に物体が存在するかOctomapから確認
//------------------------------
static bool checkExist(Eigen::Vector3d vecCamera, octomap::OcTree &ot)
{
  bool ret;
  
  octomap::point3d query(vecCamera.x(), vecCamera.y(), vecCamera.z());
  octomap::OcTreeNode* result = ot.search(query);
  
  ret = (result == NULL)? false: true;                          /* false：未発見、true：発見 */
  
  return ret;
}

//------------------------------
// voxel座標にitemが格納できるか確認する
//------------------------------
static bool checkStore(uint placeIdx, int x, int y, int z, int row, int col, int high)
{
  bool ret = true;
  
  /* はみ出しチェック */
  if (ret == true)
  {
    if (((x + row) > place[placeIdx].row_i) || ((y + col) > place[placeIdx].col_i) || ((z + high) > place[placeIdx].high_i))
    {
      ret = false;
    }
  }
  
  /* 存在チェック */
  if (ret == true)
  {
    for (int i=0; i<row; i++)
    {
      for (int j=0; j<col; j++)
      {
        for (int k=0; k<high; k++)
        {
          if (place[placeIdx].voxel[x+i][y+j][z+k] == true)
          {
            /* すでに別のアイテムがある */
            ret = false;
            break;
          }
        }
        if (ret == false)
        {
          break;
        }
      }
      if (ret == false)
      {
        break;
      }
    }
  }
  
  return ret;
}

//------------------------------
// voxel座標にitemが格納できるか確認する(高さ方向ははみ出してもよい)
//------------------------------
static bool checkStoreProtrude(uint placeIdx, int x, int y, int z, int row, int col, int high)
{
  bool ret = true;
  int search_z_len;
  
  /* はみ出しチェック */
  if (ret == true)
  {
    if (((x + row) > place[placeIdx].row_i) || ((y + col) > place[placeIdx].col_i))
    {
      ret = false;
    }
  }
  
  if (ret == true)
  {
    search_z_len = high;
    if ((z + high) > place[placeIdx].high_i)
    {
      /* はみ出す場合 */
      search_z_len = place[placeIdx].high_i - z;                /* はみ出さない範囲でサーチする */
    }
  }
  
  /* 存在チェック */
  if (ret == true)
  {
    for (int i=0; i<row; i++)
    {
      for (int j=0; j<col; j++)
      {
        for (int k=0; k<search_z_len; k++)
        {
          if (place[placeIdx].voxel[x+i][y+j][z+k] == true)
          {
            /* すでに別のアイテムがある */
            ret = false;
            break;
          }
        }
        if (ret == false)
        {
          break;
        }
      }
      if (ret == false)
      {
        break;
      }
    }
  }
  
  return ret;
}

//------------------------------
// 内部広域変数placeの中でplace_idが一致するindexを検索する
//------------------------------
static uint findPlaceIdx(uint place_id)
{
  uint ret = UINT_MAX;
  
  for (std::size_t i=0; i<place.size(); i++)
  {
    if(place[i].place_id == place_id)
    {
      ret = (uint)i;
      break;
    }
  }
  
  return ret;
}

//------------------------------
// OctomapのRviz表示
// octo_rviz_mode: モード(0=OFF, 1=最終アクセスOctomapのみ表示, 2=全Octomap表示)
//------------------------------
static void plotOctoRviz(Place_t *placeP)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue strategyParam;
  
  /* 表示オプションを読込 */
  (void)getParamTaskStrategy(&nh, &strategyParam);              /* 返戻値は確認しない */
  
  /* RvizへのTopic配信 */
  vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.ns = "t2_task_strategy_plot_octo";
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.scale.x = placeP->octomap.resolution * (double)strategyParam["octo_rviz_mult"];
  marker.scale.y = placeP->octomap.resolution * (double)strategyParam["octo_rviz_mult"];
  marker.scale.z = placeP->octomap.resolution * (double)strategyParam["octo_rviz_mult"];
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = (double)strategyParam["octo_rviz_alpha"];
  
  /* 全削除 */
  if ((int)strategyParam["octo_rviz_mode"] != 2)
  {
    /* 全表示以外は適用 */
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::DELETE;
    
    for (std::size_t i=0; i<place.size(); i++)
    {
      marker.id = (int)place[i].place_id;
      
      while (vis_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return;
        }
        ROS_WARN_ONCE("(visualization_marker)wait subscriber");
        sleep(1);
      }
      vis_pub.publish(marker);
    }
  }
  
  /* 追加表示 */
  if ((int)strategyParam["octo_rviz_mode"] != 0)
  {
    /* OFF以外は適用 */
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;            /* 既に表示されている場合はMODIFY扱い(ADD=MODIFY=0) */
    marker.id = (int)placeP->place_id;
    
    for (std::size_t i=0; i<placeP->voxel.size(); i++)
    {
      for (std::size_t j=0; j<placeP->voxel[i].size(); j++)
      {
        for (std::size_t k=0; k<placeP->voxel[i][j].size(); k++)
        {
          if (placeP->voxel[i][j][k] == true)
          {
            Eigen::Vector3d worldPoint = coordVox2World(i, j, k, placeP->org, placeP->octomap.resolution, placeP->camera_pos);
            
            geometry_msgs::Point point;
            point.x = worldPoint.x();
            point.y = worldPoint.y();
            point.z = worldPoint.z();
            marker.points.push_back(point);
          }
        }
      }
    }
    
    while (vis_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("(visualization_marker)wait subscriber");
      sleep(1);
    }
    vis_pub.publish(marker);
  }
}

