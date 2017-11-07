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

#define T2_TASK_STRATEGY_GRASP_PACK_SRC
#include "t2_task_planner/t2_task_strategy_grasp_pack.h"

/* システムヘッダ参照 */
#include <eigen_conversions/eigen_msg.h>

/* 外部ヘッダ参照 */
#include "t2_task_planner/t2_task_strategy_component.h"
#include "t2_task_planner/t2_task_strategy_grasp.h"
#include "t2_task_planner/t2_task_strategy_pack.h"

/* 内部定数定義 */
#define LARGE_BIN_PLACE_ID              ((uint)1)
#define AMNESTY_TOTE_PLACE_ID           ((uint)12)

/* 内部型定義 */
typedef struct{
  uint place_id;                                                /* 場所 */
  double eval_val;                                              /* 評価値 */
} PLACE_EVAL_T;

/* 内部変数定義 */

/* 内部関数定義 */
static void chgFormatReq(
  uint cad_id,
  uint single_gp,
  geometry_msgs::Pose center_pose,
  std::vector<Location_t> grasp_location_list,
  std::vector<Location_t> release_location_list,
  std::vector<uint> place_id_list,
  GRASP_INPUT_REQ_T *reqP
);
static void chgFormatRsp(GRASP_OUTPUT_RSP_T *rspP, std::vector<GpRp_t> *gprp);
static void sortScore(GRASP_OUTPUT_RSP_T *rspP);
static bool compScore(GRASP_POINT_T a, GRASP_POINT_T b);
static bool graspProc(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP);
static bool packProcFix(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP);
static bool packProc(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP);
static void deleteReservePlace(GRASP_INPUT_REQ_T *reqP);
static void rmMemberListUint(std::vector<uint> *targetList, std::vector<uint> *rmMember);
static void rmNotRemainMemberListUint(std::vector<uint> *targetList, std::vector<uint> *remainMember);
static void sortVolume(GRASP_INPUT_REQ_T *reqP);
static void sortBottomArea(GRASP_INPUT_REQ_T *reqP);
static void sortEval(std::vector<PLACE_EVAL_T> *work);
static bool compEval(PLACE_EVAL_T a, PLACE_EVAL_T b);
static void reCalcRapCeilingCorrection(GRASP_OUTPUT_RSP_T *rspP);
static void reScoreSingleGp(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP);
static void deleteScoreZero(GRASP_OUTPUT_RSP_T *rspP);

/****************/
/* 外部関数処理 */
/****************/
//------------------------------
// Itemの把持・リリース姿勢の取得(決め打ちStow用)
//------------------------------
bool getGpRpFix(
  uint cad_id,
  uint single_gp,
  geometry_msgs::Pose center_pose,
  std::vector<uint> place_id_list,
  std::vector<GpRp_t> *gprp
){
  std::lock_guard<std::mutex> lock(mtx_component);
  
  GRASP_INPUT_REQ_T req;
  GRASP_OUTPUT_RSP_T rsp;
  std::vector<Location_t> dmy_loc_list;
  bool ret;
  
  dmy_loc_list.clear();
  chgFormatReq(cad_id, single_gp, center_pose, dmy_loc_list, dmy_loc_list, place_id_list, &req);
  printReqInfo(&req, "input request");
  
  ret = graspProc(&req, &rsp);
  if (ret == true)
  {
    ret = packProcFix(&req, &rsp);
  }
  
  reScoreSingleGp(&req, &rsp);
  sortScore(&rsp);
  deleteScoreZero(&rsp);
  
  printRspInfo(&rsp, "output response");
  chgFormatRsp(&rsp, gprp);
  
  return ret;
}

//------------------------------
// Itemの把持・リリース姿勢の取得(予測ありStow用)
//------------------------------
bool getGpRpWithPrediction(
  uint cad_id,
  uint single_gp,
  geometry_msgs::Pose center_pose,
  std::vector<Location_t> grasp_location_list,
  std::vector<Location_t> release_location_list,
  std::vector<uint> place_id_list,
  std::vector<GpRp_t> *gprp
){
  std::lock_guard<std::mutex> lock(mtx_component);
  
  GRASP_INPUT_REQ_T req;
  GRASP_OUTPUT_RSP_T rsp;
  bool ret;
  
  chgFormatReq(cad_id, single_gp, center_pose, grasp_location_list, release_location_list, place_id_list, &req);
  printReqInfo(&req, "input request");
  
  ret = graspProc(&req, &rsp);
  if (ret == true)
  {
    deleteReservePlace(&req);                                   /* 大きいものを入れる予定のあるplaceは、place_id_listから除外する */
    sortVolume(&req);                                           /* place_id_listを残り体積の大きい順にソート */
    ret = packProc(&req, &rsp);
  }
  
  reScoreSingleGp(&req, &rsp);
  sortScore(&rsp);
  deleteScoreZero(&rsp);
  
  printRspInfo(&rsp, "output response");
  chgFormatRsp(&rsp, gprp);
  
  return ret;
}

//------------------------------
// Itemの把持・リリース姿勢の取得(予測なしPick用)
//------------------------------
bool getGpRpWithoutPrediction(
  uint cad_id,
  uint single_gp,
  geometry_msgs::Pose center_pose,
  std::vector<uint> place_id_list,
  std::vector<GpRp_t> *gprp
){
  std::lock_guard<std::mutex> lock(mtx_component);
  
  GRASP_INPUT_REQ_T req;
  GRASP_OUTPUT_RSP_T rsp;
  std::vector<Location_t> dmy_loc_list;
  bool ret;
  
  dmy_loc_list.clear();
  chgFormatReq(cad_id, single_gp, center_pose, dmy_loc_list, dmy_loc_list, place_id_list, &req);
  printReqInfo(&req, "input request");
  
  ret = graspProc(&req, &rsp);
  if (ret == true)
  {
    sortBottomArea(&req);                                       /* place_id_listを残り底面積の広い順にソート */
    ret = packProc(&req, &rsp);
  }
  
  reScoreSingleGp(&req, &rsp);
  sortScore(&rsp);
  deleteScoreZero(&rsp);
  
  printRspInfo(&rsp, "output response");
  chgFormatRsp(&rsp, gprp);
  
  return ret;
}

/****************/
/* 内部関数処理 */
/****************/
//------------------------------
// REQフォーマット変換(乗せ替え)
//------------------------------
static void chgFormatReq(
  uint cad_id,
  uint single_gp,
  geometry_msgs::Pose center_pose,
  std::vector<Location_t> grasp_location_list,
  std::vector<Location_t> release_location_list,
  std::vector<uint> place_id_list,
  GRASP_INPUT_REQ_T *reqP
){
  reqP->cad_id = cad_id;
  reqP->single_gp = single_gp;
  tf::poseMsgToEigen(center_pose, reqP->g_center_pos);
  reqP->grasp_location_list = grasp_location_list;
  reqP->release_location_list = release_location_list;
  reqP->place_id_list = place_id_list;
}

//------------------------------
// RSPフォーマット変換(乗せ替え)
//------------------------------
static void chgFormatRsp(GRASP_OUTPUT_RSP_T *rspP, std::vector<GpRp_t> *gprp)
{
  gprp->resize(rspP->grasp_point.size());
  for (std::size_t i=0; i<gprp->size(); i++)
  {
    gprp->at(i).gp_number = rspP->grasp_point[i].gp_number;
    gprp->at(i).score = rspP->grasp_point[i].score;
    gprp->at(i).grasp_pattern = rspP->grasp_point[i].grasp_pattern;
    gprp->at(i).release_place_id = rspP->grasp_point[i].release_place_id;
    gprp->at(i).protrude_length = rspP->grasp_point[i].protrude_length;
    
    gprp->at(i).suction_strength = rspP->grasp_point[i].suction_strength;
    //gprp->at(i).length_of_pushing_for_suction = rspP->grasp_point[i].length_of_pushing_for_suction;  //★用途不明のため、IF側には未出力
    gprp->at(i).threshold_of_vacuum_for_suction = rspP->grasp_point[i].threshold_of_vacuum_for_suction;
    gprp->at(i).carry_speed = rspP->grasp_point[i].carry_speed;
    gprp->at(i).width_between_finger_for_pinch = rspP->grasp_point[i].width_between_finger_for_pinch;
    gprp->at(i).width_between_finger_for_release = rspP->grasp_point[i].width_between_finger_for_release;
    gprp->at(i).finger_intrusion_for_pinch = rspP->grasp_point[i].finger_intrusion_for_pinch;
    gprp->at(i).max_effort_for_pinch = rspP->grasp_point[i].max_effort_for_pinch;
    
    tf::poseEigenToMsg(rspP->grasp_point[i].grasp_point_item, gprp->at(i).gp);
    tf::poseEigenToMsg(rspP->grasp_point[i].approach_point_item, gprp->at(i).gap);
    tf::poseEigenToMsg(rspP->grasp_point[i].rp, gprp->at(i).rp);
    tf::poseEigenToMsg(rspP->grasp_point[i].rap, gprp->at(i).rap);
  }
}

//------------------------------
// スコアでソート
//------------------------------
static void sortScore(GRASP_OUTPUT_RSP_T *rspP)
{
  printRspDebug(rspP, "sortScore before");
  std::sort(rspP->grasp_point.begin(), rspP->grasp_point.end(), compScore);
  printRspDebug(rspP, "sortScore after");
}

//------------------------------
// ソート評価関数
//------------------------------
static bool compScore(GRASP_POINT_T a, GRASP_POINT_T b)
{
  return (a.score > b.score);
}

//------------------------------
// Grasp処理
//------------------------------
static bool graspProc(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP)
{
  bool ret;
  
  /* 通常処理 */
  graspCalc(reqP, rspP);
  reScoreBottom(reqP, rspP);
  reScorePoseY(reqP, rspP);
  graspReCalc(reqP, rspP);
  
  ret = (rspP->result == GRASP_RESULT_SUCCESS);
  
  if (ret == false)
  {
    ROS_WARN("Grasp Process return false.");
  }
  
  return ret;
}

//------------------------------
// Pack処理(決め打ち)
//------------------------------
static bool packProcFix(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP)
{
  bool ret = false;
  
  printRspDebug(rspP, "packProcFix before");
  
  for (std::size_t i=0; i<rspP->grasp_point.size(); i++)
  {
    /* GPの個数分回す */
    rspP->grasp_point[i].release_place_id = UINT_MAX;           /* 初期値を解なしとしてUINT_MAXをセット */
    
    if (rspP->grasp_point[i].score > 0.0)
    {
      /* スコアが0点でないなら、RPを計算する */
      for (std::size_t j=0; j<reqP->place_id_list.size(); j++)
      {
        /* 候補リスト分回す */
        bool result = packCalcFix(reqP, rspP, i, j);
        if (result == true)
        {
          /* 解発見 */
          rspP->grasp_point[i].release_place_id = reqP->place_id_list[j];
          ret = true;                                           /* 一つでも見つけたらtrueで上位に返す */
          break;
        }
      }
      
      if (rspP->grasp_point[i].release_place_id == UINT_MAX)
      {
        /* 候補リストから解が見つかっていない */
        rspP->grasp_point[i].score = 0.0;                       /* スコアを0に上書き */
      }
    }
  }
  
  printRspDebug(rspP, "packProcFix after");
  
  if (ret == false)
  {
    ROS_WARN("PackFix Process return false.");
  }
  
  return ret;
}

//------------------------------
// Pack処理
//------------------------------
static bool packProc(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP)
{
  bool ret = false;
  
  printRspDebug(rspP, "packProc before");
  
  /* Amnestyチェック */
  uint amnestyIdx = UINT_MAX;
  for (std::size_t k=0; k<reqP->place_id_list.size(); k++)
  {
    if (reqP->place_id_list[k] == AMNESTY_TOTE_PLACE_ID)
    {
      amnestyIdx = (uint)k;
      break;
    }
  }
  
  for (std::size_t i=0; i<rspP->grasp_point.size(); i++)
  {
    /* GPの個数分回す */
    rspP->grasp_point[i].release_place_id = UINT_MAX;           /* 初期値を解なしとしてUINT_MAXをセット */
    
    if (rspP->grasp_point[i].score > 0.0)
    {
      /* スコアが0点でないなら、RPを計算する */
      
      if (amnestyIdx == UINT_MAX)
      {
        /* 行先にAmnestyを含んでないなら箱詰めする */
        if (reqP->single_gp == (uint)0)
        {
          /* 通常処理(平面検知でない)なら演算する */
          for (std::size_t j=0; j<reqP->place_id_list.size(); j++)
          {
            /* 候補リスト分回す */
            bool result = packCalc(reqP, rspP, i, j, false);
            if (result == true)
            {
              /* 解発見 */
              rspP->grasp_point[i].release_place_id = reqP->place_id_list[j];
              ret = true;                                       /* 一つでも見つけたらtrueで上位に返す */
              break;
            }
          }
          
          if (rspP->grasp_point[i].release_place_id == UINT_MAX)
          {
            /* 候補リストからハミ出ない解が見つかっていない場合、ハミ出る解を見つける */
            for (std::size_t j=0; j<reqP->place_id_list.size(); j++)
            {
              /* 候補リスト分回す */
              bool result = packCalc(reqP, rspP, i, j, true);
              if (result == true)
              {
                /* 解発見 */
                rspP->grasp_point[i].release_place_id = reqP->place_id_list[j];
                ret = true;                                     /* 一つでも見つけたらtrueで上位に返す */
                break;
              }
            }
          }
        }
        
        if (rspP->grasp_point[i].release_place_id == UINT_MAX)
        {
          /* 候補リストから解が見つかっていない場合、place上方を解とする */
          for (std::size_t j=0; j<reqP->place_id_list.size(); j++)
          {
            /* 候補リスト分回す */
            bool result = packPlaceUpside(reqP, rspP, i, j);
            if (result == true)
            {
              /* 解発見 */
              rspP->grasp_point[i].release_place_id = reqP->place_id_list[j];
              ret = true;                                       /* 一つでも見つけたらtrueで上位に返す */
              break;
            }
          }
        }
      }
      else
      {
        /* 行先にAmnestyを含んでいるならAmnesty上方をRPとする */
        bool result = packAmnestyUpside(reqP, rspP, i, amnestyIdx);
        if (result == true)
        {
          /* 解発見 */
          rspP->grasp_point[i].release_place_id = reqP->place_id_list[amnestyIdx];
          ret = true;                                           /* 一つでも見つけたらtrueで上位に返す */
        }
      }
    }
  }
  
  reCalcRapCeilingCorrection(rspP);                             /* RAPの天井補正 */
  
  printRspDebug(rspP, "packProc after");
  
  if (ret == false)
  {
    ROS_WARN("Pack Process return false.");
  }
  
  return ret;
}

//------------------------------
// 大きいものを入れる予定のあるplaceは、place_id_listから除外する
//------------------------------
static void deleteReservePlace(GRASP_INPUT_REQ_T *reqP)
{
  bool end_flg = false;
  
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue targetParam, containerParam;
  std::vector<Location_t> gloc = reqP->grasp_location_list;
  std::vector<Location_t> rloc = reqP->release_location_list;
  
  /* 使用パラメータを準備 */
  if (end_flg == false)
  {
    /* 把持リスト読み込み(処理対象) */
    bool ret = getParamGraspInfoList(reqP->cad_id, &nh, &targetParam);
    if (ret == false)
    {
      /* パラメータロードエラー */
      end_flg = true;
    }
  }
  if (end_flg == false)
  {
    /* container_info読み込み */
    bool ret = getParamContainerInfo(&nh, &containerParam);
    if (ret == false)
    {
      /* パラメータロードエラー */
      end_flg = true;
    }
  }
  
  if (end_flg == false)
  {
    /* 処理対象アイテムがLargeBin専用か確認 */
    if ((int)targetParam["large_bin_flag"] == 1)
    {
      /* LargeBin専用アイテム */
      
      /* 行先候補の中にLargeBinがあるか確認 */
      bool lbin_exist_flg = false;
      for (std::size_t i=0; i<reqP->place_id_list.size(); i++)
      {
        if (reqP->place_id_list[i] == LARGE_BIN_PLACE_ID)
        {
          lbin_exist_flg = true;
          break;
        }
      }
      
      if (lbin_exist_flg == true)
      {
        /* 行先候補にLargeBinがある場合 */
        std::vector<uint> tmpid;
        tmpid.push_back(LARGE_BIN_PLACE_ID);
        
        /* LargeBin以外を消す */
        rmNotRemainMemberListUint(&(reqP->place_id_list), &tmpid);
        end_flg = true;
      }
    }
  }
  
  if (end_flg == false)
  {
    /* grasp_location_listにLargeBin専用アイテムが含まれているか確認 */
    std::vector<uint> rmMember;
    
    std::vector<Location_t>::iterator g_it = gloc.begin();
    while ((g_it != gloc.end()) && (end_flg == false))
    {
      std::vector<uint>::iterator cad_it = (*g_it).cad_id_list.begin();
      while ((cad_it != (*g_it).cad_id_list.end()) && (end_flg == false))
      {
        /* 把持リスト読み込み(未来のアイテム) */
        XmlRpc::XmlRpcValue workParam;
        bool ret = getParamGraspInfoList((*cad_it), &nh, &workParam);
        if (ret == false)
        {
          /* パラメータロードエラー */
          end_flg = true;
        }
        else
        {
          if ((int)workParam["large_bin_flag"] == 1)
          {
            /* LargeBinアイテムを発見 */
            /* 削除リストに登録する */
            rmMember.push_back(LARGE_BIN_PLACE_ID);
            
            /* grasp_location_listから削除 */
            cad_it = (*g_it).cad_id_list.erase(cad_it);
          }
          else
          {
            cad_it++;
          }
        }
      }
      
      g_it++;
    }
    
    rmMemberListUint(&(reqP->place_id_list), &rmMember);
  }
  
  if (end_flg == false)
  {
    /* release_location_listに空のBinがあるか確認 */
    end_flg = true;                                             /* 見つからなかったら終了 */
    
    std::vector<Location_t>::iterator r_it = rloc.begin();
    while (r_it != rloc.end())
    {
      if ((*r_it).cad_id_list.size() != 0)
      {
        /* 空でないBin */
        r_it = rloc.erase(r_it);                                /* 以降の処理で使わないので消す */
      }
      else
      {
        /* 空のBin発見 */
        end_flg = false;                                        /* 以降の処理を継続 */
        r_it++;
      }
    }
  }
  
  if (end_flg == false)
  {
    /* 処理対象アイテムがBinの体積70%超えか確認 */
    std::vector<uint> tmpid;
    
    for (std::size_t i=0; i<rloc.size(); i++)
    {
      uint containerIdx = findContainerIdx(&containerParam, rloc[i].place_id); /* 該当place_idが、container_infoのどのindexか検索する */
      
      /* そのplaceに格納した場合に何%になるか調べる */
      double percent = (double)targetParam["volume_of_item"] / (double)containerParam[containerIdx]["volume"];
      if ((0.7 <= percent) && (percent <= 1.0))
      {
        /* 70%を超える場合、そのBinに入れる */
        tmpid.push_back(rloc[i].place_id);
      }
    }
    
    /* 70%を超えるBinをみつけたか */
    if (tmpid.size() != 0)
    {
      /* 超えるBin以外は消す */
      rmNotRemainMemberListUint(&(reqP->place_id_list), &tmpid);
      end_flg = true;
    }
  }
  
  if (end_flg == false)
  {
    /* 未来のアイテムがBinの体積70%超えか確認 */
    std::vector<uint> rmMember;
    
    std::vector<Location_t>::iterator r_it = rloc.begin();
    while (r_it != rloc.end())
    {
      uint containerIdx = findContainerIdx(&containerParam, (*r_it).place_id); /* 該当place_idが、container_infoのどのindexか検索する */
      bool flg = false;
      
      for (std::size_t i=0; i<gloc.size(); i++)
      {
        std::vector<uint>::iterator g_it = gloc[i].cad_id_list.begin();
        while (g_it != gloc[i].cad_id_list.end())
        {
          /* 把持リスト読み込み(未来のアイテム) */
          XmlRpc::XmlRpcValue workParam;
          bool ret = getParamGraspInfoList((*g_it), &nh, &workParam);
          if (ret == false)
          {
            /* パラメータロードエラー */
            end_flg = true;
          }
          else
          {
            /* そのplaceに格納した場合に何%になるか調べる */
            double percent = (double)workParam["volume_of_item"] / (double)containerParam[containerIdx]["volume"];
            
            if ((0.7 <= percent) && (percent <= 1.0))
            {
              /* 70%を超える */
              rmMember.push_back((*r_it).place_id);
              flg = true;
            }
          }
          
          if (flg == true)
          {
            g_it = gloc[i].cad_id_list.erase(g_it);
            break;
          }
          else
          {
            g_it++;
          }
        }
        if (flg == true)
        {
          break;
        }
      }
      
      if (flg == true)
      {
        r_it = rloc.erase(r_it);
      }
      else
      {
        r_it++;
      }
    }
    
    /* 70%を超えるBinをみつけた場合削除する */
    rmMemberListUint(&(reqP->place_id_list), &rmMember);
  }
}

//------------------------------
// Listから指定されたメンバを消す(uint用)
//------------------------------
static void rmMemberListUint(std::vector<uint> *targetList, std::vector<uint> *rmMember)
{
  std::vector<uint>::iterator it = (*targetList).begin();
  while (it != (*targetList).end())
  {
    bool flg = false;
    
    for (std::size_t i=0; i<(*rmMember).size(); i++)
    {
      if (*it == (*rmMember)[i])
      {
        flg = true;
        break;
      }
    }
    
    if (flg == true)
    {
      it = (*targetList).erase(it);
    }
    else
    {
      it++;
    }
  }
}

//------------------------------
// Listから指定されたメンバ以外を消す(uint用)　≠　Listを指定されたメンバにする
//------------------------------
static void rmNotRemainMemberListUint(std::vector<uint> *targetList, std::vector<uint> *remainMember)
{
  std::vector<uint>::iterator it = (*targetList).begin();
  while (it != (*targetList).end())
  {
    bool flg = false;
    
    for (std::size_t i=0; i<(*remainMember).size(); i++)
    {
      if (*it == (*remainMember)[i])
      {
        flg = true;
        break;
      }
    }
    
    if (flg == false)
    {
      it = (*targetList).erase(it);
    }
    else
    {
      it++;
    }
  }
}

//------------------------------
// place_id_listを残り体積の大きい順にソート
//------------------------------
static void sortVolume(GRASP_INPUT_REQ_T *reqP)
{
  std::vector<PLACE_EVAL_T> work;
  
  /* 入力の展開と評価値の算出 */
  for (std::size_t i=0; i<reqP->place_id_list.size(); i++)
  {
    PLACE_EVAL_T tmp;
    tmp.place_id = reqP->place_id_list[i];
    tmp.eval_val = (double)calcFreeVolume(reqP->place_id_list[i]); /* 評価値はplaceの底面積 */
    work.push_back(tmp);
  }
  
  /* ソート */
  sortEval(&work);
  
  /* 更新 */
  reqP->place_id_list.clear();
  for (std::size_t i=0; i<work.size(); i++)
  {
    reqP->place_id_list.push_back(work[i].place_id);
  }
}

//------------------------------
// place_id_listを残り底面積の広い順にソート
//------------------------------
static void sortBottomArea(GRASP_INPUT_REQ_T *reqP)
{
  std::vector<PLACE_EVAL_T> work;
  
  /* 入力の展開と評価値の算出 */
  for (std::size_t i=0; i<reqP->place_id_list.size(); i++)
  {
    PLACE_EVAL_T tmp;
    tmp.place_id = reqP->place_id_list[i];
    tmp.eval_val = (double)calcFreeBottomArea(reqP->place_id_list[i]); /* 評価値はplaceの底面積 */
    work.push_back(tmp);
  }
  
  /* ソート */
  sortEval(&work);
  
  /* 更新 */
  reqP->place_id_list.clear();
  for (std::size_t i=0; i<work.size(); i++)
  {
    reqP->place_id_list.push_back(work[i].place_id);
  }
}

//------------------------------
// 評価値の大きい順でソート
//------------------------------
static void sortEval(std::vector<PLACE_EVAL_T> *work)
{
  std::sort(work->begin(), work->end(), compEval);
}

//------------------------------
// 評価関数(評価値の大きさ)
//------------------------------
static bool compEval(PLACE_EVAL_T a, PLACE_EVAL_T b)
{
  return (a.eval_val > b.eval_val);
}

//------------------------------
// RAPの天井補正
//------------------------------
static void reCalcRapCeilingCorrection(GRASP_OUTPUT_RSP_T *rspP)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue strategyParam;
  
  /* Strategyパラメータ展開 */
  bool ret = getParamTaskStrategy(&nh, &strategyParam);
  if (ret == true)
  {
    printRspDebug(rspP, "reCalcRapCeilingCorrection before");
    for (std::size_t i=0; i<rspP->grasp_point.size(); i++)
    {
      /* GPの個数分回す */
      if (rspP->grasp_point[i].score > 0.0)
      {
        /* スコア0より大きいものだけ処理 */
        if(rspP->grasp_point[i].rap.translation().z() > (double)strategyParam["rap_max_coord_z"])
        {
          /* RAPが最大Z座標を超えるなら足切り */
          rspP->grasp_point[i].rap.translation().z() = (double)strategyParam["rap_max_coord_z"];
        }
        if(rspP->grasp_point[i].rap.translation().z() < rspP->grasp_point[i].rp.translation().z())
        {
          /* RAPがRPを下回るなら切上げ */
          rspP->grasp_point[i].rap.translation().z() = rspP->grasp_point[i].rp.translation().z();
        }
      }
    }
    printRspDebug(rspP, "reCalcRapCeilingCorrection after");
  }
}

//------------------------------
// 平面検知フラグによる再スコア
//------------------------------
static void reScoreSingleGp(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue infoParam;
  
  /* 平面検知フラグ確認 */
  if (reqP->single_gp == (uint)1)
  {
    printRspDebug(rspP, "reScoreSingleGp before");
    
    bool ret = getParamGraspInfoList(reqP->cad_id, &nh, &infoParam);
    if (ret == true)
    {
      /* represent_gp以外のscoreを0にする */
      for (std::size_t i=0; i<rspP->grasp_point.size(); i++)
      {
        if ((int)(rspP->grasp_point[i].gp_number / 100) != (int)infoParam["represent_gp"]) /* オリジナルのgp_numberと比較 */
        {
          rspP->grasp_point[i].score = 0.0;
        }
      }
    }
    
    printRspDebug(rspP, "reScoreSingleGp after");
  }
}

//------------------------------
// スコア0のものをGpRpリストから削除
//------------------------------
static void deleteScoreZero(GRASP_OUTPUT_RSP_T *rspP)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue strategyParam;
  
  printRspDebug(rspP, "deleteScoreZero before");

  /* Strategyパラメータ展開 */
  bool ret = getParamTaskStrategy(&nh, &strategyParam);

  if ((bool)strategyParam["delete_score_zero"])
  {  
    std::vector<GRASP_POINT_T>::iterator it = rspP->grasp_point.begin();
    while (it != rspP->grasp_point.end())
    {
      if (it->score <= 0.0)
      {
        it = rspP->grasp_point.erase(it);
      }
      else
      {
        it++;
      }
    }
  }
  
  printRspDebug(rspP, "deleteScoreZero after");
}

