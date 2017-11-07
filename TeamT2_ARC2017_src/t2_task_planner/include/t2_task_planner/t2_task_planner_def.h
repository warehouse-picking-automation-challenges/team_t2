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

#ifndef T2_TASK_PLANNER_DEF_H
#define T2_TASK_PLANNER_DEF_H

//******************************
//計画系共通定義
//******************************

enum
{
  D_CMD_Dummy = 0,
  D_CMD_ParamInit,         // 初期設定要求
  D_CMD_Initialize,        // 初期化要求
  D_CMD_Calib,             // 補正要求
  D_CMD_Ready_GntrCamera,  // Gntr撮影準備要求
  D_CMD_Ready_ArmCamera,   // Arm撮影準備要求
  D_CMD_Reco_Bin_Gntr,     // Gntr認識要求
  D_CMD_Reco_Bin_Arm,      // Arm認識要求
  D_CMD_Pick_Plan,         // 把持計画要求
  D_CMD_Stow_Plan,         // 収納（リリース）計画
  D_CMD_MoveToPick,        // 把持前移動要求
  D_CMD_PickItem,          // 把持実行要求
  D_CMD_MoveToStow,        // 収納（リリース）移動要求
  D_CMD_StowItem,          // 収納（リリース）実行要求
  D_CMD_StopCtrl,          // 実行終了要求
  D_CMD_MoveOnlyArm,       // アーム単体移動実行要求

  D_MAIN_CMD_MAX
};

// D_CMD_MoveOnlyArm 指定時のsub_cmd_code
enum
{
  D_MoveOnly_ARM_STATE_PLAN = 0,  // ステート名指定移動(StateからState移動でPlanID保存する)
  D_MoveOnly_ARM_STATE,           // ステート名指定移動(PlanID保存なし)
  D_MoveOnly_ARM_CARTESIAN,       // 位置姿勢座標指定移動
  D_MoveOnly_ARM_JOINT,           // 関節角指定移動
  D_MoveOnly_ARM_PLAN_ID,         // 計画済みPLAN指定移動
  D_MoveOnly_ARM_GET_POSE,        // アーム現在位置姿勢取得
  D_MoveOnly_ARM_TEST_MENU,       // アーム単体移動テスト動作

  D_MoveOnly_SubCmd_MAX
};

//受信関数戻り値
enum
{
  RSP_E_OK = 0,  //正常受信
  RSP_E_NG,      //異常受信
  RSP_E_ATTN,    //注意受信（Item把持失敗,把持計画、プラン作成失敗等）
  RSP_E_ILGL,    //イリーガル受信
  RSP_E_TMOUT,   //受信なし、タイムアウト
  RSP_E_HALT,    //デバッグモード停止
  RSP_E_SHTDWN,  //シャットダウン受信
};

// boxID番号
enum
{
  D_start_ID = 0,
  D_bin_A_ID,
  D_bin_B_ID,
  D_bin_C_ID,
  D_bin_D_ID,
  D_bin_E_ID,
  D_bin_F_ID,
  D_bin_G_ID,
  D_bin_H_ID,
  D_bin_I_ID,
  D_bin_J_ID,
  D_tote_1_ID,
  D_tote_2_ID,
  D_box_1_ID,
  D_box_2_ID,
  D_box_3_ID,

  D_boxID_MAX
};
typedef struct t_boxNameId
{
  uint boxType;      // box種別（棚=0、Tote=1）
  char box_name[8];  // box名称(Bin/Tote)
} T_boxNameId;
/*
// boxID番号と同じ並びにする
T_boxNameId t_boxNameId[D_boxID_MAX] = {
  { 0, "start" },  // D_start_ID //不明または開始位置
  { 0, "bin_A" },  // D_bin_A_ID //Bin
  { 0, "bin_B" },  // D_bin_B_ID
  { 0, "bin_C" },  // D_bin_C_ID
  { 0, "bin_D" },  // D_bin_D_ID
  { 0, "bin_E" },  // D_bin_E_ID
  { 0, "bin_F" },  // D_bin_F_ID
  { 0, "bin_G" },  // D_bin_G_ID
  { 0, "bin_H" },  // D_bin_H_ID
  { 0, "bin_I" },  // D_bin_I_ID
  { 0, "bin_J" },  // D_bin_J_ID
  { 1, "tote " },  // D_tote_ID  // Tote
  { 1, "tote_1" },
  { 1, "tote_2" },
  { 1, "tote_3" },
};
*/

#define D_binTblIdx_max 10           // Bin数
#define D_binTblIdx_ofst D_bin_A_ID  // BoxID値からBin情報テーブルへのIndexオフセット

#define D_toteTblIdx_max 4           // Tote数
#define D_toteTblIdx_ofst D_tote_ID  // BoxID値からTote情報テーブルへのIndexオフセット

//アームに対する規定ポジションのグループステート名指示インデックス
// boxID番号の位置に合わせること
enum
{
  posi_arm_Start,     //競技開始位置
  posi_arm_ap_BinA,   // BinA手前位置
  posi_arm_ap_BinB,   // BinB手前位置
  posi_arm_ap_BinC,   // BinC手前位置
  posi_arm_ap_BinD,   // BinD手前位置
  posi_arm_ap_BinE,   // BinE手前位置
  posi_arm_ap_BinF,   // BinF手前位置
  posi_arm_ap_BinG,   // BinG手前位置
  posi_arm_ap_BinH,   // BinH手前位置
  posi_arm_ap_BinI,   // BinI手前位置
  posi_arm_ap_BinJ,   // BinJ手前位置
  posi_arm_ap_Tote1,  // Tote1手前位置
  posi_arm_ap_Tote2,  // Tote2手前位置
  posi_arm_ap_Box1,  // Box1手前位置
  posi_arm_ap_Box2,  // Box2手前位置
  posi_arm_ap_Box3,  // Box3手前位置
  posi_arm_Initial,   //電源OFF時姿勢位置

  D_posID_MAX  //定義最大数
};
// 規定ポジションの位置の関節角テーブル名称
typedef struct t_arm_Joint
{
  char posi_name[32];
} T_arm_Joint;
/*
T_arm_Joint t_arm_Joint_angle[D_posID_MAX] = {
  //	 01234567890123456789012345678912
  "posi_arm_Start",    // [posi_arm_Start  ] 競技開始位置
  "posi_arm_ap_BinA",  // [posi_arm_ap_BinA] BinA手前位置
  "posi_arm_ap_BinB",  // [posi_arm_ap_BinB] BinB手前位置
  "posi_arm_ap_BinC",  // [posi_arm_ap_BinC] BinC手前位置
  "posi_arm_ap_BinD",  // [posi_arm_ap_BinD] BinD手前位置
  "posi_arm_ap_BinE",  // [posi_arm_ap_BinE] BinE手前位置
  "posi_arm_ap_BinF",  // [posi_arm_ap_BinF] BinF手前位置
  "posi_arm_ap_BinG",  // [posi_arm_ap_BinG] BinG手前位置
  "posi_arm_ap_BinH",  // [posi_arm_ap_BinH] BinH手前位置
  "posi_arm_ap_BinI",  // [posi_arm_ap_BinI] BinI手前位置
  "posi_arm_ap_BinJ",  // [posi_arm_ap_BinJ] BinJ手前位置
  "posi_arm_ap_BinK",  // [posi_arm_ap_BinK] BinK手前位置
  "posi_arm_ap_BinL",  // [posi_arm_ap_BinL] BinL手前位置
  "posi_arm_ap_Tote",  // [posi_arm_ap_Tote] Tote手前位置
  "posi_arm_ap_Tote1", // [posi_arm_ap_Tote1] CardboardA手前位置
  "posi_arm_ap_Tote2", // [posi_arm_ap_Tote2] CardboardB手前位置
  "posi_arm_ap_Tote3", // [posi_arm_ap_Tote3] CardboardC手前位置
  "posi_arm_Initial"   // [posi_arm_Initial] 電源OFF時姿勢位置(boxID番号では指定不可)
};
*/

/*
T_arm_Joint plan_posi_name[D_boxID_MAX] = {
  //	 01234567890123456789012345678912
  "Start",    // 競技開始位置
  "ap_BinA",  // BinA手前位置
  "ap_BinB",  // BinB手前位置
  "ap_BinC",  // BinC手前位置
  "ap_BinD",  // BinD手前位置
  "ap_BinE",  // BinE手前位置
  "ap_BinF",  // BinF手前位置
  "ap_BinG",  // BinG手前位置
  "ap_BinH",  // BinH手前位置
  "ap_BinI",  // BinI手前位置
  "ap_BinJ",  // BinJ手前位置
  "ap_BinK",  // BinK手前位置
  "ap_BinL",  // BinL手前位置
  "ap_Tote",  // Tote手前位置
  "ap_Tote1", // Tote1手前位置
  "ap_Tote2", // Tote2手前位置
  "ap_Tote3"  // Tote3手前位置
};
*/

namespace TaskTypes
{
enum TaskType
{
  Pick,
  Stow
};
}
typedef TaskTypes::TaskType TaskType;

const std::string CONTAINER_INFO_PATH = "/t2_database/container_info";
const std::string BOX_SETTINGS_PATH = "/t2_database/box_settings";
const std::string ORDER_INPUT_PATH = "/t2_database/order/input";

//******************************
#endif  // T2_TASK_PLANNER_DEF_H

