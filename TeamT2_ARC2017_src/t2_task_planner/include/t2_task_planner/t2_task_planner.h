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

#ifndef T2_TASK_PLANNER_H
#define T2_TASK_PLANNER_H

//#define D_RecogAll			//ターゲット以外も認識要求する。コメントアウトした場合はターゲットのみ認識要求。

#define USE_GRASP_RELEASE_WITH_PREDICTION

#define USE_WEIGHT_SCALE

#include <dirent.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <t2_msgs/UIConsole.h>             //キー入力／出力メッセージ
#include <t2_msgs/GetArmPose.h>            //アーム位置姿勢取得サービス(6軸+直動)
#include <t2_msgs/GetArmGroupStatePose.h>  //アームグループステート名の姿勢取得サービス

#include <t2_robot_msgs/CaptureRes.h>
#include <t2_robot_msgs/CaptureReq.h>
#include <t2_msgs/RecognizeRes.h>
#include <t2_robot_msgs/RecognizeReq.h>
#include <t2_msgs/OccupancyReq.h>
#include <t2_msgs/OccupancyRes.h>

#include <t2_msgs/GetGraspInfo.h>
#include <t2_robot_msgs/Matrix4x4.h>  // 同次変換行列
#include <t2_robot_msgs/conversions.h>

#include <t2_msgs/GripperPump.h>
#include <t2_msgs/GripperSuction.h>

#include <t2_msgs/GetPlannedArmTrajectory.h>  //計画済みアーム軌道取得サービス

#include <t2_msgs/ArmPlanAction.h>       // 動作計画(Plan)★
#include <t2_msgs/ArmExecuteAction.h>    // 軌道実行(Execute)★
#include <t2_msgs/ArmMoveAction.h>       // 移動(Move : Plan & Execute)★

#include <t2_msgs/ArmPickPlanAction.h>   // PickPlan
#include <t2_msgs/ArmPickExecuteAction.h>// PickExecute

#include <t2_msgs/ArmPlaceMoveAction.h>  // PlaceMove

#include <t2_msgs/GripperReleaseAction.h>
#include <t2_msgs/GripperRetractAction.h>

#include <actionlib/client/simple_action_client.h>

#include <t2_msgs/AddItemsToPlanningScene.h>
#include <t2_msgs/SetContainerObjectToPlanningScene.h>

#include <t2_planning_scene_updater/capability_names.h>
#include <t2_motion_planner/capability_names.h>

#include <t2_pinching_gripper/capability_names.h>

#include <t2_msgs/GripperInitialize.h>
#include <t2_msgs/GripperFinalize.h>

#include <t2_msgs/WeightScaleStart.h>
#include <t2_msgs/WeightScaleStop.h>

#include <t2_task_planner/t2_task_planner_def.h>
#include <t2_task_planner/t2_box_info.h>
#include <t2_task_planner/t2_task_strategy_common.h>
#include <t2_task_planner/t2_task_strategy_grasp_pack.h>

#include <keba_bridge_msgs/KebaInitialize.h>
#include <keba_bridge_msgs/KebaFinalize.h>

#include <thread>
#include <mutex>

//----------------------------------
// 定数
//----------------------------------
#define RSP_TIME_LIMIT 180       // 180秒待ち
#define RSP_TIME_LIMIT_RECO 300  // 300秒待ち
#define D_RECOG_RETRY_MAX 3      // 認識撮影、認識リトライ回数

enum
{
  D_STAGE_HALT,      // 処理停止ステージ（デバッグ用）
  D_STAGE_WaiSetUp,  // 初期設定ステージ
  D_STAGE_Initial,   // 初期化ステージ
  D_STAGE_PickOrder,
  D_STAGE_StowOrder,
  D_STAGE_ErrRecover,  // エラーリカバリ処理ステージ
  D_STAGE_ProcessEnd,  // 処理終了実行ステージ
  D_STAGE_ResetHand,
  D_STAGE_ResetArm,

  D_STAGE_MAIN_MAX
};

// テスト用アーム指定位置移動ファイル登録リスト
#define D_fileNameMax 128  // ファイル名の最大文字数
#define D_fileCountMax 64  // ファイル数最大

#define D_csvlineMax 512  // ファイル読み込みの1行の最大文字数

// plan-ctrl間メッセージの置き換え　とりあえず : ここから

typedef struct
{
  uint boxID;
  uint recog_req_JobNo;
  uint cadID;
  double probability;
} T_ReqCameraData;

typedef struct
{
  uint cmd_code;
  uint sub_cmd_code;
  uint sub_cmd_data;
  uint mainDebug_mode;
  uint mainDebug_step;
  uint boxID;
  uint from_boxID;
  uint itemID;
  uint cadID;
  uint RecogJobNo;
  T_ReqCameraData ReqCameraDatas[3];
  int planID;
  uint arm_state_index;
  //	t2_robot_msgs::Matrix4x4 arm_move_pose;
  //	float	joint_positions[8];
  t2_msgs::ArmPose ArmPose;
  float velocity;  //速度
} T_PlanMainCmdMsg;

typedef struct
{
  uint rsp_code;
  int rsp_result;
  uint itemID;
  uint cadID;
  uint grasp_sore;
  T_ReqCameraData ReqCameraDatas[3];
  uint recog_req_JobNo[3];
  int planID;
  float planning_time;
  t2_msgs::ArmPose ArmPose;
  //	float			joint_positions[];
} T_PlanMainRspMsg;

typedef struct
{
  uint cadID;
  float prob;
} T_ItemCtrlResult;

// plan-ctrl間メッセージの置き換え　とりあえず : ここまで

typedef struct
{
  int move_step_count_index;  //動作ステップ番号
  int arm_move_type;          //動作指定タイプ（ステート/座標/ジョイント）
  uint state_name_index;      //ステート名インデックス
  char Name[128];             //座標名
  float velocity;             //速度
  t2_msgs::ArmPose ArmPose;   //目標位置
} T_ArmMovePose;
typedef struct
{
  T_ArmMovePose init_moveTarget;   //初期位置目標名称、座標、速度
  T_ArmMovePose start_moveTarget;  //開始位置目標名称、座標、速度
  T_ArmMovePose moveTarget;        //目標名称、座標、速度
} T_SeqArmMoveParam;

typedef actionlib::SimpleActionClient<t2_msgs::ArmPlanAction> ArmPlanActionClient;
typedef actionlib::SimpleActionClient<t2_msgs::ArmExecuteAction> ArmExecuteActionClient;
typedef actionlib::SimpleActionClient<t2_msgs::ArmMoveAction> ArmMoveActionClient;

typedef actionlib::SimpleActionClient<t2_msgs::ArmPickPlanAction> ArmPickPlanActionClient;
typedef actionlib::SimpleActionClient<t2_msgs::ArmPickExecuteAction> ArmPickExecuteActionClient;

typedef actionlib::SimpleActionClient<t2_msgs::ArmPlaceMoveAction> ArmPlaceMoveActionClient;

typedef actionlib::SimpleActionClient<t2_msgs::GripperReleaseAction> GripperReleaseActionClient;
typedef actionlib::SimpleActionClient<t2_msgs::GripperRetractAction> GripperRetractActionClient;

//
typedef struct
{  // 制御ステージコード
  uint mainStageCode;
  uint mainStageCodeOld;
} T_MainStatus;

//ガントリカメラ撮影管理情報テーブル
typedef struct
{
  uint boxID;       // boxID番号(A-L)
  uint item_keyID;  // Pick候補
  uint cadID;       // Pick候補(cadID)
  uint RecogJobNo;  // Item認識データ要求Job番号
} T_GntrRecogCntrl;

//ガントリカメラ撮影管理情報テーブル
enum
{
  D_CAMERA_OFF,  // カメラ未撮影
  D_CAMERA_REQ,  // カメラ撮影要求中
};
typedef struct
{
  uint boxID;       // boxID番号(A-L)
  uint cadID;       // Pick候補
  uint CameraInfo;  // Item認識情報
  uint RecogJobNo;  // Item認識データ要求Job番号
} T_GntrCameraCntrl;

#define D_RecogJobData_max 128  //認識結果座標保存数
typedef struct
{
  uint RecogJobNo;     // Item認識データJob番号
  uint cadID;          // Pick候補
  uint recogIndex;     // 認識インデックス
  double probability;  // itemの確からしさ（確率）
  geometry_msgs::Pose pose;
} T_RecogJobRslt;

//把持実行時に使用する重心座標情報
typedef struct
{
  uint cadID;                // Pick候補
  uint recogIndex;           // 認識インデックス
  geometry_msgs::Pose pose;  //重心座標
} T_GraspExeInfo;

// ArmPlanID作成管理用
typedef struct t_arm_planID
{
  bool result;  //プラン作成結果
} T_arm_planID;

#define D_NtcQueue_Max 20  //通知受信有無キューサイズ
typedef struct
{
  uint PutP;                        /* put pointer*/
  uint GetP;                        /* get pointer*/
  uint RecogJobNo[D_NtcQueue_Max];  //認識完了Job番号
} T_Ctrl_NtcQue;

#define D_EvtFlaDataSize 8  //
typedef struct
{
  uint result;  //キー入力ステータス
  uint data;    //キー入力データまたは文字列Index
} T_keydata;
struct S_EvtFlagQue
{
  uint read;   // read offset count
  uint write;  // write offset count
  T_keydata data[D_EvtFlaDataSize];
};

// LOG用データ
//計画済みアームPlan実行時間と軌道取得データ
typedef struct
{
  int planID;
} T_planTrjData;

namespace RecognitionTypes
{
enum RecognitionType
{
  Item,
  Octomap,
  ItemAndOctomap
};
}
typedef RecognitionTypes::RecognitionType RecognitionType;

// 把持・解放点とアイテム情報の紐付け
typedef struct
{
  uint index;   // Item index
  GpRp_t gprp;  // Grasp and release point struct
} GpRpInfo;

#define D_WeightScaleTotal 3 // 重量計の数

//----------------------------------
// プロトタイプ宣言
//----------------------------------
//メインステージ実行
void Exec_SetUp_Stage(void);
void Exec_Initial(void);
void Exec_ErrRecover(void);
void Exec_ProcEnd(void);
void Exec_Halt(void);
void Exec_ResetHand(void);
void Exec_ResetArm(void);

bool Check_setup_Robot_and_start(void);
void test_MoveOnlyArm(int TestType);

void Change_MainStage(uint main_code);
bool GetFile_ItemList(uint *itemListCount);

int console_keyIn_int(int *intKeyin);
int console_keyIn_str(char *strKeyin);
int console_out(const char *m_adr, ...);
int waiEvtDataQue(S_EvtFlagQue *evtFlgQue, int *readData);
bool wait_operating(uint *op_mode, uint mode);
void cbConsoleInput(const t2_msgs::UIConsole::ConstPtr &key_msg);

uint Exec_ParamInit(void);
uint Exec_Initialize(void);
uint Exec_OperationStart(void);
uint Exec_OperationEnd(void);
uint Exec_Calib(void);

void Exec_PickOrder(void);
void Exec_StowOrder(void);
void Exec_BinAllRecognize(bool flag);
uint Exec_BinRecognize(uint box_index, bool flag);
uint Exec_GraspPack(uint place_id, uint item_index, const std::vector<uint>& to_place_id);

uint Exec_PickPlan(uint box_index, uint item_index, const int32_t& plan_id,
    bool permit_protrusion, uint& to_box_index);
uint Exec_PickExecute(uint box_index, uint item_index, const int32_t& plan_id);
uint Exec_PlaceMove(uint box_index, uint item_index, uint from_box_index, const int32_t& pick_plan_id,
    int32_t& action_result);
uint Exec_StowPlan(uint box_index);
uint Exec_StopCtrl(void);

int getPlanID(uint box_index, uint item_index);
void runPickTaskThread(void);
void runStowTaskThread(void);

void console_out_orderinfo(void);
void console_out_boxsizeinfo(void);
void console_out_boxinfo(void);
void console_out_pickorderinfo(void);

uint Exec_TestArmMove(T_PlanMainCmdMsg cmd_msg);
void sub_menu_MoveOnlyArm(T_PlanMainCmdMsg cmd_msg);
void sub_menu_ArmMoveFilePose(T_PlanMainCmdMsg cmd_msg, int arm_exec_mode);
void GetCurrDateTime(char *time_str);
void MakeTrjOutputFileName(char *src_fileName, char *dst_fileName, char *time_str, char *ext_name);
bool Savefile_plan_log(FILE *fp_sFile_tim, FILE *fp_sFile_trj, int planID, geometry_msgs::Pose *curr_pose,
                       int loop_count, int plan_count, int step_index, int step_type, int plan_result);

int moveArmCmd_exe(T_PlanMainCmdMsg cmd_msg);

int ArmMoveToStateNamePose(uint to_arm_pose_index, float velocity);

bool GetFile_SeqMoveDataFilePath(void);

bool GetFile_SeqMoveFileList(char *file_path, uint *ListCount);

void createBoxNameIdTable();
void createPlanPosiNameTable();
void creteArmJointAngleTable();

uint executeRecognition(uint place_id, RecognitionType type);
void executeAllRecognition(RecognitionType type);

void waitRecognitionTimeout(uint place_id, RecognitionType type, ros::Time base_time);

t2_msgs::GraspPoint gprpToGraspPointMsg(const GpRp_t& gprp);

//アクション
int snd_ArmPlanAction_msg(t2_msgs::ArmPose &start_pose, t2_msgs::ArmPose &goal_pose, int plan_id,
                          const std::string &plan_group, const std::string &grasp_pattern);
int snd_ArmExecuteAction_msg(int plan_id, float velocity);
int snd_ArmMoveAction_msg(t2_msgs::ArmPose &goal_pose, float velocity, const std::string &plan_group);
int snd_ArmMoveAction_msg(std::vector<geometry_msgs::Pose> &waypoints, float velocity, const std::string &plan_group);
int snd_ArmPickAction_msg(int plan_id, float velocity, const std::string &plan_group,
                          const t2_msgs::GraspPoint &grasp_point);

void cbArmPlaceMoveActionFeedback(const t2_msgs::ArmPlaceMoveFeedbackConstPtr& feedback);

//サービス
int snd_srv_msg_GripperPump(const bool& pump);
int snd_srv_msg_GripperSuction(const bool& suction);
int snd_srv_msg_GetGraspInfo(std::vector<t2_msgs::GraspPoint> *grasp_point);
int snd_srv_msg_GetArmPose(t2_msgs::ArmPose& arm_pose, const std::string& plan_group);
int snd_srv_msg_GetArmGroupStatePose(const std::string& group_state, t2_msgs::ArmPose& arm_pose, const std::string& plan_group);

int snd_srv_msg_PinchingGripperInitialize();
int snd_srv_msg_PinchingGripperFinalize();

int snd_srv_msg_KebaInitialize(bool homing);
int snd_srv_msg_KebaFinalize();

int snd_srv_msg_SetContainerObject(const t2_msgs::SetContainerObjectToPlanningScene::Request::_object_type& object);

int snd_srv_msg_WeightScaleStart();
int snd_srv_msg_WeightScaleStop();

//トピック
int snd_topic_msg_to_recoPC_captreq(t2_robot_msgs::CaptureReq send_captreq_msg);
int snd_topic_msg_to_recoPC_recoreq(t2_robot_msgs::RecognizeReq send_recoreq_msg);
//受信解析
int rcv_topic_msg_from_recoPC_captres(int *response_code, bool *rsp_evt, int wai_time);
int rcv_topic_msg_from_recoPC_recores(int *response_code, bool *rsp_evt, int wai_time);

void msgCallback_rcv_recoPC_captres_cmd(const t2_robot_msgs::CaptureRes::ConstPtr &cmd_msg);
void msgCallback_rcv_recoPC_recores_cmd(const t2_msgs::RecognizeRes::ConstPtr &cmd_msg);

void cbArmPlaceMoveAction(const actionlib::SimpleClientGoalState& state,
                         const t2_msgs::ArmPlaceMoveResultConstPtr& result);

void receiveCaptureResCb(const t2_robot_msgs::CaptureRes::ConstPtr& msg);
void receiveRecognizeResCb(const t2_msgs::RecognizeRes::Ptr& msg);
void receiveOccupancyResCb(const t2_msgs::OccupancyRes::Ptr& msg);

#endif	//End of T2_TASK_PLANNER_H
