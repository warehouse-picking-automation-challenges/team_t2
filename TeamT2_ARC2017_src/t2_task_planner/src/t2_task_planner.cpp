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

#include <t2_task_planner/t2_task_planner.h>

//----------------------------------
// グローバル変数
//----------------------------------

//----------------------------------
//ファイルアクセス用
//----------------------------------
char selectFileListnameAll[512];  // Directry name+FileList name
char inputFileListnameAll[512];   // Directry name+FileList name
char outputFileListnameAll[512];  // Directry name+FileList name
char inputFilePath[256];          // Directry name+FileList name
char outputFilePath[256];         // Directry name+FileList name

// テスト用アーム指定位置移動ファイルパス名読込み
const char *Def_SeqMoveFilePath = "/arm_data/test_seqArmDataFilePath.csv";  // plan_main以下のパス含んで指定
// テスト用アーム指定位置移動ファイル登録リスト
#define D_fileNameMax 128                                          // ファイル名の最大文字数
#define D_fileCountMax 64                                          // ファイル数最大
const char *Def_SeqMoveFileList = "test_seqArmMoveFile_List.csv";  //ファイル名のみ

// アーム移動座標指定位置ファイル
const char *Def_armMove_subPath = "/arm_data/";  // plan_main以下のサブディレクトリパス名

// テスト用アーム指定位置移動ファイル登録リスト
uint InputSeqMoveFileCount;  //入力ファイル数
char seqArmMove_FileName[D_fileCountMax][D_fileNameMax];

T_SeqArmMoveParam t_SeqArmMoveParam;

ros::Publisher ros_console_dsp_pub;   // 配信者ノード(コンソール表示)の宣言
ros::Subscriber ros_console_key_sub;  // 購読者ノード(コンソール入力)の宣言

// RECO-PC間
ros::Publisher Ctrl_recoPC_captreq_msg;   // reco
ros::Subscriber recoPC_Ctrl_captres_msg;  // reco
ros::Publisher Ctrl_recoPC_recoreq_msg;   // reco
ros::Subscriber recoPC_Ctrl_recores_msg;  // reco
ros::Publisher occupancy_req_pub_;
ros::Subscriber occupancy_res_sub_;
// plan_grasp間
ros::Publisher PlanCtrl_grasp_msg;   // grasp
ros::Subscriber grasp_planCtrl_msg;  // grasp

// plan pcl
ros::Publisher PlanCtrl_pcl_msg;       // add
ros::ServiceClient clear_octomap_msg;  // clear

// 把持計画　サービス
//ros::ServiceClient GetGraspInfo_client;
//t2_msgs::GetGraspInfo GetGraspInfo_srv;

// SuctionGripperサービス
ros::ServiceClient GripperPump_client;
ros::ServiceClient GripperSuction_client;

// PinchingGripperサービス
ros::ServiceClient PinchingGripperInitialize_client;
ros::ServiceClient PinchingGripperFinalize_client;

// KEBA 初期化・終了サービス
ros::ServiceClient KebaInitialize_client;
ros::ServiceClient KebaFinalize_client;
ros::Publisher joint_state_pub;

// 重量計 開始・停止サービス
ros::ServiceClient WeightScaleStart_client[D_WeightScaleTotal];
ros::ServiceClient WeightScaleStop_client[D_WeightScaleTotal];

//-------------------------------------------------------
//アーム
//★位置姿勢取得サービス
ros::ServiceClient GetArmPose_client;            //アーム位置姿勢取得サービス
ros::ServiceClient GetArmGroupStatePose_client;  // アームグループステート名の姿勢取得サービス

//★計画済みアーム軌道取得サービス
ros::ServiceClient GetArmPlanTrj_client;             //計画済みアーム軌道取得サービス
t2_msgs::GetPlannedArmTrajectory GetArmPlanTrj_srv;  //サービス利用の宣言

// プランニングシーン更新サービス
//ros::ServiceClient AddCollisionObject_clinet;
//ros::ServiceClient RemoveCollitionObject_client;
//ros::ServiceClient AttachObject_client;
//ros::ServiceClient DetachObject_client;
//ros::ServiceClient SetACM_client;
ros::ServiceClient AddItemsToPlanningScene_client;
ros::ServiceClient SetContainerObjectToPlanningScene_client;

//★アクションクライアント設定
ArmPlanActionClient *arm_plan_action_client_;
ArmExecuteActionClient *arm_execute_action_client_;
ArmMoveActionClient *arm_move_action_client_;

ArmPickPlanActionClient *arm_pick_plan_action_client_;
ArmPickExecuteActionClient *arm_pick_execute_action_client_;

ArmPlaceMoveActionClient *arm_place_move_action_client_;

GripperReleaseActionClient *gripper_release_action_client_;
GripperRetractActionClient *gripper_retract_action_client_;

// アーム速度
double arm_speed_high_;      // アーム速度(高速)
double arm_speed_middle_;    // アーム速度(中速)
double arm_speed_low_;       // アーム速度(低速)
double arm_speed_gap1_;      // アーム速度(SP/RAP4/GAP4 -> GAP1)
double arm_speed_gap2_;      // アーム速度(GAP1 -> GAP2)
double arm_speed_gp_;        // アーム速度(GAP2 -> GP)
double arm_speed_gap3_;      // アーム速度(GP' -> GAP3)
double arm_speed_gap4_;      // アーム速度(GAP3 -> GAP4)
double arm_speed_back_gap2_; // アーム速度(GP' -> GAP2) : 逆再生
double arm_speed_back_gap1_; // アーム速度(GAP2 -> GAP1) : 逆再生
double arm_speed_rap1_;      // アーム速度(GAP4 -> RAP1)
double arm_speed_rap2_;      // アーム速度(RAP1 -> RAP2)
double arm_speed_rp_;        // アーム速度(RAP2 -> RP)
double arm_speed_back_rap2_; // アーム速度(RP -> RAP2) : 逆再生
double arm_speed_back_rap1_; // アーム速度(RAP2 -> RAP1) : 逆再生
double arm_speed_sp_;        // アーム速度(RAP4/GAP4 -> SP)

//-------------------------------------------------------
T_PlanMainCmdMsg t_MainCtrlCmd;
T_PlanMainRspMsg t_MainCtrlRsp;

uint rsp_msg_code;  //受信コマンド
bool evt_rsprcv;    //受信完了フラグ
bool evt_shutdown;  //ノード切断要求フラグ
uint rsp_timer;     //受信レスポンス監視タイマー

int mainReq_Cmd_code;            //メイン要求コマンド
bool evt_recoPC_captres_cmdrcv;  // reco要求受信完了フラグ
bool evt_recoPC_recores_cmdrcv;  // reco要求受信完了フラグ
bool evt_grasp_cmdrcv;           // Grasp受信完了フラグ
bool evt_arm_rsprcv;             // Arm制御動作完了フラグ

uint captreq_seq_no;  //対RECO-PC シーケンス番号
uint captreq_job_no;  //対RECO-PC 認識job番号番号

// 制御ステージコード
T_MainStatus t_MainStatus;

int machine_setup_ready;  //初期設定完了フラグ
int order_task;           //要求動作種別(Pick task=1/Stow task=2)番号
int stow_direction;       //収納先Bin=0/Tote=1(棚／箱)

bool received_halt_command_;  // 処理停止コマンド受信フラグ
bool initialized_hand_;       // ハンドリセットフラグ
bool initialized_arm_;        // アームリセットフラグ

uint mainDebug_mode;                   //動作モード 通常=0/メイン間欠=1/ctrl間欠=2
uint mainDebug_step;                   //間欠モード停止ステップ指定
uint debug_stepCount;                  //間欠モード停止ステップ数カウント
uint debug_stepSave;                   //初期設定ステージスキップ用
uint MaintenanceTest_RecogGrasp_mode;  //カメラ撮影認識テスト(1:テスト実行)

//----------変換テーブル----------
/*
//コマンド名称
typedef struct
{
  char name[32];
} T_command_name;
T_command_name t_main_cmd_name[D_MAIN_CMD_MAX] = {
  "DummyCmd",     // D_CMD_Dummy =0
  "ParamInit",    // D_CMD_ParamInit      : 初期設定要求
  "Initialize",   // D_CMD_Initialize     : 初期化要求
  "Calib",        // D_CMD_Calib        : 補正要求
  "GntrCamera",   // D_CMD_Ready_GntrCamera : Gntr撮影準備要求
  "ArmCamera",    // D_CMD_Ready_ArmCamera  : Arm撮影準備要求
  "RecoBinGntr",  // D_CMD_Reco_Bin_Gntr    : Gntr認識要求
  "RecoBinArm",   // D_CMD_Reco_Bin_Arm   : Arm認識要求
  "PickPlan",     // D_CMD_Pick_Plan      : 把持計画要求
  "StowPlan",     // D_CMD_Stow_Plan      : 収納（リリース）計画
  "MoveToPick",   // D_CMD_MoveToPick     : 把持前移動要求
  "PickItem",     // D_CMD_PickItem     : 把持実行要求
  "MoveToStow",   // D_CMD_MoveToStow     : 収納（リリース）移動要求
  "StowItem",     // D_CMD_StowItem     : 収納（リリース）実行要求
  "StopCtrl",     // D_CMD_StopCtrl     : 実行終了要求
  "MoveOnlyArm",  // D_CMD_MoveOnlyArm,   : アーム単体移動実行要求
};
*/

// box種別とbox名称
std::vector<T_boxNameId> t_boxNameId;

// グループステート名固定値
std::vector<T_arm_Joint> t_arm_Joint_angle;

//プラン作成・実行用グループステート名
//[string plan_name]用の名称[plan_XXXtoYYY]を作成する:XXX=FROM,YYY=TO
std::vector<T_arm_Joint> plan_posi_name;

//ガントリカメラ撮影管理情報テーブル
T_GntrRecogCntrl t_GntrRecogCntrl[3];  // 同時撮影Bin数。
int gntrRecogReqCount;                 // Item認識データ要求数

//ガントリカメラ撮影管理情報テーブル
T_GntrCameraCntrl t_GntrCameraCntrl[3];  //同時撮影Bin数。

T_RecogJobRslt t_RecogJobRsltData[D_RecogJobData_max];  //
uint RecogJobRsltIndex;                                 //書込み用インデックス

//把持実行時に使用する重心座標情報
T_GraspExeInfo t_grasp_exe_info[D_boxID_MAX];  //把持実行時に使用する重心座標情報

t2_msgs::ArmPose plan_goal_pose;  // PlanActionのみ軌道の最終位置姿勢(関節角を含む)応答あり
bool plan_goal_pose_result_val;   // PlanActionの結果:有効/無効

T_Ctrl_NtcQue t_Ctrl_NtcQue;  // plan_ctrlからの通知受信有無キュー

S_EvtFlagQue evtFlg_KeyIn_int;                         // int入力キーイベント用
S_EvtFlagQue evtFlg_KeyIn_str;                         // str入力キーイベント用
S_EvtFlagQue evtFlg_cosole_cmd;                        // コンソールからの指示イベント用
t2_msgs::UIConsole console_evt_msg[D_EvtFlaDataSize];  // String保存用

// configパラメータ
double grasp_point_score_threshold_;
double item_probability_threshold_;
int max_recognition_retry_count_;
int wait_item_recognition_timeout_;
int wait_octomap_recognition_timeout_;
double protrusion_threshold_;
double failed_gp_distance_threshold_;
int max_failed_gp_count_;
int pick_execute_attempts_;

// 認識リトライカウント
int recognition_retry_count_;

// 並列化用変数
std::thread check_pick_task_thread_;
std::vector<std::thread> planning_thread_;
std::vector<uint> planning_place_id_;
std::vector<bool> thread_end_;
std::mutex seq_no_mutex_;
std::mutex job_no_mutex_;
std::mutex item_recog_mutex_;
std::mutex octomap_recog_mutex_;
std::mutex item_recog_jobno_mutex_;
std::mutex octomap_recog_jobno_mutex_;
std::mutex arm_pick_plan_mutex_;
std::mutex plan_recog_status_mutex_;
bool task_active_;

// ダミー箱詰使用フラグ
bool use_dummy_pack_;

extern std::vector<Boxinfo_t> boxinfo;
extern std::vector<Orderinfo_t> orderinfo;
extern BoxSize_t boxsizeinfo;
extern int g_bin_count;
extern int g_box_count;
extern std::vector<PickOrderInfo> pick_order_;
extern std::vector<int> move_excluded_place_id_list_;

// LOG用データ
//計画済みアームPlan実行時間と軌道取得データ
T_planTrjData plan_result_Data;  //

// 認識Job番号と認識タイプの紐付け
std::map<uint, RecognitionTypes::RecognitionType> recog_no_to_type_;

// PlaceMove失敗時のフラグ
int32_t arm_place_move_result_;
OrderStatus place_move_order_status_;

//********************************
// t2_task_plannerノードのメイン関数
//********************************
//
int main(int argc, char **argv)
{
  int iKeyIndata = 0;

  // ノード名の初期化
  ros::init(argc, argv, "t2_task_planner");

  ROS_INFO("t2_task_planner node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // ROSシステムとの通信のためのノードハンドルを宣言
  ros::NodeHandle nh_main_snd_cmd;
  ros::NodeHandle nh_main_rcv_rsp;
  ros::NodeHandle nh_main_rcv_ntc;
  ros::NodeHandle nh_console_dsp;
  ros::NodeHandle nh_console_key;

  // ROSシステムとの通信のためのノードハンドルを宣言
  // plan_main
  ros::NodeHandle nh_ctrl_rcv_cmd;
  ros::NodeHandle nh_ctrl_snd_rsp;
  ros::NodeHandle nh_ctrl_snd_ntc;

  /*　recoPC　*/
  ros::NodeHandle reco_capt_input_sub;
  ros::NodeHandle reco_capt_output_sub;
  ros::NodeHandle reco_reco_input_sub;
  ros::NodeHandle reco_reco_output_sub;

  //サービス
  ros::NodeHandle nh_GetGraspInfo;

  ros::NodeHandle nh_Gripper("t2_gripper");

  ros::NodeHandle nh_PinchingGripper(t2_pinching_gripper::NODE_NAME);

  ros::NodeHandle nh_KebaBridge("keba_bridge");

  //アームサービス
  ros::NodeHandle nh_MotionPlanner(t2_motion_planner::NODE_NAME);

  // PlanningSceneUpdaterサービス
  ros::NodeHandle nh_PlanningSceneUpdater(t2_planning_scene_updater::NODE_NAME);

  // 配信（ノードplan_consoleへの画面表示データ送信）
  ros_console_dsp_pub = nh_console_dsp.advertise<t2_msgs::UIConsole>("Console_Dsp", 100);
  // 購読（ノードplan_consoleからのキーボード入力受信）
  ros_console_key_sub = nh_console_key.subscribe("Console_Key", 100, cbConsoleInput);

  /* 購読　plan to reco mesg 撮影完了 */
  recoPC_Ctrl_captres_msg = reco_capt_input_sub.subscribe("capture_res", 100, receiveCaptureResCb);
  /* 配信　reco to plan mesg 撮影要求 */
  Ctrl_recoPC_captreq_msg = reco_capt_output_sub.advertise<t2_robot_msgs::CaptureReq>("capture_req", 100);
  /* 購読　plan to reco mesg 認識完了 */
  recoPC_Ctrl_recores_msg = reco_reco_input_sub.subscribe("recognize_res", 100, receiveRecognizeResCb);
  /* 配信　reco to plan mesg 認識要求 */
  Ctrl_recoPC_recoreq_msg = reco_reco_output_sub.advertise<t2_robot_msgs::RecognizeReq>("recognize_req", 100);

  occupancy_req_pub_ = nh.advertise<t2_msgs::OccupancyReq>("occupancy_req", 100);
  occupancy_res_sub_ = nh.subscribe("occupancy_res", 100, receiveOccupancyResCb);

  //サービスクライアント作成
  GripperPump_client = nh_Gripper.serviceClient<t2_msgs::GripperPump>("pump");
  GripperSuction_client = nh_Gripper.serviceClient<t2_msgs::GripperSuction>("suction");

  //GetGraspInfo_client = nh_GetGraspInfo.serviceClient<t2_msgs::GetGraspInfo>("get_grasp_info");

  GetArmPose_client = nh_MotionPlanner.serviceClient<t2_msgs::GetArmPose>(t2_motion_planner::GET_ARM_POSE_SERVICE_NAME);
  GetArmGroupStatePose_client = nh_MotionPlanner.serviceClient<t2_msgs::GetArmGroupStatePose>(
      t2_motion_planner::GET_ARM_GROUP_STATE_POSE_SERVICE_NAME);

  GetArmPlanTrj_client = nh_MotionPlanner.serviceClient<t2_msgs::GetPlannedArmTrajectory>(
      t2_motion_planner::GET_PLANNED_ARM_TRAJECTRY_SERVICE_NAME);

  AddItemsToPlanningScene_client = nh_PlanningSceneUpdater.serviceClient<t2_msgs::AddItemsToPlanningScene>(
      t2_planning_scene_updater::ADD_ITEMS_SERVICE_NAME);

  SetContainerObjectToPlanningScene_client = nh_PlanningSceneUpdater.serviceClient<t2_msgs::SetContainerObjectToPlanningScene>(
      t2_planning_scene_updater::SET_CONTAINER_OBJECT_SERVICE_NAME);

  PinchingGripperInitialize_client =
      nh_PinchingGripper.serviceClient<t2_msgs::GripperInitialize>(t2_pinching_gripper::INITIALIZE_SERVICE_NAME);
  PinchingGripperFinalize_client =
      nh_PinchingGripper.serviceClient<t2_msgs::GripperFinalize>(t2_pinching_gripper::FINALIZE_SERVICE_NAME);

  for (int i = 0; i < D_WeightScaleTotal; i++)
  {
    std::string node_name = "/t2_weight_scale_" + std::to_string(i + 1);
    WeightScaleStart_client[i] = nh.serviceClient<t2_msgs::WeightScaleStart>(node_name + "/weight_scale_start");
    WeightScaleStop_client[i] = nh.serviceClient<t2_msgs::WeightScaleStop>(node_name + "/weight_scale_stop");
  }

  KebaInitialize_client = nh_KebaBridge.serviceClient<keba_bridge_msgs::KebaInitialize>("initialize");
  KebaFinalize_client = nh_KebaBridge.serviceClient<keba_bridge_msgs::KebaFinalize>("finalize");
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("/t2_arm_controller/joint_states", 10);

  // アクションクライアント作成
  arm_plan_action_client_ =
      new ArmPlanActionClient(t2_motion_planner::NODE_NAME + "/" + t2_motion_planner::ARM_PLAN_ACTION_NAME, true);
  arm_execute_action_client_ =
      new ArmExecuteActionClient(t2_motion_planner::NODE_NAME + "/" + t2_motion_planner::ARM_EXECUTE_ACTION_NAME, true);
  arm_move_action_client_ =
      new ArmMoveActionClient(t2_motion_planner::NODE_NAME + "/" + t2_motion_planner::ARM_MOVE_ACTION_NAME, true);

  arm_pick_plan_action_client_ = new ArmPickPlanActionClient(
      t2_motion_planner::NODE_NAME + "/" + t2_motion_planner::ARM_PICK_PLAN_ACTION_NAME, true);

  arm_pick_execute_action_client_ = new ArmPickExecuteActionClient(
      t2_motion_planner::NODE_NAME + "/" + t2_motion_planner::ARM_PICK_EXECUTE_ACTION_NAME, true);

  arm_place_move_action_client_ = new ArmPlaceMoveActionClient(
      t2_motion_planner::NODE_NAME + "/" + t2_motion_planner::ARM_PLACE_MOVE_ACTION_NAME, true);

  gripper_release_action_client_ = new GripperReleaseActionClient(
      t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::RELEASE_ACTION_NAME, true);
  gripper_retract_action_client_ = new GripperRetractActionClient(
      t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::RETRACT_ACTION_NAME, true);

  // アーム速度
  private_nh.param<double>("task_planner_configs/arm_speed_high", arm_speed_high_, 1.0);
  private_nh.param<double>("task_planner_configs/arm_speed_middle", arm_speed_middle_, 0.7);
  private_nh.param<double>("task_planner_configs/arm_speed_low", arm_speed_low_, 0.25);
  private_nh.param<double>("task_planner_configs/arm_speed_gap1", arm_speed_gap1_, arm_speed_high_);
  private_nh.param<double>("task_planner_configs/arm_speed_gap2", arm_speed_gap2_, arm_speed_middle_);
  private_nh.param<double>("task_planner_configs/arm_speed_gp", arm_speed_gp_, arm_speed_low_);
  private_nh.param<double>("task_planner_configs/arm_speed_gap3", arm_speed_gap3_, arm_speed_low_);
  private_nh.param<double>("task_planner_configs/arm_speed_gap4", arm_speed_gap4_, arm_speed_middle_);
  private_nh.param<double>("task_planner_configs/arm_speed_back_gap1", arm_speed_back_gap1_, arm_speed_high_);
  private_nh.param<double>("task_planner_configs/arm_speed_rap1", arm_speed_rap1_, arm_speed_high_);
  private_nh.param<double>("task_planner_configs/arm_speed_rap2", arm_speed_rap2_, arm_speed_middle_);
  private_nh.param<double>("task_planner_configs/arm_speed_rp", arm_speed_rp_, arm_speed_low_);
  private_nh.param<double>("task_planner_configs/arm_speed_back_rap2", arm_speed_back_rap2_, arm_speed_low_);
  private_nh.param<double>("task_planner_configs/arm_speed_back_rap1", arm_speed_back_rap1_, arm_speed_middle_);
  private_nh.param<double>("task_planner_configs/arm_speed_sp", arm_speed_sp_, arm_speed_high_);

  // ループの周期設定(10Hz:0.1秒)
  ros::Rate loop_rate(10);

  captreq_seq_no = 0;     //対RECO-PC シーケンス番号初期化
  captreq_job_no = 0;     //対RECO-PC 認識job番号初期化
  RecogJobRsltIndex = 0;  //把持用recogJob、CadID保存インデックス初期化

  //コンソールノードとの接続
  ROS_INFO("<waiting Plan_console node start>");
  while (ros_console_dsp_pub.getNumSubscribers() < 1)
  {
    ros::Duration(1).sleep();  // 時間待ち
  }

  machine_setup_ready = 0;  //初期設定完了フラグ
  initialized_hand_ = true;
  initialized_arm_ = true;

  evt_shutdown = true;  //関数内でのノード切断要求受信フラグ

  mainDebug_mode = 0;                   //通常運転
  mainDebug_step = 0;                   //間欠モード停止ステップ指定
  MaintenanceTest_RecogGrasp_mode = 0;  //-->カメラ撮影認識テストOFF

  memset(&t_Ctrl_NtcQue, 0, sizeof(t_Ctrl_NtcQue));  // plan_ctrlからの通知受信有無キュー初期化

  ros::AsyncSpinner spinner(32);
  spinner.start();

  //----------------------------------
  // メインループ
  //----------------------------------
  t_MainStatus.mainStageCodeOld = D_STAGE_WaiSetUp;  //メインステージコード初期化
  t_MainStatus.mainStageCode = D_STAGE_WaiSetUp;

  while (ros::ok())
  {
    switch (t_MainStatus.mainStageCode)
    {
      case D_STAGE_WaiSetUp:    // 初期設定ステージ
        Exec_SetUp_Stage();
        break;

      case D_STAGE_Initial:     // 初期化ステージ
        Exec_Initial();
        break;

      case D_STAGE_PickOrder:
        Exec_PickOrder();
        break;

      case D_STAGE_StowOrder:
        Exec_StowOrder();
        break;

      case D_STAGE_ErrRecover:  // エラーリカバリ処理ステージ
        Exec_ErrRecover();
        break;

      case D_STAGE_ProcessEnd:  // 処理終了実行ステージ
        Exec_ProcEnd();
        break;

      case D_STAGE_HALT:        // 処理停止ステージ
        Exec_Halt();
        break;

      case D_STAGE_ResetHand:
        Exec_ResetHand();
        break;

      case D_STAGE_ResetArm:
        Exec_ResetArm();
        break;

      default:
        break;
    }
    // ループ周期時間待ち
    loop_rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}

//********************************
//メインステージ実行
//********************************

//----------------------------------
// 初期設定ステージ
//----------------------------------
void Exec_SetUp_Stage(void)
{
  int iKeyIndata = 0;
  int ret;
  int ii;
  uint item_keyID;  //要求アイテムキー番号入力

  ros::Rate wait_rate(1);
  int machine_setup_req = 0;  //初期設定要求
  bool result;

  // 動作メニュー
  if (MaintenanceTest_RecogGrasp_mode != 1)  // テストモードの継続チェック。テスト中であればメインメニューはスキップ
  {
    for (;;)
    {
      console_out("\n*-------- Main Menu --------*\n");
      console_out(" \n");
      console_out("   0. 初期化 start (%s)\n", machine_setup_ready == 1 ? "済" : "未");
      console_out("   1. Pick運転開始(JSONファイル指示による実行) start\n");
      console_out("   2. Pick結果表示 result display\n");
      console_out("   3. Stow運転開始(JSONファイル指示による実行) start\n");
      console_out("   4. Stow結果表示 result display\n");
      console_out("   5.\n");
      console_out("   6.\n");
      console_out("   7. ハンドリセット (%s)\n", initialized_hand_ ? "済" : "未");
      console_out("   8. アームリセット (%s)\n", initialized_arm_ ? "済" : "未");
      console_out("   9. 終了\n");
      console_out(" \n");
      console_out("  11. アーム単体移動実行(ステート名指定)\n");
      console_out(" \n");
      console_out("--->Select Menu Number =? ");
      ret = console_keyIn_int(&iKeyIndata);  //番号入力
      if (ret != 1)
      {
        console_out("scanf error=%d\n", ret);
        continue;  //再入力
      }
      MaintenanceTest_RecogGrasp_mode = 0;  //-->カメラ撮影認識テストOFF
      mainDebug_mode = 0;                   //デフォルト通常運転
      if (iKeyIndata == 0)
      {
        Change_MainStage(D_STAGE_Initial);
        return;
      }
      else if (iKeyIndata == 1)
      {
        console_out("\n★ JSONファイル読み込み\n");
        system("roslaunch t2_database load_order.launch pick_task:=true");

        order_task = 1;
        break;  //-->実行開始
      }
      else if (iKeyIndata == 2)  //結果表示
      {
        console_out_orderinfo();
        console_out_boxinfo();
        console_out("pause(any keyin) ?");
        console_keyIn_int(&iKeyIndata);  //番号入力(Dummy)
        continue;                        //再入力
      }
      if (iKeyIndata == 3)
      {
        console_out("\n★ JSONファイル読み込み\n");
        system("roslaunch t2_database load_order.launch stow_task:=true");

        order_task = 2;
        break;  //-->実行開始
      }
      else if (iKeyIndata == 4) //結果表示
      {
        console_out_boxinfo();
        console_out("pause(any keyin) ?");
        console_keyIn_int(&iKeyIndata);  //番号入力(Dummy)
        continue;                        //再入力
      }
      else if (iKeyIndata == 7) // 運転停止
      {
        Change_MainStage(D_STAGE_ResetHand);
        return;
      }
      else if (iKeyIndata == 8) // 運転終了
      {
        Change_MainStage(D_STAGE_ResetArm);
        return;
      }
      else if (iKeyIndata == 9) // 終了
      {
        Change_MainStage(D_STAGE_ProcessEnd);  //-->処理終了実行ステージ処理へ
        return;
      }
      else if (iKeyIndata == 11)
      {
        //アーム駆動有り
        if (false == Check_setup_Robot_and_start())  //アーム初期設定済みチェック
        {
          continue;
        }
        test_MoveOnlyArm(iKeyIndata);  //-->テスト実行開始
        continue;
      }
      /*
      else if ((iKeyIndata == 8) || (iKeyIndata == 9) || (iKeyIndata == 10))  //アーム単体移動実行(ステート名指定)
      {
        //        mainDebug_mode =2;              //サブCtrl間欠運転で行う
        //        debug_stepCount =0;              //実行ステップ数カウントクリア

        if (iKeyIndata != 10)  // plan作成のみはチェックしない
        {
          //アーム駆動有り
          if (false == Check_setup_Robot_and_start())  //アーム初期設定済みチェック
          {
            continue;
          }
        }
        test_MoveOnlyArm(iKeyIndata);  //-->テスト実行開始
        continue;
      }
      */

      wait_rate.sleep();  // 時間待ち
    }
  }

  // 運転開始初期動作
  if (RSP_E_OK != Exec_OperationStart())
  {
    Exec_OperationEnd();
    Change_MainStage(D_STAGE_ErrRecover);
    return;
  }
}

//----------------------------------
// 初期化ステージ
//----------------------------------
void Exec_Initial(void)
{
  if (true != wait_operating(&mainDebug_mode, 1))  //動作モードチェック
  {
    //中断終了
    Change_MainStage(D_STAGE_HALT);  //-->処理停止ステージ処理へ
    return;
  }
  printf("\n");
  ROS_INFO("<<< [Exec_Initial] stage start >>>");

  // 補正要求
  console_out("\n<<< 動作開始前の補正 (To Be Determined) >>>\n");
  if (RSP_E_OK != Exec_Calib())
  {
    Change_MainStage(D_STAGE_ErrRecover);
    return;
  }

  // 初期化要求
  console_out("\n<<< 機構部 初期化 >>>\n");
  if (RSP_E_OK != Exec_Initialize())
  {
    Change_MainStage(D_STAGE_ErrRecover);
    return;
  }

  Change_MainStage(D_STAGE_WaiSetUp);
  return;
}

//----------------------------------
// エラーリカバリ処理ステージ
//----------------------------------
void Exec_ErrRecover(void)
{
  printf("\n");
  ROS_INFO("<<< [Exec_ErrRecover] stage start >>>");

  console_out("\n<<< エラーリカバリ処理 >>>\n");
  if (true != wait_operating(&mainDebug_mode, 1))  //動作モードチェック
  {
    //中断終了
    Change_MainStage(D_STAGE_HALT);  //-->処理停止ステージ処理へ
    return;
  }
  //****本来は、エラー処理後、PickItem候補決定ステージ処理へ（今は何もしていないので同じアイテムを取ろうとしてループしてしまう）
  Change_MainStage(D_STAGE_WaiSetUp);  //-->初期設定ステージ処理へ
  //  Change_MainStage(D_STAGE_WaitOrder);    //-->動作要求待ちステージ処理へ
  //  Change_MainStage(D_STAGE_ChoiceItem);    //-->PickItem候補決定ステージ処理へ

  return;
}

//----------------------------------
// 処理終了実行ステージ
//----------------------------------
void Exec_ProcEnd(void)
{
  printf("\n");
  ROS_INFO("<<< [Exec_ProcEnd] stage start >>>");
  if (true != wait_operating(&mainDebug_mode, 1))  //動作モードチェック
  {
    //中断終了
    Change_MainStage(D_STAGE_HALT);  //-->処理停止ステージ処理へ
    return;
  }
  machine_setup_ready = 0;  // 初期化フラグOFF

  console_out("\n<<< 処理終了 >>>\n");

  // 実行終了要求
  if (RSP_E_OK != Exec_StopCtrl())
  {
    Change_MainStage(D_STAGE_ErrRecover);
    return;
  }

  Change_MainStage(D_STAGE_WaiSetUp);  //-->初期設定ステージ処理へ
  return;
}

//----------------------------------
// 処理停止ステージ
//----------------------------------
void Exec_Halt(void)
{
  printf("\n");
  ROS_INFO("<<< [Exec_Halt] stage start >>>");

  console_out("\n<<< 処理停止 >>>\n");

  // 並列化用スレッド停止処理
  task_active_ = false;
  if (check_pick_task_thread_.joinable())
  {
    check_pick_task_thread_.join();
  }

  initialized_hand_ = false;  // ハンドリセットフラグOFF
  initialized_arm_ = false;   // アームリセットフラグOFF

  Change_MainStage(D_STAGE_WaiSetUp);
}

//----------------------------------
// ハンド初期化ステージ
//----------------------------------
void Exec_ResetHand(void)
{
  printf("\n");
  ROS_INFO("<<< [Exec_ResetHand] stage start >>>");

  console_out("\n<<< ハンドリセット >>>\n");

  int ret = RSP_E_OK;

  //ハンド吸着設定サービス：吸着バルブをＯＦＦする
  console_out("\n★ 吸着バルブＯＦＦ\n");
  if (RSP_E_OK != snd_srv_msg_GripperSuction(false))
  {
    console_out("  ⇛ Failed\n");
    ret = RSP_E_NG;  //異常終了
  }

  // ハンド吸着ポンプ設定サービス：真空ポンプをＯＦＦする
  console_out("★ 真空ポンプＯＦＦ\n");
  if (RSP_E_OK != snd_srv_msg_GripperPump(false))
  {
    console_out("  ⇛ Failed\n");
    ret = RSP_E_NG;  //異常終了
  }

  // PinchingGripper格納
  console_out("★ PinchingGripper格納\n");
  t2_msgs::GripperRetractGoal gripper_retract_goal;
  gripper_retract_action_client_->waitForServer();
  gripper_retract_action_client_->sendGoalAndWait(gripper_retract_goal);
  t2_msgs::GripperRetractResultConstPtr gripper_retract_result = gripper_retract_action_client_->getResult();
  if (gripper_retract_result->result != t2_msgs::GripperRetractResult::SUCCESS)
  {
    console_out("  ⇛ Failed\n");
    ret = RSP_E_NG;  //異常終了
  }

  if (ret != RSP_E_OK)
  {
    Change_MainStage(D_STAGE_ErrRecover);
    return;
  }

  initialized_hand_ = true;
  Change_MainStage(D_STAGE_WaiSetUp);
}

//----------------------------------
// アーム初期化ステージ
//----------------------------------
void Exec_ResetArm(void)
{
  printf("\n");
  ROS_INFO("<<< [Exec_ResetArm] stage start >>>");

  console_out("\n<<< アームリセット >>>\n");

  uint ret = RSP_E_OK;

  // SPへ移動
  console_out("\n★ アームを終了位置<%s>に移動\n", t_arm_Joint_angle[posi_arm_Start].posi_name);
  if (RSP_E_OK != ArmMoveToStateNamePose(posi_arm_Start, arm_speed_sp_))  //アーム移動
  {
    ret = RSP_E_NG;
  }

  // KEBA終了
  console_out("★ ロボットコントローラ停止\n");
  if (RSP_E_OK != snd_srv_msg_KebaFinalize())
  {
    ret = RSP_E_NG;
  }

  if (ret != RSP_E_OK)
  {
    Change_MainStage(D_STAGE_ErrRecover);
    return;
  }

  initialized_arm_ = true;
  Change_MainStage(D_STAGE_WaiSetUp);
}

//********************************
// テスト処理
//********************************
//----------------------------------
// アーム単体移動実行前のロボット初期設定済のチェック
//----------------------------------
bool Check_setup_Robot_and_start(void)
{
  int iKeyIndata = 0;
  char strKeyin[80];
  int ret;

  //初期設定チェック
  if (machine_setup_ready == 0)  //初期設定未完了(グローバル変数)
  {
    console_out("\n★ 初期設定未完了：ロボットコントローラ起動します( y / n ) ?");
    ret = console_keyIn_str(strKeyin);  //文字入力
    if (ret != 1)
    {
      console_out("scanf error\n");
      return false;
    }
    if (strKeyin[0] != 'y')
    {
      return false;  //実行中止
    }
    printf("\n");
    ROS_INFO("<<< [Exec_SetUp_Stage] stage start >>>");

    if (RSP_E_OK != Exec_Initialize())
    {
      return false;  //実行中止
    }

    machine_setup_ready = 1;  //初期設定完了セット
  }
  return true;
}

//----------------------------------
// アーム単体移動実行テストの起動
//----------------------------------
void test_MoveOnlyArm(int TestType)
{
  //アーム動作テスト実行指示
  t_MainCtrlCmd.cmd_code = D_CMD_MoveOnlyArm;
  t_MainCtrlCmd.sub_cmd_code = D_MoveOnly_ARM_TEST_MENU;  //テストメニュー動作指定
  t_MainCtrlCmd.sub_cmd_data = TestType;                  //テスト番号（メインメニューの番号と同じ）

  if (Exec_TestArmMove(t_MainCtrlCmd) != RSP_E_OK)
  {
    return;
  }
  console_out("\nテスト終了\n");
  return;
}

////////////////
//********************************
//  ステージ呼び出し関数
//********************************
//
//----------------------------------
// メインステージ変更
//----------------------------------
void Change_MainStage(uint main_code)
{
  t_MainStatus.mainStageCodeOld = t_MainStatus.mainStageCode;
  t_MainStatus.mainStageCode = main_code;
  if (t_MainStatus.mainStageCode != t_MainStatus.mainStageCodeOld)
  {
    ROS_INFO("Plan_Main mainstage change [%d]->[%d]", t_MainStatus.mainStageCodeOld, t_MainStatus.mainStageCode);
  }
}

//----------------------------------
// テスト用アーム指定位置移動ファイル登録リスト読込み
//----------------------------------
//
bool GetFile_SeqMoveFileList(uint *ListCount)
{
  FILE *listfp;
  int jj, kk;
  char work_sbuf[D_fileNameMax + 32];

  char *ary[32];
  int file_read_result;

  std::string path = ros::package::getPath("t2_task_planner");

  printf("*---- seq_MoveArm file list ----*\n");
  sprintf(selectFileListnameAll, "%s%s", path.c_str(), Def_SeqMoveFileList);  // itemList name
  printf(" < List file name=[ %s ] >\n\n", Def_SeqMoveFileList);
  console_out(" < List file name=[ %s ] >\n\n", Def_SeqMoveFileList);
  //移動登録リストファイル読込み
  if ((listfp = fopen(selectFileListnameAll, "r")) == NULL)
  {
    console_out("\n@@@ file open error[ %s ]\n", selectFileListnameAll);
    printf("\n@@@ file open error[ %s ]\n", selectFileListnameAll);
    return false;
  }

  file_read_result = 0;
  kk = 1;                                                  // line count
  jj = 0;                                                  // List count
  while (fgets(work_sbuf, D_fileNameMax, listfp) != NULL)  // １行読み込み（ファイルEOFまで）
  {
    //ファイル名
    ary[0] = strtok(work_sbuf, ",\r\n");  // 文字列(char配列)をカンマ,CR,LFで分割する:先頭列[0]
    if (ary[0] == NULL || !strlen(ary[0]))
    {
      console_out("@@@ file name null error\n");
      ROS_ERROR("@@@ file name null error");
      file_read_result = 2;
      break;
    }
    if (static_cast<int>(strlen(ary[0])) >= D_fileNameMax)
    {
      console_out("@@@ file name length over error\n");
      ROS_ERROR("@@@ file name length over error");
      file_read_result = 2;
      break;
    }
    strcpy(&seqArmMove_FileName[jj][0], ary[0]);  // file Name
    if (seqArmMove_FileName[jj][0] == '#')
    {
      continue;  //コメントなので次行読込み
    }
    printf("[%d]< %s >\n", jj + 1, &seqArmMove_FileName[jj][0]);
    *ListCount = ++jj;         //ファイル登録数
    if (jj >= D_fileCountMax)  //入力最大数
    {
      console_out("@@@ list buffer over count=%d\n", *ListCount);
      ROS_ERROR("@@@ list buffer over count=%d", *ListCount);
      file_read_result = 2;
      break;
    }
    kk++;
    if (kk >= 1000)  //無限ループ防止
    {
      file_read_result = 1;
      break;
    }
  }
  fclose(listfp);

  if (file_read_result != 0)
  {
    if (file_read_result == 1)
    {
      console_out("@@@ file data read error line=%d No=%d\n", kk, jj);
      ROS_ERROR("@@@ file data read error line=%d No=%d", kk, jj);
    }
    return false;
  }
  printf("\n file count=%d\n\n", *ListCount);

  return true;
}

//----------------------------------
// コンソールノード画面出力／キー入力
//----------------------------------
// （int）キー入力
int console_keyIn_int(int *intKeyin)
{
  char strKeyin[80];
  int strIndex;
  int result;
  int i;

  t2_msgs::UIConsole snd_msg;

  snd_msg.node_code = t2_msgs::UIConsole::CONSOLE_ATH_PLAN_MAIN;
  snd_msg.type_code = t2_msgs::UIConsole::CONSOLE_KEYIN_STR;  //文字列要求
  snd_msg.type_data = 0;
  ros_console_dsp_pub.publish(snd_msg);  // キー入力要求送信
  //ros::spinOnce();                       // dummy

  printf("<wait keyin intData>\n");
  result = waiEvtDataQue(&evtFlg_KeyIn_str, &strIndex);  //キー入力待ち
  if (result == 1)                                       //入力有り(int)
  {
    result = sprintf(strKeyin, "%s", console_evt_msg[strIndex].msg_string.c_str());
    result--;  // CRキー文字数分を差し引く
    if (result >= 1)
    {
      for (i = 0; i < result; i++)
      {
        if ((strKeyin[i] < '0') || (strKeyin[i] > '9'))
        {
          result = -1;
          break;
        }
      }
      if (result >= 1)
      {
        *intKeyin = atoi(&strKeyin[0]);
        printf("=keyin[%d]\n", *intKeyin);
        result = 1;
      }
    }
  }

  return result;
}

//----------------------------------
// （string）キー入力
int console_keyIn_str(char *strKeyin)
{
  int strIndex;
  int result;

  t2_msgs::UIConsole snd_msg;

  snd_msg.node_code = t2_msgs::UIConsole::CONSOLE_ATH_PLAN_MAIN;
  snd_msg.type_code = t2_msgs::UIConsole::CONSOLE_KEYIN_STR;  //文字列要求
  snd_msg.type_data = 0;
  ros_console_dsp_pub.publish(snd_msg);  // キー入力要求送信
  //ros::spinOnce();                       // dummy

  printf("<wait keyin stringData>\n");
  result = waiEvtDataQue(&evtFlg_KeyIn_str, &strIndex);  //キー入力待ち
  if (result == 1)                                       //入力有り(string)
  {
    printf("=%s", console_evt_msg[strIndex].msg_string.c_str());
    sprintf(strKeyin, "%s", console_evt_msg[strIndex].msg_string.c_str());
  }

  return result;
}

//----------------------------------
// 表示出力
int console_out(const char *m_adr, ...)
{
  char dispBuf[512];
  t2_msgs::UIConsole snd_msg;

  va_list args;
  va_start(args, m_adr);
  vsprintf(dispBuf, m_adr, args);
  va_end(args);
  snd_msg.msg_string = dispBuf;
  snd_msg.node_code = t2_msgs::UIConsole::CONSOLE_ATH_PLAN_MAIN;
  snd_msg.type_code = t2_msgs::UIConsole::CONSOLE_DSPONLY;
  snd_msg.type_data = 0;
  ros_console_dsp_pub.publish(snd_msg);  // 表示出力送信
  //ros::spinOnce();                       // dummy

  return 0;
}

//----------------------------------
// オペレーション動作待ち（間欠モード）
//----------------------------------
bool wait_operating(uint *op_mode, uint mode)
{
  char strKeyin[80];
  int ret;

  debug_stepCount++;     //デバッグステップ数カウント
  if (*op_mode != mode)  //メイン間欠=1 、CTRL間欠=2
  {
    return true;
  }
  else
  {
    if (mainDebug_step != 0)  //停止ステップ指定有り
    {
      if (mainDebug_step > debug_stepCount)
      {
        console_out("\n★ main_step[%d] continue\n", debug_stepCount);
        return true;  //続行
      }
      // else(waiting)
    }
    // 続行キー入力待ち
    for (;;)
    {
      console_out("\n★ main_step[%d] continue( step= s / non_stop= n / end= e ) ?", debug_stepCount);
      ret = console_keyIn_str(strKeyin);  //文字入力
      if (ret != 1)
      {
        console_out("scanf error\n");
        continue;
      }
      if (strKeyin[0] == 's')
      {
        return true;  //続行
      }
      else if (strKeyin[0] == 'n')
      {
        *op_mode = 0;        //モードを通常に変更
        mainDebug_step = 0;  //停止ステップ番号クリア
        return true;         // non_stop続行
      }
      else if (strKeyin[0] == 'e')
      {
        return false;  //中断終了
      }
      else
      {
        continue;
      }
    }
  }
  return false;  //中断終了
}

//----------------------------------

// main処理のコンソールノードからのキー入力待ち処理
int waiEvtDataQue(S_EvtFlagQue *evtFlgQue, int *readData)
{
  // 受信タイマー用時間設定(10Hz:0.1)
  ros::Rate rspTime_rate(10);
  uint readOffset;
  int result;

  for (;;)
  {
    while (evtFlgQue->write != evtFlgQue->read)
    {
      readOffset = evtFlgQue->read + 1;  // read pointer
      if (readOffset >= D_EvtFlaDataSize)
      {
        readOffset = 0;
      }
      *readData = evtFlgQue->data[readOffset].data;  //データ取得
      result = evtFlgQue->data[readOffset].result;   //データ取得
      evtFlgQue->read = readOffset;                  // read pointer更新
                                                     //      printf("waiRet=%d",result);
      return result;
    }
    //ros::spinOnce();       // コールバック関数を受け付ける
    rspTime_rate.sleep();  // 時間待ち
    if (ros::ok() == false)
    {
      return -1;
    }
  }
  return 0;
}

// callbackからmain処理へのintキー入力イベント
void setEvtData(S_EvtFlagQue *evtFlgQue, int setdata, int key_result)
{
  uint writeOffset;

  writeOffset = evtFlgQue->write + 1;  // write pointer
  if (writeOffset >= D_EvtFlaDataSize)
  {
    writeOffset = 0;
  }
  evtFlgQue->data[writeOffset].result = key_result;
  evtFlgQue->data[writeOffset].data = setdata;
  //  printf("keyIntRet=%d",key_result);
  evtFlgQue->write = writeOffset;  // write pointer更新
  return;
}

//----------------------------------
// callbackからmain処理へのstringキー入力イベント
void setEvtString(S_EvtFlagQue *evtFlgQue, t2_msgs::UIConsole setmsg, int key_result)
{
  uint writeOffset;

  writeOffset = evtFlgQue->write + 1;  // write pointer
  if (writeOffset >= D_EvtFlaDataSize)
  {
    writeOffset = 0;
  }
  evtFlgQue->data[writeOffset].result = key_result;
  evtFlgQue->data[writeOffset].data = writeOffset;
  console_evt_msg[writeOffset] = setmsg;
  //  printf("keyStrRet=%d",key_result);
  evtFlgQue->write = writeOffset;  // write pointer更新
  return;
}

// コンソールノードからの"Console_Key"トピック(キー入力)
void cbConsoleInput(const t2_msgs::UIConsole::ConstPtr &key_msg)
{
  if (key_msg->node_code == t2_msgs::UIConsole::CONSOLE_ATH_PLAN_MAIN)  //自ノード宛て
  {
    setEvtString(&evtFlg_KeyIn_str, *key_msg, key_msg->key_result);  //文字列保存
  }
  else if (key_msg->node_code == t2_msgs::UIConsole::CONSOLE_ATH_PLAN_CNSL)  //コンソールノード指示
  {
    //コールバック内で実行できるものはここで行う
    if (key_msg->type_code == t2_msgs::UIConsole::CONSOLE_HALT)
    {
      received_halt_command_ = true;
    }
    else if (key_msg->type_code != t2_msgs::UIConsole::CONSOLE_DSPONLY)
    {
      setEvtData(&evtFlg_cosole_cmd, key_msg->type_data, key_msg->key_result);
    }
  }
  //  printf("console Topic received form %d type=%d\n",key_msg->node_code, key_msg->type_code);
}

//----------------------------------
// 初期設定要求
//----------------------------------
uint Exec_ParamInit(void)
{
  ros::NodeHandle pnh("~");
  ros::NodeHandle config_nh(pnh, "task_planner_configs");

  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_NG;  //異常終了
  }

  printf("\n");
  ROS_INFO("<<< [Exec_ParamInit] stage start >>>");

  captreq_seq_no = 0;     // 対RECO-PC シーケンス番号初期化
  //captreq_job_no = 0;   // 対RECO-PC 認識job番号初期化
  RecogJobRsltIndex = 0;  // 把持用recogJob、CadID保存インデックス初期化

  received_halt_command_ = false;   // 運転停止受信フラグのOFF

  // Get private parameter from Parameter Server
  config_nh.param("grasp_point_score_threshold", grasp_point_score_threshold_, 0.7);
  config_nh.param("item_probability_threshold", item_probability_threshold_, 0.5);
  config_nh.param("max_recognition_retry_count", max_recognition_retry_count_, 1);
  config_nh.param("wait_item_recognition_timeout", wait_item_recognition_timeout_, 30);
  config_nh.param("wait_octomap_recognition_timeout", wait_octomap_recognition_timeout_, 30);
  config_nh.param("protrusion_threshold", protrusion_threshold_, 0.02);
  config_nh.param("failed_gp_distance_threshold", failed_gp_distance_threshold_, 0.01);
  config_nh.param("max_failed_gp_count", max_failed_gp_count_, 10);
  config_nh.param("pick_execute_attempts", pick_execute_attempts_, 3);
  config_nh.param("use_dummy_pack", use_dummy_pack_, true);

  move_excluded_place_id_list_.clear();
  XmlRpc::XmlRpcValue place_id_list;
  if (config_nh.getParam("move_excluded_place_id_list", place_id_list))
  {
    for (int i = 0; i < place_id_list.size(); ++i)
    {
      move_excluded_place_id_list_.push_back(static_cast<int>(place_id_list[i]));
    }
  }
  for (int place_id : move_excluded_place_id_list_)
  {
    ROS_INFO("move excluded place_id=%u", place_id);
  }

  initializeBoxInfo(1 == order_task ? TaskType::Pick : TaskType::Stow);
  initializeParam();

  // Set item location info to boxinfo
  if (!setItemInfo())
  {
    ROS_ERROR("setItemInfo() failed");
    console_out("\n★ アイテム情報の読み込みに失敗しました。");
    return RSP_E_NG;
  }

  createBoxNameIdTable();
  createPlanPosiNameTable();
  creteArmJointAngleTable();

  if (1 == order_task)
  {
    if (!setBoxInfo())
    {
      ROS_ERROR("setBoxInfo() failed");
      console_out("\n★ ボックス情報の読み込みに失敗しました。");
      return RSP_E_NG;
    }
    console_out_boxsizeinfo();

    if (!setBoxSize())
    {
      ROS_ERROR("setBoxSize() failed");
      console_out("\n★ ダンボール設置情報の読み込みに失敗しました。");
      return RSP_E_NG;
    }

    // Set box size to parameter server
    setBoxInfoToDatabase();

    // boxをPlanningSceneへ読み込み
    snd_srv_msg_SetContainerObject(t2_msgs::SetContainerObjectToPlanningScene::Request::BOX_OBJECT);

    // Set pick order to boxinfo
    if (!setPickInfo())
    {
      ROS_ERROR("setPickInfo() failed");
      console_out("\n★ オーダー情報の読み込みに失敗しました。");
      return RSP_E_NG;
    }

    initializePickInfo();
    console_out_orderinfo();
  }
  else if (2 == order_task)
  {
    initializeStowInfo();

    // toteをPlanningSceneへ読み込み
    snd_srv_msg_SetContainerObject(t2_msgs::SetContainerObjectToPlanningScene::Request::TOTE_OBJECT);
  }
  console_out_boxinfo();

  return RSP_E_OK;
}

//----------------------------------
// 初期化要求
//----------------------------------
uint Exec_Initialize(void)
{
  uint target_arm_pose_index;

  //  mainDebug_mode =  cmd_msg.mainDebug_mode;  // デバッグ運転モード
  //  mainDebug_step =  cmd_msg.mainDebug_step;  // デバッグ間欠モード停止ステップ指定
  debug_stepCount = debug_stepSave;  //初期設定処理からの続きにデバッグステップを設定
  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_NG;  // 停止
  }

  printf("\n");
  ROS_INFO("<<< [Exec_Initialize] stage start >>>");

  // テーブル作成
  createBoxNameIdTable();
  createPlanPosiNameTable();
  creteArmJointAngleTable();

  //アクションサーバ起動待ち
  ROS_INFO("* waiting arm_action server [arm_plan]");  //動作計画(Plan)
  if (arm_plan_action_client_->waitForServer(ros::Duration(60.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting arm_action server [arm_execute]");  //軌道実行(Execute)
  if (arm_execute_action_client_->waitForServer(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting arm_action server [arm_move]");  //移動(Move : Plan & Execute)
  if (arm_move_action_client_->waitForServer(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting arm_pick_plan_action server [arm_pick_plan]");
  if (arm_pick_plan_action_client_->waitForServer(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting arm_pick_execute_action server [arm_pick_execute]");
  if (arm_pick_execute_action_client_->waitForServer(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting arm_place_move_action server [arm_place_move]");
  if (arm_place_move_action_client_->waitForServer(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  //サービス開始待ち
  ROS_INFO("* waiting service [KebaInitialize]");
  if (KebaInitialize_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting service [KebaFinalize]");
  if (KebaFinalize_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting service [AddItemsToPlanningScene]");
  if (AddItemsToPlanningScene_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting service [SetContainerObjectToPlanningScene]");
  if (SetContainerObjectToPlanningScene_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting service [GetArmPose]");  //アーム位置姿勢取得
  if (GetArmPose_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting service [GetArmGroupStatePose]");
  if (GetArmGroupStatePose_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

  ROS_INFO("* waiting service [GripperPump]");
  if (GripperPump_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");
  ROS_INFO("* waiting service [GripperSuction]");
  if (GripperSuction_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");
  ROS_INFO("* waiting service [GripperInitialize]");  //ハンド挟持設定
  if (PinchingGripperInitialize_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");
  ROS_INFO("* waiting service [GripperFinalize]");  //ハンド挟持設定
  if (PinchingGripperFinalize_client.waitForExistence(ros::Duration(30.0)))
    ROS_INFO("-->SUCCESS");
  else
    ROS_WARN("-->FAILED");

#ifdef USE_WEIGHT_SCALE
  for (int i = 0; i < D_WeightScaleTotal; i++)
  {
    ROS_INFO("* waiting service [WeightScaleStart[%d]]", i + 1);  //重量計開始
    if (WeightScaleStart_client[i].waitForExistence(ros::Duration(30.0)))
      ROS_INFO("-->SUCCESS");
    else
      ROS_WARN("-->FAILED");
    ROS_INFO("* waiting service [WeightScaleStop[%d]]", i + 1);  //重量計停止
    if (WeightScaleStop_client[i].waitForExistence(ros::Duration(30.0)))
      ROS_INFO("-->SUCCESS");
    else
      ROS_WARN("-->FAILED");
  }
#endif

  // トピック接続待ち
  ROS_INFO("* waiting topic [CaptureReq]");
  while (Ctrl_recoPC_captreq_msg.getNumSubscribers() < 1)
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("-->SUCCESS");
  ROS_INFO("* waiting topic [CaptureRes]");
  while (recoPC_Ctrl_captres_msg.getNumPublishers() < 1)
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("-->SUCCESS");
  ROS_INFO("* waiting topic [RecognizeReq]");
  while (Ctrl_recoPC_recoreq_msg.getNumSubscribers() < 1)
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("-->SUCCESS");
  ROS_INFO("* waiting topic [RecognizeRes]");
  while (recoPC_Ctrl_recores_msg.getNumPublishers() < 1)
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("-->SUCCESS");
  ROS_INFO("* waiting topic [OccupancyReq]");
  while (occupancy_req_pub_.getNumSubscribers() < 1)
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("-->SUCCESS");
  ROS_INFO("* waiting topic [OccupancyRes]");
  while (occupancy_res_sub_.getNumPublishers() < 1)
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("-->SUCCESS");
  ROS_INFO("* waiting topic [JointStates]");
  while (joint_state_pub.getNumSubscribers() < 1)
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("-->SUCCESS");

  // Keba初期化(ホーミングありInitialize)
  console_out("\n★ ロボットコントローラ初期化\n");
  if (RSP_E_OK != snd_srv_msg_KebaInitialize(true))
  {
    return RSP_E_NG;
  }

  console_out("\n★ 真空ポンプＯＦＦ & 吸着バルブＯＦＦ\n");
  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    return RSP_E_NG;  // 異常終了
  }

  // ハンド吸着設定サービス：吸着バルブをOFFする
  if (RSP_E_OK != snd_srv_msg_GripperSuction(false))
  {
    return RSP_E_NG;  // 異常終了
  }

  // ハンド吸着ポンプ設定サービス：真空ポンプをOFFする
  if (RSP_E_OK != snd_srv_msg_GripperPump(false))
  {
    return RSP_E_NG;  // 異常終了
  }

  console_out("\n★ PinchingGripper初期化\n");
  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    return RSP_E_NG;  // 異常終了
  }
  // PinchingGripper初期化
  if (RSP_E_OK != snd_srv_msg_PinchingGripperInitialize())
  {
    return RSP_E_NG;  // 異常終了
  }

#ifdef USE_WEIGHT_SCALE
  // 重量計開始
  console_out("\n★ 重量計開始\n");
  if (RSP_E_OK != snd_srv_msg_WeightScaleStart())
  {
    return RSP_E_NG;  // 異常終了
  }
#endif

  target_arm_pose_index = posi_arm_Start;  // Start前に移動

  console_out("\n★ アームを開始位置<%s>(SP)に移動(プラン作成＆実行)\n",
              t_arm_Joint_angle[target_arm_pose_index].posi_name);
  if (RSP_E_OK != ArmMoveToStateNamePose(target_arm_pose_index, arm_speed_sp_))  //アーム移動
  {
    return RSP_E_NG;  // 異常終了
  }
  ROS_INFO("Arm_Rsp[ArmMove] recived");

  // KEBA終了
  console_out("\n★ ロボットコントローラ停止\n");
  if (RSP_E_OK != snd_srv_msg_KebaFinalize())
  {
    return RSP_E_NG;  // 異常終了
  }

  // 初期化フラグON
  machine_setup_ready = 1;
  //initialized_hand_ = true;
  //initialized_arm_ = true;

  return RSP_E_OK;  //正常終了
}

//----------------------------------
// 運転開始要求
//----------------------------------
uint Exec_OperationStart(void)
{
  if (!machine_setup_ready)
  {
    console_out("\n★ 初期設定未完了\n");
    return RSP_E_NG;
  }

  if (!initialized_hand_)
  {
    console_out("\n★ ハンドリセット未完了\n");
    return RSP_E_NG;
  }

  if (!initialized_arm_)
  {
    console_out("\n★ アームリセット未完了\n");
    return RSP_E_NG;
  }

  printf("\n");
  ROS_INFO("<<< [Exec_OperationStart] stage start >>>");

  // Keba開始(ホーミングなしInitialize)
  console_out("\n★ ロボットコントローラ開始\n");
  if (RSP_E_OK != snd_srv_msg_KebaInitialize(false))
  {
    return RSP_E_NG;
  }

  // ポンプON
  console_out("\n★ 真空ポンプＯＮ\n");
  if (RSP_E_OK != snd_srv_msg_GripperPump(true))
  {
    return RSP_E_NG;
  }

  // パラメータの初期化要求
  if (RSP_E_OK != Exec_ParamInit())
  {
    return RSP_E_NG;
  }

  if (check_pick_task_thread_.joinable())
  {
    check_pick_task_thread_.join();
  }

  // item_location_file.jsonのコピー
  std::string pkg_path = ros::package::getPath("t2_database");
  std::string src = pkg_path + "/order/input/item_location_file.json";
  std::string dst = pkg_path + "/order/output/item_location_file.json";
  pkg_path.append("/order/output");
  struct stat stat_dir;
  if (-1 == stat(pkg_path.c_str(), &stat_dir))
  {
    mkdir(pkg_path.c_str(), 0775);
  }
  system(std::string("cp " + src + " " + dst).c_str());

  // 要求タスク種別により計画を分岐
  if (order_task == 1)
  {
    task_active_ = true;
    check_pick_task_thread_ = std::thread(runPickTaskThread);
    Change_MainStage(D_STAGE_PickOrder);
  }
  else if (order_task == 2)
  {
    task_active_ = true;
    check_pick_task_thread_ = std::thread(runStowTaskThread);
    Change_MainStage(D_STAGE_StowOrder);
  }

  return RSP_E_OK;
}

//----------------------------------
// 運転終了要求
//----------------------------------
uint Exec_OperationEnd(void)
{
  printf("\n");
  ROS_INFO("<<< [Exec_OperationEnd] stage start >>>");

  // スレッド終了処理
  task_active_ = false;
  if (check_pick_task_thread_.joinable())
  {
    check_pick_task_thread_.join();
  }

  uint ret = RSP_E_OK;

  // SPへ移動
  console_out("\n★ アームを終了位置<%s>に移動\n", t_arm_Joint_angle[posi_arm_Start].posi_name);
  if (RSP_E_OK != ArmMoveToStateNamePose(posi_arm_Start, arm_speed_sp_))  //アーム移動
  {
    ret = RSP_E_NG;
  }

  // ポンプOFF
  console_out("\n★ 真空ポンプＯＦＦ\n");
  if (RSP_E_OK != snd_srv_msg_GripperPump(false))
  {
    ret = RSP_E_NG;
  }

  // KEBA終了
  console_out("★ ロボットコントローラ停止\n");
  if (RSP_E_OK != snd_srv_msg_KebaFinalize())
  {
    ret = RSP_E_NG;
  }

  Change_MainStage(D_STAGE_WaiSetUp);
  return ret;
}


//----------------------------------
// 補正要求
//----------------------------------
uint Exec_Calib(void)
{
  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック  if (true != wait_operating(&mainDebug_mode))
                                                   ////動作モードチェック
  {
    //中断終了
    return RSP_E_NG;  // 停止
  }
  printf("\n");
  ROS_INFO("<<< [Exec_Calib] stage start >>>");

  //①Tote、棚位置の補正処理未定
  //  loop_rate.sleep();  //ダミー時間待ち

  t_MainCtrlRsp.rsp_result = RSP_E_OK;  //正常終了
  return RSP_E_OK;
}

//----------------------------------
// Gntr認識要求
//----------------------------------
t2_robot_msgs::CaptureReq snd_captreq_msg;   /* 認識へのトピックデータ変数 */
t2_robot_msgs::RecognizeReq snd_recoreq_msg; /* 認識へのトピックデータ変数 */
t2_robot_msgs::CaptureRes rcv_recoPC_captres_msg;
t2_msgs::RecognizeRes rcv_reco_recores_msg;

uint t2_now_box_index;   // graspのサービス対応するまで一時的に使用
uint t2_now_item_index;  // graspのサービス対応するまで一時的に使用

void Exec_PickOrder(void)
{
  uint order_index, box_index, item_index, to_box_index = 0;
  OrderStatus status, ret;

  printf("\n");
  ROS_INFO("Exec_PickOrder() start");

  if (true != wait_operating(&mainDebug_mode, 1))
  {
    Change_MainStage(D_STAGE_HALT);
    return;
  }

  uint pick_index, from_place_id, to_place_id;
  std::vector<uint> to_place_id_list;

  OrderStatus order_status = selectPickInfo(from_place_id, to_place_id_list, item_index);

  //--------------------
  // 運転停止判定
  //--------------------
  if (received_halt_command_)
  {
    Change_MainStage(D_STAGE_HALT);
    return;
  }

  // bin間移動/bin内移動/bin間移動2(成功:ORDER_LATER, 失敗:ORDER_FAIL)
  if (ORDER_FAIL == order_status)
  {
    if (recognition_retry_count_ < max_recognition_retry_count_)
    {
      recognition_retry_count_++;
      console_out("\n★ 認識リトライ(%d/%d)\n", recognition_retry_count_, max_recognition_retry_count_);
      order_status = ORDER_FAIL;
    }
    else
    {
      // Bin間/内移動のための認識待ち
      ros::Time base_time = ros::Time::now();
      for (std::size_t i = 0; i < boxinfo.size(); ++i)
      {
        Boxinfo_t& box_info = boxinfo[i];
        if (!box_info.enabled || BoxTypes::Bin != box_info.type)
        {
          continue;
        }
        waitRecognitionTimeout(box_info.box_id, RecognitionType::Item, base_time);
        waitRecognitionTimeout(box_info.box_id, RecognitionType::Octomap, base_time);
      }

      order_status = selectMoveInfo(from_place_id, to_place_id_list, item_index);
      if (ORDER_FAIL == order_status)
      {
        recognition_retry_count_ = 0;
        resetMovedItemInfo();
        console_out("\n★ 最初からやり直し\n");
      }
    }
  }

  // 格納先の表示用
  std::string display_to_place_id = "[ ";
  for (std::size_t i = 0; i < to_place_id_list.size(); ++i)
  {
    std::ostringstream oss;
    oss << to_place_id_list[i];
    display_to_place_id.append(oss.str() + " ");
  }
  display_to_place_id.append("]");

  // ビン間/内移動表示用
  std::string move_name;

  if (order_status != ORDER_NONE)
  {
    if (ORDER_FAIL == order_status)
    {
      ROS_INFO("Retry all order items to pick");

      // すべてのbinの認識状態をリセット
      for (std::size_t i = 0; i < boxinfo.size(); ++i)
      {
        if (!boxinfo[i].enabled || BoxTypes::Bin != boxinfo[i].type)
        {
          continue;
        }
        resetRecognize(i);
      }

      // orderアイテムが存在するbinの撮影・認識(アイテム認識/箱詰認識)
      for (std::size_t i = 0; i < boxinfo.size(); ++i)
      {
        const Boxinfo_t& box_info = boxinfo[i];
        if (!box_info.enabled || BoxTypes::Bin != box_info.type)
        {
          continue;
        }
        uint place_id = box_info.box_id;
        if (hasOrderItem(place_id))
        {
          executeRecognition(place_id, RecognitionTypes::Item);
          executeRecognition(place_id, RecognitionTypes::Octomap);
        }
      }

      // orderアイテムが存在しないbinの撮影・認識(箱詰認識)
      for (std::size_t i = 0; i < boxinfo.size(); ++i)
      {
        const Boxinfo_t& box_info = boxinfo[i];
        if (!box_info.enabled || BoxTypes::Bin != box_info.type)
        {
          continue;
        }
        uint place_id = box_info.box_id;
        if (!hasOrderItem(place_id))
        {
          executeRecognition(place_id, RecognitionTypes::Item);
          executeRecognition(place_id, RecognitionTypes::Octomap);
        }
      }

      // やり直し
      for (std::size_t i = 0; i < boxinfo.size(); ++i)
      {
        Boxinfo_t& box_info = boxinfo[i];
        box_info.plan_status = PLAN_NOTYET;
        for (std::size_t j = 0; j < box_info.item_plan_status.size(); ++j)
        {
          box_info.item_plan_status[j] = PLAN_NOTYET;
          if (ORDER_LATER == box_info.item_order[j])
          {
            box_info.item_order[j] = ORDER_EXIST;
          }
        }
      }

      Change_MainStage(D_STAGE_PickOrder);
      return;
    }

    if (ORDER_EXIST == order_status || ORDER_LATER == order_status)
    {
      if (to_place_id_list.size() == 0)
      {
        ROS_ERROR("Invalid to_place_id_list.size()=%lu", to_place_id_list.size());
        Change_MainStage(D_STAGE_HALT);
        return;
      }
    }

    if (!getBoxIndex(from_place_id, &box_index))
    {
      ROS_ERROR("Invalid from_place_id=%u", from_place_id);
      Change_MainStage(D_STAGE_HALT);
      return;
    }

    if (ORDER_EXIST == order_status)
    {
      to_place_id = to_place_id_list[0];
      if (!getBoxIndex(to_place_id, &to_box_index))
      {
        ROS_ERROR("Invalid to_place_id=%u", to_place_id);
        Change_MainStage(D_STAGE_HALT);
        return;
      }
    }

    // Bin間/Bin内移動
    if (ORDER_LATER == order_status)
    {
      // Bin間移動のアイテム情報をセットする
      setPickMoveInfo(from_place_id, item_index, to_place_id_list);
    }

    if (ORDER_EXIST == order_status)
    {
      ROS_INFO("\nPickItem ..... from  (box_index:%d), %s(item_index=%d), to (box_index=%d)\n",
          box_index, boxinfo[box_index].item_name[item_index].c_str(), item_index, to_box_index);
      console_out("\n\n★ ピック開始 (%s: %s ⇒ %s)\n", boxinfo[box_index].item_name[item_index].c_str(),
          t_boxNameId[from_place_id].box_name, t_boxNameId[to_place_id].box_name);
    }
    else if (ORDER_LATER == order_status)
    {
      if (to_place_id_list.size() == 1 && from_place_id == to_place_id_list[0])
      {
        move_name = "ビン内移動";
      }
      else
      {
        move_name = "ビン間移動";
      }

      console_out("\nMoveItem ..... from (box_index:%d), %s(item_index=%d) to (box_index %d)\n",
          box_index, boxinfo[box_index].item_name[item_index].c_str(), item_index, to_box_index);
      console_out("\n\n★ %s開始 (%s: %s ⇒ %s)\n", move_name.c_str(), boxinfo[box_index].item_name[item_index].c_str(),
          t_boxNameId[from_place_id].box_name, display_to_place_id.c_str());
    }
    else
    {
      Change_MainStage(D_STAGE_HALT);
      return;
    }
  }
  else
  {
    //--------------------
    // オーダー終了
    //--------------------
    printAllBoxinfo();
    console_out_orderinfo();
    console_out_boxinfo();

    if (RSP_E_OK != Exec_OperationEnd())
    {
      Change_MainStage(D_STAGE_HALT);   // 強制終了
      return;
    }
    Change_MainStage(D_STAGE_WaiSetUp);
    return;
  }

  int32_t plan_id = getPlanID(box_index, item_index);

  if (waitForPickPlan(from_place_id, item_index) == false)
  {
    ROS_INFO("PickPlan error(place_id=%d, item_index=%d)", from_place_id, item_index);
    if (ORDER_LATER == order_status)
    {
      boxinfo[box_index].item_order[item_index] = ORDER_NONE;
      boxinfo[box_index].item_plan_status[item_index] = PLAN_NG;
    }
    Change_MainStage(D_STAGE_PickOrder);
    //console_out_boxinfo();
    return;
  }

  if (ORDER_LATER == order_status)
  {
    to_place_id = getPickGoalPlaceID(from_place_id);
    getBoxIndex(to_place_id, &to_box_index);
  }

  if (RSP_E_OK != Exec_PickExecute(box_index, item_index, plan_id))
  {
    ROS_INFO("PickExecute error(box_index=%d, item_index=%d)", box_index, item_index);

    //--------------------
    // 運転停止判定(GAP4)
    //--------------------
    if (received_halt_command_)
    {
      Change_MainStage(D_STAGE_HALT);
      return;
    }

    if (retryPickSameItem(from_place_id, item_index, pick_execute_attempts_))
    {
      // 再認識せずにBoxとアイテム状態を戻す
      setRecogStatus(box_index, RECOG_OK);
      boxinfo[box_index].status[item_index] = ITEM_RECOG_OK;
      boxinfo[box_index].item_plan_status[item_index] = PLAN_NOTYET;
      if (ORDER_EXIST == order_status)
      {
        boxinfo[box_index].plan_status = PLAN_NOTYET;
      }
    }
    else
    {
      updateOrderInfoByPlanError(from_place_id, item_index);
      resetRecognize(box_index);
      executeRecognition(from_place_id, RecognitionTypes::Item);
      executeRecognition(from_place_id, RecognitionTypes::Octomap);
      ROS_INFO("executeRecognition(place_id=%d) done", from_place_id);
    }
    setOctomapStatus(to_box_index, RECOG_OK);   // 格納先boxを箱詰認識済みに戻す
    Change_MainStage(D_STAGE_PickOrder);
    printAllBoxinfo();
    console_out_boxinfo();
    return;
  }
  updateBoxInfoByPick(box_index, item_index);

  //--------------------
  // 運転停止判定(GP)
  //--------------------
  if (received_halt_command_)
  {
    Change_MainStage(D_STAGE_HALT);
    return;
  }

  place_move_order_status_ = order_status;

  int32_t result;
  if (RSP_E_OK != Exec_PlaceMove(to_box_index, item_index, box_index, plan_id, result))
  {
    ROS_INFO("Place item(itemID:%d) in box_%d  to box_%d failed", item_index, box_index, to_box_index);

    // 次に実行中のPlanに対する処理
    if (result == t2_msgs::ArmPlaceMoveResult::PICK_ERROR || result == t2_msgs::ArmPlaceMoveResult::ITEM_ERROR)
    {
      std::lock_guard<std::mutex> lock(plan_recog_status_mutex_);
      arm_place_move_result_ = result;
      //place_move_order_status_ = order_status;
      if (PLAN_NOTYET == boxinfo[box_index].plan_status)
      {
        // 次の計画中なら
        boxinfo[box_index].plan_status = PLAN_NG;
      }
      else if (PLAN_OK == boxinfo[box_index].plan_status)
      {
        // 次の計画済みなら
        for (std::size_t i = 0; i < boxinfo[box_index].item_plan_status.size(); ++i)
        {
          // Place中のアイテム以外
          if (i == item_index)
          {
            continue;
          }
          // Plan完了済みのアイテムを無効
          if (PLAN_OK == boxinfo[box_index].item_plan_status[i])
          {
            boxinfo[box_index].item_plan_status[i] = PLAN_NOTYET;
            uint gp_index = boxinfo[box_index].grasp_point_index[i];
            uint to_place_id_local = boxinfo[box_index].grasp_release_points[i][gp_index].release_place_id;
            uint to_box_index_local;
            getBoxIndex(to_place_id_local, &to_box_index_local);

            // 箱詰認識状態を戻す
            setOctomapStatus(to_box_index_local, RECOG_OK);
          }
        }

        if (result == t2_msgs::ArmPlaceMoveResult::PICK_ERROR)
        {
          // 認識フラグをONに戻す
          setRecogStatus(box_index, RECOG_OK);
        }
        else if (result == t2_msgs::ArmPlaceMoveResult::ITEM_ERROR)
        {
          // 再認識
          resetRecognize(box_index);
          executeRecognition(from_place_id, RecognitionTypes::Item);
          executeRecognition(from_place_id, RecognitionTypes::Octomap);
        }

        // 通常Pickの場合は再プラン、ビン間/内移動の場合はプラン完了のままで次のビン間/内移動へ
        if (ORDER_EXIST == order_status)
        {
          boxinfo[box_index].plan_status = PLAN_NOTYET;
        }
      }
    }

    updateOrderInfoByPlanError(from_place_id, item_index);

    //--------------------
    // 運転停止判定(GP4)
    //--------------------
    if (received_halt_command_)
    {
      Change_MainStage(D_STAGE_HALT);
      return;
    }

    // PLAN_ERRORの場合は再認識
    if (result == t2_msgs::ArmPlaceMoveResult::PLAN_ERROR)
    {
      resetRecognize(box_index);
      executeRecognition(from_place_id, RecognitionTypes::Item);
      executeRecognition(from_place_id, RecognitionTypes::Octomap);
      ROS_INFO("executeRecognition(place_id=%d) done", from_place_id);
    }

    setOctomapStatus(to_box_index, RECOG_OK);
    Change_MainStage(D_STAGE_PickOrder);
    printAllBoxinfo();
    console_out_boxinfo();
    return;
  }
  updateBoxInfoByMove(box_index, item_index, to_box_index, TaskType::Pick);

  // bin間/内移動関連情報の初期化
  if (ORDER_LATER == order_status)
  {
    initializeParam();
    for (std::size_t i = 0; i < boxinfo.size(); ++i)
    {
      Boxinfo_t& box_info = boxinfo[i];
      if (!box_info.enabled || BoxTypes::Bin != box_info.type)
      {
        continue;
      }
      for (std::size_t j = 0; j < box_info.item_plan_status.size(); ++j)
      {
        box_info.item_plan_status[j] = PLAN_NOTYET;
      }
    }
  }

  // item_location_file.jsonに出力
  ROS_INFO("Output item_location_file.json");
  writeItemLocationJson();

  //--------------------
  // 運転停止判定(RAP4)
  //--------------------
  if (received_halt_command_)
  {
    Change_MainStage(D_STAGE_HALT);
    return;
  }

  executeRecognition(to_place_id, RecognitionTypes::Octomap);   // 箱詰認識
  // Bin間移動の場合
  if (ORDER_LATER == order_status)
  {
    executeRecognition(to_place_id, RecognitionTypes::Item);    // アイテム認識
  }

  if (ORDER_EXIST == order_status)
  {
    ROS_INFO("\nPick Success (box_index=%d, item_index=%d, to_box_index=%d)\n", box_index, item_index, to_box_index);
    console_out("\n★ Pick成功 (%s: %s ⇒ %s)\n\n\n", boxinfo[box_index].item_name[item_index].c_str(),
                t_boxNameId[boxinfo[box_index].box_id].box_name, t_boxNameId[boxinfo[to_box_index].box_id].box_name);
  }
  else
  {
    ROS_INFO("\nMove Success (box_index=%d, item_index=%d, to_box_index=%d)\n", box_index, item_index, to_box_index);
    console_out("\n★ %s移動成功 (%s: %s ⇒ %s)\n\n\n", move_name.c_str(), boxinfo[box_index].item_name[item_index].c_str(),
                t_boxNameId[boxinfo[box_index].box_id].box_name, t_boxNameId[boxinfo[to_box_index].box_id].box_name);
  }

  Change_MainStage(D_STAGE_PickOrder);

  printAllBoxinfo();
  console_out_boxinfo();
  return;
}

int getPlanID(uint box_index, uint item_index)
{
  return box_index * 100 + item_index;
}

void threadEndProcess(uint thread_index)
{
  thread_end_[thread_index] = true;
  planning_place_id_[thread_index] = 0;
}

bool runPickPlanThread(uint place_id, const std::vector<uint>& to_place_id_vec, uint item_index,
    uint target_recog_jobno, OrderStatus order_status)
{
  ros::Duration duration(0.1);

  Boxinfo_t *box_info;
  getBoxInfo(place_id, &box_info);

  uint box_index;
  getBoxIndex(place_id, &box_index);

  // PickExecute失敗後のリトライでは把持・箱詰計画を省略
  if (box_info->failed_gp_numbers[item_index].size() == 0)
  {
    // 把持・箱詰計画
    if (RSP_E_OK != Exec_GraspPack(place_id, item_index, to_place_id_vec))
    {
      ROS_INFO("runPickPlanThread(%s) Execute grasp and pack plan NG", box_info->box_name.c_str());

      if (ITEM_RECOG_OK == box_info->status[item_index])
      {
        // GraspPlan失敗
        console_out("\n★ 把持・箱詰計画失敗 : %s\n", box_info->item_name[item_index].c_str());
        updateOrderInfoByPlanError(place_id, item_index);
      }
      else
      {
        // 認識失敗
        console_out("\n★ 認識失敗 : %s\n", box_info->item_name[item_index].c_str());
        updateOrderInfoByRecogError(place_id, item_index);
      }

      if (ORDER_MOVE == order_status)
      {
        box_info->item_plan_status[item_index] = PLAN_NG;
      }

      return false;
    }
    ROS_INFO("runPickPlanThread(%s) Execute grasp and pack plan OK", box_info->box_name.c_str());
  }

  // PlaceMove(FailedItem)による再撮影・再認識済み
  {
    std::lock_guard<std::mutex> lock(plan_recog_status_mutex_);
    if (PLAN_NG == box_info->plan_status)
    {
      ROS_INFO("Plan NG for PlaceMove failed");
      if (ORDER_EXIST == place_move_order_status_)
      { // 通常Pickの場合は同じBinを再Plan
        box_info->plan_status = PLAN_NOTYET;
      }
      else
      { // Bin間/内移動の場合はPlan完了
        box_info->plan_status = PLAN_OK;
      }

      if (t2_msgs::ArmPlaceMoveResult::PICK_ERROR == arm_place_move_result_)
      {
        // 誤認識されたPick中アイテムのアイテム状態を変更し、Plan開始
        int pick_ng_index = -1;
        for (std::size_t i = 0; i < box_info->status.size(); ++i)
        {
          if (ITEM_PICK_NG == box_info->status[i])
          {
            pick_ng_index = static_cast<int>(i);
            break;
          }
        }
        if (pick_ng_index >= 0)
        {
          for (std::size_t i = 0; i < box_info->status.size(); ++i)
          {
            if (ITEM_RECOG_OK == box_info->status[i] && ITEM_MISRECOG == box_info->item_recog_status[i] &&
                box_info->cad_id[pick_ng_index] == box_info->cad_id[i])
            {
              box_info->status[i] = ITEM_RECOG_NOTYET;
              box_info->status[pick_ng_index] = ITEM_RECOG_OK;
            }
          }
        }
        if (ORDER_MOVE == place_move_order_status_)
        {
          return false;
        }
      }
      else if (t2_msgs::ArmPlaceMoveResult::ITEM_ERROR == arm_place_move_result_)
      {
        resetRecognize(box_index);
        executeRecognition(place_id, RecognitionTypes::Item);
        executeRecognition(place_id, RecognitionTypes::Octomap);
        ROS_INFO("runPickPlanThread(%s) executeRecognition(place_id=%d) done", box_info->box_name.c_str(), place_id);
        return false;
      }
    }
  }

  // Pick計画
  int32_t plan_id = getPlanID(box_index, item_index);
  bool permit_protrusion = true;
  if (ORDER_MOVE == order_status)
  {
    permit_protrusion = permitProtrusion();
  }
  uint to_box_index;
  if (RSP_E_OK != Exec_PickPlan(box_index, item_index, plan_id, permit_protrusion, to_box_index))
  {
    // Plan失敗
    ROS_INFO("runPickPlanThread(%s) Execute pick plan NG", box_info->box_name.c_str());
    updateOrderInfoByPlanError(place_id, item_index);
    return false;
  }

  uint to_place_id = boxinfo[to_box_index].box_id;

  // PlaceMove失敗(FailedItem)時のPlanStatusと格納先boxのRecogStatusの衝突を防ぐ
  {
    std::lock_guard<std::mutex> lock(plan_recog_status_mutex_);

    if (ORDER_EXIST == order_status)
    {
      // 格納先boxが先に使われていた場合(認識job_noが異なる／未認識状態(計画完了))
      const Boxinfo_t& to_box_info = boxinfo[to_box_index];
      if (target_recog_jobno != to_box_info.octomap_recog_jobno || RECOG_NOTYET == to_box_info.octomap_status)
      {
        ROS_INFO("runPickPlanThread(%s) Box(place_id=%u) is used", box_info->box_name.c_str(), to_place_id);
        // アイテム選択のやりなおし
        return false;
      }
    }
    ROS_INFO("runPickPlanThread(%s) Execute pick plan OK", box_info->box_name.c_str());

    // PlaceMove(FailedItem)による再撮影・再認識済み
    if (PLAN_NG == box_info->plan_status)
    {
      ROS_INFO("Plan NG for PlaceMove failed");
      if (ORDER_EXIST == place_move_order_status_)
      { // 通常Pickの場合は同じBinを再Plan
        box_info->plan_status = PLAN_NOTYET;
      }
      else
      { // Bin間/内移動の場合はPlan完了
        box_info->plan_status = PLAN_OK;
      }

      if (t2_msgs::ArmPlaceMoveResult::PICK_ERROR == arm_place_move_result_)
      {
        // 誤認識されたPick中アイテムのアイテム状態を変更し、Plan開始
        int pick_ng_index = -1;
        for (std::size_t i = 0; i < box_info->status.size(); ++i)
        {
          if (ITEM_PICK_NG == box_info->status[i])
          {
            pick_ng_index = static_cast<int>(i);
            break;
          }
        }
        if (pick_ng_index >= 0)
        {
          for (std::size_t i = 0; i < box_info->status.size(); ++i)
          {
            if (ITEM_RECOG_OK == box_info->status[i] && ITEM_MISRECOG == box_info->item_recog_status[i] &&
                box_info->cad_id[pick_ng_index] == box_info->cad_id[i])
            {
              box_info->status[i] = ITEM_RECOG_NOTYET;
              box_info->status[pick_ng_index] = ITEM_RECOG_OK;
            }
          }
        }
      }
      else if (t2_msgs::ArmPlaceMoveResult::ITEM_ERROR == arm_place_move_result_)
      {
        resetRecognize(box_index);
        executeRecognition(place_id, RecognitionTypes::Item);
        executeRecognition(place_id, RecognitionTypes::Octomap);
        ROS_INFO("runPickPlanThread(%s) executeRecognition(place_id=%d) done", box_info->box_name.c_str(), place_id);
      }
      return false;   // 再プラン
    }

    // 対象のBinとBoxの認識フラグをOFF
    setRecogStatus(box_index, RECOG_NOTYET);
    setOctomapStatus(to_box_index, RECOG_NOTYET);

    // アイテム状態を計画成功、bin状態を計画完了
    setPickGoalPlaceID(place_id, to_place_id);
    box_info->item_plan_status[item_index] = PLAN_OK;
    box_info->plan_status = PLAN_OK;
  }

  if (ORDER_MOVE == order_status)
  {
    box_info->item_order[item_index] = ORDER_NONE;
  }

  return true;
}

void runPickTaskBinThread(uint place_id, OrderStatus order_status)
{
  ros::Duration duration(0.1);

  std::ostringstream oss;
  oss << std::this_thread::get_id();
  std::string thread_name = oss.str();

  uint box_index;
  getBoxIndex(place_id, &box_index);

  Boxinfo_t *box_info;
  getBoxInfo(place_id, &box_info);

  // アイテム選択スタート
  while (ros::ok() && task_active_)
  {
    // 計画完了の場合は、pick/place実行完了まで待つ
    if (PLAN_OK == box_info->plan_status)
    {
      duration.sleep();
      continue;
    }

    // binの認識フラグ完了待ち
    waitRecognitionTimeout(box_info->box_id, RecognitionType::Item, ros::Time::now());

    int target_item_index = -1;
    uint to_place_id;
    uint target_recog_jobno = 0;
    bool not_recognized = false;

    std::vector<uint> to_place_id_list;
    double max_probability = 0;

    for (std::size_t i = 0; i < box_info->item_order.size(); ++i)
    {
      // orderアイテム
      if (ORDER_EXIST != box_info->item_order[i])
      {
        continue;
      }

      // アイテム状態が認識成功
      if (ITEM_RECOG_OK != box_info->status[i])
      {
        continue;
      }

      // 認識スコアが閾値以上
      double probability = box_info->item_data[i].probability;
      if (probability < item_probability_threshold_)
      {
        continue;
      }

      // 格納先のplace_id
      uint dest_place_id = getPickDestinationPlaceID(place_id, i);
      if (dest_place_id == 0)
      {
        ROS_ERROR("ORDER_EXIT item has no order");
        continue;
      }

      std::vector<uint>::iterator it = std::find(to_place_id_list.begin(), to_place_id_list.end(), dest_place_id);
      if (it == to_place_id_list.end())
      {
        to_place_id_list.push_back(dest_place_id);
      }

      uint dest_box_index;
      getBoxIndex(dest_place_id, &dest_box_index);
      // 格納先boxの箱詰認識が完了している
      if (RECOG_OK != boxinfo[dest_box_index].octomap_status)
      {
        not_recognized = true;
        continue;
      }
      uint recog_jobno = boxinfo[dest_box_index].octomap_recog_jobno;

      // 最大認識スコア
      if (probability > max_probability)
      {
        target_item_index = i;
        to_place_id = dest_place_id;
        target_recog_jobno = recog_jobno;
        max_probability = probability;
      }
      else if (probability == max_probability)
      {
        double grasp_easiness = getGraspEasiness(box_info->cad_id[i]);
        if (grasp_easiness > getGraspEasiness(box_info->cad_id[target_item_index]))
        {
          target_item_index = i;
          to_place_id = dest_place_id;
          target_recog_jobno = recog_jobno;
        }
      }
    }

    // アイテムがない場合
    if (target_item_index < 0)
    {
      // 箱詰認識が未完了のアイテムがあるとき
      if (not_recognized)
      {
        // 箱詰認識完了まで待つ
        ros::Time base_time = ros::Time::now();
        for (std::size_t i = 0; i < to_place_id_list.size(); ++i)
        {
          waitRecognitionTimeout(to_place_id_list[i], RecognitionType::Octomap, base_time);
        }
        continue;
      }
      else
      {
        // アイテムなし
        box_info->plan_status = PLAN_OK;
        while (PLAN_OK == box_info->plan_status && task_active_)
        {
          duration.sleep();
          continue;
        }
        continue;
      }
    }

    // アイテムあり
    std::vector<uint> to_place_id_vec = { to_place_id };

    ROS_INFO("runPickTaskBinThread(%s) plan... from_place_id=%u, to_place_id=%u, item_index=%d",
        box_info->box_name.c_str(), place_id, to_place_id, target_item_index);

    if (!runPickPlanThread(place_id, to_place_id_vec, target_item_index, target_recog_jobno, ORDER_EXIST))
    {
      continue;
    }
    duration.sleep();
  }
}

void runPickTaskThread(void)
{
  ros::Duration duration(0.1);  // ループの待ち時間

  uint bin_count = getBinCount();
  planning_thread_.resize(bin_count);
  thread_end_.assign(bin_count + 1, true);

  // 全boxの撮影・認識(箱詰認識)
  executeRecognition(D_box_1_ID, RecognitionTypes::Octomap);
  executeRecognition(D_box_2_ID, RecognitionTypes::Octomap);
  executeRecognition(D_box_3_ID, RecognitionTypes::Octomap);

  // orderアイテムが存在するbinの撮影・認識(アイテム認識/箱詰認識)
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    const Boxinfo_t& box_info = boxinfo[i];
    if (!box_info.enabled || BoxTypes::Bin != box_info.type)
    {
      continue;
    }
    uint place_id = box_info.box_id;
    if (hasOrderItem(place_id))
    {
      executeRecognition(place_id, RecognitionTypes::Item);
      executeRecognition(place_id, RecognitionTypes::Octomap);
    }
  }

  // orderアイテムが存在しないbinの撮影・認識(箱詰認識)
  for (std::size_t i = 0; i < boxinfo.size(); ++i)
  {
    const Boxinfo_t& box_info = boxinfo[i];
    if (!box_info.enabled || BoxTypes::Bin != box_info.type)
    {
      continue;
    }
    uint place_id = box_info.box_id;
    if (!hasOrderItem(place_id))
    {
      executeRecognition(place_id, RecognitionTypes::Item);
      executeRecognition(place_id, RecognitionTypes::Octomap);
    }
  }

  // bin毎のスレッド作成
  for (std::size_t i = 0; i < planning_thread_.size(); ++i)
  {
    std::thread& thread = planning_thread_[i];
    if (thread.joinable())
    {
      thread.join();
    }

    uint place_id = 0;
    uint bin_count = 0;
    for (std::size_t j = 0; j < boxinfo.size(); ++j)
    {
      const Boxinfo_t box_info = boxinfo[j];
      if (!box_info.enabled || BoxTypes::Bin != box_info.type)
      {
        continue;
      }
      if (bin_count == i)
      {
        place_id = box_info.box_id;
        break;
      }
      bin_count++;
    }
    thread = std::thread(runPickTaskBinThread, place_id, ORDER_EXIST);
  }

  while (ros::ok() && task_active_)
  {
    for (std::size_t i = 0; i < boxinfo.size(); ++i)
    {
      const Boxinfo_t& box_info = boxinfo[i];
      if (!box_info.enabled || BoxTypes::Bin != box_info.type)
      {
        continue;
      }

      for (std::size_t j = 0; j < box_info.item_order.size(); ++j)
      {
        // bin間/内移動
        if (ORDER_MOVE == box_info.item_order[j] && PLAN_NOTYET == box_info.item_plan_status[j])
        {
          // 移動アイテム, 移動先の設定
          uint item_index = getPickItemIndex(box_info.box_id);
          const std::vector<uint>& to_place_id_list = getPickGoalPlaceIDList(box_info.box_id);
          runPickPlanThread(box_info.box_id, to_place_id_list, item_index, 0, ORDER_MOVE);
        }
      }
    }

    // 終了判定
    bool end_task = true;
    for (std::size_t i = 0; i < boxinfo.size(); ++i)
    {
      const Boxinfo_t& box_info = boxinfo[i];
      for (std::size_t j = 0; j < box_info.item_order.size(); ++j)
      {
        OrderStatus item_order = boxinfo[i].item_order[j];
        if (ORDER_EXIST == item_order || ORDER_LATER == item_order)
        {
          end_task = false;
          break;
        }
      }
      if (!end_task)
      {
        break;
      }
    }
    if (end_task)
    {
      ROS_INFO("runPickTaskThread() All pick order is done.");
      for (std::size_t i = 0; i < planning_thread_.size(); ++i)
      {
        if (planning_thread_[i].joinable())
        {
          planning_thread_[i].join();
        }
      }
      return;
    }
    duration.sleep();
  }
}

void runStowTaskThread(void)
{
  ros::Duration duration(0.1);  // ループの待ち時間

  uint tote_index;
  getBoxIndex(D_tote_1_ID, &tote_index);
  const Boxinfo_t& tote = boxinfo[tote_index];

  // 全Binの撮影・認識(箱詰認識)
  executeAllRecognition(RecognitionTypes::Octomap);

  // AmnestyToteの撮影・認識(箱詰認識)
  executeRecognition(D_tote_2_ID, RecognitionTypes::Octomap);

  while (ros::ok() && task_active_)
  {
    // Toteの撮影・認識(アイテム認識)
    executeRecognition(D_tote_1_ID, RecognitionTypes::Item);

    while (ros::ok() && task_active_)
    {
      // Toteのアイテム認識が完了するまで待つ
      waitRecognitionTimeout(D_tote_1_ID, RecognitionType::Item, ros::Time::now());
      ROS_INFO("runStowTaskThread() Recognition status OK");

      // 箱詰認識が完了しているBinが1つ以上になるまで待つ
      ros::Time start = ros::Time::now();
      bool recog_finished = false;
      do
      {
        if (!task_active_)
        {
          return;
        }
        for (std::size_t i = 0; i < boxinfo.size(); ++i)
        {
          if (!boxinfo[i].enabled || BoxTypes::Bin != boxinfo[i].type)
          {
            continue;
          }
          if (RECOG_OK == boxinfo[i].octomap_status)
          {
            recog_finished = true;
            break;
          }
        }
        if (!recog_finished && ros::Time::now() - start > ros::Duration(wait_octomap_recognition_timeout_))
        {
          console_out("\n★ 箱詰認識タイムアウト : 全Bin");
          ROS_INFO("runStowTaskThread() Recognition octomap timeout");
          {
            std::lock_guard<std::mutex> lock(octomap_recog_mutex_);
            for (std::size_t i = 0; i < boxinfo.size(); ++i)
            {
              if (!boxinfo[i].enabled || BoxTypes::Bin != boxinfo[i].type)
              {
                continue;
              }
              boxinfo[i].capture_octomap_allowed = true;
            }
          }
          executeAllRecognition(RecognitionTypes::Octomap);
          start = ros::Time::now();
        }
        duration.sleep();
      } while (!recog_finished);

      // アイテムがないとき終了
      while (getTargetItemCount(D_tote_1_ID) == 0)
      {
        if (!task_active_)
        {
          return;
        }
        duration.sleep();
      }

      // PlaceMove判定
      {
        std::lock_guard<std::mutex> lock(plan_recog_status_mutex_);
        if (PLAN_NG == boxinfo[tote_index].plan_status)
        {
          // Plan状態を未に設定
          setStowPlanStatus(PLAN_NOTYET);
          if (t2_msgs::ArmPlaceMoveResult::PICK_ERROR == arm_place_move_result_)
          {
            // 誤認識されたPick中アイテムのアイテム状態を変更し、Plan開始
            int pick_ng_index = -1;
            for (std::size_t i = 0; i < tote.status.size(); ++i)
            {
              if (ITEM_PICK_NG == tote.status[i])
              {
                pick_ng_index = static_cast<int>(i);
                break;
              }
            }
            if (pick_ng_index >= 0)
            {
              for (std::size_t i = 0; i < tote.status.size(); ++i)
              {
                if (ITEM_RECOG_OK == tote.status[i] && ITEM_MISRECOG == tote.item_recog_status[i] &&
                    tote.cad_id[pick_ng_index] == tote.cad_id[i])
                {
                  boxinfo[tote_index].status[i] = ITEM_RECOG_NOTYET;
                  boxinfo[tote_index].status[pick_ng_index] = ITEM_RECOG_OK;
                }
              }
            }
          }
          else if (t2_msgs::ArmPlaceMoveResult::ITEM_ERROR == arm_place_move_result_)
          {
            // Toteの再認識
            resetRecognize(tote_index);
            break;
          }
        }
      }

      // 予測あり把持・箱詰計画
      if (RSP_E_OK != Exec_StowPlan(tote_index))
      {
        ROS_INFO("runStowTaskThread() plan failed");
        resetRecognize(tote_index);
        break;
      }

      ROS_INFO("runStowTaskThread() Execute stow plan OK");

      // PlaceMove失敗の衝突回避
      {
        std::lock_guard<std::mutex> lock(plan_recog_status_mutex_);
        if (PLAN_NG == boxinfo[tote_index].plan_status)
        {
          // Plan状態を未に設定
          setStowPlanStatus(PLAN_NOTYET);

          if (t2_msgs::ArmPlaceMoveResult::PICK_ERROR == arm_place_move_result_)
          {
            // 誤認識されたPick中アイテムのアイテム状態を変更し、再Plan
            int pick_ng_index = -1;
            for (std::size_t i = 0; i < tote.status.size(); ++i)
            {
              if (ITEM_PICK_NG == tote.status[i])
              {
                pick_ng_index = static_cast<int>(i);
                break;
              }
            }
            if (pick_ng_index >= 0)
            {
              for (std::size_t i = 0; i < tote.status.size(); ++i)
              {
                if (ITEM_RECOG_OK == tote.status[i] && ITEM_MISRECOG == tote.item_recog_status[i] &&
                    tote.cad_id[pick_ng_index] == tote.cad_id[i])
                {
                  boxinfo[tote_index].status[i] = ITEM_RECOG_NOTYET;
                  boxinfo[tote_index].status[pick_ng_index] = ITEM_RECOG_OK;
                }
              }
              continue;   // 再プラン
            }
            else
            {
              ROS_ERROR("Failed to change pikced item status");
              continue;
            }
          }
          else if (t2_msgs::ArmPlaceMoveResult::ITEM_ERROR == arm_place_move_result_)
          {
            // Toteの再認識
            resetRecognize(tote_index);
            break;
          }
        }

        setRecogStatus(tote_index, RECOG_NOTYET);
        setStowPlanStatus(PLAN_OK);
      }
    }

    duration.sleep();
  }
}

void console_out_orderinfo(void)
{
  int i, j;
  const std::vector<std::string> status_names{ "未処理", "認識完了", "ピック失敗",
                                               "プレイス失敗", "ムーブ済", "オーダー完了" };

  console_out("\n++++++++++ ピックオーダー結果 +++++++++++++++++++++++++");
  int box_no = 1;
  for (std::size_t k = 0; k < boxinfo.size(); ++k)
  {
    if (!boxinfo[k].enabled || BoxTypes::Box != boxinfo[k].type)
    {
      continue;
    }

    for (i = 0; i < orderinfo.size(); i++)
    {
      if (orderinfo[i].to_box_index != k)
      {
        continue;
      }
      console_out("\nピックオーダー%d. ⇒BOX%d[%s]\n", box_no, box_no, boxinfo[orderinfo[i].to_box_index].box_name.c_str());
      for (j = 0; j < orderinfo[i].item_name.size(); j++)
      {
        console_out("   %s %s \n", orderinfo[i].item_name[j].c_str(),
            status_names[orderinfo[i].status[j]].c_str());
      }
      box_no++;
      break;
    }
  }
  console_out("++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");
  return;
}

void console_out_boxsizeinfo(void)
{
  int i;

  console_out("\n++++++++++ boxsize　 +++++++++++++++++++++++++\n");
  for (i = 0; i < boxsizeinfo.size_id.size(); i++)
  {
    console_out("   %d. size_id( %s ),  volume( %lf )\n", i + 1, boxsizeinfo.size_id[i].c_str(), boxsizeinfo.volume[i]);
  }
  console_out("++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");
  return;
}

void console_out_boxinfo(void)
{
  int i, j, itemcount;
  const std::vector<std::string> status_names{ "未処理", "認識完了", "ピック失敗",
                                               "プレイス失敗", "ムーブ済", "オーダー完了" };

  console_out("\n++++++++++ Bins, Boxes, tote +++++++++++++++++++++++++");
  for (i = 0; i < boxinfo.size(); i++)
  {
    if (!boxinfo[i].enabled)
    {
      continue;
    }
    console_out("\n%s  (boxvolume=%lf, volume_of_items=%lf)\n", boxinfo[i].box_name.c_str(), boxinfo[i].boxvolume,
                boxinfo[i].volume_of_items);

    itemcount = boxinfo[i].item_name.size();
    if (itemcount > 0)
    {
      for (j = 0; j < itemcount; j++)
      {
        if (boxinfo[i].item_recog_status[j] == ITEM_UNKNOWN && boxinfo[i].status[j] == ITEM_RECOG_NOTYET)
        {
          continue;
        }
        console_out("   %s (cad_id=%d, volume=%lf, item_order=%d, item_recog_status=%d) : %s)\n",
            boxinfo[i].item_name[j].c_str(), boxinfo[i].cad_id[j], boxinfo[i].volume[j],
            boxinfo[i].item_order[j], boxinfo[i].item_recog_status[j],
            status_names[boxinfo[i].status[j]].c_str());
      }
    }
  }
  console_out("++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");
  return;
}

void console_out_pickorderinfo(void)
{
  const std::vector<std::string> order_status{ "オーダーなし", "オーダー完了", "オーダー失敗",
                                               "オーダーあり", "後回し", "移動中"};

  console_out("\n++++++++++++++++++++ ピックオーダー結果 ++++++++++++++++++++");
  for (int i = 0; i < pick_order_.size(); i++)
  {
    Orderinfo_t* order_info;
    if (!getOrderInfo(pick_order_[i].order_no, &order_info))
    {
      ROS_WARN("Invalid order_no=%u", pick_order_[i].order_no);
      continue;
    }
    int index = pick_order_[i].content_index;

    std::string from = boxinfo[order_info->from_box_index[pick_order_[i].content_index]].box_name;
    std::string to = boxinfo[order_info->to_box_index].box_name;
    std::string item_name = order_info->item_name[index];
    console_out("\nピックオーダー%d: [%s]⇒[%s]\n", i + 1, from.c_str(), to.c_str());
    console_out("   %s %s\n", item_name.c_str(), order_status[pick_order_[i].status].c_str());
  }
  console_out("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");
}

void Exec_StowOrder(void)
{
  uint box_index, to_box_index = 0;

  printf("\n");
  ROS_INFO("Exec_StowOrder() start");

  getBoxIndex(D_tote_1_ID, &box_index);
  uint place_id = boxinfo[box_index].box_id;

  if (getTargetItemCount(place_id) == 0)
  {
    //--------------------
    // オーダー終了
    //--------------------
    ROS_INFO("No item(box_index=%u)", box_index);
    console_out_boxinfo();

    if (RSP_E_OK != Exec_OperationEnd())
    {
      Change_MainStage(D_STAGE_HALT);   // 強制終了
      return;
    }
    Change_MainStage(D_STAGE_WaiSetUp);
    return;
  }

  if (!waitForStowPlan())
  {
    Change_MainStage(D_STAGE_StowOrder);
    return;
  }

  //--------------------
  // 運転停止判定
  //--------------------
  if (received_halt_command_)
  {
    Change_MainStage(D_STAGE_HALT);
    return;
  }

  uint item_index = getStowItemIndex();
  int32_t plan_id = getPlanID(box_index, item_index);

  // GPの座標を保持
  uint grasp_point_index = boxinfo[box_index].grasp_point_index[item_index];
  geometry_msgs::Point gp_pos = boxinfo[box_index].grasp_release_points[item_index][grasp_point_index].gp.position;

  if (RSP_E_OK != Exec_PickExecute(box_index, item_index, plan_id))
  {
    ROS_INFO("PickExecute error (box_index=%d, item_index=%d)", box_index, item_index);
    updateStowInfoByPlanError(place_id, item_index, gp_pos);

    //--------------------
    // 運転停止判定(GAP4)
    //--------------------
    if (received_halt_command_)
    {
      Change_MainStage(D_STAGE_HALT);
      return;
    }

    if (getFailedGPList().size() > max_failed_gp_count_)
    {
      // 失敗GPリストのクリア
      clearFailedGPList();
      recognition_retry_count_ = 0;
      resetRecognize(box_index);
      executeRecognition(place_id, RecognitionTypes::Item);
      ROS_INFO("executeRecognition(place_id=%d) done", place_id);
    }
    else
    {
      boxinfo[box_index].recog_status = RECOG_OK;
    }
    setStowPlanStatus(PLAN_NOTYET);
    setStowPreviousPlaceID();

    Change_MainStage(D_STAGE_StowOrder);
    printAllBoxinfo();
    console_out_boxinfo();
    return;
  }
  updateBoxInfoByPick(box_index, item_index);

  //--------------------
  // 運転停止判定(GP)
  //--------------------
  if (received_halt_command_)
  {
    Change_MainStage(D_STAGE_HALT);
    return;
  }

  uint release_place_id = boxinfo[box_index].grasp_release_points[item_index][grasp_point_index].release_place_id;
  int32_t result;
  getBoxIndex(release_place_id, &to_box_index);
  setStowPlaceMoveStatus(true);
  if (RSP_E_OK != Exec_PlaceMove(to_box_index, item_index, box_index, plan_id, result))
  {
    ROS_INFO("Place item(itemID:%d) in box_%d  to box_%d failed", item_index, box_index, to_box_index);
    setStowPlaceMoveStatus(false);

    // 次に実行中のPlanに対する処理
    if (result == t2_msgs::ArmPlaceMoveResult::ITEM_ERROR || result == t2_msgs::ArmPlaceMoveResult::PICK_ERROR)
    {
      std::lock_guard<std::mutex> lock(plan_recog_status_mutex_);
      arm_place_move_result_ = result;
      if (PLAN_NOTYET == boxinfo[box_index].plan_status)
      {
        // 次の計画中なら
        boxinfo[box_index].plan_status = PLAN_NG;
      }
      else if (PLAN_OK == boxinfo[box_index].plan_status)
      {
        // 次の計画済みなら
        if (result == t2_msgs::ArmPlaceMoveResult::PICK_ERROR)
        {
          // 認識フラグON
          setRecogStatus(box_index, RECOG_OK);
        }
        else if (result == t2_msgs::ArmPlaceMoveResult::ITEM_ERROR)
        {
          // 再認識
          resetRecognize(box_index);
          executeRecognition(place_id, RecognitionTypes::Item);
        }
        // 再プラン
        boxinfo[box_index].plan_status = PLAN_NOTYET;
      }
    }

    updateStowInfoByPlanError(place_id, item_index, gp_pos);

    //--------------------
    // 運転停止判定(GAP4)
    //--------------------
    if (received_halt_command_)
    {
      Change_MainStage(D_STAGE_HALT);
      return;
    }

    if (getFailedGPList().size() > max_failed_gp_count_)
    {
      // 失敗GPリストのクリア
      clearFailedGPList();
      recognition_retry_count_ = 0;
    }

    // PLAN_ERRORの場合は再認識
    if (result == t2_msgs::ArmPlaceMoveResult::PLAN_ERROR)
    {
      resetRecognize(box_index);
      executeRecognition(place_id, RecognitionTypes::Item);
    }
    setStowStartPlaceID(place_id);
    Change_MainStage(D_STAGE_StowOrder);
    printAllBoxinfo();
    console_out_boxinfo();
    return;
  }
  updateBoxInfoByMove(box_index, item_index, to_box_index, TaskType::Stow);

  // 失敗GPリストのクリア
  clearFailedGPList();

  // item_location_file.jsonに出力
  ROS_INFO("Output item_location_file.json");
  writeItemLocationJson();

  //--------------------
  // 運転停止判定(RAP4)
  //--------------------
  if (received_halt_command_)
  {
    Change_MainStage(D_STAGE_HALT);
    return;
  }

  if (getTargetItemCount(place_id) != 0)
  {
    executeRecognition(boxinfo[to_box_index].box_id, RecognitionTypes::Octomap);
    ROS_INFO("executeRecognition(place_id=%d) done", boxinfo[to_box_index].box_id);
  }

  ROS_INFO("Stow Success (box_index=%d, item_index=%d, to_box_index=%d)", box_index, item_index, to_box_index);
  console_out("\n★ Stow成功 (%s: %s ⇒ %s)\n\n\n", boxinfo[box_index].item_name[item_index].c_str(),
                t_boxNameId[boxinfo[box_index].box_id].box_name, t_boxNameId[boxinfo[to_box_index].box_id].box_name);

  Change_MainStage(D_STAGE_StowOrder);

  printAllBoxinfo();
  console_out_boxinfo();
}

void Exec_BinAllRecognize(bool flag)
{
  uint box_index;
  printf("\n");
  ROS_INFO("Exec_BinAllRecognize()");

  for (int i = 0; i < boxinfo.size(); i++)
  {
    // Temporal
    if (boxinfo[i].box_name < "G" || boxinfo[i].box_name > "I")
    {
      continue;
    }

    if (RECOG_NOTYET == boxinfo[i].recog_status)
    {
      Exec_BinRecognize(i, flag);
    }
  }
  return;
}

uint Exec_BinRecognize(uint box_index, bool flag)
{
  int CmdCode, i;

  printf("\n");
  ROS_INFO("Exec_BinRecognize(box_index=%d, flag=%d) start", box_index, flag);

  if (boxinfo[box_index].cad_id.size() == 0)
  {
    ROS_INFO("No need to recognize.  box_index(%d) has no items", box_index);
    return RSP_E_NG;
  }

  if (boxinfo[box_index].recog_status == RECOG_OK)
  {
    ROS_INFO("Already recognized. box_index(%d)", box_index);
    return RSP_E_OK;
  }

  captreq_seq_no++;
  captreq_job_no++;

  snd_captreq_msg.seq_no = captreq_seq_no;
  snd_captreq_msg.job_no = captreq_job_no;
  snd_captreq_msg.cam_id = boxinfo[box_index].camera_id;

  printf("<captreq> seq_no:%d job_no:%d, cam_id:%d \n", snd_captreq_msg.seq_no, snd_captreq_msg.job_no,
         boxinfo[box_index].box_id);
  evt_recoPC_captres_cmdrcv = false;
  CmdCode = 1;

  if (RSP_E_OK != snd_topic_msg_to_recoPC_captreq(snd_captreq_msg))
  {
    ROS_INFO("send /cap_request failed");
    return RSP_E_NG;
  }

  if (RSP_E_OK != rcv_topic_msg_from_recoPC_captres(&CmdCode, &evt_recoPC_captres_cmdrcv, RSP_TIME_LIMIT))
  {
    ROS_INFO("receive /cap_request failed");
    // capture error//....
    return RSP_E_NG;
  }

  boxinfo[box_index].item_recog_jobno = captreq_job_no;

  captreq_seq_no++;

  snd_recoreq_msg.seq_no = captreq_seq_no;
  snd_recoreq_msg.job_no = captreq_job_no;
  snd_recoreq_msg.cad_id.clear();
  snd_recoreq_msg.category_id.clear();

  for (i = 0; i < boxinfo[box_index].cad_id.size(); i++)
  {
    if ((boxinfo[box_index].status[i] != ITEM_PICKED) && (boxinfo[box_index].cad_id[i] > 0))
    {
      snd_recoreq_msg.cad_id.push_back(boxinfo[box_index].cad_id[i]);
      // snd_recoreq_msg.category_id.push_back(GetCategoryID_from_itemlist(boxinfo[box_index].cad_id[i]));
      snd_recoreq_msg.category_id.push_back(0);  // TODO:
    }
  }

  if (snd_recoreq_msg.cad_id.size() == 0)
  {
    ROS_INFO("No need to request recognize.  box_index(%d) has no items", box_index);
    return RSP_E_NG;
  }

  printf("<recog_req> seq_no:%d job_no:%d, ", snd_recoreq_msg.seq_no, snd_recoreq_msg.job_no);
  for (i = 0; i < snd_recoreq_msg.cad_id.size(); i++)
  {
    printf("i=%d, cadid=%d categoryid=%d\n", i, snd_recoreq_msg.cad_id[i], snd_recoreq_msg.category_id[i]);
  }

  ROS_INFO("<recog_req>job_num=%d, box_id=%d itemcount=%lu", snd_recoreq_msg.job_no, boxinfo[box_index].box_id,
           snd_recoreq_msg.cad_id.size());
  console_out("\n★ 認識ＰＣに認識要求[Job=%d %s](itemcount=%lu)を送信\n", snd_recoreq_msg.job_no,
              t_boxNameId[boxinfo[box_index].box_id].box_name, snd_recoreq_msg.cad_id.size());

  if (RSP_E_OK != snd_topic_msg_to_recoPC_recoreq(snd_recoreq_msg))
  {
    ROS_INFO("send /req_recognize failed");
    // capture error//....
    return RSP_E_NG;
  }

  if (flag == false)
  {
    ROS_INFO(" Exec_BinRecognize success");

    return RSP_E_OK;
  }

  evt_recoPC_recores_cmdrcv = false;
  CmdCode = 1;
  if (RSP_E_OK != rcv_topic_msg_from_recoPC_recores(&CmdCode, &evt_recoPC_recores_cmdrcv, RSP_TIME_LIMIT))
  {
    ROS_INFO("receive /res_request failed");
    // capture error//....
    return RSP_E_NG;
  }

  boxinfo[box_index].recog_status = RECOG_OK;

  ROS_INFO(" Exec_BinRecognize success");

  return RSP_E_OK;
}

uint executeRecognition(uint place_id, RecognitionType type)
{
  ROS_INFO("executeRecognition(place_id=%u, RecognitionType=%d)", place_id, type);
  Boxinfo_t* box_info;
  if (!getBoxInfo(place_id, &box_info))
  {
    ROS_INFO("Invalid place_id=%u", place_id);
    return RSP_E_NG;
  }

  if (RecognitionTypes::Item == type)
  {
    // 認識済みの場合
    if (box_info->recog_status == RECOG_OK)
    {
      ROS_INFO("Already recognized item. name=%s", box_info->box_name.c_str());
      return RSP_E_OK;
    }

    // アイテムがない場合
    int item_count = 0;
    for (int i = 0; i < box_info->cad_id.size(); ++i)
    {
      if (ITEM_PICKED != box_info->status[i] && ITEM_NORMAL == box_info->item_recog_status[i])
      {
        item_count++;
      }
    }
    if (item_count == 0)
    {
      ROS_INFO("No need to recognize. %s has no items", box_info->box_name.c_str());
      box_info->recog_status = RECOG_OK;
      return RSP_E_NG;
    }
  }
  else if (RecognitionTypes::Octomap == type)
  {
    // 認識済みの場合
    if (box_info->octomap_status == RECOG_OK)
    {
      ROS_INFO("Already recognized octomap. name=%s", box_info->box_name.c_str());
      return RSP_E_OK;
    }
  }
  else if (RecognitionTypes::ItemAndOctomap == type)
  {
    // TODO
  }

  t2_robot_msgs::CaptureReq req;
  {
    std::lock_guard<std::mutex> lock(seq_no_mutex_);
    captreq_seq_no++;
    req.seq_no = captreq_seq_no;
  }
  {
    std::lock_guard<std::mutex> lock(job_no_mutex_);
    captreq_job_no++;
    req.job_no = captreq_job_no;
  }
  req.cam_id = getCameraID(place_id);
  if (RecognitionTypes::Item == type)
  {
    std::lock_guard<std::mutex> lock(item_recog_jobno_mutex_);
    box_info->item_recog_jobno = req.job_no;
  }
  else
  {
    std::lock_guard<std::mutex> lock(octomap_recog_jobno_mutex_);
    box_info->octomap_recog_jobno = req.job_no;
  }
  recog_no_to_type_.insert( { req.job_no, type });

  ROS_INFO("<capture_req> seq_no=%d job_no=%d, cam_id=%d", req.seq_no, req.job_no, req.cam_id);
  ROS_ASSERT(req.cam_id > 0);
  Ctrl_recoPC_captreq_msg.publish(req);

  ROS_INFO("executeRecognition(place_id=%u, RecognitionType=%d) done", place_id, type);
  return RSP_E_OK;
}

void executeAllRecognition(RecognitionType type)
{
  ROS_INFO("executeAllRecognition()");
  for (int i = 0; i < boxinfo.size(); i++)
  {
    if (!boxinfo[i].enabled || BoxTypes::Bin != boxinfo[i].type)
    {
      continue;
    }
    executeRecognition(boxinfo[i].box_id, type);
  }
  return;
}

void waitRecognitionTimeout(uint place_id, RecognitionType type, ros::Time base_time)
{
  Boxinfo_t *box_info;
  getBoxInfo(place_id, &box_info);

  ros::Time start = base_time;
  if (RecognitionType::Item == type)
  {
    while (RECOG_OK != box_info->recog_status)
    {
      if (!task_active_)
      {
        return;
      }
      if (ros::Time::now() - start > ros::Duration(wait_item_recognition_timeout_))
      {
        console_out("\n★ アイテム認識タイムアウト : %s\n", box_info->box_name.c_str());
        ROS_INFO("Recognition item timeout. name=%s, timeout=%d",
            box_info->box_name.c_str(), wait_item_recognition_timeout_);
        {
          std::lock_guard<std::mutex> lock(item_recog_mutex_);
          box_info->capture_item_allowed = true;
        }
        executeRecognition(place_id, RecognitionTypes::Item);
        start = ros::Time::now();
      }
      ros::Duration(0.1).sleep();
    }
  }
  else if (RecognitionType::Octomap == type)
  {
    while (RECOG_OK != box_info->octomap_status)
    {
      if (!task_active_)
      {
        return;
      }
      if (ros::Time::now() - start > ros::Duration(wait_octomap_recognition_timeout_))
      {
        console_out("\n★ 箱詰認識タイムアウト : %s\n", box_info->box_name.c_str());
        ROS_INFO("Recognition octomap timeout. name=%s, timeout=%d",
            box_info->box_name.c_str(), wait_octomap_recognition_timeout_);
        {
          std::lock_guard<std::mutex> lock(octomap_recog_mutex_);
          box_info->capture_octomap_allowed = true;
        }
        executeRecognition(place_id, RecognitionTypes::Octomap);
        start = ros::Time::now();
      }
      ros::Duration(0.1).sleep();
    }
  }
}

uint Exec_GraspPack(uint place_id, uint item_index, const std::vector<uint>& to_place_id)
{
  printf("\n");
  ROS_INFO("Exec_GraspPack(place_id=%u, item_index=%u) start", place_id, item_index);
  if (true != wait_operating(&mainDebug_mode, 1))
  {
    Change_MainStage(D_STAGE_HALT);
    return RSP_E_NG;
  }

  Boxinfo_t *box_info;
  if (!getBoxInfo(place_id, &box_info))
  {
    ROS_ERROR("Invalid place_id=%u", place_id);
    return RSP_E_NG;
  }
  ItemStatus item_status = box_info->status[item_index];
  if (ITEM_RECOG_OK != item_status)
  {
    ROS_ERROR("Exec_GraspPack() Invalid item recognition status %d", item_status);
    return RSP_E_NG;
  }

  std::vector<GpRpInfo> gprp_info;

  uint cad_id = box_info->cad_id[item_index];
  geometry_msgs::Pose pose = box_info->item_data[item_index].pose;
  GpRpArray gprp_array;
  if (!getGpRpWithoutPrediction(cad_id, box_info->item_data[item_index].single_gp, pose, to_place_id, &gprp_array))
  {
    ROS_INFO("getGpRpWithoutPrediction(cad_id=%u, to_place_id.size()=%lu) fail", cad_id, to_place_id.size());
    return RSP_E_ATTN;
  }

  if (gprp_array.size() == 0)
  {
    ROS_WARN("No grasp and release point to pick");
    return RSP_E_ATTN;
  }

  uint box_index, to_box_index;
  getBoxIndex(place_id, &box_index);
  updateGraspAndRelease(box_index, item_index, gprp_array);

  if (boxinfo[box_index].grasp_release_points[item_index].size() == 0)
  {
    ROS_WARN("No grasp and release point over score 0");
    return RSP_E_ATTN;
  }

  ROS_INFO("Exec_GraspPack() end");
  return RSP_E_OK;
}

uint Exec_PickPlan(uint box_index, uint item_index, const int32_t& plan_id,
    bool permit_protrusion, uint& to_box_index)
{
  ROS_INFO("Exec_PickPlan(box_index=%u, item_index=%u, plan_id=%d)",
      box_index, item_index, plan_id);

  ros::NodeHandle private_config_nh("~/task_planner_configs");

  int plan_result = RSP_E_ATTN;

  Boxinfo_t &pick_boxinfo = boxinfo[box_index];

  console_out("\n★ PickPlan開始 : %s\n  GAP1(%s) ⇒ GAP2 ⇒ GP\n",
      pick_boxinfo.item_name[item_index].c_str(), t_boxNameId[pick_boxinfo.box_id].box_name);

  const GpRpArray& grasp_release_points = pick_boxinfo.grasp_release_points[item_index];
  const GpNumberArray& failed_gp_numbers = pick_boxinfo.failed_gp_numbers[item_index];

  uint start_index = pick_boxinfo.failed_gp_numbers[item_index].size() == 0 ? 0 : pick_boxinfo.grasp_point_index[item_index] + 1;

  for (std::size_t i = start_index; i < grasp_release_points.size(); i++)
  {
    console_out("  GP[%ld/%ld] score = %f, ", i + 1, grasp_release_points.size(), grasp_release_points[i].score);

    // はみ出しが許可されていない
    if (!permit_protrusion)
    {
      double protrude_length = grasp_release_points[i].protrude_length;
      ROS_INFO("GP[%ld/%ld] protrude length = %f", i + 1, grasp_release_points.size(), protrude_length);
      if (protrude_length > protrusion_threshold_)
      {
        console_out("skip protrude length is %f (threshold=%f)\n", protrude_length, protrusion_threshold_);
        continue;
      }
    }

    // 把持スコア値をチェックする（スコアは昇順にソートされている）
    if (grasp_release_points[i].score < grasp_point_score_threshold_)
    {
      console_out("skip below the threshold(%f)\n", grasp_point_score_threshold_);
      continue;  // 把持スコアなし
    }

    // PickExecute方向違いのGPはPlan対象外
    uint gp_number = grasp_release_points[i].gp_number;
    bool same_gp = false;
    for (std::size_t j = 0; j < failed_gp_numbers.size(); ++j)
    {
      if (gp_number / 100 == failed_gp_numbers[j] / 100)
      {
        same_gp = true;
        break;
      }
    }
    if (same_gp)
    {
      ROS_INFO("reject gp_number = %u", gp_number);
      console_out("skip the failed grasp point (gp_number=%u)\n", gp_number);
      continue;
    }

    console_out("pattern = %s, plan\n", grasp_release_points[i].grasp_pattern.c_str());

    // PickPlan
    t2_msgs::ArmPickPlanGoal pick_plan_goal;
    pick_plan_goal.plan_id = plan_id;
    pick_plan_goal.place_id = pick_boxinfo.box_id;
    pick_plan_goal.cad_id = pick_boxinfo.cad_id[item_index];
    pick_plan_goal.job_no = pick_boxinfo.item_recog_jobno;
    pick_plan_goal.recog_id = pick_boxinfo.recog_index[item_index];
    pick_plan_goal.grasp_point = gprpToGraspPointMsg(grasp_release_points[i]);
    pick_plan_goal.goal.type.type = t2_msgs::ArmPoseType::GROUP_STATE;
    pick_plan_goal.goal.group_state = t_arm_Joint_angle[pick_boxinfo.box_id].posi_name;
    pick_plan_goal.gap2_velocity = arm_speed_gap2_;
    pick_plan_goal.gp_velocity = arm_speed_gp_;
    pick_plan_goal.back_gap1_velocity = arm_speed_back_gap1_;

    geometry_msgs::Pose &gap2_pose = pick_plan_goal.grasp_point.approach_point_item;
    geometry_msgs::Pose &gp_pose = pick_plan_goal.grasp_point.grasp_point_item;

    console_out("   GAP2(x, y, z) = (%f, %f, %f)\n", gap2_pose.position.x, gap2_pose.position.y, gap2_pose.position.z);
    console_out("   GP(x, y, z) = (%f, %f, %f)\n", gp_pose.position.x, gp_pose.position.y, gp_pose.position.z);

    {
      std::lock_guard<std::mutex> lock(arm_pick_plan_mutex_);
      arm_pick_plan_action_client_->sendGoal(pick_plan_goal);
      bool before_timeout = arm_pick_plan_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

      t2_msgs::ArmPickPlanResultConstPtr arm_pick_plan_result = arm_pick_plan_action_client_->getResult();

      if ((arm_pick_plan_result->result == t2_msgs::ArmPickPlanResult::SUCCESS) && before_timeout)
      {
        pick_boxinfo.grasp_point_index[item_index] = i;
        getBoxIndex(grasp_release_points[i].release_place_id, &to_box_index);
        console_out("   => plan success. : plan_time = %0.3f[s], attempt = %d\n", arm_pick_plan_result->planning_time,
            arm_pick_plan_result->planning_attempt);
        console_out("\n★ PickPlan成功 : %s\n  GAP1(%s) ⇒ GAP2 ⇒ GP\n",
            pick_boxinfo.item_name[item_index].c_str(), t_boxNameId[pick_boxinfo.box_id].box_name);
        ROS_INFO("@@@ Exec_PickPlan success");
        plan_result = RSP_E_OK;
        break;
      }
      else
      {
        ROS_INFO("@@@ Exec_PickPlan failed\n");
        console_out("   => plan failed : %s\n", arm_pick_plan_result->error_msg.c_str());
      }
    }
  }

  if (plan_result != RSP_E_OK)
  {
    ROS_INFO("@@@ Exec_PickPlan failed\n");
    console_out("\n★ PickPlan失敗 : %s\n  current ⇒ GAP1(%s) ⇒ GAP2 ⇒ GP\n",
        pick_boxinfo.item_name[item_index].c_str(), t_boxNameId[pick_boxinfo.box_id].box_name);
  }

  return plan_result;
}

uint Exec_PickExecute(uint box_index, uint item_index, const int32_t& plan_id)
{
  console_out("\n★ PickExecute開始 : %s\n  current ⇒ GAP1(%s) ⇒ GAP2 ⇒ GP\n",
                boxinfo[box_index].item_name[item_index].c_str(), t_boxNameId[boxinfo[box_index].box_id].box_name);

  uint result = RSP_E_NG;

  ros::NodeHandle private_config_nh("~/task_planner_configs");

  double lift_up_length = 0.03;  // default
  if (!private_config_nh.getParam("gap_lift_up_length", lift_up_length))
  {
    ROS_WARN("Failed to get gap_lift_up_length. Use dafault value = %f", lift_up_length);
  }

  t2_msgs::ArmPickExecuteGoal pick_execute_goal;

  pick_execute_goal.place_id = boxinfo[box_index].box_id;
  pick_execute_goal.stay_level = true;
  pick_execute_goal.plan_id = plan_id;
  pick_execute_goal.gap1_velocity = arm_speed_gap1_;

  arm_pick_execute_action_client_->sendGoal(pick_execute_goal);
  bool before_timeout = arm_pick_execute_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

  t2_msgs::ArmPickExecuteResultConstPtr arm_pick_execute_result = arm_pick_execute_action_client_->getResult();

  if ((arm_pick_execute_result->result == t2_msgs::ArmPickExecuteResult::SUCCESS) && before_timeout)
  {
    console_out("\n★ PickExecute成功 : %s\n  current ⇒ GAP1(%s) ⇒ GAP2 ⇒ GP\n",
                boxinfo[box_index].item_name[item_index].c_str(), t_boxNameId[boxinfo[box_index].box_id].box_name);
    ROS_INFO("@@@ Exec_PickExecute success");
    result = RSP_E_OK;
  }
  else
  {
    ROS_INFO("@@@ Exec_PickExecute failed\n");
    console_out("\n★ PickExecute失敗 : %s\n  %s ⇒ AP\n  => Error : %s\n", boxinfo[box_index].item_name[item_index].c_str(),
                t_boxNameId[boxinfo[box_index].box_id].box_name, arm_pick_execute_result->error_msg.c_str());
  }

  return result;
}

//----------------------------------
// プレース実行要求
//----------------------------------
uint Exec_PlaceMove(uint box_index, uint item_index, uint from_box_index, const int32_t& pick_plan_id,
    int32_t& action_result)
{
  int mat_num;
  t2_robot_msgs::Matrix4x4 arm_relese_pose;    //把持のリリース位置姿勢
  t2_robot_msgs::Matrix4x4 relativePoseValue;  //移動量
  t2_msgs::ArmPose goal_pose;
  geometry_msgs::Point rap2_point, rp_point;
  geometry_msgs::Pose rap1_pose, rap3_pose;
  uint result;
  std::string plan_group;
  ros::NodeHandle nh;
  ros::NodeHandle private_config_nh("~/task_planner_configs");
  bool relative = true;
  std::vector<geometry_msgs::Pose> waypoints;

  if (true != wait_operating(&mainDebug_mode, 1))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;  // 停止
  }
  printf("\n");
  ROS_INFO("<<< [Exec_PlaceMove] stage start >>>");

  mat_num = boxinfo[from_box_index].grasp_point_index[item_index];
  const GpRp_t& grasp_release_point = boxinfo[from_box_index].grasp_release_points[item_index][mat_num];
  t2_msgs::GraspPoint grasp_point = gprpToGraspPointMsg(grasp_release_point);

  std::string item_name = boxinfo[from_box_index].item_name[item_index];
  console_out("\n★ PlaceMove開始 : %s\n  current ⇒ GAP3 ⇒ GAP4(GAP1)(%s) ⇒ RAP1(%s) ⇒ RAP2 ⇒ RP ⇒ RAP3 ⇒ RAP4\n  pattern = %s\n",
      item_name.c_str(), t_boxNameId[boxinfo[from_box_index].box_id].box_name,
      t_boxNameId[boxinfo[box_index].box_id].box_name, grasp_point.grasp_pattern.c_str());

  if (grasp_point.grasp_pattern == "suction")
  {
    // 吸着
    plan_group = "arm_suction";
  }
  else if (grasp_point.grasp_pattern == "pinch")
  {
    // 挟持
    plan_group = "arm_pinch";
  }
  else
  {
    ROS_ERROR("Unknown grasp_pattern %s", grasp_point.grasp_pattern.c_str());
    return RSP_E_NG;
  }

  t2_msgs::ArmPose ap1_state_pose;
  snd_srv_msg_GetArmGroupStatePose(t_arm_Joint_angle[boxinfo[box_index].box_id].posi_name, ap1_state_pose, plan_group);

  rap1_pose = ap1_state_pose.pose;  // 現在位置姿勢(RAP1)の姿勢を保持

  use_dummy_pack_ = false;

  if (use_dummy_pack_)
  {
    // RAP1(現在位置姿勢)からx,y,z方向にconfigで指定した距離を移動したRPの算出
    std::vector<double> diff_vector;
    XmlRpc::XmlRpcValue diff_length;
    if (!private_config_nh.getParam("rap1_rp_diff_length", diff_length))
    {
      diff_vector.push_back(0);
      diff_vector.push_back(0);
      diff_vector.push_back(-0.2);  // default
      ROS_WARN("Failed to get rap1_rp_diff_length. Use dafault value (dx, dy, dz) = (%lf, %lf, %lf)", diff_vector[0],
               diff_vector[1], diff_vector[2]);
    }
    else
    {
      ROS_ASSERT(diff_length.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int box_location = 1;
      if (box_index > g_bin_count + 1 && box_index < boxinfo.size())
      {
        box_location = box_index - g_bin_count - 1;
      }

      uint place_id = boxinfo[box_index].box_id;
      for (int i = 0; i < diff_length.size(); i++)
      {
        if (static_cast<int>(diff_length[i]["place_id"]) == place_id)
        {
          // Box内のアイテム数に応じてindexを設定
          int index = getCurrentItemCount(box_index) % diff_length[i]["diff"].size();
          //int index = boxinfo[box_index].item_name.size() % diff_length.size();
          XmlRpc::XmlRpcValue diff = diff_length[i]["diff"][index];
          ROS_ASSERT(diff.getType() == XmlRpc::XmlRpcValue::TypeArray);
          for (int j = 0; j < diff.size(); j++)
          {
            ROS_ASSERT(diff[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            diff_vector.push_back(static_cast<double>(diff[j]));
          }
          break;
        }
      }
    }

    // 現在座標位置(RAP1)に差分距離を加算しBox把持解放点(RP)への座標を取得
    rp_point.x = rap1_pose.position.x + diff_vector[0];
    rp_point.y = rap1_pose.position.y + diff_vector[1];
    rp_point.z = rap1_pose.position.z + diff_vector[2];

    // RAP2の算出(RP+差分座標)
    diff_vector.clear();
    if (!private_config_nh.getParam("rp_rap2_diff_length", diff_length))
    {
      // default
      diff_vector.push_back(0);
      diff_vector.push_back(0);
      diff_vector.push_back(0.1);
      ROS_WARN("Failed to get rp_rap2_diff_length. Use dafault value (dx, dy, dz) = (%lf, %lf, %lf)", diff_vector[0],
               diff_vector[1], diff_vector[2]);
    }
    else
    {
      ROS_ASSERT(diff_length.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int i = 0; i < diff_length.size(); i++)
      {
        ROS_ASSERT(diff_length[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        diff_vector.push_back(static_cast<double>(diff_length[i]));
      }
      // Box把持解放点(RP)に差分座標を加算
      rap2_point.x = rp_point.x + diff_vector[0];
      rap2_point.y = rp_point.y + diff_vector[1];
      rap2_point.z = rp_point.z + diff_vector[2];
    }
  }
  else
  {
    rap2_point = grasp_release_point.rap.position;
    rp_point = grasp_release_point.rp.position;
  }

  double rap_lift_up_length = 0.03;  // z軸方向の差分(default)
  if (!private_config_nh.getParam("rap_lift_up_length", rap_lift_up_length))
  {
    ROS_WARN("Failed to get gap_lift_up_length. Use dafault value = %f", rap_lift_up_length);
  }
  ROS_INFO("RP to RAP3 rap_lift_up_length: %lf", rap_lift_up_length);

  // 吸着位置(現在位置姿勢)からZ方向にconfigで指定した距離移動
  double lift_up_length = 0.03;  // default
  if (!private_config_nh.getParam("gap_lift_up_length", lift_up_length))
  {
    ROS_WARN("Failed to get gap_lift_up_length. Use dafault value = %f", lift_up_length);
  }

  t2_msgs::ArmPlaceMoveGoal place_move_goal;

  place_move_goal.goal_place_id = boxinfo[box_index].box_id;

  // set gp_lift_up_length
  place_move_goal.gp_lift_up_length = lift_up_length;

  // set GAP4(GAP1)
  place_move_goal.gap4_goal.type.type = t2_msgs::ArmPoseType::GROUP_STATE;
  place_move_goal.gap4_goal.group_state = t_arm_Joint_angle[boxinfo[from_box_index].box_id].posi_name;

  // set RAP1
  place_move_goal.rap1_goal.type.type = t2_msgs::ArmPoseType::GROUP_STATE;
  place_move_goal.rap1_goal.group_state = t_arm_Joint_angle[boxinfo[box_index].box_id].posi_name;

#if 0
  // set RAP2
  place_move_goal.rap2_pose.orientation = rap1_pose.orientation;
  place_move_goal.rap2_pose.position = rap2_point;

  // set RP
  place_move_goal.rp_pose.orientation = rap1_pose.orientation;
  place_move_goal.rp_pose.position = rp_point;
#else
  // set RAP2
  if (use_dummy_pack_)
  {
    place_move_goal.rap2_pose.orientation = rap1_pose.orientation;
  }
  else
  {
    place_move_goal.rap2_pose.orientation = grasp_release_point.rap.orientation;
  }
  place_move_goal.rap2_pose.position = rap2_point;

  // set RP
  if (use_dummy_pack_)
  {
    place_move_goal.rp_pose.orientation = rap1_pose.orientation;
  }
  else
  {
    place_move_goal.rp_pose.orientation = grasp_release_point.rp.orientation;
  }
  place_move_goal.rp_pose.position = rp_point;
#endif

  // set rp_lift_up_length
  place_move_goal.rp_lift_up_length = rap_lift_up_length;

  place_move_goal.stay_level = true;

  place_move_goal.gap3_velocity = arm_speed_gap3_ * grasp_point.carry_speed;
  place_move_goal.gap4_velocity = arm_speed_gap4_ * grasp_point.carry_speed;
  place_move_goal.rap1_velocity = arm_speed_rap1_ * grasp_point.carry_speed;
  place_move_goal.rap2_velocity = arm_speed_rap2_ * grasp_point.carry_speed;
  place_move_goal.rp_velocity = arm_speed_rp_ * grasp_point.carry_speed;
  place_move_goal.back_rap2_velocity = arm_speed_back_rap2_;
  place_move_goal.back_rap1_velocity = arm_speed_back_rap1_;

  console_out("  GAP3 lift_up = %f\n", lift_up_length);
  console_out("  RAP1(x, y, z) = (%f, %f, %f)\n", rap1_pose.position.x, rap1_pose.position.y, rap1_pose.position.z);
  console_out("  RAP2(x, y, z) = (%f, %f, %f)\n", rap2_point.x, rap2_point.y, rap2_point.z);
  console_out("  RP(x, y, z) = (%f, %f, %f)\n", rp_point.x, rp_point.y, rp_point.z);
  console_out("  RAP3 lift_up = %f\n", rap_lift_up_length);

  arm_place_move_action_client_->sendGoal(place_move_goal, NULL, NULL, &cbArmPlaceMoveActionFeedback);
  bool before_timeout = arm_place_move_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

  t2_msgs::ArmPlaceMoveResultConstPtr arm_place_move_result = arm_place_move_action_client_->getResult();
  action_result = arm_place_move_result->result;
  ROS_INFO("ArmPlaceMoveResult result=%d", action_result);

  if ((arm_place_move_result->result == t2_msgs::ArmPlaceMoveResult::SUCCESS) && before_timeout)
  {
    console_out("\n★ PlaceMove成功 : %s\n  => plan_time = %0.3f[s], attempt = %d\n", item_name.c_str(),
        arm_place_move_result->planning_time, arm_place_move_result->planning_attempt);
  }
  else
  {
    console_out("\n★ PlaceMove失敗 : %s\n  %s\n", item_name.c_str(), arm_place_move_result->error_msg.c_str());
    return RSP_E_NG;  //異常終了
  }

  result = RSP_E_OK;  //正常終了
  return result;
}

bool operator<(const GpRpInfo& left, const GpRpInfo& right)
{
  return left.gprp.gp.position.z > right.gprp.gp.position.z;
}

std::string getStowStartPositionName()
{
  uint start_place_id = getStowStartPlaceID();
  std::string name;
  if (start_place_id != 0)
  {
    name = "RAP4("+ std::string(t_boxNameId[start_place_id].box_name) + ")";
  }
  else
  {
    name = "current";
  }
  return name;
}

uint Exec_StowPlan(uint box_index, ItemRecogStatus item_recog_status, const std::vector<uint>& place_id_list,
    bool below_protrusion)
{
  ROS_INFO("Exec_StowPlan(box_index=%u, item_recog_status=%d, place_id_list.size()=%lu, below_protrusion=%d)",
      box_index, item_recog_status, place_id_list.size(), below_protrusion);

  ROS_INFO("failed GP size = %lu", getFailedGPList().size());

  int plan_result = RSP_E_ATTN;
  Boxinfo_t& box_info = boxinfo[box_index];

  std::vector<uint> item_index_array = getItemIndexList(box_index, item_recog_status, item_probability_threshold_);

  if (item_index_array.size() == 0)
  {
    return RSP_E_NG;
  }

  std::string display_to_place_id = "[ ";
  for (std::size_t i = 0; i < place_id_list.size(); ++i)
  {
    std::ostringstream oss;
    oss << place_id_list[i];
    display_to_place_id.append(oss.str() + " ");
  }
  display_to_place_id.append("]");
  if (ITEM_NORMAL == item_recog_status)
  {
    console_out("\n  予測あり把持&箱詰(%s ⇒ %s), はみ出し条件(%d)\n", t_boxNameId[boxinfo[box_index].box_id].box_name,
        display_to_place_id.c_str(), below_protrusion);
  }
  else
  {
    console_out("\n  予測なし把持&箱詰(%s ⇒ %s)\n", t_boxNameId[boxinfo[box_index].box_id].box_name,
        display_to_place_id.c_str());
  }

  std::vector<GpRpInfo> gprp_info;

  for (auto it = item_index_array.begin(); it != item_index_array.end(); ++it)
  {
    uint index = *it;
    uint cad_id = box_info.cad_id[index];
    uint single_gp = box_info.item_data[index].single_gp;
    geometry_msgs::Pose pose = box_info.item_data[index].pose;
    GpRpArray gprp_array;

#ifdef USE_GRASP_RELEASE_WITH_PREDICTION
    if (ITEM_NORMAL == item_recog_status)
    {
      // misrecognition/unknown以外のアイテムで予測あり把持・箱詰計画
      std::vector<uint> grasp_place_id_list = { boxinfo[box_index].box_id };
      std::vector<Location_t> grasp_location_list = getLocationList(grasp_place_id_list);

      // 現在のアイテムはcad_id_listから除外
      if (grasp_location_list.size() > 0)
      {
        std::vector<uint>& cad_id_list = grasp_location_list[0].cad_id_list;
        std::vector<uint>::iterator cad_it = std::find(cad_id_list.begin(), cad_id_list.end(), cad_id);
        if (cad_it != cad_id_list.end())
        {
          cad_id_list.erase(cad_it);
        }
        else
        {
          ROS_WARN("No cad_id=%u in cad_id_list", cad_id);
        }
      }
      else
      {
        ROS_WARN("grasp_location_list.size()=0");
      }

      std::vector<Location_t> release_location_list = getLocationList(place_id_list);

      // 現在carry中のアイテムがある
      if (whileStowPlaceMove())
      {
        uint move_place_id = getStowStartPlaceID();
        for (std::size_t i = 0; i < release_location_list.size(); ++i)
        {
          Location_t location = release_location_list[i];
          if (location.place_id == move_place_id)
          {
            uint cad_id = box_info.cad_id[getStowItemIndex()];
            location.cad_id_list.push_back(cad_id);
            break;
          }
        }
      }
      if (!getGpRpWithPrediction(cad_id, single_gp, pose, grasp_location_list, release_location_list, place_id_list, &gprp_array))
      {
        // TODO
        ROS_INFO("getGpRpWithPrediction(cad_id=%u, place_id_list.size()=%lu) fail", cad_id, place_id_list.size());
        continue;
      }
    }
    else
    {
      // misrecognition/unknownアイテムで予測なし把持・箱詰計画
      if (!getGpRpWithoutPrediction(cad_id, single_gp, pose, place_id_list, &gprp_array))
      {
        // TODO
        ROS_INFO("getGpRpWithoutPrediction(cad_id=%u, place_id_list.size()=%lu) fail", cad_id, place_id_list.size());
        continue;
      }
    }
#else
    if (!getGpRpFix(cad_id, single_gp, pose, place_id_list, &gprp_array))
    {
      // TODO
      ROS_INFO("getGpRpFix(cad_id=%u, place_id_list.size()=%lu) fail", cad_id, place_id_list.size());
      continue;
    }
#endif

    for (auto gprp_it = gprp_array.begin(); gprp_it != gprp_array.end(); ++gprp_it)
    {
      GpRpInfo info = { index, *gprp_it };
      gprp_info.push_back(info);
    }
    updateGraspAndRelease(box_index, index, gprp_array);
  }

  if (gprp_info.size() == 0)
  {
    ROS_WARN("No grasp and release point to stow");
    return plan_result;
  }

  uint start_place_id = getStowStartPlaceID();

  // 把持点のz座標を基に降順にソート
  std::sort(gprp_info.begin(), gprp_info.end());

  for (int i = 0; i < gprp_info.size(); ++i)
  {
    const GpRp_t& gprp = gprp_info[i].gprp;
    uint item_index = gprp_info[i].index;

    console_out("  GP[%ld/%ld] score = %f, z = %f, %s, ", i + 1, gprp_info.size(),
        gprp.score, gprp_info[i].gprp.gp.position.z, box_info.item_name[item_index].c_str());

    // 失敗GPと一致する場合は除外
    std::vector<geometry_msgs::Point> failed_gp_list = getFailedGPList();
    const geometry_msgs::Point& point = gprp.gp.position;
    bool failed = false;
    for (std::size_t j = 0; j < failed_gp_list.size(); ++j)
    {
      const geometry_msgs::Point& failed_point = failed_gp_list[j];
      double distance = sqrt(pow(point.x - failed_point.x, 2) + pow(point.y - failed_point.y, 2) + pow(point.z - failed_point.z, 2));
      if (distance < failed_gp_distance_threshold_)
      {
        failed = true;
        break;
      }
    }
    if (failed)
    {
      console_out("skip the failed grasp point\n");
      continue;
    }

    // 把持点のスコアが閾値未満は除外
    if (gprp.score < grasp_point_score_threshold_)
    {
      console_out("skip below the threshold(%f)\n", grasp_point_score_threshold_);
      continue;
    }

    // はみ出し量の閾値による判定
    if (below_protrusion != (gprp.protrude_length <= protrusion_threshold_))
    {
      std::string below_or_over = below_protrusion ? "over" : "below";
      console_out("skip %s protrusion threshold\n", below_or_over.c_str(), protrusion_threshold_);
      continue;
    }

    console_out("pattern = %s, plan\n", gprp_info[i].gprp.grasp_pattern.c_str());

    int32_t plan_id = getPlanID(box_index, item_index);

    // GraspAndReleasePoint
    int gp_index = -1;
    for (int j = 0; j < box_info.grasp_release_points[item_index].size(); ++j)
    {
      if (gprp.gp_number == box_info.grasp_release_points[item_index][j].gp_number)
      {
        gp_index = j;
        break;
      }
    }
    if (gp_index < 0)
    {
      ROS_INFO("No grasp point(gp_number=%u) in box info", gprp.gp_number);
      // TODO
    }

    // StowPlan
    t2_msgs::ArmPickPlanGoal pick_plan_goal;
    pick_plan_goal.plan_id = plan_id;
    pick_plan_goal.place_id = box_info.box_id;
    pick_plan_goal.cad_id = box_info.cad_id[item_index];
    pick_plan_goal.job_no = box_info.item_recog_jobno;
    pick_plan_goal.recog_id = box_info.recog_index[item_index];

    // GraspPoint
    pick_plan_goal.grasp_point = gprpToGraspPointMsg(gprp);
    pick_plan_goal.goal.type.type = t2_msgs::ArmPoseType::GROUP_STATE;
    pick_plan_goal.goal.group_state = t_arm_Joint_angle[box_info.box_id].posi_name;
    pick_plan_goal.gap2_velocity = arm_speed_gap2_;
    pick_plan_goal.gp_velocity = arm_speed_gp_;
    pick_plan_goal.back_gap1_velocity = arm_speed_back_gap1_;

    geometry_msgs::Pose &gap2_pose = pick_plan_goal.grasp_point.approach_point_item;
    geometry_msgs::Pose &gp_pose = pick_plan_goal.grasp_point.grasp_point_item;

    console_out("   GAP2(x, y, z) = (%f, %f, %f)\n", gap2_pose.position.x, gap2_pose.position.y, gap2_pose.position.z);
    console_out("   GP(x, y, z) = (%f, %f, %f)\n", gp_pose.position.x, gp_pose.position.y, gp_pose.position.z);

    arm_pick_plan_action_client_->sendGoal(pick_plan_goal);
    bool before_timeout = arm_pick_plan_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

    t2_msgs::ArmPickPlanResultConstPtr arm_pick_plan_result = arm_pick_plan_action_client_->getResult();

    if ((arm_pick_plan_result->result == t2_msgs::ArmPickPlanResult::SUCCESS) && before_timeout)
    {
      setStowItemIndex(item_index);
      box_info.grasp_point_index[item_index] = gp_index;
      console_out("   => plan success. : plan_time = %0.3f[s], attempt = %d\n", arm_pick_plan_result->planning_time, arm_pick_plan_result->planning_attempt);
      console_out("\n★ StowPlan成功 : %s\n  GAP1(%s) ⇒ GAP2 ⇒ GP\n", box_info.item_name[item_index].c_str(), t_boxNameId[box_info.box_id].box_name);
      ROS_INFO("@@@ Exec_StowPlan success");
      plan_result = RSP_E_OK;
      setStowStartPlaceID(gprp.release_place_id);
      break;
    }
    else
    {
      ROS_INFO("@@@ Exec_StowPlan failed\n");
      console_out("   => plan failed : %s\n", arm_pick_plan_result->error_msg.c_str());
    }
  }

  /*
  if (plan_result != RSP_E_OK)
  {
    console_out("\n★ StowPlan失敗 : \n  %s ⇒ GAP1(%s) ⇒ GAP2 ⇒ GP\n",
        getStowStartPositionName().c_str(), t_boxNameId[box_info.box_id].box_name);
  }
  */

  return plan_result;
}

uint Exec_StowPlan(uint box_index)
{
  console_out("\n★ StowPlan開始\n  GAP1(%s) ⇒ GAP2 ⇒ GP\n", t_boxNameId[boxinfo[box_index].box_id].box_name);

  ros::Duration duration(0.1);

  ItemRecogStatus item_recog_status = ITEM_NORMAL;
  bool below_protrusion = true;

  while (true)
  {
    // Octomap認識済みのBinリストを取得
    std::vector<uint> place_id_array = getRecognizedBinID();

    uint ret = Exec_StowPlan(box_index, item_recog_status, place_id_array, below_protrusion);
    if (RSP_E_OK == ret)
    {
      return RSP_E_OK;
    }
    else if (RSP_E_ATTN == ret)
    {
      // 格納先Bin候補が全Binでない場合
      if (place_id_array.size() != getBinCount())
      {
        ROS_INFO("Wait all bin recongnition to stow");
        ros::Time base_time = ros::Time::now();
        for (std::size_t i = 0; i < boxinfo.size(); ++i)
        {
          if (!boxinfo[i].enabled || BoxTypes::Bin != boxinfo[i].type)
          {
            continue;
          }
          waitRecognitionTimeout(boxinfo[i].box_id, RecognitionType::Octomap, base_time);
        }
        continue;
      }

      // はみ出し量の条件あり
      if (below_protrusion)
      {
        ROS_INFO("Change protrusion condition");
        below_protrusion = false;
        continue;
      }
    }
    else
    {
      // 条件を満たすアイテムが存在しない
    }

    // 認識リトライ上限以内
    if (recognition_retry_count_ < max_recognition_retry_count_)
    {
      // 失敗GPリストのクリア
      clearFailedGPList();

      recognition_retry_count_++;
      console_out("\n★ 認識リトライ[%u/%u]\n", recognition_retry_count_, max_recognition_retry_count_);
      ROS_INFO("Stow recognition retry [%u/%u]", recognition_retry_count_, max_recognition_retry_count_);
      return RSP_E_NG;
    }

    // 認識リトライカウンタをリセット
    recognition_retry_count_ = 0;

    break;
  }

  // AmnestyToteの箱詰認識待ち
  waitRecognitionTimeout(D_tote_2_ID, RecognitionType::Octomap, ros::Time::now());

  // misrecogアイテム
  std::vector<uint> amnesty_tote_id = { D_tote_2_ID };
  if (RSP_E_OK == Exec_StowPlan(box_index, ITEM_MISRECOG, amnesty_tote_id, true))
  {
    return RSP_E_OK;
  }

  // unknownアイテム
  if (RSP_E_OK == Exec_StowPlan(box_index, ITEM_UNKNOWN, amnesty_tote_id, true))
  {
    return RSP_E_OK;
  }

  // 失敗GPリストのクリア
  clearFailedGPList();

  console_out("\n★ StowPlan失敗 : \n  GAP1(%s) ⇒ GAP2 ⇒ GP\n", t_boxNameId[boxinfo[box_index].box_id].box_name);

  return RSP_E_NG;
}

t2_msgs::GraspPoint gprpToGraspPointMsg(const GpRp_t& gprp)
{
  t2_msgs::GraspPoint gp;
  gp.gp_number = gprp.gp_number;
  gp.score = gprp.score;
  gp.grasp_pattern = gprp.grasp_pattern;
  gp.suction_strength = gprp.suction_strength;
  gp.threshold_of_vacuum_for_suction = gprp.threshold_of_vacuum_for_suction;
  gp.carry_speed = gprp.carry_speed;
  gp.width_between_finger_for_pinch = gprp.width_between_finger_for_pinch;
  gp.width_between_finger_for_release = gprp.width_between_finger_for_release;
  gp.finger_intrusion_for_pinch = gprp.finger_intrusion_for_pinch;
  gp.max_effort_for_pinch = gprp.max_effort_for_pinch;
  gp.grasp_point_item = gprp.gp;
  gp.approach_point_item = gprp.gap;
  return gp;
}

//----------------------------------
// 実行終了要求
//----------------------------------
uint Exec_StopCtrl(void)
{
  uint result;
  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;  // 停止
  }
  console_out("\n");
  ROS_INFO("<<< [Exec_StopCtrl] stage start >>>");

  result = RSP_E_OK;  //正常終了をセットしておく（エラーでも一通り実行する）

  //ハンド吸着設定サービス：吸着バルブをＯＦＦする
  console_out("★ 吸着バルブＯＦＦ\n");
  if (RSP_E_OK != snd_srv_msg_GripperSuction(false))
  {
    result = RSP_E_NG;  //異常終了
  }

  //ハンド吸着ポンプ設定サービス：真空ポンプをＯＦＦする
  console_out("★ 真空ポンプＯＦＦ\n");
  if (RSP_E_OK != snd_srv_msg_GripperPump(false))
  {
    result = RSP_E_NG;  //異常終了
  }

  console_out("★ PinchingGripper終了\n");
  if (RSP_E_OK != snd_srv_msg_PinchingGripperFinalize())
  {
    result = RSP_E_NG;  //異常終了
  }

  // ロボット状態設定サービス（KEBAを停止する）
  console_out("★ ロボットコントローラ停止\n");
  if (RSP_E_OK != snd_srv_msg_KebaFinalize())
  {
    result = RSP_E_NG;  //異常終了
  }

#ifdef USE_WEIGHT_SCALE
  // 重量計停止
  if (RSP_E_OK != snd_srv_msg_WeightScaleStop())
  {
    result = RSP_E_NG;  //異常終了
  }
#endif

  return result;
}

//----------------------------------
// アーム単体移動実行テスト要求
//----------------------------------
uint Exec_TestArmMove(T_PlanMainCmdMsg cmd_msg)
{
  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;  // 停止
  }
  printf("\n");
  ROS_INFO("<<< [Exec_MoveOnlyArm][code=%d][data=%d] start >>>", cmd_msg.sub_cmd_code, cmd_msg.sub_cmd_data);
  if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_TEST_MENU)  //ステート名指定による移動プラン作成＆実行（PlanID保存）
  {
    // ctrl側でメニュー表示し実行する
    //メインメニュー 8:ステート名指定実行 9:ファイル指定実行(プラン作成＆実行) 10:ファイル指定実行(プラン作成のみ)
    if (cmd_msg.sub_cmd_data == 11)
    {
      sub_menu_MoveOnlyArm(cmd_msg);  //-->ステート名指定実行メニュー
    }
    /*
    else if (cmd_msg.sub_cmd_data == 9)
    {
      sub_menu_ArmMoveFilePose(cmd_msg, 0);  //-->ファイル指定実行メニュー
    }
    else if (cmd_msg.sub_cmd_data == 10)
    {
      sub_menu_ArmMoveFilePose(cmd_msg, 1);  //-->ファイル指定実行メニュー
    }
    t_MainCtrlRsp.rsp_result = RSP_E_OK;  //メインへの応答終了コード
    */
  }
  else
  {
    //メインからコマンド直接指定
    t_MainCtrlRsp.rsp_result = moveArmCmd_exe(cmd_msg);  //アーム単体移動処理実行
  }
  return RSP_E_OK;
}

//****************************************************
//  コマンド実行呼び出し関数
//****************************************************
//
//----------------------------------
// アーム単体移動実行メニュー
//----------------------------------
void sub_menu_MoveOnlyArm(T_PlanMainCmdMsg cmd_msg)
{
  int iKeyIndata = 0;
  char strKeyin[80];
  int ret;
  int ii;
  int result;
  T_PlanMainCmdMsg ctrl_cmd_msg = cmd_msg;  //メインからのコマンドコピー

  system("roslaunch t2_database load_order.launch pick_task:=true");
  // 各テーブルの初期化
  initializeBoxInfo(TaskType::Pick);
  createBoxNameIdTable();
  createPlanPosiNameTable();
  creteArmJointAngleTable();

  order_task = 0;
  if (RSP_E_OK != Exec_OperationStart())
  {
    return;
  }

  //テスト実行
  for (;;)
  {
    console_out("\n★ テスト用アーム単体移動指定(現在位置から目標位置への移動)\n");
    for (ii = 0; ii < t_arm_Joint_angle.size(); ii++)
    {
      console_out("   %2d. %s\n", ii, t_arm_Joint_angle[ii].posi_name);
    }
    console_out("\n★ 目標位置番号入力( Number = 0-%d / end=99 ) ?", (t_arm_Joint_angle.size() - 1));
    ret = console_keyIn_int(&iKeyIndata);  //番号入力
    if (ret != 1)
    {
      console_out("scanf error=%d\n", ret);
      continue;  //再入力
    }
    if ((0 <= iKeyIndata) && (iKeyIndata < t_arm_Joint_angle.size()))
    {
      console_out("\n★ < %s > に移動しますか( y / n ) ?", t_arm_Joint_angle[iKeyIndata].posi_name);
      ret = console_keyIn_str(strKeyin);  //文字入力
      if (ret != 1)
      {
        console_out("scanf error\n");
        continue;
      }
      if (strKeyin[0] != 'y')
      {
        continue;  //再入力
      }

      //アーム動作実行指示
      ctrl_cmd_msg.cmd_code = D_CMD_MoveOnlyArm;
      ctrl_cmd_msg.sub_cmd_code = D_MoveOnly_ARM_STATE_PLAN;  //ステート名指定(PlanID保存)
      ctrl_cmd_msg.sub_cmd_data = 0;                          //本コマンドではデータ指定なし
      ctrl_cmd_msg.arm_state_index = iKeyIndata;              //ステート名指示インデックス
      ctrl_cmd_msg.velocity = arm_speed_high_;               //速度
      result = moveArmCmd_exe(ctrl_cmd_msg);                  //アーム単体処理コマンド実行
      if (RSP_E_OK != result)
      {
        console_out("アーム移動動作異常終了\n");
      }
      continue;  //再入力
    }
    else if (iKeyIndata == 99)  //終了選択
    {
      break;
    }
    else
    {
      console_out("num error\n");
      continue;  //再入力
    }
  }

  Exec_OperationEnd();

  return;
}
//----------------------------------
// アーム指定位置移動（ファイル指定）
//
//※とりあえず機能追加、追加で作っていたら長くなってしまった。
//
//----------------------------------
//
void sub_menu_ArmMoveFilePose(T_PlanMainCmdMsg cmd_msg, int arm_exec_mode)
{
  int ii, jj, kk, ret;
  char strKeyin[80];
  int iKeyIndata = 0;
  int arm_move_type;
  int arm_move_step_type;
  int arm_move_step_return_mode;
  bool nonStopFlag;
  int nonStop_LoopCount;
  int nonStop_LoopCountMax;
  bool result;
  FILE *fp_input_movePose = NULL;
  FILE *fp_output_plan_tim = NULL;
  FILE *fp_output_plan_trj = NULL;
  char work_sbuf[D_csvlineMax];
  char work_string[100];
  char *ary[32];
  int aryOffset;
  int file_read_result;
  uint arm_pose_index;
  int arm_exec_fileNo;
  int plan_count;
  int move_step_count_index;
  int current_move_step_count_index;
  int plan_result;
  int move_result;
  int arm_retry_Count;
  bool plan_goal_pose_set = false;  //プラン作成用の前回移動終了位置or初期位置設定済フラグ
  char trjoutFileName_tim[128];
  char trjoutFileName_trj[128];
  int joint_num;                            //関節角数
  t2_msgs::ArmPose current_arm_pose;        //アームの現在位置姿勢
  T_PlanMainCmdMsg ctrl_cmd_msg = cmd_msg;  //メインからのコマンドコピー

  geometry_msgs::Pose curr_pose;         //重心座標
  t2_robot_msgs::Matrix4x4 tmp_pose;  //一時保存用座標
  const std::string plan_group = "arm";

  //---------------------------------------
  // アーム動作ファイル指定移動メインループ
  for (;;)  //メニュー終了、動作異常で抜ける
  {
    //---------------------------------------
    // 動作指定ファイルのパス名を取得選択する
    result = GetFile_SeqMoveDataFilePath();  // inputFilePath,
// outputFilePathにファイルパスを取得する；デフォルトパスはarm_data
#if 0  //ファイルないときはデフォルトパスを使用する
    if (result != true)    //ファイルの登録なし
    {
      console_out("パス指定ファイルが見つかりません。またはデータ異常です\n");
      console_out("pause(any keyin) ?");
      console_keyIn_int(&iKeyIndata);    //番号入力(dummy)
      return;
    }
#endif
    //---------------------------------------
    // 動作指定ファイルを選択する
    console_out("\n★ テスト用ファイル指定アーム移動（ %s ）\n",
                ((arm_exec_mode == 0) ? "アーム単体実機移動" : "プラン作成のみ"));
    InputSeqMoveFileCount = 0;
    result = GetFile_SeqMoveFileList(&inputFilePath[0], &InputSeqMoveFileCount);  // filelistファイル名取得
    if ((result != true) || (InputSeqMoveFileCount == 0))                         //ファイルの登録なし
    {
      if (result != true)
      {
        console_out("リストファイルが見つかりません。またはデータ異常です\n");
      }
      else
      {
        console_out("ファイル名リストが空です[%d]\n", InputSeqMoveFileCount);
      }
      console_out("pause(any keyin) ?");
      console_keyIn_int(&iKeyIndata);  //番号入力(dummy)
      return;
    }
    arm_exec_fileNo = 0;
    if (InputSeqMoveFileCount != 1)  //ファイル複数個なら選択問い合わせする
    {
      for (ii = 0; ii < InputSeqMoveFileCount; ii++)
      {
        console_out("   %2d. %s\n", ii + 1, seqArmMove_FileName[ii]);
      }
      console_out("\n★ ファイル番号入力( Number = 1-%d / end=99 ) ?", InputSeqMoveFileCount);
      ret = console_keyIn_int(&iKeyIndata);  //番号入力
      if (ret != 1)
      {
        console_out("scanf error=%d\n", ret);
        continue;  //再入力
      }
      if ((0 < iKeyIndata) && (iKeyIndata <= InputSeqMoveFileCount))
      {
        arm_exec_fileNo = iKeyIndata - 1;
      }
      else if (iKeyIndata == 99)  //終了選択
      {
        return;
      }
      else
      {
        console_out("num error\n");
        continue;  //再入力
      }
    }
    //---------------------------------------
    // 動作指定ファイル名の設定と表示
    printf("\n*---- Arm Move pose file ----*\n");
    //フォルダファイル指定
    sprintf(inputFileListnameAll, "%s%s", inputFilePath,
            seqArmMove_FileName[arm_exec_fileNo]);  // target input file name

    console_out(" < file name=[ %s ] >\n", seqArmMove_FileName[arm_exec_fileNo]);
    printf(" < input file name all=[ %s ] >\n", inputFileListnameAll);

    console_out("\n★ PLAN結果ファイル出力 ( all= a / time= t / non= n /end= e ) ?",
                t_arm_Joint_angle[iKeyIndata].posi_name);
    ret = console_keyIn_str(strKeyin);  //文字入力
    if (ret != 1)
    {
      console_out("scanf error\n");
      continue;
    }
    if ((strKeyin[0] == 'a') || (strKeyin[0] == 't'))
    {
      GetCurrDateTime(work_string);  //現在時刻取得
      MakeTrjOutputFileName(&seqArmMove_FileName[arm_exec_fileNo][0], trjoutFileName_tim, work_string,
                            const_cast<char *>("tim"));  //結果出力ファイル名生成
      // Plan作成時間結果ファイル出力
      sprintf(outputFileListnameAll, "%s%s", outputFilePath, trjoutFileName_tim);  // target output file name
      if ((fp_output_plan_tim = fopen(outputFileListnameAll, "w")) == NULL)
      {
        console_out("\n@@@ file open error[ %s ]\n", outputFileListnameAll);
        ROS_ERROR("@@@ file open error[ %s ]", outputFileListnameAll);
        continue;  //再入力
      }
      else
      {
        console_out("< output file name=[ %s ] >\n", trjoutFileName_tim);
        printf(" < output file name all=[ %s ] >\n", outputFileListnameAll);
        fprintf(fp_output_plan_tim, "fileName,%s\n", seqArmMove_FileName[arm_exec_fileNo]);  //１行ファイル出力
      }
      if (strKeyin[0] == 'a')
      {
        //軌道結果ファイル出力
        MakeTrjOutputFileName(&seqArmMove_FileName[arm_exec_fileNo][0], trjoutFileName_trj, work_string,
                              const_cast<char *>("trj"));                            //結果出力ファイル名生成
        sprintf(outputFileListnameAll, "%s%s", outputFilePath, trjoutFileName_trj);  // target output file name
        if ((fp_output_plan_trj = fopen(outputFileListnameAll, "w")) == NULL)
        {
          console_out("\n@@@ file open error[ %s ]\n", outputFileListnameAll);
          ROS_ERROR("@@@ file open error[ %s ]", outputFileListnameAll);
          continue;  //再入力
        }
        else
        {
          console_out("< output file name=[ %s ] >\n", trjoutFileName_trj);
          printf(" < output file name all=[ %s ] >\n", outputFileListnameAll);
          fprintf(fp_output_plan_trj, "fileName,%s\n", seqArmMove_FileName[arm_exec_fileNo]);  //１行ファイル出力
        }
      }
      //⇒ファイル出力あり実行
    }
    else if (strKeyin[0] == 'n')
    {
      fp_output_plan_tim = NULL;
      fp_output_plan_trj = NULL;
      //⇒ファイル出力なし実行
    }
    else if (strKeyin[0] == 'e')
    {
      return;  //実行終了
    }
    else
    {
      continue;  //再入力
    }

    //---------------------------------------
    nonStop_LoopCountMax = 1;  //連続実行回数
    console_out("\n★ ステップ実行 or 連続実行 ( step= s / non_stop= n / end= e ) ?",
                t_arm_Joint_angle[iKeyIndata].posi_name);
    ret = console_keyIn_str(strKeyin);  //文字入力
    if (ret != 1)
    {
      console_out("scanf error\n");
      continue;
    }
    if (strKeyin[0] == 'n')
    {
      if (arm_exec_mode != 0)  //連続実行でプラン作成時のみ問い合わせ
      {
        console_out("\n★ プラン作成 繰返し回数 = ?", InputSeqMoveFileCount);
        ret = console_keyIn_int(&nonStop_LoopCountMax);  //回数入力
        if (ret != 1)
        {
          console_out("scanf error=%d\n", ret);
          continue;  //再入力
        }
      }
      else
      {
        nonStop_LoopCountMax = 1;  //連続実行回数
      }
      nonStopFlag = true;  //連続実行
    }
    else if (strKeyin[0] == 's')
    {
      nonStopFlag = false;  //ステップ実行
    }
    else if (strKeyin[0] == 'e')
    {
      if (InputSeqMoveFileCount == 1)
      {
        return;  //実行終了
      }
      continue;  //再入力
    }
    else
    {
      continue;  //再入力
    }
    /*
        //---------------------------------------
        // 現在のアーム位置姿勢取得する（plan_armの出力座標を見て参考にする）
        if (GetArmPose_client.call(GetArmPose_srv) == true)
        {
          if (GetArmPose_srv.response.result == t2_msgs::GetArmPoseResponse::SUCCESS)
          {
            printf("<current ArmMatrix>\n");
            for (ii = 0; ii < 16; ++ii)
            {
              printf("%.4f",GetArmPose_srv.response.pose.matrix.m[ii]);
              if ((ii+1)%4 == 0)
              {
                printf("\n");
              }
              else
              {
                printf(", ");
              }
            }
            printf("<current ArmJoint>\n");  //姿勢取得時の先頭Joint[0]は未使用？
            for (ii = 1; ii < GetArmPose_srv.response.pose.joint_positions.size(); ++ii)
            {
              printf("[%d]%.4f ", ii, GetArmPose_srv.response.pose.joint_positions[ii]);
            }
            printf("\n\n");
          }
        }
    */
    //---------------------------------------
    //プラン作成のみの場合、指定回数の繰り返しあり
    for (nonStop_LoopCount = 0; nonStop_LoopCount < nonStop_LoopCountMax; nonStop_LoopCount++)
    {
      //アーム姿勢座標ファイル読込み
      printf("input file open = [ %s ] >\n", seqArmMove_FileName[arm_exec_fileNo]);
      if ((fp_input_movePose = fopen(inputFileListnameAll, "r")) == NULL)
      {
        console_out("\n@@@ file open error[ %s ]\n", inputFileListnameAll);
        ROS_ERROR("@@@ file open error[ %s ]", inputFileListnameAll);
        file_read_result = 2;
        break;
      }
      // 動作指定ファイルからステート名、座標を読込み実行する
      file_read_result = 0;
      memset(&t_SeqArmMoveParam, 0, sizeof(t_SeqArmMoveParam));
      kk = 1;                                                            // Line counter
      plan_count = 0;                                                    //今回プラン作成回数
      move_step_count_index = 0;                                         //ターゲット座標Index
      while (fgets(work_sbuf, D_csvlineMax, fp_input_movePose) != NULL)  // １行読み込み（ファイルEOFまで）
      {
        //---------------------------------------
        // １行読み込み、動作モードを解析する
        arm_move_type = 0;                //アーム動作タイプ指定クリア
        arm_move_step_type = 0;           //開始位置指定クリア
        arm_move_step_return_mode = 0;    // Homeへの戻りフラグ
        ary[0] = strtok(work_sbuf, ",");  // 文字列(char配列)をカンマで分割する:先頭列[0]
        if (strcmp("group_state", ary[0]) == 0)
        {
          arm_move_type = D_MoveOnly_ARM_STATE;  //ステート名指定
        }
        else if (strcmp("cartesian", ary[0]) == 0)
        {
          arm_move_type = D_MoveOnly_ARM_CARTESIAN;  //座標指定
        }
        else if (strcmp("joint", ary[0]) == 0)
        {
          arm_move_type = D_MoveOnly_ARM_JOINT;  //関節角指定
        }
        if (arm_move_type != 0)  //どれか？
        {
          //---------------------------------------
          // 列毎に読込み
          move_step_count_index++;         //ターゲットファイルIndex
          ary[1] = strtok(NULL, ",");      // name
          ary[2] = strtok(NULL, ",\r\n");  //速度
          if (ary[1] == NULL || !strlen(ary[1]))
          {
            console_out("@@@ pose_name null error\n");
            ROS_ERROR("@@@ pose_name null error");
            file_read_result = 2;
            break;
          }
          //４列目の移動ステップ指定
          aryOffset = 3;                           //読込み列
          ary[aryOffset] = strtok(NULL, ",\r\n");  //スタート位置指定
          if (ary[aryOffset] != NULL && strlen(ary[aryOffset]))
          {
            strcpy(&work_string[0], ary[aryOffset]);  //ステップ指定
            if (strcmp(work_string, "home") == 0)
            {
              printf("\n-->to home position\n");
              arm_move_step_type = 1;  //開始位置
              aryOffset++;             //次列
            }
            else if (strcmp(work_string, "ret") == 0)
            {
              printf("\n-->to next position & ret\n");
              arm_move_step_type = 2;         //実行後、Homeへの戻り有り
              arm_move_step_return_mode = 1;  // Homeへの戻りフラグ
              aryOffset++;                    //次列
            }
            else if (strcmp(work_string, "init") == 0)
            {
              printf("\n-->set init position\n");
              arm_move_step_type = 3;  //初期位置
              aryOffset++;             //次列
            }
            else if (strcmp(work_string, "next") == 0)
            {
              printf("\n-->to next position\n");
              arm_move_step_type = 0;  //実行後、次ステップ
              aryOffset++;             //次列
            }
            else
            {
              printf("\n-->next position(default)\n");
              arm_move_step_type = 0;  //実行後、次ステップ
            }
          }
          else
          {  // null
            printf("\n-->next position(default)\n");
            arm_move_step_type = 0;  //実行後、次ステップ
            aryOffset = 0;           //次の行を読み込む
          }
          //---------------------------------------
          // アーム動作指定タイプ別のファイル入力データ解析
          if (arm_move_type == D_MoveOnly_ARM_STATE)  //ステート名指定
          {
            //開始ステート名が登録されているかチェック
            strcpy(&work_string[0], ary[1]);  // state Name
            arm_pose_index = D_posID_MAX;
            for (ii = 0; ii < D_posID_MAX; ii++)
            {
              if (strcmp(t_arm_Joint_angle[ii].posi_name, work_string) == 0)
              {
                arm_pose_index = ii;  //一致有り
              }
            }
            if (arm_pose_index >= D_posID_MAX)
            {
              console_out("\n@@@ group_state name error[ %s ]\n", work_string);
              ROS_ERROR("@@@ group_state name error[ %s ]", work_string);
              file_read_result = 2;
              break;
            }
            //ステート名指定への移動設定
            t_SeqArmMoveParam.moveTarget.state_name_index = arm_pose_index;
            t_SeqArmMoveParam.moveTarget.velocity = atof(ary[2]);  //速度
            printf("state_name=< %s >,v=%.2f\n",
                   t_arm_Joint_angle[t_SeqArmMoveParam.moveTarget.state_name_index].posi_name,
                   t_SeqArmMoveParam.moveTarget.velocity);
          }
          else if (arm_move_type == D_MoveOnly_ARM_CARTESIAN)  //座標指定
          {
            //座標指定への移動設定
            strcpy(&t_SeqArmMoveParam.moveTarget.Name[0], ary[1]);  // target pose Name
            t_SeqArmMoveParam.moveTarget.velocity = atof(ary[2]);   //速度
            printf("[%d]< %s >,v=%.2f\n", move_step_count_index, t_SeqArmMoveParam.moveTarget.Name,
                   t_SeqArmMoveParam.moveTarget.velocity);

            // アーム座標マトリクス読み込み
            jj = 0;  //アーム座標インデックス
            while (file_read_result == 0)
            {
              if (aryOffset == 0)
              {
                kk++;                                                           // Line counter
                if (fgets(work_sbuf, D_csvlineMax, fp_input_movePose) != NULL)  // １行読み込み（ファイルEOFまで）
                {
                  aryOffset = 0;
                  ary[aryOffset] = strtok(work_sbuf, ",\r\n");  // 文字列(char配列)をカンマで分割する:先頭列[0]
                  if (ary[aryOffset] == NULL || !strlen(ary[aryOffset]))
                  {
                    console_out("\n@@@ arm pose matrix null data error poseNo=[%d]\n", move_step_count_index);
                    ROS_ERROR("@@@ arm pose matrix null data error poseNo=[%d]", move_step_count_index);
                    file_read_result = 2;
                    break;
                  }
                }
              }
              else
              {
                if ((aryOffset != 3) || (jj != 0))  //読込み列が先頭の場合でステップ指定がないときのみスキップ
                {
                  aryOffset++;
                  ary[aryOffset] = strtok(NULL, ",\r\n");  //値指定or\(改行マーク)
                }
                else
                {
                }
              }
              if (ary[aryOffset] == NULL || !strlen(ary[aryOffset]))
              {
                //データなし再読み込み
                aryOffset = 0;
              }
              else
              {
                if (ary[aryOffset] != 0)  //数字チェック
                {
                  tmp_pose.m[jj] = atof(ary[aryOffset]);  //アーム座標
                                                          //                  printf("%.4f",tmp_pose.m[jj]);

                  jj++;
                  if (jj >= 16)
                  {
                    t_SeqArmMoveParam.moveTarget.ArmPose.pose = t2_robot_msgs::matrix4x4ToPoseMsg(tmp_pose);
                    break;  //読込み終了
                  }
                  //                  else if (jj%4 == 0)
                  //                  {
                  //                    printf("\n");
                  //                  }
                  //                  else
                  //                  {
                  //                    printf(", ");
                  //                  }
                }
                else
                {
                  console_out("\n@@@ arm pose matrix data error poseNo=[%d]\n", move_step_count_index);
                  ROS_ERROR("@@@ arm pose matrix data error poseNo=[%d]", move_step_count_index);
                  file_read_result = 2;
                  break;
                }
                aryOffset++;  //次列
              }

              kk++;
              if (kk >= 100000)  //無限ループ防止
              {
                file_read_result = 1;
                break;
              }
            }
            printf("\n");
            if (file_read_result != 0)  //座標読取り中エラー
            {
              break;  //読取り中断
            }
            if (t_SeqArmMoveParam.moveTarget.ArmPose.pose.orientation.w == 0)  //座標データチェック
            {
              console_out("\n@@@ arm pose [orientation.w == 0] data error poseNo=[%d]\n", move_step_count_index);
              ROS_ERROR("@@@ arm pose [orientation.w == 0] data error poseNo=[%d]", move_step_count_index);
              file_read_result = 2;
              break;
            }
          }
          else if (arm_move_type == D_MoveOnly_ARM_JOINT)  //関節角指定
          {
            //関節角指定の移動設定
            strcpy(&t_SeqArmMoveParam.moveTarget.Name[0], ary[1]);  // target pose Name
            t_SeqArmMoveParam.moveTarget.velocity = atof(ary[2]);   //速度
            printf("[%d]< %s >,v=%.2f\n", move_step_count_index, t_SeqArmMoveParam.moveTarget.Name,
                   t_SeqArmMoveParam.moveTarget.velocity);

            // 関節角読み込み
            jj = 0;  //関節角インデックス
            t_SeqArmMoveParam.moveTarget.ArmPose.joint_positions.resize(0);
            joint_num = GetArmPlanTrj_srv.response.joint_trajectory.joint_names.size();
            while (file_read_result == 0)
            {
              if (aryOffset == 0)
              {
                kk++;                                                           // Line counter
                if (fgets(work_sbuf, D_csvlineMax, fp_input_movePose) != NULL)  // １行読み込み（ファイルEOFまで）
                {
                  aryOffset = 0;
                  ary[aryOffset] = strtok(work_sbuf, ",\r\n");  // 文字列(char配列)をカンマで分割する:先頭列[0]
                  if (ary[aryOffset] == NULL || !strlen(ary[aryOffset]))
                  {
                    console_out("\n@@@ arm joint null data error poseNo=[%d]\n", move_step_count_index);
                    ROS_ERROR("@@@ arm joint null data error poseNo=[%d]", move_step_count_index);
                    file_read_result = 2;
                    break;
                  }
                }
              }
              else
              {
                if ((aryOffset != 3) || (jj != 0))  //読込み列が先頭の場合でステップ指定がないときのみスキップ
                {
                  aryOffset++;
                  ary[aryOffset] = strtok(NULL, ",\r\n");  //値指定or\(改行マーク)
                }
              }
              if (ary[aryOffset] == NULL || !strlen(ary[aryOffset]))
              {
                //データなし再読み込み
                aryOffset = 0;
              }
              else
              {
                if (ary[aryOffset] != 0)  //数字チェック
                {
                  t_SeqArmMoveParam.moveTarget.ArmPose.joint_positions.resize(jj + 1);
                  t_SeqArmMoveParam.moveTarget.ArmPose.joint_positions[jj] = atof(ary[aryOffset]);  //関節角
                  printf("%.4f, ", t_SeqArmMoveParam.moveTarget.ArmPose.joint_positions[jj]);

                  jj++;
                  if (jj >= joint_num)
                  {
                    break;  //読込み終了
                  }
                }
                else
                {
                  console_out("\n@@@ arm joint data error poseNo=[%d]\n", move_step_count_index);
                  ROS_ERROR("@@@ arm joint data error poseNo=[%d]", move_step_count_index);
                  file_read_result = 2;
                  break;
                }
                aryOffset++;  //次列
              }

              kk++;
              if (kk >= 100000)  //無限ループ防止
              {
                file_read_result = 1;
                break;
              }
            }
            printf("\n");
          }
          else
          {
            file_read_result = 1;  //論理エラー
            break;
          }
          t_SeqArmMoveParam.moveTarget.arm_move_type = arm_move_type;  //動作指定タイプ保存（ステート/座標/ジョイント）
          t_SeqArmMoveParam.moveTarget.move_step_count_index = move_step_count_index;  //現在の実行ステップ

          //移動ステップ指定のチェック
          if (arm_move_step_type == 3)  //初期位置
          {
            if (arm_exec_mode == 0)  //アーム実機動作実行有り
            {
              console_out("\n-->実機移動での初期位置指定[init]は、[Home]指定として動作します\n");
              arm_move_step_type = 1;  //開始位置に設定変更する
            }
            else
            {
              //移動ステップが初期位置で目標動作指定タイプがJointの場合、アーム現在位置をこの初期位置に設定する
              // Plan_ctrlが保持している現在位置を設定するだけなので、アーム動作には関係なし
              if (arm_move_type == D_MoveOnly_ARM_JOINT)  //関節角指定
              {
                printf("initialize setting joint_position[%d]< %s >\n", move_step_count_index,
                       t_SeqArmMoveParam.moveTarget.Name);
                console_out("\n-->[%d] 初期位置設定< %s >\n", move_step_count_index, t_SeqArmMoveParam.moveTarget.Name);
                ctrl_cmd_msg.cmd_code = D_CMD_MoveOnlyArm;
                ctrl_cmd_msg.sub_cmd_code = D_MoveOnly_ARM_JOINT;  //関節角指定
                ctrl_cmd_msg.sub_cmd_data = 3;  //内部保存の移動終了位置を指定の関節角に設定
                ctrl_cmd_msg.velocity = 0;      //速度
                ctrl_cmd_msg.ArmPose.joint_positions = t_SeqArmMoveParam.moveTarget.ArmPose.joint_positions;  //関節角
                //コマンド送信
                plan_result = moveArmCmd_exe(ctrl_cmd_msg);  //アーム単体処理コマンド実行
                if (plan_result != RSP_E_OK)
                {
                  console_out("アーム初期位置設定異常\n");
                  file_read_result = 2;
                  break;
                }
                kk++;                                                              // Line counter
                t_SeqArmMoveParam.init_moveTarget = t_SeqArmMoveParam.moveTarget;  //初期位置の目標座標等保存
                plan_goal_pose_set = true;  //プラン作成用の初期位置設定済フラグ
                continue;                   //正常で次のステップ
              }
              else
              {
                //ステート名指定と座標指定の場合は、アーム初期位置を設定できないのでhomeとして扱う
                //最初に動かすときはアーム現在位置（初期位置）からの移動になる
                //ステート名から関節角がわかるはずだが・・・やり方知らない
                console_out("\n-->Joint指定以外の初期位置指定[init]は、[Home]指定として動作します\n");
                arm_move_step_type = 1;  //開始位置に設定変更する
              }
            }
          }
          // 移動ステップ指定が開始位置指定であれば、戻り動作のため目標動作位置を保存しておく
          if (arm_move_step_type == 1)
          {
            t_SeqArmMoveParam.start_moveTarget = t_SeqArmMoveParam.moveTarget;  //開始位置の目標座標等保存
          }

          //---------------------------------------
          // 実行ループ：ステップ指定の"home"と"next"は１回で抜ける。"ret"は、Homeへの戻りで２回で抜ける。
          current_move_step_count_index = move_step_count_index;  //現在のステップカウントは保持
          for (;;)
          {
            for (;;)  // 続行キー入力待ち
            {
              //---------------------------------------
              // 目標位置の表示
              console_out("\n-->[%d]", current_move_step_count_index);
              if (arm_move_step_type == 1)  //開始位置指定
              {
                console_out(" 原点");
              }
              else if (arm_move_step_return_mode == 2)  //戻り移動
              {
                console_out(" 戻り");
              }
              else if (arm_move_step_return_mode == 3)  //戻り移動
              {
                console_out(" 初期");
              }
              else
              {
                console_out(" 次点");
              }
              // type
              if (arm_move_type == D_MoveOnly_ARM_STATE)
              {
                console_out("s");
              }
              else if (arm_move_type == D_MoveOnly_ARM_CARTESIAN)
              {
                console_out("c");
              }
              else if (arm_move_type == D_MoveOnly_ARM_JOINT)
              {
                console_out("j");
              }
              else
              {
                console_out(" ");
              }
              // name
              if (t_SeqArmMoveParam.moveTarget.arm_move_type == D_MoveOnly_ARM_STATE)  //ステート名指定
              {
                console_out("< %s (%.2f)>", t_arm_Joint_angle[t_SeqArmMoveParam.moveTarget.state_name_index].posi_name,
                            t_SeqArmMoveParam.moveTarget.velocity);
              }
              else
              {
                console_out("< %s (%.2f)>", t_SeqArmMoveParam.moveTarget.Name, t_SeqArmMoveParam.moveTarget.velocity);
              }
              //---------------------------------------
              // 実行開始の確認キー入力
              if (nonStopFlag == true)
              {
                console_out("\n");
                strKeyin[0] = 's';  //連続実行
                break;              // Loop Out
              }
              else
              {
                console_out("に移動 ( step= s / pass= p / end= e ) ?");
                ret = console_keyIn_str(strKeyin);  //文字入力
                if (ret != 1)
                {
                  console_out("scanf error\n");
                  continue;
                }
                if ((strKeyin[0] == 's') || (strKeyin[0] == 'p') || (strKeyin[0] == 'e'))
                {
                  break;  // Loop Out
                }
                console_out("key-in error\n");
              }
            }
            if (strKeyin[0] == 's')
            {
              arm_retry_Count = 0;  //リトライカウント
              for (;;)
              {
                //---------------------------------------
                // アーム実機動作有無の設定 (0:実行有り 1:プラン作成(現在位置) 2:プラン作成(前回移動位置))
                if (arm_exec_mode == 0)  //アーム動作実行有り
                {
                  //                  ctrl_cmd_msg.sub_cmd_data = 0;    //動作実行
                  ctrl_cmd_msg.sub_cmd_data = 1;  //プラン作成(開始位置：現在位置)→実機は作成後にPlan実行する
                }
                else
                {
                  if (plan_goal_pose_set == true)  //プラン作成用の移動終了位置 or 初期位置設定済フラグ
                  {
                    ctrl_cmd_msg.sub_cmd_data = 2;  //プラン作成(開始位置：前回移動終了位置)
                  }
                  else
                  {
                    ctrl_cmd_msg.sub_cmd_data = 1;  //プラン作成(開始位置：現在位置)
                  }
                }
                //アーム動作指示コマンド設定
                ctrl_cmd_msg.cmd_code = D_CMD_MoveOnlyArm;
                ctrl_cmd_msg.sub_cmd_code = t_SeqArmMoveParam.moveTarget.arm_move_type;  //移動指定タイプ
                ctrl_cmd_msg.velocity = t_SeqArmMoveParam.moveTarget.velocity;           //速度
                if (t_SeqArmMoveParam.moveTarget.arm_move_type == D_MoveOnly_ARM_STATE)  //ステート名指定
                {
                  ctrl_cmd_msg.arm_state_index = t_SeqArmMoveParam.moveTarget.state_name_index;  //ステート名
                }
                else if (t_SeqArmMoveParam.moveTarget.arm_move_type == D_MoveOnly_ARM_CARTESIAN)  //座標指定
                {
                  ctrl_cmd_msg.ArmPose.pose = t_SeqArmMoveParam.moveTarget.ArmPose.pose;  //座標
                }
                else if (t_SeqArmMoveParam.moveTarget.arm_move_type == D_MoveOnly_ARM_JOINT)  //関節角指定
                {
                  ctrl_cmd_msg.ArmPose.joint_positions = t_SeqArmMoveParam.moveTarget.ArmPose.joint_positions;  //関節角
                }
                else
                {
                  file_read_result = 1;  //論理エラー
                  break;
                }
                //コマンド送信
                plan_result = moveArmCmd_exe(ctrl_cmd_msg);  //アーム単体処理コマンド実行
                if (plan_result == RSP_E_ATTN)
                {
                  // プラン作成or実行リトライ：連続動作中のみ
                  if (nonStopFlag == true)
                  {
                    arm_retry_Count++;
                    if (arm_retry_Count > 3)
                    {
                      ROS_ERROR("@@@ arm plan failed");
                      console_out("Plan作成失敗しました\n");
                    }
                    else
                    {
                      ROS_WARN("* arm plan failed retry");
                      console_out("* arm plan failed retry(%d/3)\n", arm_retry_Count);
                      continue;  //リトライする：注意続行
                    }
                  }
                  else
                  {
                    console_out("Plan作成失敗しました\n");
                  }
                }
                else if (plan_result != RSP_E_OK)
                {
                  ROS_ERROR("@@@ arm plan error");
                  console_out("アーム動作設定データ異常、または動作異常\n");
                }
                //
                move_result = plan_result;  //動作実行でプラン結果ステータスを変えないようにする
                if (move_result == RSP_E_OK)  //プラン正常終了のとき
                {
                  plan_goal_pose_set = true;  //前回Plan設定済フラグセット
                  //--------------------------------------------
                  //
                  if (arm_exec_mode == 0)  //アーム動作実行有り
                  {
                    ctrl_cmd_msg.cmd_code = D_CMD_MoveOnlyArm;
                    ctrl_cmd_msg.sub_cmd_code = D_MoveOnly_ARM_PLAN_ID;             //移動指定タイプ:Plan実行
                    ctrl_cmd_msg.planID = t_MainCtrlRsp.planID;                     //計画済みPlanID
                    ctrl_cmd_msg.velocity = t_SeqArmMoveParam.moveTarget.velocity;  //速度
                    move_result = moveArmCmd_exe(ctrl_cmd_msg);  //アーム単体処理コマンド実行
                    if (move_result != RSP_E_OK)
                    {
                      ROS_ERROR("@@@ arm excute error");
                      console_out("アーム動作異常\n");
                    }
                  }
                }

                memset(&curr_pose, 0, sizeof(curr_pose));

                if (plan_result == RSP_E_OK)  //プラン正常終了のとき
                {
                  //--------------------------------------------
                  // plan作成結果の関節角を表示する
                  printf("<plan Goal ArmJoint>\n");
                  for (ii = 0; ii < plan_goal_pose.joint_positions.size(); ++ii)
                  {
                    printf("[%d]%.4f ", ii, plan_goal_pose.joint_positions[ii]);
                  }
                  printf("\n");
                  if (arm_exec_mode == 0)  //アーム動作実行有り
                  {
                    t2_msgs::ArmPose arm_pose;
                    if (RSP_E_OK != snd_srv_msg_GetArmPose(arm_pose, plan_group))
                    {
                      curr_pose = arm_pose.pose;
                    }
                  }
                  else
                  {
                    curr_pose = plan_goal_pose.pose;  //プラン作成結果
                  }
                }
                //プラン結果をファイルに保存（実行後の姿勢を保存するので、ここでファイル出力実行）
                if (false == Savefile_plan_log(fp_output_plan_tim, fp_output_plan_trj, t_MainCtrlRsp.planID, &curr_pose,
                                               nonStop_LoopCount, plan_count, current_move_step_count_index,
                                               arm_move_step_type, plan_result))
                {
                  console_out("PLANログ保存異常です\n");
                  file_read_result = 2;
                  break;
                }
                plan_count++;  //プラン作成実行回数
                //
                if (move_result != RSP_E_OK)  //注意、異常終了のとき
                {
                  // プラン作成or実行リトライキー確認：ステップ実行中のみ
                  if (nonStopFlag != true)
                  {
                    for (;;)  // リトライ続行キー入力待ち
                    {
                      console_out("\n★ リトライしますか ( yes= y / pass= p  /end= e ) ?");
                      ret = console_keyIn_str(strKeyin);  //文字入力
                      if (ret != 1)
                      {
                        console_out("scanf error\n");
                        continue;
                      }
                      if ((strKeyin[0] == 'y') || (strKeyin[0] == 'p') || (strKeyin[0] == 'e'))
                      {
                        break;  // Loop Out
                      }
                    }
                    if (strKeyin[0] == 'y')
                    {
                      ROS_WARN("* arm plan failed retry");
                      continue;  //リトライする：注意続行
                    }
                    else if (strKeyin[0] == 'e')
                    {
                      console_out("中断しました\n");
                      file_read_result = 2;
                    }
                    else if (strKeyin[0] == 'p')
                    {
                      // passの場合は戻り実行なし
                      arm_move_step_return_mode = 0;  // Home戻りフラグをクリア
                    }
                  }
                  else
                  {
                    //連続実行時
                    arm_move_step_return_mode = 0;  // Home戻りフラグをクリア
                    file_read_result = 2;           //異常中断
                  }
                }
                break;
              }  // retry loop
            }
            else if (strKeyin[0] != 'p')  // passではない→end
            {
              console_out("中断しました\n");
              file_read_result = 2;
              break;  //再入力
            }
            if (file_read_result != 0)
            {
              break;  //終了
            }

            //---------------------------------------
            //動作終了で、移動ステップ指定がretで、Homeへの戻り有り指定がある場合
            if (arm_move_step_return_mode == 1)  // Homeへの戻りフラグ
            {
              if (t_SeqArmMoveParam.start_moveTarget.arm_move_type != 0)  //アーム動作タイプ設定済み
              {
                current_move_step_count_index = t_SeqArmMoveParam.start_moveTarget.move_step_count_index;
                t_SeqArmMoveParam.moveTarget =
                    t_SeqArmMoveParam.start_moveTarget;  //開始位置の目標座標等 保存データ読込み
                arm_move_step_return_mode++;             // Home戻りフラグ更新実行

                if (t_SeqArmMoveParam.moveTarget.arm_move_type == D_MoveOnly_ARM_STATE)  //ステート名指定
                {
                  printf("\n-->ret to home< %s (%.2f)> position\n",
                         t_arm_Joint_angle[t_SeqArmMoveParam.moveTarget.state_name_index].posi_name,
                         t_SeqArmMoveParam.moveTarget.velocity);
                }
                else
                {
                  printf("\n-->ret to home< %s (%.2f)> position\n", t_SeqArmMoveParam.moveTarget.Name,
                         t_SeqArmMoveParam.moveTarget.velocity);
                }
                continue;
              }
              else
              {
                printf("\n-->[notdef home] to next position\n");
                arm_move_step_return_mode = 0;  // Home登録ない場合は、フラグクリアして次のステップに進む
              }
            }
            break;
          }  //実行ループ(ret戻りでloop)
          if (file_read_result != 0)
          {
            break;
          }
        }
        kk++;
        if (kk >= 100000)  //無限ループ防止
        {
          file_read_result = 1;
          break;
        }
      }  // file read １行loop
      fclose(fp_input_movePose);
      printf("input file closed\n");
      if (file_read_result != 0)
      {
        if (file_read_result == 1)
        {
          console_out("@@@ file data read error line=%d step[%d]\n", kk, move_step_count_index);
          ROS_ERROR("@@@ file data read error line=%d step[%d]", kk, move_step_count_index);
        }
        break;
      }
      console_out("* execution count( %d / %d )\n", nonStop_LoopCount + 1, nonStop_LoopCountMax);
    }
    if (fp_output_plan_tim != NULL)
    {
      fclose(fp_output_plan_tim);
      printf("output tim file closed\n");
    }
    if (fp_output_plan_trj != NULL)
    {
      fclose(fp_output_plan_trj);
      printf("output trj file closed\n");
    }
  }
  return;
}

//出力ファイル名編集
//
void GetCurrDateTime(char *time_str)
{
  struct timeval sTimeVal;
  struct tm *psTm = NULL;

  gettimeofday(&sTimeVal, NULL);
  psTm = localtime(&sTimeVal.tv_sec);

  sprintf(time_str, "%04d%02d%02d-%02d%02d%02d", psTm->tm_year + 1900, psTm->tm_mon + 1, psTm->tm_mday, psTm->tm_hour,
          psTm->tm_min, psTm->tm_sec);

  return;
}

void MakeTrjOutputFileName(char *src_fileName, char *dst_fileName, char *time_str, char *ext_name)
{
  char OutFileName[128];

  sprintf(OutFileName, "%s", src_fileName);  // file name copy
  strtok(OutFileName, ".");                  //.csv削除( . ->NULLに)
  sprintf(dst_fileName, "%s_%s_%s.csv", OutFileName, time_str, ext_name);
  return;
}

//----------------------------------
//プラン結果ファイル保存
//----------------------------------
bool Savefile_plan_log(FILE *fp_sFile_tim, FILE *fp_sFile_trj, int planID, geometry_msgs::Pose *curr_pose,
                       int loop_count, int plan_count, int step_index, int step_type, int plan_result)
{
  int ii, jj;
  int joint_num;
  int trj_count;
  float planning_time = 0;
  float execution_time = 0;

  char work_output_sbuf[D_csvlineMax];

  if (plan_result == RSP_E_OK)  //正常終了
  {
    //計画済みアーム軌道を取得する
    ROS_INFO("* GetArmPlanTrj_srv<request>");
    GetArmPlanTrj_srv.request.plan_id = planID;  //直前の計画成功したPlanIDを設定
    if (GetArmPlanTrj_client.call(GetArmPlanTrj_srv) == true)
    {
      if (GetArmPlanTrj_srv.response.result == t2_msgs::GetPlannedArmTrajectoryResponse::SUCCESS)
      {
        //正常取得
        t_MainCtrlRsp.planning_time = GetArmPlanTrj_srv.response.planning_time;  //これは関数外に出したほうが良い
        planning_time = GetArmPlanTrj_srv.response.planning_time;                //プラン実行時間
        joint_num = GetArmPlanTrj_srv.response.joint_trajectory.joint_names.size();
        trj_count = GetArmPlanTrj_srv.response.joint_trajectory.points.size();
        if (trj_count == 0)
        {
          ROS_ERROR("* GetPlannedArmTrajectory response no data");
          return false;
        }
        execution_time =
            GetArmPlanTrj_srv.response.joint_trajectory.points[(trj_count - 1)].time_from_start.toSec();  //実行時間
        //        printf("joint_num=%d joint_trajectory count= %d\n", joint_num, trj_count);
        console_out("\n result: plan_time= %.4f, excute_time= %.4f xyz(%.4f, %.4f, %.4f)\n", planning_time,
                    execution_time, curr_pose->position.x, curr_pose->position.y, curr_pose->position.z);  //画面表示
        //先頭に列タイトル
        if ((loop_count == 0) && (plan_count == 0))
        {
          sprintf(work_output_sbuf, "loop,planCnt,[step],TrjNo,");  //行先頭
          sprintf(work_output_sbuf, "%spositions[%d]", work_output_sbuf, joint_num);
          for (ii = 0; ii < joint_num; ++ii)
          {
            sprintf(work_output_sbuf, "%s,", work_output_sbuf);  //列区切り
          }
          sprintf(work_output_sbuf, "%svelocities[%d]", work_output_sbuf, joint_num);
          for (ii = 0; ii < joint_num; ++ii)
          {
            sprintf(work_output_sbuf, "%s,", work_output_sbuf);  //列区切り
          }
          sprintf(work_output_sbuf, "%saccelerations[%d]", work_output_sbuf, joint_num);
          for (ii = 0; ii < joint_num; ++ii)
          {
            sprintf(work_output_sbuf, "%s,", work_output_sbuf);  //列区切り
          }
          sprintf(work_output_sbuf, "%stime_from_start\n", work_output_sbuf);
          if (fp_sFile_trj != NULL)
          {
            fprintf(fp_sFile_trj, "%s", work_output_sbuf);  //１行ファイル出力
          }
        }
        //各列データ
        for (jj = 0; jj < trj_count; ++jj)
        {
          sprintf(work_output_sbuf, "%d,%d,[%d],%d,", loop_count, plan_count, step_index,
                  jj);  // loop,planCnt,step,TrjNo,
          for (ii = 0; ii < joint_num; ++ii)
          {
            sprintf(work_output_sbuf, "%s%.4f,", work_output_sbuf,
                    GetArmPlanTrj_srv.response.joint_trajectory.points[jj].positions[ii]);
          }
          for (ii = 0; ii < joint_num; ++ii)
          {
            sprintf(work_output_sbuf, "%s%.4f,", work_output_sbuf,
                    GetArmPlanTrj_srv.response.joint_trajectory.points[jj].velocities[ii]);
          }
          for (ii = 0; ii < joint_num; ++ii)
          {
            sprintf(work_output_sbuf, "%s%.4f,", work_output_sbuf,
                    GetArmPlanTrj_srv.response.joint_trajectory.points[jj].accelerations[ii]);
          }
          sprintf(work_output_sbuf, "%s%f\n", work_output_sbuf,
                  GetArmPlanTrj_srv.response.joint_trajectory.points[jj].time_from_start.toSec());
          if (fp_sFile_trj != NULL)
          {
            fprintf(fp_sFile_trj, "%s", work_output_sbuf);  //１行ファイル出力
          }
        }
      }
      else
      {
        ROS_ERROR("* GetPlannedArmTrajectory response error=%d", GetArmPlanTrj_srv.response.result);
        return false;
      }
    }
    else
    {
      ROS_ERROR("* GetPlannedArmTrajectory service connet error");
      return false;
    }
  }
  else
  {
    printf("Planエラー\n");
  }
  //
  //実行内容(エラーでも出力に残す)
  sprintf(work_output_sbuf, "%d,%d,[%d],", loop_count, plan_count, step_index);  //先頭列
  switch (t_SeqArmMoveParam.moveTarget.arm_move_type)
  {
    case D_MoveOnly_ARM_STATE:
      sprintf(work_output_sbuf, "%sstate,%s,", work_output_sbuf,
              t_arm_Joint_angle[t_SeqArmMoveParam.moveTarget.state_name_index].posi_name);
      break;
    case D_MoveOnly_ARM_CARTESIAN:
      sprintf(work_output_sbuf, "%scartesian,%s,", work_output_sbuf, t_SeqArmMoveParam.moveTarget.Name);
      break;
    case D_MoveOnly_ARM_JOINT:
      sprintf(work_output_sbuf, "%sjoint,%s,", work_output_sbuf, t_SeqArmMoveParam.moveTarget.Name);
      break;
    default:
      sprintf(work_output_sbuf, "%s,%s,", work_output_sbuf, t_SeqArmMoveParam.moveTarget.Name);
      break;
  }
  switch (step_type)
  {
    case 1:
      sprintf(work_output_sbuf, "%shome,", work_output_sbuf);
      break;
    case 2:
      sprintf(work_output_sbuf, "%sret,", work_output_sbuf);
      break;
    case 3:
      sprintf(work_output_sbuf, "%sinit,", work_output_sbuf);
      break;
    default:
      sprintf(work_output_sbuf, "%snext,", work_output_sbuf);
      break;
  }
  if (plan_result == RSP_E_OK)  //正常終了
  {
    sprintf(work_output_sbuf, "%splanTime=,%.4f,execTime=,%.4f,xyz=,%.4f,%.4f,%.4f\n", work_output_sbuf, planning_time,
            execution_time, curr_pose->position.x, curr_pose->position.y, curr_pose->position.z);
  }
  else
  {
    sprintf(work_output_sbuf, "%splanTime=,%.4f,execTime=,%.4f,xyz=,%.4f,%.4f,%.4f,plan_NG\n", work_output_sbuf,
            planning_time, execution_time, curr_pose->position.x, curr_pose->position.y, curr_pose->position.z);
  }
  if (fp_sFile_tim != NULL)
  {
    fprintf(fp_sFile_tim, "%s", work_output_sbuf);  //１行ファイル出力
  }
  printf("%s", work_output_sbuf);  //１行出力
  return true;
}

//----------------------------------
//アーム単体移動 実行処理
//----------------------------------
int moveArmCmd_exe(T_PlanMainCmdMsg cmd_msg)
{
  t2_msgs::ArmPose current_arm_pose;  //サービスで取得したアームの現在位置姿勢
  t2_msgs::ArmPose start_pose;
  t2_msgs::ArmPose goal_pose;
  uint arm_pose_index;
  float velocity;
  int planID_moveOnly;  //プラン管理用ID
  int plan_result;

  const std::string plan_group = "arm";

  printf("\n");
  ROS_INFO("<<< [moveArmCmd_exe][code=%d][data=%d] start >>>", cmd_msg.sub_cmd_code, cmd_msg.sub_cmd_data);

  //アーム速度
  velocity = cmd_msg.velocity;
  if (velocity > arm_speed_high_)
  {
    velocity = arm_speed_high_;  //上限
  }
  if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_STATE_PLAN)  //ステート名指定による移動プラン作成＆実行（PlanID保存）
  {
    if (cmd_msg.arm_state_index >= D_posID_MAX)
    {
      ROS_ERROR("* request command posi_arm data illgal<%d>", cmd_msg.arm_state_index);
      return RSP_E_NG;  //異常終了
    }
    // プラン作成＆実行（PlanID保存）
    plan_result = ArmMoveToStateNamePose(cmd_msg.arm_state_index, velocity);  //アーム移動
    if (plan_result != RSP_E_OK)
    {
      return plan_result;  //異常終了
    }
    //アームの現在位置姿勢を取得する
    if (RSP_E_OK != snd_srv_msg_GetArmPose(current_arm_pose, plan_group))
    {
      return RSP_E_NG;  //異常終了
    }
    //正常終了
  }
  else if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_PLAN_ID)  //計画済みPLAN指定による移動
  {
    if (cmd_msg.planID != 0)
    {
      ROS_INFO("Arm_Cmd[ArmExec]->plan_id=%d", cmd_msg.planID);
      // t_ArmPlanStartPoseはダミーでセット(planIDが有効なので)
      if (RSP_E_OK != snd_ArmExecuteAction_msg(cmd_msg.planID, velocity))  //コマンド送信／受信
      {
        return RSP_E_NG;  //異常終了
      }
      ROS_INFO("Arm_Rsp[ArmExec] recived");
      return RSP_E_OK;  //正常終了
    }
    else
    {
      ROS_ERROR("* request command illgal plan_id=0");
      return RSP_E_NG;  //異常終了
    }
  }
  else if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_GET_POSE)  //アーム現在位置姿勢取得
  {
    t2_msgs::ArmPose arm_pose;
    if (RSP_E_OK != snd_srv_msg_GetArmPose(arm_pose, plan_group))
    {
      ROS_ERROR("* GetArmPose response error");  // 異常受信
    }
    else
    {
      t_MainCtrlRsp.ArmPose.joint_positions = arm_pose.joint_positions;  //現在関節角応答
    }
  }
  else
  {
    //動作指定値のチェック
    if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_STATE)  //ステート名指定による移動（PlanID保存しない）
    {
      //ステート名指定インデックスチェック
      if (cmd_msg.arm_state_index >= D_posID_MAX)
      {
        ROS_ERROR("* request command posi_arm data illgal<%d>", cmd_msg.arm_state_index);
        return RSP_E_NG;  //異常終了
      }
    }
    else if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_CARTESIAN)  //座標指定による移動
    {
      //目標位置姿勢座標指定のチェック
      if (cmd_msg.ArmPose.pose.orientation.w == 0)  // 姿勢データチェック
      {
        ROS_ERROR("@@@ arm pose matrix data error m[15]!=1");
        return RSP_E_NG;  //異常終了
      }
    }
    else if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_JOINT)  //関節角指定による移動
    {
      //関節角指定のチェック
      if (cmd_msg.ArmPose.joint_positions.size() == 0)
      {
        ROS_ERROR("@@@ arm pose joint data error null");
        return RSP_E_NG;  //異常終了
      }
    }

    else
    {
      ROS_ERROR("* request command sub_cmd_code illgal<%d>", cmd_msg.sub_cmd_code);
      return RSP_E_NG;  //異常終了
    }

    // アーム動作実行有無による分岐
    if (cmd_msg.sub_cmd_data ==
        0)  //(0:実行有り 1:プラン作成(現在位置) 2:プラン作成(前回移動位置) 3:アーム初期位置設定)
    {
      //動作実行
      if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_STATE)  //ステート名指定による移動（PlanID保存しない）
      {
        goal_pose.type.type = t2_msgs::ArmPoseType::GROUP_STATE;  //グループステート名指定
        goal_pose.group_state = t_arm_Joint_angle[cmd_msg.arm_state_index].posi_name;
        ROS_INFO("Arm_Cmd[ArmMove](state)->%s", goal_pose.group_state.c_str());
      }
      else if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_CARTESIAN)  //座標指定による移動
      {
        goal_pose.type.type = t2_msgs::ArmPoseType::CARTESIAN_POSITION;  //手先位置姿勢指定
        goal_pose.pose = cmd_msg.ArmPose.pose;                           //座標
        ROS_INFO("Arm_Cmd[ArmMove](cartesian)");
      }
      else if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_JOINT)  //関節角指定による移動
      {
        goal_pose.type.type = t2_msgs::ArmPoseType::JOINT_POSITION;  //関節角指定
        goal_pose.joint_positions = cmd_msg.ArmPose.joint_positions;
        ROS_INFO("Arm_Cmd[ArmMove](joint)");
      }
      //コマンド送信
      if (RSP_E_OK != snd_ArmMoveAction_msg(goal_pose, velocity, plan_group))  //コマンド送信／受信
      {
        return RSP_E_NG;  //異常終了
      }

      ROS_INFO("Arm_Rsp[ArmMove] recived");
      //正常終了
    }
    else if (cmd_msg.sub_cmd_data == 3)  //アーム初期位置設定
    {
      if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_JOINT)  //関節角指定による
      {
        plan_goal_pose.joint_positions = cmd_msg.ArmPose.joint_positions;  //前回関節角結果を差し替え
        if (plan_goal_pose.joint_positions.size() != 0)
        {
          plan_goal_pose_result_val = true;  // PlanActionの結果:有効をセットする
          printf("set init joint_positions data \n");
        }
        else
        {
          ROS_ERROR("@@@ nothing data(init joint_positions)");
          return RSP_E_NG;  //異常終了
        }
      }
      else
      {
        ROS_ERROR("@@@ pose type error(init joint_positions)");
        return RSP_E_NG;  //異常終了
      }
      //正常終了
    }
    else
    {
      //プラン作成
      planID_moveOnly = (t_boxNameId.size() * t_boxNameId.size());  //最大値指定
      //プランスタート位置
      start_pose.type.type = t2_msgs::ArmPoseType::CURRENT_POSITION;  //現在位置指定(デフォルト)
      if (cmd_msg.sub_cmd_data == 2)                                  //前回移動位置指定
      {
        if (plan_goal_pose_result_val == true)  // PlanActionの結果:有効
        {
          start_pose.joint_positions = plan_goal_pose.joint_positions;  //前回結果コピー
          if (start_pose.joint_positions.size() != 0)
          {
#if 0
            printf("plan_start JointPos =\n");
            for (i = 0; i < start_pose.joint_positions.size(); ++i)
            {
              printf("[%d]%1f ", i , plan_goal_pose.joint_positions[i]);
            }
            printf("\n");
#endif
            start_pose.type.type = t2_msgs::ArmPoseType::JOINT_POSITION;  //前回終了時の関節角指定
          }
          else
          {
            printf("nothing JointPos data --> change to current\n");
          }
        }
        else
        {
          printf("non saved JointPos --> change to current\n");
        }
      }
      //プランゴール位置
      if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_STATE)  //ステート名指定による移動（PlanID保存しない）
      {
        goal_pose.type.type = t2_msgs::ArmPoseType::GROUP_STATE;  //グループステート名指定
        goal_pose.group_state = t_arm_Joint_angle[cmd_msg.arm_state_index].posi_name;
        printf("\nArmPlan(plan_id=%d) to<%s>\n", planID_moveOnly, goal_pose.group_state.c_str());
      }
      else if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_CARTESIAN)  //座標指定による移動
      {
        goal_pose.type.type = t2_msgs::ArmPoseType::CARTESIAN_POSITION;  //手先位置姿勢指定
        goal_pose.pose = cmd_msg.ArmPose.pose;                           //座標
        printf("\nArmPlan(plan_id=%d) to cartesian pose\n", planID_moveOnly);
      }
      else if (cmd_msg.sub_cmd_code == D_MoveOnly_ARM_JOINT)  //関節角指定による移動
      {
        goal_pose.type.type = t2_msgs::ArmPoseType::JOINT_POSITION;  //関節角指定
        goal_pose.joint_positions = cmd_msg.ArmPose.joint_positions;
        ROS_INFO("Arm_Cmd[ArmMove](joint)");
      }
      //コマンド送信
      if (RSP_E_OK !=
          snd_ArmPlanAction_msg(start_pose, goal_pose, planID_moveOnly, plan_group, ""))  //コマンド送信／受信
      {
        t_MainCtrlRsp.planID = 0;
        t_MainCtrlRsp.ArmPose.joint_positions.resize(0);
        return RSP_E_NG;  //異常終了
      }

      //正常
      t_MainCtrlRsp.planID = planID_moveOnly;                                  //プラン管理用ID(応答用)
      t_MainCtrlRsp.ArmPose.joint_positions = plan_goal_pose.joint_positions;  //プラン作成時のゴール関節角を応答する
      ROS_INFO("Arm_Rsp[ArmPlan] recived");
    }
    //正常終了
  }

  return RSP_E_OK;  //正常終了
}

//----------------------------------
//  アームを現在位置から所定位置に移動する
//----------------------------------
int ArmMoveToStateNamePose(uint to_arm_pose_index, float velocity)
{
  t2_msgs::ArmMoveGoal move_goal;

  move_goal.plan_title = "### current -> " + std::string(t_arm_Joint_angle[to_arm_pose_index].posi_name);
  move_goal.plan_group = "arm_suction";
  move_goal.start.type.type = t2_msgs::ArmPoseType::CURRENT_POSITION;
  move_goal.goal.type.type = t2_msgs::ArmPoseType::GROUP_STATE;
  move_goal.goal.group_state = t_arm_Joint_angle[to_arm_pose_index].posi_name;
  move_goal.stay_level = true;
  move_goal.velocity = velocity;

  arm_move_action_client_->sendGoal(move_goal);
  bool before_timeout = arm_move_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

  if ((arm_move_action_client_->getResult()->result == t2_msgs::ArmMoveResult::SUCCESS) && before_timeout)
  {
	ROS_INFO("@@@ ArmMoveToStateNamePose success");
    console_out("\n★ Move成功 (current ⇒ %s)\n", move_goal.goal.group_state.c_str());
  }
  else
  {
    ROS_INFO("@@@ ArmMoveToStateNamePose failed\n");
    console_out("\n★ Move失敗 (current ⇒ %s)\n", move_goal.goal.group_state.c_str());
    return RSP_E_NG;  //異常終了
  }

  return RSP_E_OK;  //正常終了
}

//----------------------------------
// テスト用アーム指定位置移動ファイルパス読込み
//----------------------------------
//
bool GetFile_SeqMoveDataFilePath(void)
{
  FILE *listfp;
  int jj, kk;
  char work_sbuf[D_fileNameMax + 32];

  char *ary[32];
  int file_read_result;
  DIR *sDir = NULL;

  std::string path = ros::package::getPath("t2_task_planner");

  //デフォルトパス設定しておく
  sprintf(inputFilePath, "%s%s", path.c_str(), Def_armMove_subPath);   // target input file path
  sprintf(outputFilePath, "%s%s", path.c_str(), Def_armMove_subPath);  // target output file path

  sprintf(selectFileListnameAll, "%s%s", path.c_str(), Def_SeqMoveFilePath);  // file path name
  //移動登録リストファイル読込み
  if ((listfp = fopen(selectFileListnameAll, "r")) == NULL)
  {
    console_out("\n@@@ file open error[ %s ]\n", selectFileListnameAll);
    printf("\n@@@ file open error[ %s ]\n", selectFileListnameAll);
    return false;
  }
  printf(" < path file name=[ %s ] >\n\n", selectFileListnameAll);

  file_read_result = 0;
  kk = 1;                                                  // line count
  jj = 0;                                                  // List count
  while (fgets(work_sbuf, D_fileNameMax, listfp) != NULL)  // １行読み込み（ファイルEOFまで）
  {
    //ファイル名
    ary[0] = strtok(work_sbuf, ",\r\n");  // 文字列(char配列)をカンマ,CR,LFで分割する:先頭列[0]
    if (ary[0] == NULL || !strlen(ary[0]))
    {
      continue;  //次行読込み
    }
    if (static_cast<int>(strlen(ary[0])) >= D_fileNameMax)
    {
      console_out("@@@ file name length over error\n");
      ROS_ERROR("@@@ file name length over error");
      file_read_result = 2;
      break;
    }
    if (strcmp("input_file_path", ary[0]) == 0)  //入力フォルダパス名
    {
      ary[1] = strtok(NULL, ",\r\n");  // folder name
      strcpy(&inputFilePath[0], ary[1]);
      if (inputFilePath[0] != 0)
      {
        //パスの有無
        if ((sDir = opendir(inputFilePath)) == NULL)
        {
          printf("not found input_file_path =[ %s ]\n", inputFilePath);
          sprintf(inputFilePath, "%s%s", path.c_str(), Def_armMove_subPath);  // デフォルトパスに戻す
        }
        else
        {
          closedir(sDir);
          printf("input_file_path =[ %s ]\n", inputFilePath);
        }
      }
      continue;  //次行読込み
    }
    if (strcmp("output_file_path", ary[0]) == 0)  //入力フォルダパス名
    {
      ary[1] = strtok(NULL, ",\r\n");  // folder name
      strcpy(&outputFilePath[0], ary[1]);
      if (outputFilePath[0] != 0)
      {
        //パスの有無
        if ((sDir = opendir(outputFilePath)) == NULL)
        {
          printf("not found output_file_Path =[ %s ]\n", outputFilePath);
          sprintf(outputFilePath, "%s%s", path.c_str(), Def_armMove_subPath);  // デフォルトパスに戻す
        }
        else
        {
          closedir(sDir);
          printf("output_file_Path =[ %s ]\n", outputFilePath);
        }
      }
      continue;  //次行読込み
    }
    kk++;
    if (kk >= 1000)  //無限ループ防止
    {
      file_read_result = 1;
      break;
    }
  }
  fclose(listfp);

  if (file_read_result != 0)
  {
    if (file_read_result == 1)
    {
      console_out("@@@ file data read error line=%d No=%d\n", kk, jj);
      ROS_ERROR("@@@ file data read error line=%d No=%d", kk, jj);
    }
    printf(" < file path name=[ %s ] >\n\n", Def_SeqMoveFilePath);
    return false;
  }
  return true;
}

//----------------------------------
// テスト用アーム指定位置移動ファイル登録リスト読込み
//----------------------------------
//
bool GetFile_SeqMoveFileList(char *file_path, uint *ListCount)
{
  FILE *listfp;
  int jj, kk;
  char work_sbuf[D_fileNameMax + 32];

  char *ary[32];
  int file_read_result;

  printf("*---- seq_MoveArm file list ----*\n");
  sprintf(selectFileListnameAll, "%s%s", file_path, Def_SeqMoveFileList);  // itemList name
  printf(" < List file name=[ %s ] >\n\n", Def_SeqMoveFileList);
  console_out(" < List file name=[ %s ] >\n\n", Def_SeqMoveFileList);
  //移動登録リストファイル読込み
  if ((listfp = fopen(selectFileListnameAll, "r")) == NULL)
  {
    console_out("\n@@@ file open error[ %s ]\n", selectFileListnameAll);
    printf("\n@@@ file open error[ %s ]\n", selectFileListnameAll);
    return false;
  }

  file_read_result = 0;
  kk = 1;                                                  // line count
  jj = 0;                                                  // List count
  while (fgets(work_sbuf, D_fileNameMax, listfp) != NULL)  // １行読み込み（ファイルEOFまで）
  {
    //ファイル名
    ary[0] = strtok(work_sbuf, ",\r\n");  // 文字列(char配列)をカンマ,CR,LFで分割する:先頭列[0]
    if (ary[0] == NULL || !strlen(ary[0]))
    {
      continue;  //次行読込み
    }
    if (static_cast<int>(strlen(ary[0])) >= D_fileNameMax)
    {
      console_out("@@@ file name length over error\n");
      ROS_ERROR("@@@ file name length over error");
      file_read_result = 2;
      break;
    }
    strcpy(&seqArmMove_FileName[jj][0], ary[0]);  // file Name
    if (seqArmMove_FileName[jj][0] == '#')
    {
      continue;  //コメントなので次行読込み
    }
    printf("[%d]< %s >\n", jj + 1, &seqArmMove_FileName[jj][0]);
    *ListCount = ++jj;         //ファイル登録数
    if (jj >= D_fileCountMax)  //入力最大数
    {
      console_out("@@@ list buffer over count=%d\n", *ListCount);
      ROS_ERROR("@@@ list buffer over count=%d", *ListCount);
      file_read_result = 2;
      break;
    }
    kk++;
    if (kk >= 1000)  //無限ループ防止
    {
      file_read_result = 1;
      break;
    }
  }
  fclose(listfp);

  if (file_read_result != 0)
  {
    if (file_read_result == 1)
    {
      console_out("@@@ file data read error line=%d No=%d\n", kk, jj);
      ROS_ERROR("@@@ file data read error line=%d No=%d", kk, jj);
    }
    return false;
  }
  printf("\n file count=%d\n\n", *ListCount);

  return true;
}

//----------------------------------
// テーブル作成
//----------------------------------
void createBoxNameIdTable()
{
  t_boxNameId.clear();

  // start
  T_boxNameId start = { 0, "Start" };
  t_boxNameId.push_back(start);

  // bin, tote, box
  for (int i = 0; i < boxinfo.size(); i++)
  {
    if (i < g_bin_count)
    {
      /* BIN */
      std::string name = "Bin" + boxinfo[i].box_name;
      T_boxNameId bin = {};
      bin.boxType = 0;
      std::strncpy(bin.box_name, name.c_str(), 8);
      t_boxNameId.push_back(bin);
    }
    else if (i > g_bin_count + 1)
    {
      /* BOX */
      std::ostringstream oss;
      oss << (i - g_bin_count - 1);
      std::string name = "Box" + oss.str();
      T_boxNameId box = {};
      box.boxType = 1;
      std::strncpy(box.box_name, name.c_str(), 8);
      t_boxNameId.push_back(box);
    }
    else
    {
      /* TOTE */
      std::ostringstream oss;
      oss << (i - g_bin_count + 1);
      std::string name = "Tote" + oss.str();
      T_boxNameId box = {};
      box.boxType = 1;
      std::strncpy(box.box_name, name.c_str(), 8);
      t_boxNameId.push_back(box);
    }
  }
}

void createPlanPosiNameTable()
{
  plan_posi_name.clear();

  std::string box = "Box";
  std::string tote = "Tote";
  for (int i = 0; i < t_boxNameId.size(); i++)
  {
    std::string name(t_boxNameId[i].box_name);
    if (name != "Start")
    {
      std::size_t pos = name.find(box);
      if (pos != std::string::npos)
      {
        name.replace(name.find(box), box.size(), tote);
      }
      name.insert(0, "ap_");
    }
    T_arm_Joint joint = {};
    std::strncpy(joint.posi_name, name.c_str(), 32);
    plan_posi_name.push_back(joint);
  }
}

void creteArmJointAngleTable()
{
  t_arm_Joint_angle.clear();

  std::string prefix = "posi_arm_";
  for (int i = 0; i < plan_posi_name.size(); i++)
  {
    std::string name = prefix + plan_posi_name[i].posi_name;
    T_arm_Joint state = {};
    std::strncpy(state.posi_name, name.c_str(), 32);
    t_arm_Joint_angle.push_back(state);
  }
  T_arm_Joint initial_state = {"posi_arm_Initial"};
  t_arm_Joint_angle.push_back(initial_state);

  // Update by container_info.yaml
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue container_info_array;
  if (!nh.getParam(CONTAINER_INFO_PATH, container_info_array))
  {
    ROS_ERROR("getParam() error: %s", CONTAINER_INFO_PATH.c_str());
    return;
  }
  ROS_ASSERT(container_info_array.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int i = 0; i < container_info_array.size(); ++i)
  {
    int place_id = static_cast<int>(container_info_array[i]["place_id"]);
    if (place_id < t_arm_Joint_angle.size())
    {
      std::string ap_name = static_cast<std::string>(container_info_array[i]["ap_name"]);
      std::strncpy(t_arm_Joint_angle[place_id].posi_name, ap_name.c_str(), 32);
    }
  }
}

//*******************************************************************************
/* recoPC 撮影要求　送信 */
int snd_topic_msg_to_recoPC_captreq(t2_robot_msgs::CaptureReq send_captreq_msg)
{
  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;  // 停止
  }
  ROS_INFO("send RECO_PC! cmd_msg %s", send_captreq_msg.header.frame_id.c_str());
  Ctrl_recoPC_captreq_msg.publish(send_captreq_msg);
  return RSP_E_OK;  // 正常終了
}

/* recoPC 認識要求　送信 */
int snd_topic_msg_to_recoPC_recoreq(t2_robot_msgs::RecognizeReq send_recoreq_msg)
{
  ROS_INFO("send RECO_PC! cmd_msg %s", send_recoreq_msg.header.frame_id.c_str());
  if (true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;  // 停止
  }
  Ctrl_recoPC_recoreq_msg.publish(send_recoreq_msg);
  return RSP_E_OK;  // 正常終了
}

//----------------------------------
// アクション送受信処理
//----------------------------------
//----------------------------------
//  アームアクション 動作計画(Plan)
//

//*Action
int snd_ArmPlanAction_msg(t2_msgs::ArmPose& start_pose, t2_msgs::ArmPose& goal_pose, int plan_id, const std::string& plan_group, const std::string& grasp_pattern)
{
  t2_msgs::ArmPlanGoal plan_goal;

  evt_arm_rsprcv = false;  //受信イベントフラグクリア

  //④プラン管理ＩＤ
  plan_goal.plan_id = plan_id;
  if (plan_goal.plan_id == 0)
  {
    ROS_INFO("send arm plan_action plan_id illegal");
    return RSP_E_ILGL;  // データ異常終了
  }
  //①計画グループ
  plan_goal.plan_group = plan_group;

  //把持方法(把持計画の際に利用 通常は指定しない)
  plan_goal.grasp_pattern = grasp_pattern;

  //③目標位置姿勢を設定する
  plan_goal.start = start_pose;  //開始位置姿勢設定値をコピー
  plan_goal.goal = goal_pose;    //目標位置姿勢設定値をコピー

  //水平を保つ
  plan_goal.stay_level = true;

  //テストモード時ステップ動作待ち
  if ( true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;    // 停止
  }

  // アクションの実行
  arm_plan_action_client_->sendGoal(plan_goal);

  bool before_timeout = arm_plan_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

  if ((arm_plan_action_client_->getResult()->result == t2_msgs::ArmPlanResult::SUCCESS) && before_timeout)
  {
    return RSP_E_OK;
  }
  else
  {
	return RSP_E_NG;  // 異常受信
  }

  return RSP_E_OK;    // 終了
}

//*Action
int snd_ArmExecuteAction_msg(int plan_id, float velocity)
{
  t2_msgs::ArmExecuteGoal execute_goal;

  evt_arm_rsprcv = false;      //受信イベントフラグクリア

  //②速度指定
  execute_goal.velocity = velocity;

  //①プラン管理ＩＤ
  execute_goal.plan_id = plan_id;    //オンメモリのプランを実行する場合は、plan_id(1以上)を指定する。plan_nameは無視される。
  if (execute_goal.plan_id == 0)
  {
    ROS_ERROR("Not support. plan_id = 0");
  }
  else
  {
    printf("send arm execute_action v=%.2f id=%d,\n", execute_goal.velocity, execute_goal.plan_id);
  }

  //テストモード時ステップ動作待ち
  if ( true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;    // 停止
  }

  // アクションの実行
  arm_execute_action_client_->sendGoal(execute_goal);

  bool before_timeout = arm_execute_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

  if ((arm_execute_action_client_->getResult()->result == t2_msgs::ArmExecuteResult::SUCCESS) && before_timeout)
  {
    return RSP_E_OK;
  }
  else
  {
	return RSP_E_NG;  // 異常受信
  }

  return RSP_E_OK;    // 終了
}

//----------------------------------
//  アームアクション  移動(Move : Plan & Execute)
//

//*Action
int snd_ArmMoveAction_msg(t2_msgs::ArmPose& goal_pose, float velocity, const std::string& plan_group)
{
  t2_msgs::ArmMoveGoal move_goal;

  evt_arm_rsprcv = false;      //受信イベントフラグクリア

  //④速度指定
  move_goal.velocity = velocity;

  //①計画グループ
  move_goal.plan_group = plan_group;

  //②開始位置指定（プロトタイプ機は現在位置から）
  move_goal.start.type.type = t2_msgs::ArmPoseType::CURRENT_POSITION;

  //③目標位置姿勢を設定する
  move_goal.goal = goal_pose;    //目標位置姿勢設定値をコピー

  //水平を保つ
  move_goal.stay_level = true;

  //テストモード時ステップ動作待ち
  if ( true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;    // 停止
  }

  // アクションの実行
  arm_move_action_client_->sendGoal(move_goal);

  bool before_timeout = arm_move_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

  if ((arm_move_action_client_->getResult()->result == t2_msgs::ArmMoveResult::SUCCESS) && before_timeout)
  {
    return RSP_E_OK;
  }
  else
  {
	return RSP_E_NG;  // 異常受信
  }

  return RSP_E_OK;    // 終了
}

int snd_ArmMoveAction_msg(std::vector<geometry_msgs::Pose>& waypoints, float velocity, const std::string& plan_group)
{
  t2_msgs::ArmMoveGoal move_goal;

  evt_arm_rsprcv = false;      //受信イベントフラグクリア

  //④速度指定
  move_goal.velocity = velocity;

  //経由点
  move_goal.waypoints = waypoints;

  //①計画グループ
  move_goal.plan_group = plan_group;

  //②開始位置指定（プロトタイプ機は現在位置から）
  move_goal.start.type.type = t2_msgs::ArmPoseType::CURRENT_POSITION;

  //テストモード時ステップ動作待ち
  if ( true != wait_operating(&mainDebug_mode, 2))  //動作モードチェック
  {
    //中断終了
    return RSP_E_HALT;    // 停止
  }

  // アクションの実行
  arm_move_action_client_->sendGoal(move_goal);


  bool before_timeout = arm_move_action_client_->waitForResult(ros::Duration(RSP_TIME_LIMIT));

  if ((arm_move_action_client_->getResult()->result == t2_msgs::ArmMoveResult::SUCCESS) && before_timeout)
  {
    return RSP_E_OK;
  }
  else
  {
	return RSP_E_NG;  // 異常受信
  }

  return RSP_E_OK;    // 終了
}

void cbArmPlaceMoveActionFeedback(const t2_msgs::ArmPlaceMoveFeedbackConstPtr& feedback)
{
  ROS_INFO("cbArmPlaceMoveActionFeedback() event = %s", feedback->event.c_str());

  if (feedback->event == t2_motion_planner::FEEDBACK_EVENT_LEAVE_CONTAINER)
  {
    const uint& place_id = feedback->place_id;
    uint box_index;
    getBoxIndex(place_id, &box_index);
    if (order_task == 2 || place_move_order_status_ == ORDER_EXIST)
    {
      boxinfo[box_index].plan_status = PLAN_NOTYET;
    }
    resetRecognize(box_index);
    executeRecognition(place_id, RecognitionTypes::Item);
    // Pick task
    if (order_task == 1)
    {
      executeRecognition(place_id, RecognitionTypes::Octomap);
    }
  }
}

//ハンド吸着ポンプ設定サービス
int snd_srv_msg_GripperPump(const bool& pump)
{
  t2_msgs::GripperPump gripper_pump_srv;

  if (pump)
  {
	ROS_INFO("* GripperPump_srv<PUMP_ON>");
	gripper_pump_srv.request.pump = t2_msgs::GripperPumpRequest::PUMP_ON;
  }
  else
  {
	ROS_INFO("* GripperPump_srv<PUMP_OFF>");
	gripper_pump_srv.request.pump = t2_msgs::GripperPumpRequest::PUMP_OFF;
  }

  if (GripperPump_client.call(gripper_pump_srv) == true)
  {
    if (gripper_pump_srv.response.result == t2_msgs::GripperPumpResponse::SUCCESS)
    {
      return RSP_E_OK;  // 正常受信
    }
    ROS_ERROR("* GripprPump response error=%d", gripper_pump_srv.response.result);
    return RSP_E_NG;  // 異常受信
  }
  ROS_ERROR("* GripprPump service connet error");
  return RSP_E_TMOUT;  // 接続なし
}

//ハンド吸着設定サービス
int snd_srv_msg_GripperSuction(const bool& suction)
{
  t2_msgs::GripperSuction gripper_suction_srv;

  if (suction)
  {
    ROS_INFO("* gripper_suction_srv<SUCTION_ON>");
    gripper_suction_srv.request.suction = t2_msgs::GripperSuctionRequest::SUCTION_ON;
  }
  else
  {
    ROS_INFO("* gripper_suction_srv<SUCTION_OFF>");
    gripper_suction_srv.request.suction = t2_msgs::GripperSuctionRequest::SUCTION_OFF;
  }

  if (GripperSuction_client.call(gripper_suction_srv) == true)
  {
    if (gripper_suction_srv.response.result == t2_msgs::GripperSuctionResponse::SUCCESS)
    {
      return RSP_E_OK;  // 正常受信
    }
    ROS_ERROR("* gripper_suction_srv response error=%d", gripper_suction_srv.response.result);
    return RSP_E_NG;  // 異常受信
  }
  ROS_ERROR("* gripper_suction_srv service connet error");
  return RSP_E_TMOUT;  // 接続なし
}

/*
int snd_srv_msg_GetGraspInfo(std::vector<t2_msgs::GraspPoint> *grasp_point)
{
  ROS_INFO("* GetGraspInfo_srv<request>");

  if (GetGraspInfo_client.call(GetGraspInfo_srv) == true)
  {
    if (GetGraspInfo_srv.response.result == t2_msgs::GetGraspInfoResponse::SUCCESS)
    {
      *grasp_point = GetGraspInfo_srv.response.grasp_point;
      return RSP_E_OK;  // 正常受信
    }
    ROS_ERROR("* GetGraspInfo response error=%d", GetGraspInfo_srv.response.result);
    return RSP_E_NG;  // 異常受信
  }
  ROS_ERROR("* GetGraspInfo service connet error");
  return RSP_E_TMOUT;  // 接続なし
}
*/

//アーム位置姿勢取得サービス
int snd_srv_msg_GetArmPose(t2_msgs::ArmPose &arm_pose, const std::string &plan_group)
{
  ROS_INFO("* GetArmPose_srv<request>");

  t2_msgs::GetArmPose arm_pose_srv;
  arm_pose_srv.request.plan_group = plan_group;

  if (!GetArmPose_client.call(arm_pose_srv))
  {
    ROS_ERROR("* GetArmPose service connet error");
    return RSP_E_TMOUT;  // 接続なし
  }

  if (arm_pose_srv.response.result == t2_msgs::GetArmPoseResponse::SUCCESS)
  {
    // アームの現在位置姿勢座標コピー
    arm_pose = arm_pose_srv.response.pose;
    return RSP_E_OK;  // 正常受信
  }
  else
  {
    ROS_ERROR("* GetArmPose response error");
    return RSP_E_NG;  // 異常受信
  }
}

int snd_srv_msg_GetArmGroupStatePose(const std::string& group_state, t2_msgs::ArmPose& arm_pose, const std::string& plan_group)
{
  ROS_INFO("* GetArmGroupStatePose_srv<request>");

  t2_msgs::GetArmGroupStatePose arm_group_state_pose_srv;
  arm_group_state_pose_srv.request.plan_group = plan_group;
  arm_group_state_pose_srv.request.group_state = group_state;

  if (!GetArmGroupStatePose_client.call(arm_group_state_pose_srv))
  {
    ROS_ERROR("* GetArmPose service connet error");
    return RSP_E_TMOUT;  // 接続なし
  }

  if (arm_group_state_pose_srv.response.result == t2_msgs::GetArmGroupStatePoseResponse::SUCCESS)
  {
    // アームの現在位置姿勢座標コピー
    arm_pose = arm_group_state_pose_srv.response.pose;
    return RSP_E_OK;  // 正常受信
  }
  else
  {
    ROS_ERROR("* GetArmGroupStatePose response error");
    return RSP_E_NG;  // 異常受信
  }
}

int snd_srv_msg_PinchingGripperInitialize()
{
  ROS_INFO("* PinchingGripperInitialize_srv<request>");

  t2_msgs::GripperInitialize gripper_init_srv;

  if (!PinchingGripperInitialize_client.call(gripper_init_srv))
  {
    ROS_ERROR("Failed to call GripperInitialize service.");
    return RSP_E_NG;
  }

  if (gripper_init_srv.response.result != t2_msgs::GripperInitializeResponse::SUCCESS)
  {
    ROS_ERROR("Failed to call GripperInitialize service.");
    return RSP_E_NG;
  }

  return RSP_E_OK;
}

int snd_srv_msg_PinchingGripperFinalize()
{
  ROS_INFO("* PinchingGripperFinalize_srv<request>");

  t2_msgs::GripperFinalize gripper_fin_srv;

  if (!PinchingGripperFinalize_client.call(gripper_fin_srv))
  {
    ROS_ERROR("Failed to call GripperFinalize service.");
    return RSP_E_NG;
  }

  if (gripper_fin_srv.response.result != t2_msgs::GripperFinalizeResponse::SUCCESS)
  {
    ROS_ERROR("Failed to call GripperFinalize service.");
    return RSP_E_NG;
  }

  return RSP_E_OK;
}

int snd_srv_msg_KebaInitialize(bool homing)
{
  ROS_INFO("* KebaInitialize_srv<request>");

  keba_bridge_msgs::KebaInitialize keba_init_msg;
  keba_init_msg.request.homing = homing;

  if (!KebaInitialize_client.call(keba_init_msg))
  {
    ROS_ERROR("Failed to call KebaInitialize service.");
    return RSP_E_NG;
  }

  // joint state
  joint_state_pub.publish(keba_init_msg.response.joint_state);

  return RSP_E_OK;
}

int snd_srv_msg_KebaFinalize()
{
  ROS_INFO("* KebaFinalize_srv<request>");

  keba_bridge_msgs::KebaFinalize keba_fin_msg;

  if (!KebaFinalize_client.call(keba_fin_msg))
  {
    ROS_ERROR("Failed to call KebaFinalize service.");
    return RSP_E_NG;
  }

  return RSP_E_OK;
}

int snd_srv_msg_SetContainerObject(const t2_msgs::SetContainerObjectToPlanningScene::Request::_object_type& object)
{
  t2_msgs::SetContainerObjectToPlanningScene sco;
  sco.request.object = object;
  if (!SetContainerObjectToPlanningScene_client.call(sco))
  {
    ROS_ERROR("Failed to call SetContainerObjectToPlanningScene service");
    return RSP_E_NG;
  }
  return RSP_E_OK;
}

int snd_srv_msg_WeightScaleStart()
{
  for (int i = 0; i < D_WeightScaleTotal; i++)
  {
    t2_msgs::WeightScaleStart wss;
    if (!WeightScaleStart_client[i].call(wss))
    {
      ROS_ERROR("Failed to call WeightScaleStart[%d] service", i + 1);
      return RSP_E_NG;
    }
  }
  return RSP_E_OK;
}

int snd_srv_msg_WeightScaleStop()
{
  for (int i = 0; i < D_WeightScaleTotal; i++)
  {
    t2_msgs::WeightScaleStop wss;
    if (!WeightScaleStop_client[i].call(wss))
    {
      ROS_ERROR("Failed to call WeightScaleStop[%d] service", i + 1);
      return RSP_E_NG;
    }
  }
  return RSP_E_OK;
}

//----------------------------------
// トピック受信待ち処理
//----------------------------------
//
/* recoPC 撮影応答 */
int rcv_topic_msg_from_recoPC_captres(int *response_code, bool *rsp_evt, int wai_time)
{
  // 受信タイマー用時間設定(1Hz:1秒->10Hz:0.1)
  ros::Rate rspTime_rate(10);

  if (wai_time == 0)
  {
    //ros::spinOnce();  // コールバック関数を受け付ける
    if (*rsp_evt == true)
    {
      //受信コードチェック（未実装）
      //異常→  *rsp_evt = false;  // next wait
      //*response_code = mainReq_Cmd_code;    // 受信コマンド
      ROS_INFO("receive from recoPC %d", mainReq_Cmd_code);
      return RSP_E_OK;  // 受信有り
    }
  }
  else
  {
    ROS_INFO("response wairsp from <recoPC_capt>, waitm=%d", wai_time);
    for (rsp_timer = 0; rsp_timer < wai_time * 10; rsp_timer++)  // 受信待ち
    {
      //ros::spinOnce();  // コールバック関数を受け付ける
      if (*rsp_evt == true)
      {
        //受信コードチェック（未実装）
        //異常→  *rsp_evt = false;  // next wait
        //*response_code = mainReq_Cmd_code;    // 受信コマンド
        ROS_INFO("receive from recoPC %d", mainReq_Cmd_code);
        return RSP_E_OK;  // 受信有り
      }
      rspTime_rate.sleep();  // 時間待ち
      if (ros::ok() == false)
      {
        ROS_INFO("shutdown recived");  // shutdown受けているので表示されない
        evt_shutdown = false;
        return RSP_E_SHTDWN;
      }
      if (mainDebug_mode != 0)  //通常運転ではなく間欠の場合には、タイムアウトしないようにする
      {
        rsp_timer = 0;  //無限待ち
      }
    }
  }
  return RSP_E_TMOUT;
}

/* recoPC 認識応答 */
int rcv_topic_msg_from_recoPC_recores(int *response_code, bool *rsp_evt, int wai_time)
{
  ROS_INFO("rcv_topic_msg_from_recoPC_recores() start");

  // 受信タイマー用時間設定(1Hz:1秒->10Hz:0.1)
  ros::Rate rspTime_rate(10);

  if (wai_time == 0)
  {
    //ros::spinOnce();  // コールバック関数を受け付ける
    if (*rsp_evt == true)
    {
      //受信コードチェック（未実装）
      //異常→  *rsp_evt = false;  // next wait
      //*response_code = mainReq_Cmd_code;    // 受信コマンド
      ROS_INFO("receive from recoPC %d", mainReq_Cmd_code);
      return RSP_E_OK;  // 受信有り
    }
  }
  else
  {
    ROS_INFO("response wairsp from <recoPC_recog>, waitm=%d", wai_time);
    for (rsp_timer = 0; rsp_timer < wai_time * 10; rsp_timer++)  // 受信待ち
    {
      //ros::spinOnce();  // コールバック関数を受け付ける
      if (*rsp_evt == true)
      {
        //受信コードチェック（未実装）
        //異常→  *rsp_evt = false;  // next wait
        //*response_code = mainReq_Cmd_code;    // 受信コマンド
        ROS_INFO("receive from recoPC %d", mainReq_Cmd_code);
        return RSP_E_OK;  // 受信有り
      }
      rspTime_rate.sleep();  // 時間待ち
      if (ros::ok() == false)
      {
        ROS_INFO("shutdown recived");  // shutdown受けているので表示されない
        evt_shutdown = false;
        return RSP_E_SHTDWN;
      }
      if (mainDebug_mode != 0)  //通常運転ではなく間欠の場合には、タイムアウトしないようにする
      {
        rsp_timer = 0;  //無限待ち
      }
    }
  }

  ROS_INFO("rcv_topic_msg_from_recoPC_recores() timeout");
  return RSP_E_TMOUT;
}

// reco_PCからの撮影応答トピック
void msgCallback_rcv_recoPC_captres_cmd(const t2_robot_msgs::CaptureRes::ConstPtr& cmd_msg)
{
  ROS_INFO("[TOPIC]received RECO_PC! cmd_msg camera_on_res");
  //mainReq_Cmd_code = cmd_msg->data;    //受信コマンド保存
  evt_recoPC_captres_cmdrcv = true;          // コマンド受信イベントセット
}

// reco_PCからのトピック
void msgCallback_rcv_recoPC_recores_cmd(const t2_msgs::RecognizeRes::ConstPtr& cmd_msg)
{
  ROS_INFO("[TOPIC]received RECO_PC! cmd_msg recog_res");
  ROS_INFO("cmdmsg->jobno=%d", cmd_msg->job_no);

  for (int i = 0; i < g_bin_count; i++)
  {
    if (boxinfo[i].item_recog_jobno == cmd_msg->job_no)
    {
      ROS_INFO("update boxinfo and Plannnig Scene");

      // PlanningSceneへ認識アイテムを追加
      t2_msgs::AddItemsToPlanningScene abi_srv;
      abi_srv.request.place_id = boxinfo[i].box_id;
      abi_srv.request.job_no = boxinfo[i].item_recog_jobno;
      abi_srv.request.items = cmd_msg->items;
      if (!AddItemsToPlanningScene_client.call(abi_srv))
      {
        ROS_ERROR("Failed to call AddItemsToPlanningScene service");
      }

      if (abi_srv.response.result != t2_msgs::AddItemsToPlanningScene::Response::SUCCESS)
      {
        ROS_ERROR("AddItemsToPlanningScene() failed");
        console_out("\n★ PlanningSceneへの認識アイテム追加に失敗\n");
      }

      // update boxinfo
      updateRecognize(i, cmd_msg->items);
      ROS_INFO("msgCallback_rcv_recoPC_recores_cmd() success");

      evt_recoPC_recores_cmdrcv = true;
      break;
    }
  }
  if (evt_recoPC_recores_cmdrcv == true)
  {
    ROS_INFO("msgCallback_rcv_recoPC_recores_cmd() success");
  }
  else
  {
    ROS_INFO("msgCallback_rcv_recoPC_recores_cmd() not received");
  }

  return;
}

void receiveCaptureResCb(const t2_robot_msgs::CaptureRes::ConstPtr& msg)
{
  ROS_INFO("[TOPIC] receiveCaptureResCb(job_no=%u)", msg->job_no);

  uint job_no = msg->job_no;

  RecognitionTypes::RecognitionType type = recog_no_to_type_[job_no];
  ROS_INFO("RecognitionType %d", type);

  if (RecognitionTypes::Item == type ||
      RecognitionTypes::ItemAndOctomap == type)
  {
    t2_robot_msgs::RecognizeReq req;
    req.job_no = job_no;
    req.cad_id.clear();
    req.category_id.clear();

    uint place_id = 0;
    std::string name;
    for (Boxinfo_t& box_info : boxinfo)
    {
      if (job_no == box_info.item_recog_jobno)
      {
        place_id = box_info.box_id;
        name = box_info.box_name;

        // 認識中なら待つ
        bool recognizing = false;
        {
          std::lock_guard<std::mutex> lock(item_recog_mutex_);
          if (!box_info.capture_item_allowed)
          {
            ROS_INFO("Wait for previous item recognition. name=%s", name.c_str());
            recognizing = true;
          }
          else
          {
            box_info.pre_item_recog_jobno = job_no;
            box_info.capture_item_allowed = false;
          }
        }
        if (recognizing)
        {
          ros::Time start = ros::Time::now();
          while (!box_info.capture_item_allowed)
          {
            if (ros::Time::now() - start > ros::Duration(wait_item_recognition_timeout_))
            {
              ROS_INFO("Previous item recognition timeout. name=%s", name.c_str());
              break;
            }
            ros::Duration(0.1).sleep();
            continue;
          }
          std::lock_guard<std::mutex> lock(item_recog_jobno_mutex_);
          box_info.pre_item_recog_jobno = job_no;
          box_info.capture_item_allowed = false;
        }

        for (int i = 0; i < box_info.cad_id.size(); i++)
        {
          if (box_info.status[i] != ITEM_PICKED && box_info.cad_id[i] > 0 && ITEM_NORMAL == box_info.item_recog_status[i])
          {
            req.cad_id.push_back(box_info.cad_id[i]);
            req.category_id.push_back(0);   // TODO: temporary
          }
        }
        if (req.cad_id.size() == 0)
        {
          ROS_ERROR("No need to request recognition. place_id=%u has no item", box_info.box_id);
          box_info.recog_status = RECOG_OK;
          return;
        }
        break;
      }
    }
    if (place_id == 0)
    {
      ROS_ERROR("Invalid value job_no=%u", job_no);
      return;
    }

    ROS_INFO("<recognize_req> job_no=%d, place_id=%d itemcount=%lu", req.job_no, place_id, req.cad_id.size());
    for (int i = 0; i < req.cad_id.size(); ++i)
    {
      ROS_INFO("i=%d, cad_id=%u, category_id=%d", i, req.cad_id[i], req.category_id[i]);
    }
    console_out("\n★ 認識ＰＣにアイテム認識要求[Job=%d %s](itemcount=%lu)を送信\n",
        req.job_no, name.c_str(), req.cad_id.size());

    {
      std::lock_guard<std::mutex> lock(seq_no_mutex_);
      req.seq_no = ++captreq_seq_no;
    }
    Ctrl_recoPC_recoreq_msg.publish(req);
  }

  if (RecognitionTypes::Octomap == type ||
      RecognitionTypes::ItemAndOctomap == type)
  {
    t2_msgs::OccupancyReq req;
    req.job_no = job_no;

    uint place_id = 0;
    std::string name;
    for (int i = 0; i < boxinfo.size(); ++i)
    {
      if (job_no == boxinfo[i].octomap_recog_jobno)
      {
        place_id = boxinfo[i].box_id;
        name = boxinfo[i].box_name;

        // 認識中なら待つ
        bool recognizing = false;
        {
          std::lock_guard<std::mutex> lock(octomap_recog_mutex_);
          if (!boxinfo[i].capture_octomap_allowed)
          {
            ROS_INFO("Wait for previous octomap recognition. name=%s", name.c_str());
            recognizing = true;
          }
          else
          {
            boxinfo[i].pre_octomap_recog_jobno = job_no;
            boxinfo[i].capture_octomap_allowed = false;
          }
        }
        if (recognizing)
        {
          ros::Time start = ros::Time::now();
          while (!boxinfo[i].capture_octomap_allowed)
          {
            if (ros::Time::now() - start > ros::Duration(wait_octomap_recognition_timeout_))
            {
              ROS_INFO("Previous octomap recognition timeout. name=%s", name.c_str());
              break;
            }
            ros::Duration(0.1).sleep();
            continue;
          }
          std::lock_guard<std::mutex> lock(octomap_recog_jobno_mutex_);
          boxinfo[i].pre_octomap_recog_jobno = job_no;
          boxinfo[i].capture_octomap_allowed = false;
        }

        break;
      }
    }
    if (place_id == 0)
    {
      ROS_ERROR("Invalid value job_no=%u", job_no);
      return;
    }
    ROS_INFO("<occupancy_req> job_no=%d, place_id=%d", req.job_no, place_id);
    console_out("\n★ 認識ＰＣに箱詰認識要求[Job=%d %s]を送信\n", req.job_no, name.c_str());

    {
      std::lock_guard<std::mutex> lock(seq_no_mutex_);
      req.seq_no = ++captreq_seq_no;
    }
    occupancy_req_pub_.publish(req);
  }
}

void receiveRecognizeResCb(const t2_msgs::RecognizeRes::Ptr& msg)
{
  ROS_INFO("[TOPIC] receiveRecognizeResCb(job_no=%u)", msg->job_no);

  bool recog_flag = false;
  bool current_recog = true;
  std::size_t box_index = 0;
  {
    std::lock_guard<std::mutex> lock(item_recog_jobno_mutex_);
    for (std::size_t i = 0; i < boxinfo.size(); ++i)
    {
      if (boxinfo[i].item_recog_jobno == msg->job_no)
      {
        recog_flag = true;
        box_index = i;
        break;
      }
      else if (boxinfo[i].pre_item_recog_jobno == msg->job_no)
      {
        recog_flag = true;
        current_recog = false;
        box_index = i;
        break;
      }
    }
  }

  if (recog_flag)
  {
    Boxinfo_t& box_info = boxinfo[box_index];
    if (current_recog)
    {
      console_out("\n★ 認識結果[Job=%d %s](itemcount=%lu)を受信\n",
          msg->job_no, box_info.box_name.c_str(), msg->items.size());
      for (int j = 0; j < msg->items.size(); ++j)
      {
        const t2_msgs::ItemData& item = msg->items[j];
        console_out("   cad_id = %u pos(x, y, z) = (%.3f, %.3f, %.3f) ori(x, y, z, w) = (%.3f, %.3f, %.3f, %.3f)\n",
            item.cad_id, item.pose.position.x, item.pose.position.y, item.pose.position.z,
            item.pose.orientation.x, item.pose.orientation.y, item.pose.orientation.z, item.pose.orientation.w);
      }

      ROS_INFO("Update box info and plannnig scene");

      // PlanningSceneへ認識アイテムを追加
      t2_msgs::AddItemsToPlanningScene srv;
      srv.request.place_id = box_info.box_id;
      srv.request.job_no = box_info.item_recog_jobno;
      srv.request.items = msg->items;
      AddItemsToPlanningScene_client.call(srv);

      // Update boxinfo
      updateRecognize(box_index, msg->items);
      box_info.capture_item_allowed = true;
      ROS_INFO("receiveRecognizeResCb() success");
    }
    else
    {
      console_out("\n★ 認識結果[Job=%d %s](itemcount=%lu)を受信\n",
          msg->job_no, box_info.box_name.c_str(), msg->items.size());
      std::lock_guard<std::mutex> lock(item_recog_jobno_mutex_);
      if (msg->job_no == box_info.pre_item_recog_jobno)
      {
        box_info.capture_item_allowed = true;
      }
    }
  }
  else
  {
    ROS_ERROR("receiveRecognizeResCb() fail. boxinfo have no item_recog_jobno=%u", msg->job_no);
  }
}

void receiveOccupancyResCb(const t2_msgs::OccupancyRes::Ptr& msg)
{
  ROS_INFO("[TOPIC] receiveOccupancyResCb(job_no=%u)", msg->job_no);

  bool recog_flag = false;
  bool current_recog = true;
  std::size_t box_index = 0;
  {
    std::lock_guard<std::mutex> lock(octomap_recog_jobno_mutex_);
    for (std::size_t i = 0; i < boxinfo.size(); ++i)
    {
      if (boxinfo[i].octomap_recog_jobno == msg->job_no)
      {
        recog_flag = true;
        box_index = i;
        break;
      }
      else if (boxinfo[i].pre_octomap_recog_jobno == msg->job_no)
      {
        recog_flag = true;
        current_recog = false;
        box_index = i;
        break;
      }
    }
  }

  if (recog_flag)
  {
    Boxinfo_t& box_info = boxinfo[box_index];
    if (current_recog)
    {
      console_out("\n★ 箱詰認識結果[Job=%d %s]を受信\n",
          msg->job_no, box_info.box_name.c_str());

      // Update boxinfo
      updateOctomap(box_info.box_id, msg->octomap_pose);
      updateOctomapStatus(box_index);
      box_info.capture_octomap_allowed = true;
      ROS_INFO("receiveOccupancyResCb() success");
    }
    else
    {
      console_out("\n★ 箱詰認識結果[Job=%d %s]を受信\n", msg->job_no, box_info.box_name.c_str());
      std::lock_guard<std::mutex> lock(octomap_recog_jobno_mutex_);
      if (msg->job_no == box_info.pre_octomap_recog_jobno)
      {
        box_info.capture_octomap_allowed = true;
      }
    }
  }
  else
  {
    ROS_ERROR("receiveOccupancyResCb() fail. boxinfo have no item_recog_jobno=%u", msg->job_no);
  }
}
