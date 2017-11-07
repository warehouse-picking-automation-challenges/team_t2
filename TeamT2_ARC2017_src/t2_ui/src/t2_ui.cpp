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

#include <ros/ros.h>
#include <ros/package.h>
#include <t2_msgs/UIConsole.h>  // キー入力／出力メッセージ
#include <t2_ui/hitkey.h>

//----------------------------------
// 定数
//----------------------------------
const char *Def_PlanMainPrompt = "main";
const char *Def_PlanCntlPrompt = "ctrl";
const char *Def_PlanUnKownPrompt = "";

//----------------------------------
// グローバル変数
//----------------------------------
ros::Publisher ros_console_key_pub;   // 配信者ノードの宣言
ros::Subscriber ros_console_dsp_sub;  // 購読者ノードの宣言

//----------------------------------
// プロトタイプ宣言
//----------------------------------
void cmdInput_from_console(void);
void cbIntervalTimer(const ros::TimerEvent &);

//********************************
// ノードからの画面表示データ"Console_Dsp"トピック
//********************************
void cbConsoleDisp(const t2_msgs::UIConsole::ConstPtr &dsp_msg)
{
  int iKeyIndata = 0;
  int ret = 0;
  char strKeyin[80];
  std::string strNodeName;
  t2_msgs::UIConsole console_msg;

  console_msg.node_code = dsp_msg->node_code;  // 折り返し
  console_msg.type_code = dsp_msg->type_code;  // 折り返し
  console_msg.msg_string = "";
  console_msg.type_data = 0;

  switch (console_msg.node_code)  // ノード名表示用
  {
    case t2_msgs::UIConsole::CONSOLE_ATH_PLAN_MAIN:
      strNodeName = Def_PlanMainPrompt;
      break;

    case t2_msgs::UIConsole::CONSOLE_ATH_PLAN_CTRL:
      strNodeName = Def_PlanCntlPrompt;
      break;

    default:
      strNodeName = Def_PlanUnKownPrompt;
      break;
  }
  // タイプ別に処理
  if (console_msg.type_code == t2_msgs::UIConsole::CONSOLE_DSPONLY)  // 表示のみ
  {
    console_msg.msg_string = dsp_msg->msg_string;
    std::cout << console_msg.msg_string;
    console_msg.key_result = 0;  // KeyIn status
  }
  else
  {
    // キー入力
    hitkey_end();  // 周期呼出しからの入力OFF
    for (;;)
    {
      printf("%s>", strNodeName.c_str());
      if (fgets(strKeyin, sizeof(strKeyin), stdin))
      {
        if (strKeyin[0] == 1)  // CTRL-Aキー受け付け
        {
          cmdInput_from_console();  // コンソール指示メニュー
          ret = -1;
          break;  // キー入力中なので、エラーを送りメニューを再表示させる
        }
        else
        {
          // printf("%s",strKeyin);
          // console_msg.type_data = atoi(&strKeyin[0]);
          console_msg.msg_string = strKeyin;  //入力正常
          ret = 1;
          break;
        }
      }
      ret = -1;
      break;
    }
    console_msg.key_result = ret;              // KeyIn status
    ros_console_key_pub.publish(console_msg);  // 文字列キー入力データ送信
    hitkey_init();                             // HitKey再開
    while (hitkey() != -1)
    {
    }
  }
}

//********************************
// コンソール指示メニュー
//********************************
void cmdInput_from_console(void)
{
  char cKeyIndata;
  char buf[256];
  t2_msgs::UIConsole console_msg;

  printf("\n*-------- console menu start --------*\n");
  printf(" \n");
  bool valid_input = false;
  while (!valid_input)
  {
    printf("停止しますか?[Y/N]");
    if (fgets(buf, sizeof(buf), stdin))
    {
      cKeyIndata = buf[0];  // 文字入力
      switch (cKeyIndata)
      {
        case 'Y':
        case 'y':
          console_msg.msg_string = "";
          console_msg.key_result = 1;
          console_msg.node_code = t2_msgs::UIConsole::CONSOLE_ATH_PLAN_CNSL;
          console_msg.type_code = t2_msgs::UIConsole::CONSOLE_HALT;
          console_msg.type_data = 0;
          ros_console_key_pub.publish(console_msg);
          valid_input = true;
          break;
        case 'N':
        case 'n':
          valid_input = true;
          break;
        default:
          printf("Invalid input \'%c\'\n", cKeyIndata);
          break;
      }
    }
  }
  printf("\n*-------- console menu end ----------*\n");
}
//----------------------------------
// 一定周期呼出し(コンソールからのキー入力有無をチェックする)
//----------------------------------
void cbIntervalTimer(const ros::TimerEvent &)
{
  int keyData;
  t2_msgs::UIConsole console_msg;

  // 接続済み
  keyData = hitkey();
  if (keyData != -1)  // キー入力有り
  {
    while (getchar() != -1)
    {
    }
    if (keyData == 0x1B)  // ESCキー受け付け
    {
      hitkey_end();             // 周期呼出しからの入力OFF
      cmdInput_from_console();  // コンソール指示メニュー
      hitkey_init();            // HitKey再開
    }
  }
}

//***********************************************************************
int main(int argc, char **argv)
{
  // ノード名の初期化
  ros::init(argc, argv, "t2_ui");
  // ROSシステムとの通信のためのノードハンドルを宣言
  ros::NodeHandle nh_timer;
  ros::NodeHandle nh_console_dsp;
  ros::NodeHandle nh_console_key;

  /* トピック管理の設定 */
  // 配信（ノードplan_xxxへのキーボード入力送信）
  ros_console_key_pub = nh_console_dsp.advertise<t2_msgs::UIConsole>("Console_Key", 100);
  // 購読（ノードplan_xxxからの画面表示データ受信）
  ros_console_dsp_sub = nh_console_key.subscribe("Console_Dsp", 100, cbConsoleDisp);

  // タイマ
  ros::Timer timer_;
  // ブロックなしキー入力設定
  hitkey_init();

  // TaskPlanner起動待ち
  ROS_INFO("<waiting task planner node start>");
  while (ros_console_key_pub.getNumSubscribers() < 1)
  {
    ros::Duration(1).sleep();  // 時間待ち
  }

  printf("\n*-------- console menu < [ESC] key> --------*\n");
  // 定周期呼出しタイマ(100ms)
  timer_ = nh_timer.createTimer(ros::Duration(0.1), cbIntervalTimer);
  ros::spin();

  // 終了：キー入力設定を元に戻す
  printf("end of console\n");
  hitkey_end();
  return 0;
}
