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

#include <stdio.h>
#include <termios.h>
#include <ctype.h>
#include <t2_ui/hitkey.h>

static struct termios g_oldtio_stdin;  // バックアップ格納
static int is_initialized = 0;         // 多重初期化防止フラグ

void hitkey_init(void)
{
  if (!is_initialized)
  {
    struct termios newtio;
    tcgetattr(0, &g_oldtio_stdin);  // 現在の設定をバックアップ
    tcgetattr(0, &newtio);
    newtio.c_lflag &= ~ICANON;  // 非カノニカルモード(入力文字即時利用)
    // newtio.c_lflag &= ~ ECHO; // エコーしない
    newtio.c_cc[VTIME] = 0;  // タイムアウトなし
    newtio.c_cc[VMIN] = 0;   // ブロックしない
    tcsetattr(0, TCSANOW, &newtio);
    is_initialized = 1;
  }
}

void hitkey_end(void)
{
  if (is_initialized)
  {
    tcsetattr(0, TCSANOW, &g_oldtio_stdin);  // バックアップをリストア
    putchar('\n');
    is_initialized = 0;
  }
}

int hitkey(void)
{
  int key = getchar(); // 未入力時:-1
  if (isgraph(key)) putchar('\n');
  //if(key == '\n') key = '\r';
  return(key);
}
