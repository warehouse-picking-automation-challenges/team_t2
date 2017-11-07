#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Toshiba Corporation, 
#                     Toshiba Infrastructure Systems & Solutions Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#       * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#       * Neither the name of the Toshiba Corporation, nor the Toshiba
#       Infrastructure Systems & Solutions Corporation, nor the names
#       of its contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import rospkg
import glob
import shutil
import os
import sys
import re
import subprocess

if __name__ == '__main__':
    rospy.loginfo('(gptest_update) start.')

    # ROSノード初期化
    rospy.init_node('gptest_update')
    rospack = rospkg.RosPack()

    # アイテムデータコピー元ホームディレクトリ
    src_dir = os.environ['HOME'] + '/Documents/ARC_FakeSVN/release/'
    # 数字10桁で構成されるディレクトリのうち、最も数字の大きいもの
    # src_dir_list = glob.glob(os.environ['HOME'] + '/Documents/ARC_FakeSVN/[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]')
    # if len(src_dir_list) > 0: src_dir = src_dir_list[-1]    

    # アイテムデータコピー元ディレクトリ
    src_dir_list = \
    [
        src_dir + '/grasp_info_list_launch_xml/', # (7)CAD-ID紐付けxml
        src_dir + '/Fake_vision_yaml/',           # (8)FakeVision用yaml
        src_dir + '/Item_stl/',                   # (9')仮アイテムSTLファイル
        src_dir + '/Grasp_info_yaml/',            # (10')仮把持点情報
        src_dir + '/Item_stl/',                   # (9)アイテムSTLファイル
        src_dir + '/Grasp_info_yaml/',            # (10)把持点情報
    ]

    # アイテムデータ検索規則
    src_file_rule = \
    [
        'grasp_info_list.launch.xml',
        'World_*.yaml',
        '*.stl',
        '[0-9]*.yaml',
        'auto_*.stl',
        'auto_[0-9]*.yaml',
    ]
        
    # アイテムデータコピー先ディレクトリ
    dest_dir_list = \
    [
        rospack.get_path('t2_database') + '/launch/',
        rospack.get_path('t2_fake_vision') + '/data/',
        rospack.get_path('t2_database') + '/meshes/items/',
        rospack.get_path('t2_database') + '/data/grasp_info_list/',
        rospack.get_path('t2_database') + '/meshes/items/',
        rospack.get_path('t2_database') + '/data/grasp_info_list/',
    ]

    last_file_list = [[] for elem in range(len(src_dir_list))]
    last_timestamp_list = [[] for elem in range(len(src_dir_list))]
    update_file_list = [[] for elem in range(len(src_dir_list))]

    rate = rospy.Rate(10) # ファイル監視は0.1秒周期

    while not rospy.is_shutdown():

        for i in range(len(src_dir_list)):
            src_dir = src_dir_list[i]
            src_rule = src_file_rule[i]
            dest_dir = dest_dir_list[i]
            last = last_file_list[i]
            last_timestamp = last_timestamp_list[i]

            # ファイルリストとタイムスタンプを取得
            file_list = glob.glob(src_dir + src_rule)
            timestamp_list = [os.stat(file).st_mtime for file in file_list]

            # 新しく作られたファイルのリストを取得
            update_list = list(set(file_list) - set(last))
            
            # タイムスタンプが更新されたファイルのリストを取得
            compare_list = list(set(file_list) & set(last))
            for cmp_file in compare_list:
                cmp_file_i = file_list.index(cmp_file)
                last_cmp_file_i = last.index(cmp_file)
                if timestamp_list[cmp_file_i] != last_timestamp[last_cmp_file_i]:
                    update_list.append(cmp_file)

            last_file_list[i] = file_list # ファイルリスト更新
            last_timestamp_list[i] = timestamp_list # タイムスタンプ更新
        
            # 更新されたファイルをコピー
            for file in update_list:
                try:
                    # ファイルに何かのプロセスがアクセス中(=ファイル更新中)かどうかを調べる
                    ret = subprocess.check_output(['lsof', file])
                    last_timestamp_list[i].pop(last_file_list[i].index(file))
                    last_file_list[i].remove(file)
                    rospy.loginfo('(gptest_update) waiting file copy.')
                    continue
                except:
                    pass

                # "auto_"付きファイルの場合はコピー先のファイル名から"auto_"を削除
                if re.match('auto_*', os.path.basename(file)):
                    dest_dir = dest_dir + re.sub('auto_', '', os.path.basename(file))

                # ファイルのコピー
                shutil.copy(file, dest_dir)                        
                rospy.loginfo('(gptest_update) file copy. \n"{}".'.format(os.path.basename(file)))
        
        
        rate.sleep()
