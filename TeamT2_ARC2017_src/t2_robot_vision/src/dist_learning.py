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
import os
import rospkg
import numpy as np
import cv2
from chainer import cuda
import dist_learning_rcg as dlr


from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image
from T2_robot_vision.msg import RecognizedItem, SegmentedData, ItemizedData, RecognizedSegment
from cv_bridge import CvBridge



def callback(data):
    jn = data.recog_target.job_no
    rospy.loginfo("job_no:%d Detection data received", jn)
    
    
    #出力先準備
    item = RecognizedItem()
    item.header            = data.header
    item.recog_target      = data.recog_target
    item.calibrated_points = data.calibrated_points
    item.segmented_data    = data.segmented_data
    item.itemized_data     = []
    item.rcg_module_index  = 5 #YOLO+距離学習
    
    for itm in data.itemized_data: # YOLO結果をループ
        # YOLO結果から値取り出し
        cg           = itm.category
        seg_id       = itm.seg_id
        seg          = data.segmented_data[seg_id]
        sx,ex,sy,ey  = seg.sx, seg.ex, seg.sy, seg.ey
        img_idx      = seg.img_idx
        
        #存在しないカテゴリ番号はスキップ
        if cg not in glb_modelSet:
        #if cg != 27:
            #item.itemized_data.append(itm)
            continue
        
        #カテゴリ番号に対応する辞書を得る
        glb_model  = glb_modelSet [cg]
        glb_dic_co = glb_dic_coSet[cg]
        glb_dic_lb = glb_dic_lbSet[cg]

        # RGB画像を得る
        converter = CvBridge()
        rgbImg = converter.imgmsg_to_cv2(data.recog_target.data[img_idx].rgb,
                                         desired_encoding="passthrough")
        rgbImg = rgbImg[:,:,::-1] #誤ってBGRで学習したため、暫定でBGRに変換。後で消す。
        
        # 位置情報を中心と半径に変換
        cx = (sx+ex)/2.
        cy = (sy+ey)/2.
        r  = max( (ex-sx)/2., (ey-sy)/2. )
        locates=[[cx,cy,r]]
        
        # 認識
        result_cyprd=dlr.Recog_dist_learning(rgbImg,locates,glb_model,glb_dic_co,
                                             glb_dic_lb,glb_outSize,glb_xp)
        #print result_cyprd
        
        #閾値処理
        if result_cyprd[4] > glb_kyori_rej_th: continue
        
        #出力先に結果格納
        resultItem_data = ItemizedData()
        resultItem_data.module_index = 5 #YOLO+距離学習
        resultItem_data.category     = cg
        resultItem_data.yaw          = result_cyprd[1]
        resultItem_data.pitch        = result_cyprd[2]
        #resultItem_data.roll         = result_cyprd[3]
        resultItem_data.roll         = (360-result_cyprd[3]) % 360 #rollを反転して学習したため、暫定で変換。後で戻す
        resultItem_data.score        = 100-result_cyprd[4]/2
        if resultItem_data.score < 0:
                resultItem_data.score = 0
        resultItem_data.seg_id       = seg_id
        item.itemized_data.append(resultItem_data)
    
    pub.publish(item)



rospack = rospkg.RosPack()

#callback.counter = 1

# パラメータ
glb_d_th=15.
use_gpu=0 #0:使わない、1:使う
glb_outSize=56
glb_kyori_rej_th = 50. #距離がこの値以上ならリジェクト

glb_xp = cuda.cupy if use_gpu == 1 else np

# モデル・辞書 読み込み
file_path = '%s/data/dist_learning'%rospack.get_path('T2_robot_vision')
#glb_model, glb_dic_co, glb_dic_lb = dlr.LoadModel_Dic(file_path,glb_d_th,glb_xp)
glb_modelSet, glb_dic_coSet, glb_dic_lbSet = dlr.LoadModel_Dic(file_path,glb_d_th)
print("Dist Learning model_dic load done.")


pub = rospy.Publisher("recognized_item", RecognizedItem, queue_size=10)
rospy.init_node('dist_learning')
r = rospy.Rate(1) # 10hz

rospy.Subscriber("yolov2_results", RecognizedItem, callback)

while not rospy.is_shutdown():

    r.sleep()

