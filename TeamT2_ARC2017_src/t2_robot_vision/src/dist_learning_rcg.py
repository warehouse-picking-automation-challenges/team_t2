# -*- coding: utf-8 -*-
"""
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
"""

#import rospy #debug用
#import rospkg #debug用
import numpy as np
from chainer import cuda
from chainer import serializers
from chainer import Chain
import chainer.functions as F
import chainer.links as L
import cv2
#from scipy.spatial.distance import cdist

from dist_embed3 import dist_embed


class DistEmbeddingNet(Chain):
    def __init__(self, d_th):
        if d_th <= 0:
            raise ValueError('out_th should be positive value.')
        self.d_th=d_th
        self.outDim=10
        super(DistEmbeddingNet,self).__init__(
            bnorm0=L.BatchNormalization(3),
            conv1 =L.Convolution2D(in_channels=3, out_channels=16,ksize=5,pad=2),
            bnorm1=L.BatchNormalization(16),
            conv2 =L.Convolution2D(in_channels=16,out_channels=64,ksize=5,pad=2),
            bnorm2=L.BatchNormalization(64),
            fc1=L.Linear(in_size=12544,out_size=1000),
            bnormFc1=L.BatchNormalization(1000),
            fc2=L.Linear(in_size=1000,out_size=1000),
            bnormFc2=L.BatchNormalization(1000),
            fc3=L.Linear(in_size=1000,out_size=1000),
            bnormFc3=L.BatchNormalization(1000),
            fc4=L.Linear(in_size=1000,out_size=self.outDim),
        )
        
    def forward_once(self,x,train=False):
        h = self.bnorm0(x,test=not train)
        #h = F.relu(self.conv1(x))
        #h = F.relu(self.bnorm1(self.conv1(x),test=not train))
        h = F.relu(self.bnorm1(self.conv1(h),test=not train))
        h = F.max_pooling_2d(h, ksize=2)
        #h = F.relu(self.conv2(h))
        h = F.relu(self.bnorm2(self.conv2(h),test=not train))
        h = F.max_pooling_2d(h, ksize=2)
        h = F.dropout(h,train=train)
        #h = F.relu(self.fc1(h))
        h = F.relu(self.bnormFc1(self.fc1(h),test=not train))
        h = F.dropout(h,train=train)
        #h = F.relu(self.fc2(h))
        h = F.relu(self.bnormFc2(self.fc2(h),test=not train))
        h = F.dropout(h,train=train)
        #h = F.relu(self.fc3(h))
        h = F.relu(self.bnormFc3(self.fc3(h),test=not train))
        h = F.dropout(h,train=train)
        z = self.fc4(h)
        return z
    
    def forward(self,x0,x1,dist):
        z0=self.forward_once(x0,train=True)
        z1=self.forward_once(x1,train=True)
        return dist_embed(z0,z1,dist,self.d_th)

        
        
#ベクトル集合aとbの距離行列を得る
#距離行列[i,j]＝a[i]とb[j]の距離の2乗
def DistMatCalc(A,B,xp):
    aNum=A.shape[0]
    bNum=B.shape[0]
    a2 = xp.sum(A**2,axis=1)
    a2Mat=xp.tile(a2[:,xp.newaxis],bNum)
    b2 = xp.sum(B**2,axis=1)
    b2Mat=xp.tile(b2,(aNum,1))
    dist = a2Mat + b2Mat - 2*xp.dot(A,B.T)
    return dist
        
    

def Recog_core03(test_data,model,dic_co,dic_lb,xp):
    batchsize=100
    testNum=test_data.shape[0]

    #結果格納場所準備
    bestN=3
    bestNidx=xp.empty((testNum,bestN),dtype=int)#近い順bestN個の辞書index
    distN   =xp.empty((testNum,bestN))          #近い順bestN個の距離2乗

    #近い順にN個を辞書から探す
    for i in range(0,testNum,batchsize):
        test_batch = test_data[i : i+batchsize]
        test_batchLen=test_batch.shape[0]

        #認識させる
        test_batch = xp.asarray(test_batch, dtype=xp.float32)
        z = model.forward_once(test_batch)

        #distB=cdist(z.data,dic_co,'sqeuclidean')#dists[i,j]:第iテストと第j辞書の距離2乗
        dist=DistMatCalc(z.data,dic_co,xp) #dists[i,j]:第iテストと第j辞書の距離2乗
        bestNidx_batch=dist.argsort()[:,:bestN]#近い順にbestN点のindex
        distN_batch   =xp.sort(dist) [:,:bestN] 
        bestNidx[i : i+test_batchLen]=bestNidx_batch
        distN   [i : i+test_batchLen]=distN_batch

    bestN_cypr = dic_lb[bestNidx]
    bestN_cyprd=xp.concatenate( (bestN_cypr, distN[:,:,xp.newaxis]), axis=2)

    return bestN_cyprd



#入力
#rgbImg:openCV形式のRGB画像
#locates:中心x,yと半径rのリスト
def Recog_dist_learning(rgbImg,locates,model,dic_co,dic_lb,outSize,xp):
    
    testNum=len(locates)
    test_data=xp.empty((testNum,3,outSize,outSize),dtype=xp.float32)
    
    ## パッチ画像にする
    # 出力 test_data
    for idx,(cx,cy,r) in enumerate(locates):
        r /= 0.95
        pts1 = [[cx-r,cy-r], [     cx+r,cy-r], [     cx+r,     cy+r]]
        pts2 = [[   0,   0], [outSize-1,   0], [outSize-1,outSize-1]]
        M = cv2.getAffineTransform(xp.float32(pts1),xp.float32(pts2))
        testPatch=cv2.warpAffine(rgbImg,M,(outSize,outSize))
        test_data[idx]=testPatch[:,:,::-1].transpose(2,0,1).astype(xp.float32)
        
        #debug
        #rospack = rospkg.RosPack()
        #cv2.imwrite('{}/data/hama_cx{}cy{}r{}.png'.format(rospack.get_path('T2_robot_vision'),cx,cy,r),
        #            test_data[0][:,:,::-1].transpose(1,2,0))
    
    ## 認識
    bestN_cyprd=Recog_core03(test_data,model,dic_co,dic_lb,xp)
    
    ## 検証
    # ここでリジェクトなどの各種判断をする
    #暫定で1データ目(locates[0])の1位結果を返す
    result_cyprd=bestN_cyprd[0][0]
    
    return result_cyprd


def LoadModel_Dic(file_path,d_th):
    modelSet={}
    dic_coSet={}
    dic_lbSet={}

    #catNoList = [1,2,25,27,43,48,51,56,59,60,61]
    #catNoList = [73,2,25,27,43,48,51,77,59,60,61]
    #catNoList = [62,63,64,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,84,86,87,91,92,93,94,96,97,98,99]
    catNoList = [63,68,70,72,74,76,80,86,94]
    for catNo in catNoList:
        #モデル読み込み
        model = DistEmbeddingNet(d_th)
        serializers.load_npz('{}/m_{:02d}.model'.format(file_path,catNo),model)
    
        #辞書読み込み
        npfile=np.load('{}/dic_{:02d}.npz'.format(file_path,catNo) )
        dic_co=npfile['co']
        dic_lb=npfile['lb']

        #Listに入れる
        modelSet [catNo] = model
        dic_coSet[catNo] = dic_co
        dic_lbSet[catNo] = dic_lb
        print ("Loading dist learning model ({}/{})".format(len(modelSet),len(catNoList)) )
    
    return modelSet, dic_coSet, dic_lbSet

