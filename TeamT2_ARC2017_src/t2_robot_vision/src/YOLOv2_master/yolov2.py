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

import numpy.lib.stride_tricks as np
import numpy as np
from chainer import cuda, Function, gradient_check, Variable, optimizers, serializers, utils
from chainer import Link, Chain, ChainList
import chainer.links as L
import chainer.functions as F
from lib.utils import *
from lib.functions import *


nF01 = 32/4
nF02 = 64/4
nF03 = 128/4
nF04 = 64/4
nF05 = 128/4
nF06 = 256/4
nF07 = 128/4
nF08 = 256/4
nF09 = 512/4
nF10 = 256/4
nF11 = 512/4
nF12 = 256/4
nF13 = 512/4
nF14 = 1024/4
nF15 = 512/4
nF16 = 1024/4
nF17 = 512/4
nF18 = 1024/4
nF19 = 1024/4
          

class YOLOv2(Chain):

    """
    YOLOv2
    - It takes (416, 416, 3) sized image as input
    """

    def __init__(self, n_classes, n_boxes):
        super(YOLOv2, self).__init__(
            ##### common layers for both pretrained layers and yolov2 #####
            conv1  = L.Convolution2D(3, nF01, ksize=3, stride=1, pad=1, nobias=True),
            bn1    = L.BatchNormalization(nF01, use_beta=False, eps=2e-5),
            bias1  = L.Bias(shape=(nF01,)),
            conv2  = L.Convolution2D(nF01, nF02, ksize=3, stride=1, pad=1, nobias=True),
            bn2    = L.BatchNormalization(nF02, use_beta=False, eps=2e-5),
            bias2  = L.Bias(shape=(nF02,)),
            conv3  = L.Convolution2D(nF02, nF03, ksize=3, stride=1, pad=1, nobias=True),
            bn3    = L.BatchNormalization(nF03, use_beta=False, eps=2e-5),
            bias3  = L.Bias(shape=(nF03,)),
            conv4  = L.Convolution2D(nF03, nF04, ksize=1, stride=1, pad=0, nobias=True),
            bn4    = L.BatchNormalization(nF04, use_beta=False, eps=2e-5),
            bias4  = L.Bias(shape=(nF04,)),
            conv5  = L.Convolution2D(nF04, nF05, ksize=3, stride=1, pad=1, nobias=True),
            bn5    = L.BatchNormalization(nF05, use_beta=False, eps=2e-5),
            bias5  = L.Bias(shape=(nF05,)),
            conv6  = L.Convolution2D(nF05, nF06, ksize=3, stride=1, pad=1, nobias=True),
            bn6    = L.BatchNormalization(nF06, use_beta=False, eps=2e-5),
            bias6  = L.Bias(shape=(nF06,)),
            conv7  = L.Convolution2D(nF06, nF07, ksize=1, stride=1, pad=0, nobias=True),
            bn7    = L.BatchNormalization(nF07, use_beta=False, eps=2e-5),
            bias7  = L.Bias(shape=(nF07,)),
            conv8  = L.Convolution2D(nF07, nF08, ksize=3, stride=1, pad=1, nobias=True),
            bn8    = L.BatchNormalization(nF08, use_beta=False, eps=2e-5),
            bias8  = L.Bias(shape=(nF08,)),
            conv9  = L.Convolution2D(nF08, nF09, ksize=3, stride=1, pad=1, nobias=True),
            bn9    = L.BatchNormalization(nF09, use_beta=False, eps=2e-5),
            bias9  = L.Bias(shape=(nF09,)),
            conv10 = L.Convolution2D(nF09, nF10, ksize=1, stride=1, pad=0, nobias=True),
            bn10   = L.BatchNormalization(nF10, use_beta=False, eps=2e-5),
            bias10 = L.Bias(shape=(nF10,)),
            conv11 = L.Convolution2D(nF10, nF11, ksize=3, stride=1, pad=1, nobias=True),
            bn11   = L.BatchNormalization(nF11, use_beta=False, eps=2e-5),
            bias11 = L.Bias(shape=(nF11,)),
            conv12 = L.Convolution2D(nF11, nF12, ksize=1, stride=1, pad=0, nobias=True),
            bn12   = L.BatchNormalization(nF12, use_beta=False, eps=2e-5),
            bias12 = L.Bias(shape=(nF12,)),
            conv13 = L.Convolution2D(nF12, nF13, ksize=3, stride=1, pad=1, nobias=True),
            bn13   = L.BatchNormalization(nF13, use_beta=False, eps=2e-5),
            bias13 = L.Bias(shape=(nF13,)),
            conv14 = L.Convolution2D(nF13, nF14, ksize=3, stride=1, pad=1, nobias=True),
            bn14   = L.BatchNormalization(nF14, use_beta=False, eps=2e-5),
            bias14 = L.Bias(shape=(nF14,)),
            conv15 = L.Convolution2D(nF14, nF15, ksize=1, stride=1, pad=0, nobias=True),
            bn15   = L.BatchNormalization(nF15, use_beta=False, eps=2e-5),
            bias15 = L.Bias(shape=(nF15,)),
            conv16 = L.Convolution2D(nF15, nF16, ksize=3, stride=1, pad=1, nobias=True),
            bn16   = L.BatchNormalization(nF16, use_beta=False, eps=2e-5),
            bias16 = L.Bias(shape=(nF16,)),
            conv17 = L.Convolution2D(nF16, nF17, ksize=1, stride=1, pad=0, nobias=True),
            bn17   = L.BatchNormalization(nF17, use_beta=False, eps=2e-5),
            bias17 = L.Bias(shape=(nF17,)),
            conv18 = L.Convolution2D(nF17, nF18, ksize=3, stride=1, pad=1, nobias=True),
            bn18   = L.BatchNormalization(nF18, use_beta=False, eps=2e-5),
            bias18 = L.Bias(shape=(nF18,)),

            ###### new layer
            conv19 = L.Convolution2D(nF19, nF19, ksize=3, stride=1, pad=1, nobias=True),
            bn19   = L.BatchNormalization(nF19, use_beta=False),
            bias19 = L.Bias(shape=(nF19,)),
            conv20 = L.Convolution2D(nF19, nF19, ksize=3, stride=1, pad=1, nobias=True),
            bn20   = L.BatchNormalization(nF19, use_beta=False),
            bias20 = L.Bias(shape=(nF19,)),
            conv21 = L.Convolution2D(nF19*3, nF19, ksize=3, stride=1, pad=1, nobias=True),
            bn21   = L.BatchNormalization(nF19, use_beta=False),
            bias21 = L.Bias(shape=(nF19,)),
            conv22 = L.Convolution2D(nF19, n_boxes * (5 + n_classes), ksize=1, stride=1, pad=0, nobias=True),
            bias22 = L.Bias(shape=(n_boxes * (5 + n_classes),)),
        )
        self.train = False
        self.finetune = False
        self.n_boxes = n_boxes
        self.n_classes = n_classes

    def __call__(self, x):
	#x.to_cpu()
    	#cv2.imshow('image',x.data[0].transpose(1,2,0))
    	#cv2.waitKey(0)
    	#cv2.destroyAllWindows()
	#x.to_gpu()
        ##### common layer
        h = F.leaky_relu(self.bias1(self.bn1(self.conv1(x), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.max_pooling_2d(h, ksize=2, stride=2, pad=0)
        h = F.leaky_relu(self.bias2(self.bn2(self.conv2(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.max_pooling_2d(h, ksize=2, stride=2, pad=0)
        h = F.leaky_relu(self.bias3(self.bn3(self.conv3(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias4(self.bn4(self.conv4(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias5(self.bn5(self.conv5(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.max_pooling_2d(h, ksize=2, stride=2, pad=0)
        h = F.leaky_relu(self.bias6(self.bn6(self.conv6(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias7(self.bn7(self.conv7(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias8(self.bn8(self.conv8(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.max_pooling_2d(h, ksize=2, stride=2, pad=0)
        h = F.leaky_relu(self.bias9(self.bn9(self.conv9(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias10(self.bn10(self.conv10(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias11(self.bn11(self.conv11(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias12(self.bn12(self.conv12(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias13(self.bn13(self.conv13(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        high_resolution_feature = reorg(h) # 高解像度特徴量をreorgでサイズ落として保存しておく
        h = F.max_pooling_2d(h, ksize=2, stride=2, pad=0)
        h = F.leaky_relu(self.bias14(self.bn14(self.conv14(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias15(self.bn15(self.conv15(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias16(self.bn16(self.conv16(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias17(self.bn17(self.conv17(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias18(self.bn18(self.conv18(h), test=not self.train, finetune=self.finetune)), slope=0.1)

        ###### new layer
        h = F.leaky_relu(self.bias19(self.bn19(self.conv19(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.leaky_relu(self.bias20(self.bn20(self.conv20(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = F.concat((high_resolution_feature, h), axis=1) # output concatenation
        h = F.leaky_relu(self.bias21(self.bn21(self.conv21(h), test=not self.train, finetune=self.finetune)), slope=0.1)
        h = self.bias22(self.conv22(h))

        return h

class YOLOv2Predictor(Chain):
    def __init__(self, predictor):
        super(YOLOv2Predictor, self).__init__(predictor=predictor)
        self.anchors = [[5.375, 5.03125], [5.40625, 4.6875], [2.96875, 2.53125], [2.59375, 2.78125], [1.9375, 3.25]]
        self.thresh = 0.6
        self.seen = 0
        self.unstable_seen = 5000

    def __call__(self, input_x, t):
        isVola = input_x.volatile
        output = self.predictor(input_x)
        batch_size, _, grid_h, grid_w = output.shape
        if self.predictor.train == True:
            self.seen += batch_size
        x, y, w, h, conf, prob = F.split_axis(F.reshape(output, (batch_size, self.predictor.n_boxes, self.predictor.n_classes+5, grid_h, grid_w)), (1, 2, 3, 4, 5), axis=2)
        x = F.sigmoid(x) # xのactivation
        y = F.sigmoid(y) # yのactivation
        conf = F.sigmoid(conf) # confのactivation
        prob = F.transpose(prob, (0, 2, 1, 3, 4))
        prob = F.softmax(prob) # probabilityのactivation


        # 教師データの用意
        tw = np.zeros(w.shape, dtype=np.float32) # wとhが0になるように学習(e^wとe^hは1に近づく -> 担当するbboxの倍率1)
        th = np.zeros(h.shape, dtype=np.float32)
        tx = np.tile(0.5, x.shape).astype(np.float32) # 活性化後のxとyが0.5になるように学習()
        ty = np.tile(0.5, y.shape).astype(np.float32)

        if self.seen < self.unstable_seen: # centerの存在しないbbox誤差学習スケールは基本0.1
            box_learning_scale = np.tile(0.1, x.shape).astype(np.float32)
        else:
            box_learning_scale = np.tile(0, x.shape).astype(np.float32)

        tconf = np.zeros(conf.shape, dtype=np.float32) # confidenceのtruthは基本0、iouがthresh以上のものは学習しない、ただしobjectの存在するgridのbest_boxのみ真のIOUに近づかせる
        conf_learning_scale = np.tile(0.1, conf.shape).astype(np.float32)

        tprob = prob.data.copy() # best_anchor以外は学習させない(自身との二乗和誤差 = 0)

        # 全bboxとtruthのiouを計算(batch単位で計算する)
        x_shift = Variable(np.broadcast_to(np.arange(grid_w, dtype=np.float32), x.shape[1:]), volatile=isVola)
        y_shift = Variable(np.broadcast_to(np.arange(grid_h, dtype=np.float32).reshape(grid_h, 1), y.shape[1:]), volatile=isVola)
        w_anchor = Variable(np.broadcast_to(np.reshape(np.array(self.anchors, dtype=np.float32)[:, 0], (self.predictor.n_boxes, 1, 1, 1)), w.shape[1:]), volatile=isVola)
        h_anchor = Variable(np.broadcast_to(np.reshape(np.array(self.anchors, dtype=np.float32)[:, 1], (self.predictor.n_boxes, 1, 1, 1)), h.shape[1:]), volatile=isVola)
        x_shift.to_gpu(), y_shift.to_gpu(), w_anchor.to_gpu(), h_anchor.to_gpu()
        best_ious = []
        for batch in range(batch_size):
            n_truth_boxes = len(t[batch])
            box_x = (x[batch] + x_shift) * 1.0 / grid_w
            box_y = (y[batch] + y_shift) * 1.0 / grid_h
            box_w = F.exp(w[batch]) * w_anchor * 1.0 / grid_w
            box_h = F.exp(h[batch]) * h_anchor * 1.0 / grid_h

            ious = []
            for truth_index in range(n_truth_boxes):
                truth_box_x = Variable(np.broadcast_to(np.array(t[batch][truth_index]["x"], dtype=np.float32), box_x.shape), volatile=isVola)
                truth_box_y = Variable(np.broadcast_to(np.array(t[batch][truth_index]["y"], dtype=np.float32), box_y.shape), volatile=isVola)
                truth_box_w = Variable(np.broadcast_to(np.array(t[batch][truth_index]["w"], dtype=np.float32), box_w.shape), volatile=isVola)
                truth_box_h = Variable(np.broadcast_to(np.array(t[batch][truth_index]["h"], dtype=np.float32), box_h.shape), volatile=isVola)
                truth_box_x.to_gpu(), truth_box_y.to_gpu(), truth_box_w.to_gpu(), truth_box_h.to_gpu()
                ious.append(multi_box_iou(Box(box_x, box_y, box_w, box_h), Box(truth_box_x, truth_box_y, truth_box_w, truth_box_h)).data.get())  
            ious = np.array(ious)
            best_ious.append(np.max(ious, axis=0))
        best_ious = np.array(best_ious)

        # 一定以上のiouを持つanchorに対しては、confを0に下げないようにする(truthの周りのgridはconfをそのまま維持)。
        tconf[best_ious > self.thresh] = conf.data.get()[best_ious > self.thresh]
        conf_learning_scale[best_ious > self.thresh] = 0

        # objectの存在するanchor boxのみ、x、y、w、h、conf、probを個別修正
        abs_anchors = self.anchors / np.array([grid_w, grid_h])
        for batch in range(batch_size):
            for truth_box in t[batch]:
                truth_w = int(float(truth_box["x"]) * grid_w)
                truth_h = int(float(truth_box["y"]) * grid_h)
                truth_n = 0
                best_iou = 0.0
                for anchor_index, abs_anchor in enumerate(abs_anchors):
                    iou = box_iou(Box(0, 0, float(truth_box["w"]), float(truth_box["h"])), Box(0, 0, abs_anchor[0], abs_anchor[1]))
                    if best_iou < iou:
                        best_iou = iou
                        truth_n = anchor_index

                # objectの存在するanchorについて、centerを0.5ではなく、真の座標に近づかせる。anchorのスケールを1ではなく真のスケールに近づかせる。学習スケールを1にする。
                box_learning_scale[batch, truth_n, :, truth_h, truth_w] = 1.0 
                tx[batch, truth_n, :, truth_h, truth_w] = float(truth_box["x"]) * grid_w - truth_w
                ty[batch, truth_n, :, truth_h, truth_w] = float(truth_box["y"]) * grid_h - truth_h
                tw[batch, truth_n, :, truth_h, truth_w] = np.log(float(truth_box["w"]) * 1.0 / abs_anchors[truth_n][0])
                th[batch, truth_n, :, truth_h, truth_w] = np.log(float(truth_box["h"]) * 1.0 / abs_anchors[truth_n][1])
                tprob[batch, :, truth_n, truth_h, truth_w] = 0
                tprob[batch, int(truth_box["label"]), truth_n, truth_h, truth_w] = 1

                # IOUの観測
                full_truth_box = Box(float(truth_box["x"]), float(truth_box["y"]), float(truth_box["w"]), float(truth_box["h"]))
                predicted_box = Box(
                    (x[batch][truth_n][0][truth_h][truth_w].data.get() + truth_w) * 1.0 / grid_w, 
                    (y[batch][truth_n][0][truth_h][truth_w].data.get() + truth_h) * 1.0 / grid_h,
                    np.exp(w[batch][truth_n][0][truth_h][truth_w].data.get()) * abs_anchors[truth_n][0],
                    np.exp(h[batch][truth_n][0][truth_h][truth_w].data.get()) * abs_anchors[truth_n][1]
                )
                predicted_iou = box_iou(full_truth_box, predicted_box)
                tconf[batch, truth_n, :, truth_h, truth_w] = predicted_iou
                conf_learning_scale[batch, truth_n, :, truth_h, truth_w] = 10.0

            # debug prints
            maps = F.transpose(prob[batch], (2, 3, 1, 0)).data
#            print("best confidences and best conditional probability and predicted class of each grid:")
#            for i in range(grid_h):
#                for j in range(grid_w):
#                    print("%2d" % (int(conf[batch, :, :, i, j].data.max() * 100)), end=" ")
#                print("     ", end="")
#                for j in range(grid_w):
#                    print("%2d" % (maps[i][j][int(maps[i][j].max(axis=1).argmax())].argmax()), end=" ")
#                print("     ", end="")
#                for j in range(grid_w):
#                    print("%2d" % (maps[i][j][int(maps[i][j].max(axis=1).argmax())].max()*100), end=" ")
#                print()
#
#            print("best default iou: %.2f   predicted iou: %.2f   confidence: %.2f   class: %s" % (best_iou, predicted_iou, conf[batch][truth_n][0][truth_h][truth_w].data, t[batch][0]["label"]))
#            print("-------------------------------")
        #print("seen = %d" % self.seen)

        # loss計算
        tx, ty, tw, th, tconf, tprob = Variable(tx, volatile=isVola), Variable(ty, volatile=isVola), Variable(tw, volatile=isVola), Variable(th, volatile=isVola), Variable(tconf, volatile=isVola), Variable(tprob, volatile=isVola)
        box_learning_scale, conf_learning_scale = Variable(box_learning_scale, volatile=isVola), Variable(conf_learning_scale, volatile=isVola)
        tx.to_gpu(), ty.to_gpu(), tw.to_gpu(), th.to_gpu(), tconf.to_gpu(), tprob.to_gpu()
        box_learning_scale.to_gpu()
        conf_learning_scale.to_gpu()

        x_loss = F.sum((tx - x) ** 2 * box_learning_scale) / 2.0
        y_loss = F.sum((ty - y) ** 2 * box_learning_scale) / 2.0
        w_loss = F.sum((tw - w) ** 2 * box_learning_scale) / 2.0
        h_loss = F.sum((th - h) ** 2 * box_learning_scale) / 2.0
        c_loss = F.sum((tconf - conf) ** 2 * conf_learning_scale) / 2.0
        p_loss = F.sum((tprob - prob) ** 2) / 2.0
        print("x_loss: %f  y_loss: %f  w_loss: %f  h_loss: %f  c_loss: %f   p_loss: %f" % 
            (F.sum(x_loss).data, F.sum(y_loss).data, F.sum(w_loss).data, F.sum(h_loss).data, F.sum(c_loss).data, F.sum(p_loss).data)
        )

        loss = x_loss + y_loss + w_loss + h_loss + c_loss + p_loss
        return loss

    def init_anchor(self, anchors):
        self.anchors = anchors

    def predict(self, input_x):
        output = self.predictor(input_x)
        batch_size, input_channel, input_h, input_w = input_x.shape
        batch_size, _, grid_h, grid_w = output.shape
        x, y, w, h, conf, prob = F.split_axis(F.reshape(output, (batch_size, self.predictor.n_boxes, self.predictor.n_classes+5, grid_h, grid_w)), (1, 2, 3, 4, 5), axis=2)
        x = F.sigmoid(x) # xのactivation
        y = F.sigmoid(y) # yのactivation
        conf = F.sigmoid(conf) # confのactivation
        prob = F.transpose(prob, (0, 2, 1, 3, 4))
        prob = F.softmax(prob) # probablitiyのacitivation
        prob = F.transpose(prob, (0, 2, 1, 3, 4))

        # x, y, w, hを絶対座標へ変換
        x_shift = Variable(np.broadcast_to(np.arange(grid_w, dtype=np.float32), x.shape))
        y_shift = Variable(np.broadcast_to(np.arange(grid_h, dtype=np.float32).reshape(grid_h, 1), y.shape))
        w_anchor = Variable(np.broadcast_to(np.reshape(np.array(self.anchors, dtype=np.float32)[:, 0], (self.predictor.n_boxes, 1, 1, 1)), w.shape))
        h_anchor = Variable(np.broadcast_to(np.reshape(np.array(self.anchors, dtype=np.float32)[:, 1], (self.predictor.n_boxes, 1, 1, 1)), h.shape))
        #x_shift.to_gpu(), y_shift.to_gpu(), w_anchor.to_gpu(), h_anchor.to_gpu()
        box_x = (x + x_shift) * 1.0 / grid_w
        box_y = (y + y_shift) * 1.0 / grid_h
        box_w = F.exp(w) * w_anchor * 1.0 / grid_w
        box_h = F.exp(h) * h_anchor * 1.0 / grid_h

        return box_x, box_y, box_w, box_h, conf, prob
