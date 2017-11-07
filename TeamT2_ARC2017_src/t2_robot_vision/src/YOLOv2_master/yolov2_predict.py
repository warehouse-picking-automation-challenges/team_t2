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

import time
import cv2
import numpy as np
from chainer import serializers, Variable
import chainer.functions as F
import rospkg
import argparse
from yolov2 import *

class AnimalPredictor:
    def __init__(self):
        # 関数化用にrospack追加
        rospack = rospkg.RosPack()
        # hyper parameters
        weight_file = "{}/src/YOLOv2_master/backup/yolov2.model".format(rospack.get_path('T2_robot_vision'))
        self.n_classes = 35 #42 #17 #10
        self.n_boxes = 5
        self.detection_thresh = 0.3 #0.5 #0.05
        self.iou_thresh = 0.3 #0.05
        self.label_file = "{}/src/YOLOv2_master/data/label.txt".format(rospack.get_path('T2_robot_vision'))
        with open(self.label_file, "r") as f:
            self.labels = f.read().strip().split("\n")

        # load model
        print("loading yolov2 model...")
        yolov2 = YOLOv2(n_classes=self.n_classes, n_boxes=self.n_boxes)
        model = YOLOv2Predictor(yolov2)
        serializers.load_hdf5(weight_file, model) # load saved model
        model.predictor.train = False
        model.predictor.finetune = False
        self.model = model

    def __call__(self, orig_img):
        orig_input_height, orig_input_width, _ = orig_img.shape
        #img = cv2.resize(orig_img, (640, 640))
        img = reshape_to_yolo_size(orig_img)
        input_height, input_width, _ = img.shape
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.asarray(img, dtype=np.float32) / 255.0
        img = img.transpose(2, 0, 1)

        # forward
        x_data = img[np.newaxis, :, :, :]
        x = Variable(x_data)
        x, y, w, h, conf, prob = self.model.predict(x)

        # parse results
        _, _, _, grid_h, grid_w = x.shape
        x = F.reshape(x, (self.n_boxes, grid_h, grid_w)).data
        y = F.reshape(y, (self.n_boxes, grid_h, grid_w)).data
        w = F.reshape(w, (self.n_boxes, grid_h, grid_w)).data
        h = F.reshape(h, (self.n_boxes, grid_h, grid_w)).data

        conf = F.reshape(conf, (self.n_boxes, grid_h, grid_w)).data
        prob = F.transpose(F.reshape(prob, (self.n_boxes, self.n_classes, grid_h, grid_w)), (1, 0, 2, 3)).data

        detected_indices = (conf * prob).max(axis=0) > self.detection_thresh


        results = []
        for i in range(detected_indices.sum()):
            results.append({
                "label": self.labels[prob.transpose(1, 2, 3, 0)[detected_indices][i].argmax()],
                "probs": prob.transpose(1, 2, 3, 0)[detected_indices][i],
                "conf" : conf[detected_indices][i],
                "objectness": conf[detected_indices][i] * prob.transpose(1, 2, 3, 0)[detected_indices][i].max(),
                "box"  : Box(
                            x[detected_indices][i]*orig_input_width,
                            y[detected_indices][i]*orig_input_height,
                            w[detected_indices][i]*orig_input_width,
                            h[detected_indices][i]*orig_input_height).crop_region(orig_input_height, orig_input_width)
            })

        # nms
        nms_results = nms(results, self.iou_thresh)
        return nms_results

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    # argument parse
    parser = argparse.ArgumentParser(description="指定したパスの画像を読み込み、bbox及びクラスの予測を行う")
    parser.add_argument('path', help="画像ファイルへのパスを指定")
    args = parser.parse_args()
    image_file = args.path

    # read image
    print("loading image...")
    orig_img = cv2.imread(image_file)

    predictor = AnimalPredictor()
    nms_results = predictor(orig_img)

    # draw result
    f = open("{}/data/temp_image/result.txt".format(rospack.get_path('T2_robot_vision')),"w")
    for result in nms_results:
        left, top = result["box"].int_left_top()
        cv2.rectangle(
            orig_img,
            result["box"].int_left_top(), result["box"].int_right_bottom(),
            (255, 0, 255),
            3
        )
	f.write("{}, {}, {}, {}, {}, {}, \n".format(result["label"],result["probs"].max()*result["conf"]*100,result["box"].x,result["box"].y,result["box"].w,result["box"].h))
        text = '%s(%2d%%)' % (result["label"], result["probs"].max()*result["conf"]*100)
        cv2.putText(orig_img, text, (left, top-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        #print(text)
    f.close()
    cv2.imshow('image',orig_img)
    cv2.imwrite("./Predicted.jpg", orig_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
