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
import numpy as np
import cv2
import os
import rospkg
import subprocess


from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image
from T2_robot_vision.msg import RecognizedItem, SegmentedData, ItemizedData, RecognizedSegment
from cv_bridge import CvBridge

import chainer
import chainer.functions as F
import chainer.links as L
from chainer import Variable
from chainer import optimizers, serializers

import YOLOv2_master.yolov2_predict


def object_detection(data):
	answer = np.zeros(7)
	data = data.astype(np.float32)
	answer[np.argmax(object_detector(np.expand_dims(data,0)).data)] = 1
	print(answer)
	obj = np.argmax(answer)
	score = np.max(object_detector(np.expand_dims(data/1.0,0)).data)
	return score, obj
 
def autoencoder_input(data):
	data = data/255.0
	return rollangle_detector(np.expand_dims(data,0), train = False)[0].data

def callback(data):
	jn = data.recog_target.job_no
	rospy.loginfo("job_no:%d Segmentation data received", jn)
	item = RecognizedItem()
	item.header = data.header
	item.recog_target = data.recog_target
	item.calibrated_points = data.calibrated_points
	item.rcg_module_index = 3
	item.segmented_data = []
	item.itemized_data = []

	if len(data.segmented_data) != 0:
		maxscore = 0
		converter = CvBridge()
		segment = converter.imgmsg_to_cv2(data.segmented_data[0].mask, desired_encoding="passthrough")
		image = converter.imgmsg_to_cv2(item.recog_target.data[0].rgb, desired_encoding="passthrough")
		depth = converter.imgmsg_to_cv2(item.recog_target.data[0].depth, desired_encoding="passthrough")
		image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
		nms_results = predictor(image_bgr)
		i = 0
		for result in nms_results:
			detection_thresh = 0.8
			for j in range(len(data.recog_target.category_id)):
				category_id = data.recog_target.category_id[j]
				if object_names[result["label"]] == category_id:
					detection_thresh = 0.5
			print "detection_thresh:%0.2f" % (detection_thresh)

			if result["probs"].max()*result["conf"] > detection_thresh:
				left, top = result["box"].int_left_top()
				cv2.rectangle(
					image_bgr,
					result["box"].int_left_top(), result["box"].int_right_bottom(),
					(255, 0, 255), 3)
				seg = SegmentedData()
				seg.module_index = 3
				seg.sx = int(result["box"].x - (result["box"].w / 2.0))
				seg.sy = int(result["box"].y - (result["box"].h / 2.0))
				seg.ex = int(result["box"].x + (result["box"].w / 2.0))
				seg.ey = int(result["box"].y + (result["box"].h / 2.0))
				seg.img_idx = 0
				seg.img_type = 1
				item.segmented_data.append(seg)
				
				test_data = ItemizedData()
				test_data.category = object_names[result["label"]]
				test_data.score = int(float(result["probs"].max()*result["conf"]*100))
				test_data.roll = 0
				test_data.yaw = 0
				test_data.pitch = 0
				test_data.module_index = 3
				test_data.seg_id = i
				i = i + 1
				item.itemized_data.append(test_data)			
				text = 'itemNo:%d, %s(%2d%%)' % (object_names[result["label"]], result["label"], result["probs"].max()*result["conf"]*100)
				cv2.putText(image_bgr, text, (left, top-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

				print(text)

		print "yolo func finished"
	else:
		rospy.loginfo("job_no:%d No segment data received!", jn)
	pub.publish(item)

print("Neural networks loaded!")
rospack = rospkg.RosPack()

object_names = {
	"01_Robots_Everywhere":92,
	"02_Colgate_Toothbrush_4PK":68,
	"03_Band_Aid_Tape":64,
	"04_Hand_Weight":77,
	"05_Tissue_Box":98,
	"06_Scotch_Sponges":93,
	"07_Pie_Plates":87,
	"08_Expo_Eraser":73,
	"09_Reynolds_Wrap":90,
	"10_Avery_Binder":62,
	"11_Burts_Bees_Baby_Wipes":67,
	"12_Ice_Cube_Tray":80,
	"13_Toilet_Brush":99,
	"15_Tennis_Ball_Container":96,
	"17_Table_Cloth":95,
	"18_Epsom_Salts":72,
	"19_Irish_Spring_Soap":81,
	"20_Laugh_Out_Loud_Jokes":82,
	"21_White_Facecloth":100,
	"22_Speed_Stick":94,
	"23_Ticonderoga_Pencils":97,
	"24_Glue_Sticks":76,
	"25_Mouse_Traps":86,
	"26_Duct_Tape":71,
	"27_Composition_Book":69,
	"29_Bath_Sponge":65,
	"30_Flashlight":75,
	"31_Crayons":70,
	"32_Measuring_Spoon":84,
	"33_Black_Fashion_Gloves":66,
	"34_Balloons":63,
	"35_Hanes_Socks":78,
	"36_Robots_DVD":91,
	"39_Hinged_Ruled_Index_Cards":79,
	"40_Fiskars_Scissors":74
}
predictor = YOLOv2_master.yolov2_predict.AnimalPredictor()
print "yolo predictor load done"
rospy.loginfo("Recognition program is running")
pub = rospy.Publisher("yolov2_results", RecognizedItem, queue_size=10)
rospy.init_node('robot_vision_recognition')
r = rospy.Rate(1) # 10hz

rospy.Subscriber("recognized_segment", RecognizedSegment, callback)

while not rospy.is_shutdown():

	r.sleep()
