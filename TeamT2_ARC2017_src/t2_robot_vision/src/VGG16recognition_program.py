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


from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image
from T2_robot_vision.msg import RecognizedItem, SegmentedData, ItemizedData, RecognizedSegment
from cv_bridge import CvBridge

import chainer
import chainer.functions as F
import chainer.links as L
from chainer import Variable
from chainer import optimizers, serializers

class VGGNet(chainer.Chain):
    def __init__(self):
        super(VGGNet, self).__init__(
	    bnorm1 = L.BatchNormalization(3),
            conv1_1=L.Convolution2D(3, 64, 3, stride=1, pad=1),
            conv1_2=L.Convolution2D(64, 64, 3, stride=1, pad=1),
	    bnorm2 = L.BatchNormalization(64),

            conv2_1=L.Convolution2D(64, 128, 3, stride=1, pad=1),
            conv2_2=L.Convolution2D(128, 128, 3, stride=1, pad=1),
	    bnorm3 = L.BatchNormalization(128),

            conv3_1=L.Convolution2D(128, 256, 3, stride=1, pad=1),
            conv3_2=L.Convolution2D(256, 256, 3, stride=1, pad=1),
            conv3_3=L.Convolution2D(256, 256, 3, stride=1, pad=1),
	    bnorm4 = L.BatchNormalization(256),

            conv4_1=L.Convolution2D(256, 512, 3, stride=1, pad=1),
            conv4_2=L.Convolution2D(512, 512, 3, stride=1, pad=1),
            conv4_3=L.Convolution2D(512, 512, 3, stride=1, pad=1),
	    bnorm5 = L.BatchNormalization(512),

            conv5_1=L.Convolution2D(512, 512, 3, stride=1, pad=1),

	    bnorm6 = L.BatchNormalization(512),

        )
	self.train = False
    def __call__(self, x, train=False):
        x = chainer.Variable(x)
	x.to_gpu()
	h = self.bnorm1(x, test = not train)

        h = F.relu(self.conv1_1(h))
        h = F.relu(self.conv1_2(h))
        h = F.max_pooling_2d(h, 2, stride=2)

	h = self.bnorm2(h, test = not train)

        h = F.relu(self.conv2_1(h))
        h = F.relu(self.conv2_2(h))
        h = F.max_pooling_2d(h, 2, stride=2)

	h = self.bnorm3(h, test = not train)

        h = F.relu(self.conv3_1(h))
        h = F.relu(self.conv3_2(h))
        h = F.relu(self.conv3_3(h))
        h = F.max_pooling_2d(h, 2, stride=2)

	h = self.bnorm4(h, test = not train)

        h = F.relu(self.conv4_1(h))
        h = F.relu(self.conv4_2(h))
        h = F.relu(self.conv4_3(h))
        h = F.max_pooling_2d(h, 2, stride=2)

	h = self.bnorm5(h, test = not train)

        h = F.relu(self.conv5_1(h))

	h = self.bnorm6(h, test = not train)

	return h

class ARCClassifier(chainer.Chain):
    def __init__(self, predictor):
        super(ARCClassifier, self).__init__(
		predictor=predictor,
            	conv5_2=L.Convolution2D(512, 512, 3, stride=1, pad=1),
            	conv5_3=L.Convolution2D(512, 512, 3, stride=1, pad=1),
            	fc6=L.Linear(25088, 4096),
            	fc7=L.Linear(4096, 4096),
            	fc8=L.Linear(4096, 7),
            )
        self.train = False
    def __call__(self, x):
        h = self.predictor(x, self.train)
        h = F.relu(self.conv5_2(h))
        h = F.relu(self.conv5_3(h))
        h = F.max_pooling_2d(h, 2, stride=2)
        h = F.dropout(F.relu(self.fc6(h)), train=self.train, ratio=0.5)
        h = F.dropout(F.relu(self.fc7(h)), train=self.train, ratio=0.5)
        h = self.fc8(h)
        loss = F.softmax(h)
	loss.to_cpu()
        return loss

            
class AutoEncoder(chainer.Chain):
    def __init__(self):
        super(AutoEncoder, self).__init__(
		bnorm0 = L.BatchNormalization(3),
		conv1 = L.Convolution2D(in_channels=3, out_channels=32, ksize = 3, stride = 1, pad = 1),
		bnorm1 = L.BatchNormalization(32),
		conv2 = L.Convolution2D(in_channels=32, out_channels=32, ksize = 3, stride = 1, pad = 1),
		bnorm2 = L.BatchNormalization(32),

		conv3 = L.Convolution2D(in_channels=32, out_channels=64, ksize = 3, stride = 1, pad = 1),
		bnorm3 = L.BatchNormalization(64),
		conv4 = L.Convolution2D(in_channels=64, out_channels=64, ksize = 3, stride = 1, pad = 1),
		bnorm4 = L.BatchNormalization(64),

		conv5 = L.Convolution2D(in_channels=64, out_channels=128, ksize = 3, stride = 1, pad = 1),
		bnorm5 = L.BatchNormalization(128),
		conv6 = L.Convolution2D(in_channels=128, out_channels=128, ksize = 3, stride = 1, pad = 1),
		bnorm6 = L.BatchNormalization(128),
		conv7 = L.Convolution2D(in_channels=128, out_channels=128, ksize = 3, stride = 1, pad = 1),
		bnorm7 = L.BatchNormalization(128),
		lin1 = L.Linear(in_size=100352,out_size=256),
	   	conv8 = L.Convolution2D(in_channels=128, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm8 = L.BatchNormalization(256),
    		conv9 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm9 = L.BatchNormalization(256),
    		conv10 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm10 = L.BatchNormalization(256),

    		conv11 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm11 = L.BatchNormalization(256),
    		conv12 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm12 = L.BatchNormalization(256),
    		conv13 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm13 = L.BatchNormalization(256),

    		conv14 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm14 = L.BatchNormalization(256),
    		conv15 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm15 = L.BatchNormalization(256),
    		conv16 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm16 = L.BatchNormalization(256),

    		conv17 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm17 = L.BatchNormalization(256),
    		conv18 = L.Convolution2D(in_channels=256, out_channels=256, ksize = 3, stride = 1, pad = 1),
    		bnorm18 = L.BatchNormalization(256),
    		conv19 = L.Convolution2D(in_channels=256, out_channels=128, ksize = 3, stride = 1, pad = 1),
    		bnorm19 = L.BatchNormalization(128),
		lin2 = L.Linear(in_size=256,out_size=100352),
		conv20 = L.Convolution2D(in_channels=128, out_channels=128, ksize = 3, stride = 1, pad = 1),
		bnorm20 = L.BatchNormalization(128),
		conv21 = L.Convolution2D(in_channels=128, out_channels=128, ksize = 3, stride = 1, pad = 1),
		bnorm21 = L.BatchNormalization(128),
		conv22 = L.Convolution2D(in_channels=128, out_channels=64, ksize = 3, stride = 1, pad = 1),
		bnorm22 = L.BatchNormalization(64),

      		conv23 = L.Convolution2D(in_channels=64, out_channels=64, ksize = 3, stride = 1, pad = 1),
		bnorm23 = L.BatchNormalization(64),
		conv24 = L.Convolution2D(in_channels=64, out_channels=32, ksize = 3, stride = 1, pad = 1),
		bnorm24 = L.BatchNormalization(32),

      		conv25 = L.Convolution2D(in_channels=32, out_channels=32, ksize = 3, stride = 1, pad = 1),
		bnorm25 = L.BatchNormalization(32),
		conv26 = L.Convolution2D(in_channels=32, out_channels=3, ksize = 3, stride = 1, pad = 1),
		bnorm26 = L.BatchNormalization(3),
        )
    def __call__(self, x, train=False):
        x = chainer.Variable(x)
	x.to_gpu()
	h = self.bnorm0(x, test = not train)
        h = F.relu(self.bnorm2(self.conv2(F.relu(self.bnorm1(self.conv1(h), test = not train))), test = not train))
	h = F.max_pooling_2d(h, ksize = 3, stride = 2)
        h = F.relu(self.bnorm4(self.conv4(F.relu(self.bnorm3(self.conv3(h), test = not train))), test = not train))
	h = F.max_pooling_2d(h, ksize = 3, stride = 2)
        h = F.relu(self.bnorm7(self.conv7(F.relu(self.bnorm6(self.conv6(F.relu(self.bnorm5(self.conv5(h), test = not train))), test = not train))), test = not train))
	h = F.max_pooling_2d(h, ksize = 3, stride = 2)
	#h = F.relu(self.bnorm10(self.conv10(F.relu(self.bnorm9(self.conv9(F.relu(self.bnorm8(self.conv8(h), test = not train))), test = not train))), test = not train))


	#h = F.max_pooling_2d(h, ksize = 3, stride = 2)
        #h = F.relu(self.bnorm13(self.conv13(F.relu(self.bnorm12(self.conv12(F.relu(self.bnorm11(self.conv11(h)))))))))
	#h = F.max_pooling_2d(h, ksize = 3, stride = 2)
	h_dim = h.shape
	h = self.lin1(h)
	h = self.lin2(h)
	h = F.reshape(h,h_dim)
	#h = F.unpooling_2d(h, ksize = 3, stride = 2)
        #h = F.relu(self.bnorm16(self.conv16(F.relu(self.bnorm15(self.conv15(F.relu(self.bnorm14(self.conv14(h)))))))))
	#h = F.unpooling_2d(h, ksize = 3, stride = 2)


	#h = F.relu(self.bnorm19(self.conv19(F.relu(self.bnorm18(self.conv18(F.relu(self.bnorm17(self.conv17(h), test = not train))), test = not train))), test = not train))
	h = F.unpooling_2d(h, ksize = 3, stride = 2)
        h = F.relu(self.bnorm22(self.conv22(F.relu(self.bnorm21(self.conv21(F.relu(self.bnorm20(self.conv20(h), test = not train))), test = not train))), test = not train))
	h = F.unpooling_2d(h, ksize = 3, stride = 2)
        h = F.relu(self.bnorm24(self.conv24(F.relu(self.bnorm23(self.conv23(h), test = not train))), test = not train))
	h = F.unpooling_2d(h, ksize = 3, stride = 2)
        h = F.relu(self.bnorm26(self.conv26(F.relu(self.bnorm25(self.conv25(h), test = not train))), test = not train))
        return F.mean_squared_error(h, x), h

def object_detection(data):
	answer = np.zeros(7)
	data = data.astype(np.float32)
	answer[np.argmax(object_detector(np.expand_dims(data,0)).data)] = 1
	print(answer)
	#answer = answer.reshape((1,7,24,3))
	obj = np.argmax(answer)
	#yaw = answer[0][2]
	#pitch = answer[0][3]
	score = np.max(object_detector(np.expand_dims(data/1.0,0)).data)
	return score, obj
 
def autoencoder_input(data):
	data = data/255.0
	return rollangle_detector(np.expand_dims(data,0), train = False)[0].data

def callback(data):
	jn = data.recog_target.job_no
	rospy.loginfo("job_no:%d Segmentation data received", jn)
	#rospy.loginfo("Segmentation data received")
	item = RecognizedItem()
	item.header = data.header
	item.recog_target = data.recog_target
	item.calibrated_points = data.calibrated_points
	item.segmented_data = data.segmented_data

	if len(data.segmented_data) != 0:
		maxscore = 0
		#Convert images from ROS format to OpenCV format:
		converter = CvBridge()
		segment = converter.imgmsg_to_cv2(data.segmented_data[0].mask, desired_encoding="passthrough")
		image = converter.imgmsg_to_cv2(item.recog_target.data[0].rgb_1, desired_encoding="passthrough")
		depth = converter.imgmsg_to_cv2(item.recog_target.data[0].depth, desired_encoding="passthrough")
        	#cv2.imshow('image',(image.astype(np.uint8)))
        	#cv2.waitKey(0)
        	#cv2.destroyAllWindows()

		#Cropping + masking:
		cropped_image = cv2.bitwise_and(image[data.segmented_data[0].sy:data.segmented_data[0].ey+1,data.segmented_data[0].sx:data.segmented_data[0].ex+1,:],image[data.segmented_data[0].sy:data.segmented_data[0].ey+1,data.segmented_data[0].sx:data.segmented_data[0].ex+1,:],mask = segment)
		cropped_depth = cv2.bitwise_and(depth[data.segmented_data[0].sy:data.segmented_data[0].ey+1,data.segmented_data[0].sx:data.segmented_data[0].ex+1],depth[data.segmented_data[0].sy:data.segmented_data[0].ey+1,data.segmented_data[0].sx:data.segmented_data[0].ex+1],mask = segment)

		#Save images for debugging:
		#cv2.imwrite('%s/data/%d_cropped_image.bmp'%(rospack.get_path('T2_robot_vision'), callback.counter),cropped_image)
		#cv2.imwrite('%s/data/%d_cropped_depth.png'%(rospack.get_path('T2_robot_vision'), callback.counter),cropped_depth)
		
		#Convert to np.float32
		cropped_image = cropped_image.astype(np.float32)
		cropped_depth = cropped_depth.astype(np.float32)

		#Taking depth image's average non-zero values for image scaling:
		depth_mean = np.mean(cropped_depth.ravel()[np.flatnonzero(cropped_depth)])

           	prepared_image = cropped_image

		#See which rotation angle has best score:
		min_error = 100000
		for angle in range(0,360,15):
			#Scaling by depth_mean:
			#If it's realtime detection the depth value is 10x larger, so we should change the 290.0 below to 2900.0
			rotated_image = cv2.resize(prepared_image,None,fx=(depth_mean/290.0), fy=(depth_mean/290.0), interpolation = cv2.INTER_LINEAR)
			cropped_image = rotated_image
	
			#Padding borders with 0's in order to make the image 228x228:
			if cropped_image.shape[0] > 224:
				cropped_image = cropped_image[cropped_image.shape[0]/2 - 112:cropped_image.shape[0]/2 + 112,:]
			if cropped_image.shape[1] > 224:
				cropped_image = cropped_image[:,cropped_image.shape[1]/2 - 112:cropped_image.shape[1]/2 + 112]
			resized_image = cv2.copyMakeBorder(cropped_image,(224 - cropped_image.shape[0])/2,(224 - cropped_image.shape[0])/2,(224 - cropped_image.shape[1])/2,(224 - cropped_image.shape[1])/2,cv2.BORDER_CONSTANT,value=0)
			resized_image = cv2.copyMakeBorder(resized_image,(224 - resized_image.shape[0]),0,(224 - resized_image.shape[1]),0,cv2.BORDER_CONSTANT,value=0)

			#Rotate image:
			M = cv2.getRotationMatrix2D((112,112),angle,1)
			rotated_image = cv2.warpAffine(resized_image,M,(224,224))

			#Resize image to 56x56:
			#resized_image = cv2.resize(rotated_image,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_LINEAR)

			#Normalize color intensity:
           		color_coef = 128.0/np.mean(resized_image.ravel()[np.flatnonzero(resized_image)])
			resized_image = resized_image*color_coef

			#Get autoencoder error at each angle:
			error = autoencoder_input(resized_image.transpose(2,0,1))

			#print("CNN Score: %f, Roll: %d, Category: %d, Yaw: %d, Pitch: %d" %(score, angle, obj, roll*15 + 5, yaw*15 + 25))
			#rospy.logdebug("Autoencoder error: %f, Roll: %d" %(error, angle))
			print("Autoencoder error: %f, Roll: %d" %(error, angle))
			if error < min_error:
				min_error = error
				correct_angle = angle


		#Scaling by depth_mean:
		#If it's realtime detection the depth value is 10x larger, so we should change the 290.0 below to 2900.0
		resized_image = cv2.resize(prepared_image,None,fx=(depth_mean/290.0), fy=(depth_mean/290.0), interpolation = cv2.INTER_LINEAR)
		#cv2.imshow('image',resized_image)
		#cv2.waitKey(0)
		#cv2.destroyAllWindows()
		cropped_image = resized_image
		#Padding borders with 0's in order to make the image 224x224:
		if cropped_image.shape[0] > 224:
			cropped_image = cropped_image[cropped_image.shape[0]/2 - 112:cropped_image.shape[0]/2 + 112,:]
		if cropped_image.shape[1] > 224:
			cropped_image = cropped_image[:,cropped_image.shape[1]/2 - 112:cropped_image.shape[1]/2 + 112]
		resized_image = cv2.copyMakeBorder(cropped_image,(224 - cropped_image.shape[0])/2,(224 - cropped_image.shape[0])/2,(224 - cropped_image.shape[1])/2,(224 - cropped_image.shape[1])/2,cv2.BORDER_CONSTANT,value=0)
		resized_image = cv2.copyMakeBorder(resized_image,(224 - resized_image.shape[0]),0,(224 - resized_image.shape[1]),0,cv2.BORDER_CONSTANT,value=0)

		#Rotate image:
		M = cv2.getRotationMatrix2D((112,112), correct_angle,1)
		rotated_image = cv2.warpAffine(resized_image,M,(224,224))

		#Resize image to 57x57:
		#resized_image = cv2.resize(rotated_image,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_LINEAR)

		#Normalize color intensity:
           	#color_coef = 128.0/np.mean(resized_image.ravel()[np.flatnonzero(resized_image)])
		#resized_image = resized_image*color_coef

		#Subtract mean:
		final_image = resized_image - np.asarray([43.38757584,  46.62129445,  49.26503253])


		#Detect object:
		score, obj = object_detection(final_image.transpose(2,0,1)[:,:,::-1])
		print("Detected category at %d degrees: %d, with largest confidence score %f" %(correct_angle, obj, score))
		#rospy.loginfo("job_no:%d Detected category at %d degrees: %d, with largest confidence score %f" %(jn, correct_angle, obj, score))
		#rospy.loginfo("Detected category at %d degrees: %d, with largest confidence score %f" %(correct_angle, obj, score))
		
		#Save detection results in file:
		#f.write("%d,%d,%d,%d,%d\n" %(callback.counter,obj,correct_angle,yaw*15+5,pitch*15+25))
		#callback.counter = callback.counter + 1

		#Insert recognition results in the variables below:
		item.itemized_data = []
		test_data = ItemizedData()
		test_data.category = obj
		test_data.roll = -correct_angle
		test_data.yaw = 0
		test_data.pitch = 0
		test_data.score = 100.0*score
		test_data.seg_id = 0
		item.itemized_data.append(test_data)
		rospy.loginfo("job_no:%d Detected category:%d pitch %d yaw: %d roll:%d score %f seg_id%d"
		 %(jn, test_data.category, test_data.pitch, test_data.yaw, test_data.roll, test_data.score, test_data.seg_id))

	else:
		#print("No segment data received!")	
		rospy.loginfo("job_no:%d No segment data received!", jn)
		#rospy.loginfo("No segment data received!")

	pub.publish(item)

print("Neural networks loaded!")
object_detector = ARCClassifier(VGGNet())
rollangle_detector = AutoEncoder()
object_detector.to_gpu()
rollangle_detector.to_gpu()

#callback.counter = 1

rospack = rospkg.RosPack()
#print("Copy the net_param folder inside '\\133.115.57.8\prj\APCプロジェクト\サブＷＧ\認識\個人フォルダ\caio' to the data folder, otherwise this program won't work!")
#f = open('%s/data/neural_results.csv' %rospack.get_path('T2_robot_vision'), 'w')
serializers.load_npz('%s/data/net_param/VGG16_based_network.npz' %rospack.get_path('T2_robot_vision'), object_detector)
serializers.load_npz('%s/data/net_param/ARCAutoEncoder.npz' %rospack.get_path('T2_robot_vision'), rollangle_detector)

rospy.loginfo("Recognition program is running")
pub = rospy.Publisher("recognized_item", RecognizedItem, queue_size=10)
rospy.init_node('robot_vision_recognition')
r = rospy.Rate(1) # 10hz

rospy.Subscriber("recognized_segment", RecognizedSegment, callback)

while not rospy.is_shutdown():

	r.sleep()
