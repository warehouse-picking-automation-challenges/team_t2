#!/usr/bin/env python

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


from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image
from T2_robot_vision.msg import CalibratedData, RecognizedItem, SegmentedData, ItemizedData, RecognizedSegment
from cv_bridge import CvBridge

import chainer
import chainer.functions  as F
import chainer.links as L

"""
Parameter Settings
"""
rate = 2
img_h = 128*rate
img_w = 128*rate
#score, cx, cy, xl, yl
d_elem = 5
#mask
d_info = d_elem
#
d_sq = 4
#channels
d_ech = (d_elem)*d_sq
d_ch = d_info*d_sq
#feature map size.
fs = [32*rate, 16*rate, 8*rate, 4*rate]
#size total
fs_total = 0
fs_tlp = [0]#for loop
for cs in fs:
    fs_total += cs**2
    fs_tlp.append(fs_total)
fs_tlp.append(img_h*img_w)
fd = [0.1, 0.25, 0.4, 0.55]

"""
Default Box Info
"""
dflt_info = np.zeros((fs_total, d_sq, d_elem), np.float32)
for l in range(len(fs)):
    for y in range(fs[l]):
        for x in range(fs[l]):
            cx = (x+0.5)*img_w/fs[l]
            cy = (y+0.5)*img_h/fs[l]
            xl = img_w*fd[l]
            yl = img_h*fd[l]
            for q in range(d_sq):
                dflt_info[x+y*fs[l]+fs_tlp[l], q, 0] = 0
                dflt_info[x+y*fs[l]+fs_tlp[l], q, 1] = cx
                dflt_info[x+y*fs[l]+fs_tlp[l], q, 2] = cy            
            dflt_info[x+y*fs[l]+fs_tlp[l], 0, 3] = xl
            dflt_info[x+y*fs[l]+fs_tlp[l], 0, 4] = yl             
            dflt_info[x+y*fs[l]+fs_tlp[l], 1, 3] = xl*2
            dflt_info[x+y*fs[l]+fs_tlp[l], 1, 4] = yl    
            dflt_info[x+y*fs[l]+fs_tlp[l], 2, 3] = xl
            dflt_info[x+y*fs[l]+fs_tlp[l], 2, 4] = yl*2 
            dflt_info[x+y*fs[l]+fs_tlp[l], 3, 3] = xl
            dflt_info[x+y*fs[l]+fs_tlp[l], 3, 4] = yl*3

"""
Draw rect area for jaccard overlap
"""            
def rect4jaccard(image, cx, cy, xl, yl):
    xs = int(cx - xl/2)
    xe = int(cx + xl/2)
    ys = int(cy - yl/2)
    ye = int(cy + yl/2)
    if xs < 0:
        xs = 0
    if xe >= img_w:
        xe = img_w-1
    if ys < 0:
        ys = 0
    if ye >= img_h:
        ye = img_h-1
    image[ys:ye+1, xs:xe+1] = 1
    

def merge_detected_rect(p_result, m_result, th=0.5, u_sig=False, merge_th=0.5, neighbor_th=0.2):
    assert merge_th > neighbor_th, "must merge_th > neighbor_th"
    assert (p_result.shape[0], p_result.shape[2]) == m_result.shape, "array size error"
    merge_info = np.zeros((fs_total*d_sq, d_info), np.float32)
    score = 0
    find_rect_counter = 0
    for i in range(fs_total):
        for j in range(d_sq):
            if u_sig:
                score = 1/(1+np.exp(-p_result[i,j,0]))
                if score < th:
                    continue
            else:
                score = p_result[i,j,0]
                if score < th:
                    continue
            cx = p_result[i,j,1] + dflt_info[i,j,1]
            cy = p_result[i,j,2] + dflt_info[i,j,2]
            xl = p_result[i,j,3] + dflt_info[i,j,3]
            yl = p_result[i,j,4] + dflt_info[i,j,4]
            merge_info[find_rect_counter, 0] = score
            merge_info[find_rect_counter, 1] = cx
            merge_info[find_rect_counter, 2] = cy
            merge_info[find_rect_counter, 3] = xl
            merge_info[find_rect_counter, 4] = yl
            find_rect_counter += 1
    
    merge_info = merge_info[merge_info[:,0].argsort()[::-1]]
    merge_result = np.zeros((fs_total, d_info+1), np.float32)
    merge_counter = -1
    for i in range(find_rect_counter):
        if merge_info[i,0] < th:
            continue
        merge_counter += 1
        cx = merge_info[i,1]
        cy = merge_info[i,2]
        xl = merge_info[i,3]
        yl = merge_info[i,4]
        img_base = np.zeros((img_h, img_w), np.int32)
        rect4jaccard(img_base, cx, cy, xl, yl)
        if len(img_base[img_base>0])==0:
            merge_counter -= 1
            continue
        merge_result[merge_counter, 0] = merge_info[i,0]
        merge_result[merge_counter, 1] = cx
        merge_result[merge_counter, 2] = cy
        merge_result[merge_counter, 3] = xl
        merge_result[merge_counter, 4] = yl
        merge_result[merge_counter, d_info] = 1
        #merge check
        for j in range(i,find_rect_counter):
            if merge_info[j,0] < th:
                continue
            cx_d = merge_info[j,1]
            cy_d = merge_info[j,2]
            xl_d = merge_info[j,3]
            yl_d = merge_info[j,4]
            img_tmp = np.zeros((img_h, img_w), np.int32)
            rect4jaccard(img_tmp, cx_d, cy_d, xl_d, yl_d)
            sum_map = img_base + img_tmp
            count_or = len(sum_map[sum_map>0])
            count_and = len(sum_map[sum_map==2])
            if count_and*1.0/count_or < neighbor_th :
                continue
            elif count_and*1.0/count_or < merge_th :
                merge_info[j,0] = -i
                continue
            merge_info[j,0] = -i
            #continue
            merge_result[merge_counter, 1] += cx_d
            merge_result[merge_counter, 2] += cy_d
            merge_result[merge_counter, 3] += xl_d
            merge_result[merge_counter, 4] += yl_d
            merge_result[merge_counter, d_info] += 1
            
    merge_counter = 0
    for i in range(fs_total):
        if merge_result[i,0] == 0 or merge_result[i,d_info] == 0:
            break
        m_result[merge_counter, 0] = merge_result[i, 0]
        m_result[merge_counter, 1] = merge_result[i, 1]/merge_result[i,d_info]
        m_result[merge_counter, 2] = merge_result[i, 2]/merge_result[i,d_info]
        m_result[merge_counter, 3] = merge_result[i, 3]/merge_result[i,d_info]
        m_result[merge_counter, 4] = merge_result[i, 4]/merge_result[i,d_info]
        merge_counter += 1
      
class SSD_MASK2_DEPTH_PICK(chainer.Chain):
    def __init__(self):
        super(SSD_MASK2_DEPTH_PICK, self).__init__(
        #detph
        cd0 = L.Convolution2D(1,4,3,1,1),
        bnd0 = L.BatchNormalization(4),
        cd1 = L.Convolution2D(4,16,3,1,1),
        bnd1 = L.BatchNormalization(16),
        cd2 = L.Convolution2D(16,32,3,1,1),
        bnd2 = L.BatchNormalization(32),
        cd3 = L.Convolution2D(32,64,3,1,1),
        bnd3 = L.BatchNormalization(64),
        #RGB
        c0 = L.Convolution2D(3,12,3,1,1),
        bn0 = L.BatchNormalization(12),
        c1 = L.Convolution2D(12,48,3,1,1),
        bn1 = L.BatchNormalization(48),
        c2 = L.Convolution2D(64,128,3,1,1),#48+16
        bn2 = L.BatchNormalization(128),
        c3 = L.Convolution2D(128,192,3,1,1),
        bn3 = L.BatchNormalization(192),
        c4 = L.Convolution2D(256,512,3,1,1),#192+64
        bn4 = L.BatchNormalization(512),

        #feature
        f1 = L.Convolution2D(512,512,3,1,1),
        bnr1 = L.BatchNormalization(512),
        f2 = L.Convolution2D(512,256,3,2,1),
        bnr2 = L.BatchNormalization(256),
        f3 = L.Convolution2D(256,128,3,2,1),
        bnr3 = L.BatchNormalization(128),
        f4 = L.Convolution2D(128,128,3,2,1),
        bnr4 = L.BatchNormalization(128),

        #classifier
        e1p1 = L.Convolution2D(512,64,3,1,1),#f1->D
        e1p2 = L.Convolution2D(64,d_ech,3,1,1),#f1->D
        
        e2p1 = L.Convolution2D(256,64,3,1,1),#f2->D
        e2p2 = L.Convolution2D(64,d_ech,3,1,1),#f2->D
        
        e3p1 = L.Convolution2D(128,64,3,1,1),#f3->D
        e3p2 = L.Convolution2D(64,d_ech,3,1,1),#f3->D
        
        e4p1 = L.Convolution2D(128,64,3,1,1),#f4->D
        e4p2 = L.Convolution2D(64,d_ech,3,1,1),#f4->D
        
        )
        
    def __call__(self, x):        
        xc, xd= F.split_axis(x, [3], axis=1)
        
        cdr0 = F.relu(self.bnd0(self.cd0(xd)))
        cdr1 = F.relu(self.bnd1(self.cd1(cdr0)))
        cdr2 = F.max_pooling_2d(F.relu(self.bnd2(self.cd2(cdr1))),2)
        cdr3 = F.relu(self.bnd3(self.cd3(cdr2)))
        
        cr0 = F.relu(self.bn0(self.c0(xc)))
        cr1 = F.relu(self.bn1(self.c1(cr0)))
        c1_d = F.concat((cr1, cdr1), axis=1)
        cr2 = F.max_pooling_2d(F.relu(self.bn2(self.c2(c1_d))),2)
        cr3 = F.relu(self.bn3(self.c3(cr2)))
        c3_d = F.concat((cr3, cdr3), axis=1)
        cr4 = F.max_pooling_2d(F.relu(self.bn4(self.c4(c3_d))),2)
        
        fr1 = F.relu(self.bnr1(self.f1(cr4)))#32x32
        fr2 = F.relu(self.bnr2(self.f2(fr1)))#16x16
        fr3 = F.relu(self.bnr3(self.f3(fr2)))#8x8
        fr4 = F.relu(self.bnr4(self.f4(fr3)))#4x4
        
        b_size = x.shape[0]
        er1r = F.reshape(self.e1p2(self.e1p1(fr1)),(b_size, d_ech, fs[0]**2))
        
        er2r = F.reshape(self.e2p2(self.e2p1(fr2)),(b_size, d_ech, fs[1]**2))
        
        er3r = F.reshape(self.e3p2(self.e3p1(fr3)),(b_size, d_ech, fs[2]**2))
        
        er4r = F.reshape(self.e4p2(self.e4p1(fr4)),(b_size, d_ech, fs[3]**2))
        
        hc = F.concat((er1r, er2r, er3r, er4r), axis=2)
        ht = F.transpose(hc, (0, 2, 1))
        h = F.reshape(ht, (b_size, fs_total, d_sq, d_info))
        
        return h
        
class SSD_MASK_PICK_Net(chainer.Chain):
    def __init__(self, predictor):
        super(SSD_MASK_PICK_Net, self).__init__(predictor=predictor)
       
model_SSD_MASK_PICK = SSD_MASK_PICK_Net(SSD_MASK2_DEPTH_PICK())

file_path = '%s'%rospkg.RosPack().get_path('T2_robot_vision')
print file_path
chainer.serializers.load_npz(file_path+'/data/pick_ssd_ARC_gpu.model', model_SSD_MASK_PICK)
print 'seg data loaded'

def seg_ARC_predict(image_c, image_d):
    test_images = np.empty((1, img_h, img_w, 4))
    image_d = cv2.medianBlur(image_d, 3)
    sx = 40
    sy = 20
    sub_w = 500
    sub_h = 375
    image_c = image_c[sy:sy+sub_h, sx:sx+sub_w] 
    image_d = image_d[sy:sy+sub_h, sx:sx+sub_h] 
    image_c = cv2.resize(image_c,(img_h, img_w)) 
    image_d = cv2.resize(image_d,(img_h, img_w)) 
    test_images[0,:,:,0] = image_c[:,:,0]/255.0
    test_images[0,:,:,1] = image_c[:,:,1]/255.0
    test_images[0,:,:,2] = image_c[:,:,2]/255.0
    test_images[0,:,:,3] = image_d[:,:]/255.0
    data_test = test_images.transpose(0,3,1,2)
    data_test = data_test.astype(np.float32)
    z = model_SSD_MASK_PICK.predictor(data_test)
    merge_result = np.zeros((fs_total, d_info), np.float32)
    merge_detected_rect(z.data[0,:,:,:], merge_result, 0.8, True, 0.8, 0.1)
    #merge_result = merge_result[merge_result[:,0].argsort()[::-1]]
    for i in range(merge_result.shape[0]):
        merge_result[i,1] = merge_result[i,1]*sub_w/img_w + sx
        merge_result[i,2] = merge_result[i,2]*sub_h/img_h + sy
        merge_result[i,3] = merge_result[i,3]*sub_w/img_w
        merge_result[i,4] = merge_result[i,4]*sub_h/img_h
    return merge_result

def callback(data):
    rospy.loginfo("segment_python node start")
    item = RecognizedSegment()
    item.header            = data.header
    item.recog_target      = data.recog_target
    item.calibrated_points = data.calibrated_points
    item.segmented_data    = []

    converter = CvBridge()
    bgrImg = converter.imgmsg_to_cv2(data.recog_target.data[0].rgb_1,
                                     desired_encoding="passthrough")
    rgbImg = cv2.cvtColor(bgrImg, cv2.COLOR_RGB2BGR)
    depthImg = converter.imgmsg_to_cv2(data.recog_target.data[0].depth,
                                       desired_encoding="passthrough")
    merge_result = seg_ARC_predict(rgbImg, depthImg)
    for i in range(5):
        if merge_result[i, 0] < 0.8:
            break
        sx = int(merge_result[i, 1] - merge_result[i, 3]/2)
        ex = int(merge_result[i, 1] + merge_result[i, 3]/2)
        sy = int(merge_result[i, 2] - merge_result[i, 4]/2)
        ey = int(merge_result[i, 2] + merge_result[i, 4]/2)
        if sx < 0:
            sx = 0
        if sx >= 640:
            sx = 640-1
        if sy < 0:
            sy = 0
        if sy >= 480:
            sy = 480-1
        if ex < 0:
            ex = 0
        if ex >= 640:
            ex = 640-1
        if ey < 0:
            ey = 0
        if ey >= 480:
            ey = 480-1
        resultItem_data = SegmentedData()
        resultItem_data.sx = sx
        resultItem_data.sy = sy
        resultItem_data.ex = ex
        resultItem_data.ey = ey
        #print sx, sy, ex, ey
        cv2.rectangle(rgbImg, (sx,sy),(ex,ey), (0,0,255), 1)
        item.segmented_data.append(resultItem_data)
    pub.publish(item)    
    cv2.imwrite(file_path + "/data/debug/seg_result.png", rgbImg)
    rospy.loginfo("segment_python node end")

pub = rospy.Publisher("recognized_segment_sub", RecognizedSegment, queue_size=10)
rospy.init_node('seg_ARC_ROS')
r = rospy.Rate(1)
rospy.Subscriber("calibrated_data_sub", CalibratedData, callback)

while not rospy.is_shutdown():

    r.sleep()

