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

import numpy

from chainer import cuda
from chainer import function
from chainer.utils import type_check


class DistEmbed(function.Function):
    
    """Distance Embedding loss function."""
    
    def __init__(self, d_th):
        if d_th <= 0:
            raise ValueError('out_th should be positive value.')
        self.d_th = d_th
 
        
    def check_type_forward(self, in_types):
        type_check.expect(in_types.size() == 3)        

        zi, zj, dist = in_types
        
        type_check.expect(
            zi.dtype   == numpy.float32,
            zj.dtype   == numpy.float32,
            dist.dtype == numpy.float32,
            zi.shape == zj.shape,
            zi.shape[0] == dist.shape[0],
            zi.shape[0] > 0
        )

    def forward(self, inputs):
        # data1つ分のloss
        #  = ┌     | dij - ||zi-zj|| |   if dij < d_th
        #    └ max(0,dij - ||zi-zj||)    if dij > d_th
        #data数の分平均したものを出力する
        
        xp = cuda.get_array_module(*inputs)
        zi,zj,dij = inputs
        N=zi.shape[0]
        d_of_zi_zj = xp.linalg.norm(zi-zj,axis=1)
        
        ### dij<d_thの時のloss
        isInTh = dij<self.d_th
        lossAll = xp.linalg.norm( (dij - d_of_zi_zj) * isInTh,  ord=1)
        
        ### dij>d_thの時のloss
        lossAll += xp.sum( (1-isInTh) * xp.maximum(dij-d_of_zi_zj, 0) )
        
        loss=lossAll/N
        
        return xp.array(loss, dtype=xp.float32),


    def backward(self, inputs, grad_outputs):
        xp = cuda.get_array_module(*inputs)
        zi,zj,dij = inputs
        dE_dLoss, = grad_outputs #この値は1のはず

        sa=zi-zj
        d_of_zi_zj=xp.linalg.norm(sa,axis=1)
        d_of_zi_zj=xp.maximum(d_of_zi_zj,1e-8) # avoid division by zero
        d_of_zi_zj=d_of_zi_zj[:,xp.newaxis] #縦ベクトル化
        dij=dij[:,xp.newaxis] #縦ベクトル化
        
        A=(d_of_zi_zj<dij)
        C=(dij<self.d_th)
        
        #signの値
        # +1,  if   dij    <   d_th   < ||zi-zj||
        # +1,  if   dij    < ||zi-zj||<   d_th
        #  0,  if   d_th   <   dij    < ||zi-zj||
        # -1,  if   d_th   < ||zi-zj||<   dij
        # -1,  if ||zi-zj||<   dij    <   d_th
        # -1,  if ||zi-zj||<   d_th   <   dij
        sign = -1*A + (1-A)*C

        dLoss_dzi = sign*sa/d_of_zi_zj
        dE_dzi = (dE_dLoss*dLoss_dzi).astype(xp.float32)
        
        return dE_dzi, -dE_dzi, None
        
        
def dist_embed(zi,zj,dij, d_th=15.):
    """Computes Distance embedding loss."""
        
    return DistEmbed(d_th)(zi,zj,dij)
        
        
        
        
        
        
        