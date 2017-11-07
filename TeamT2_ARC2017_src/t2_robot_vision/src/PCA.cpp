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

#include <math.h>

#include <PCA.h>


double unitvec( double v[] )
{
  double d, eps=0.00001;
  if( (d=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])) < eps ){
    return 0.;
  }
  v[0] /= d;  v[1] /= d;  v[2] /= d;  
  return d;
}

void CrossProduct(double u1[3],double u2[3],double u3[3])
{
  u3[0] = u1[1]*u2[2] - u1[2]*u2[1];
  u3[1] = u1[2]*u2[0] - u1[0]*u2[2];
  u3[2] = u1[0]*u2[1] - u1[1]*u2[0];
}

void SortLum(double lu[3],double ve1[3],double ve2[3],double ve3[3])
{
  double temp=0.0;
  double vtemp = 0.0;
  double vector[3][3];
  int i,j,k;

  // データ読み込み
  for(i = 0; i< 3; i++){
    vector[0][i] = ve1[i];
    vector[1][i] = ve2[i];
    vector[2][i] = ve3[i];
  }
  // バブルソートを使って、降順に並び替え
  for (i = 0; i < 2/*n - 1*/; i++) {
    for (j = 2/*n - 1*/; j > i; j--) {
      if (lu[j - 1] < lu[j]) {  // 前の要素の方が小さかったら 
        temp = lu[j];      // 交換する 
        lu[j] = lu[j - 1];
        lu[j - 1]= temp;
        for(k = 0; k < 3; k++){
          vtemp = vector[j][k];
          vector[j][k] = vector[j-1][k];
          vector[j-1][k] = vtemp;
        }
      }
    }  
  }
  // データ書き込み
  for(i = 0; i< 3; i++){
    ve1[i] = vector[0][i];
    ve2[i] = vector[1][i];
    ve3[i] = vector[2][i];
  }
}

int PCA3( double *x, double *y, double *z, int n, double *_cen,
      double *eval, double *evec)
{
  int  i;
  double cen[3],v1[3],v2[3],v3[3];
  double a, b, c, d, e, f, g, h, j, k, ab, q, q2, q3, lum[3];
  double m, n1, n2, t;
  double r,r1,s,s1,eps=0.001;
  double x1,y1,x2,y2,x3,y3,x4,tmp,y4,x5,y5,x6,y6;

  if(!_cen){
    // 中心が設定されていない場合平均位置を中心とする
    cen[0] = cen[1] = cen[2] = 0;
    for(i = 0; i < n; i++){
      cen[0] += x[i];  cen[1] += y[i];  cen[2] += z[i];
    }
    cen[0] /= n;  cen[1] /= n;  cen[2] /= n;
  } else {
    cen[0] = _cen[0];  cen[1] = _cen[1];  cen[2] = _cen[2];
  }

  a = b = c = d = e = f = 0.;
  for( i=0; i < n; i++ ){
    a += (x[i] - cen[0])*(x[i] - cen[0]);
    b += (y[i] - cen[1])*(y[i] - cen[1]);
    c += (z[i] - cen[2])*(z[i] - cen[2]);
    d += (x[i] - cen[0])*(y[i] - cen[1]);
    f += (y[i] - cen[1])*(z[i] - cen[2]);
    e += (z[i] - cen[2])*(x[i] - cen[0]);
  }
  a /= (double)n; b /= (double)n; c /= (double)n;
  d /= (double)n; e /= (double)n; f /= (double)n;

  g = a + b + c;
  h = -(a*a + b*b + c*c) + a*b + b*c + c*a - 3.*d*d - 3.*e*e - 3.*f*f;
  j = a*b + b*c + c*a - d*d - e*e - f*f;
  k = - a*b*c + c*d*d +b*e*e - 2.*d*e*f + a*f*f;
  m = 2.*g*g*g - 9.*g*j -27.*k;
  n1 = 4.*h*h*h + m*m;

  if( n1 >= 0. ){
    return -1;
  }
  n2 = sqrt(-n1);

  ab = pow( sqrt(m*m - n1),(1./3.) );  

  // 090616 ykohno追加
  if(!ab || !e){  // 分母
    return -1;
  }

  t = atan2(n2,m)/3.;
  r = ab*cos(t); 
  s = ab*sin(t);
  ab = 1./ab;  r1 = ab*cos(t);  s1 = -ab*sin(t);
  q = pow(2.,(1./3.));
  q2 = pow(2.,(2./3.));
  q3 = sqrt(3.);
  lum[0] = (2.*g + q2*r - 2.*q*h*r1)/6.;
  lum[1] = (4.*g - q*(q*r - 2.*h*r1 + q*q3*s + 2.*q3*h*s1))/12.;
  lum[2] = (4.*g + q*(-q*r + 2.*h*r1 + q*q3*s + 2.*q3*h*s1))/12.;
    
  x1 = b*e - d*f - e*(2.*g + q2*r - 2.*q*h*r1)/6.;
  y1 = -e*(q2*s - 2.*q*h*s1)/6.;
  x2 = c +(-2.*g - q2*r + 2.*q*h*r1)/6.;
  y2 = (-q2*s + 2.*q*h*s1)/6.;
  ab = x1*x1 + y1*y1;
  v1[0]= (f*(e*f*x1 - d*(x1*x2 + y1*y2)) - x2*ab)/(e*ab);
  v1[1]= (-e*f*x1 + d*x1*x2 + d*y1*y2)/ab;
  v1[2]= 1.;
    
  tmp = q*(q*r - 2.*h*r1 + q*q3*s + 2.*q3*h*s1)/12.;
  x3 = b - g/3. + tmp;
  y3 = (q*q3*r - q*s + 2.*h*(q3*r1 + s1))/(-6.*q2);
  x4 = c - g/3. + tmp;
  y4 = y3;
  ab = (e*x3 - d*f)*(e*x3 - d*f) + e*e*y3*y3;
  v2[0]= (f*(-e*e*f*x3 - d*d*f*x4 + d*e*(f*f + x3*x4 + y3*y4))
          + x4*ab) / (-e*ab);
  v2[1]= (-e*e*f*x3 -d*d*f*x4 + d*e*(f*f + x3*x4 + y3*y4))/ab;
  v2[2]= 1.;
    
  tmp = q*(q*r - 2.*h*r1 - q*q3*s - 2.*q3*h*s1)/12.;
  x5 = b - g/3. + tmp;
  y5 = (q*q3*r + 2.*q3*h*r1 + q*s - 2.*h*s1)/(6.*q2);
  x6 = c - g/3. + tmp;
  y6 = y5;
  ab = (e*x5 - d*f)*(e*x5 - d*f) + e*e*y5*y5;
  v3[0]= (f*(-e*e*f*x5 - d*d*f*x6 + d*e*(f*f + x5*x6 + y5*y6)) + x6*ab)/(-e*ab);
  v3[1]= (-e*e*f*x5 - d*d*f*x6 + d*e*(f*f + x5*x6 + y5*y6))/ab;
  v3[2]= 1.;
    
  // 単位ベクトルにする
  unitvec(v1); unitvec(v2); unitvec(v3);
  // 固有値の大きい順位、ベクトルを並べる
  SortLum(lum,v1,v2,v3);
  // 外積を計算する
  CrossProduct(v1,v2,v3);
#if 0
  // 固有ベクトルにする
  for( i=0; i<3; i++ ){
    v1[i] *= (double)sqrt((double)lum[0]);
    v2[i] *= (double)sqrt((double)lum[1]);
    v3[i] *= (double)sqrt((double)lum[2]);
    }
#endif

  // 固有値、固有ベクトルを格納する
  if(eval){
    eval[0] = lum[0];  eval[1] = lum[1];  eval[2] = lum[2];
  }
  if(evec){
    evec[0] = v1[0];  evec[1] = v1[1];  evec[2] = v1[2];  evec[3] = 0;
    evec[4] = v2[0];  evec[5] = v2[1];  evec[6] = v2[2];  evec[7] = 0;
    evec[8] = v3[0];  evec[9] = v3[1];  evec[10] = v3[2];  evec[11] = 0;
    evec[12] = cen[0];  evec[13] = cen[1];  evec[14] = cen[2];  evec[15] = 1;
  }
  return 0;
}

