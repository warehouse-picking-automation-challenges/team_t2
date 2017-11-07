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

#include <T2_robot_vision/RecognizedResult.h>
#include <PCA.h>
#include <robot_vision_unification.h>

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
//#include <pcl/visualization/cloud_viewer.h>
 
#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <iostream>
#include <fstream>
#include <string>

#include <stdlib.h>
#include <float.h>
#include <memory.h>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

#define TRUE 1
#define FALSE 0

//template<class T> inline std::string toString(T x) {std::ostringstream sout;sout<<x;return sout.str();}

// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_unification, nodelet::Nodelet);

namespace T2_robot_vision
{

std::string int_to_string(int n)
{
    std::stringstream ss;
    ss << n;
    std::string result = ss.str();

    return result;
}

int matcout(cv::Mat m1){  
  // 行数
  std::cout << "rows:" << m1.rows <<std::endl;
  // 列数
  std::cout << "cols:" << m1.cols << std::endl;
  // 次元数
  std::cout << "dims:" << m1.dims << std::endl;
  // サイズ（2次元の場合）
  std::cout << "size[]:" << m1.size().width << "," << m1.size().height << std::endl;
  // ビット深度ID
  std::cout << "depth (ID):" << m1.depth() << "(=" << CV_64F << ")" << std::endl;
  // チャンネル数
  std::cout << "channels:" << m1.channels() << std::endl;
  // （複数チャンネルから成る）1要素のサイズ [バイト単位]
  std::cout << "elemSize:" << m1.elemSize() << "[byte]" << std::endl;
  // 1要素内の1チャンネル分のサイズ [バイト単位]
  std::cout << "elemSize1 (elemSize/channels):" << m1.elemSize1() << "[byte]" << std::endl;
  // 要素の総数
  std::cout << "total:" << m1.total() << std::endl;
  // ステップ数 [バイト単位]
  std::cout << "step:" << m1.step << "[byte]" << std::endl;
  // 1ステップ内のチャンネル総数
  std::cout << "step1 (step/elemSize1):" << m1.step1()  << std::endl;
  // データは連続か？
  std::cout << "isContinuous:" << (m1.isContinuous()?"true":"false") << std::endl;
  // 部分行列か？
  std::cout << "isSubmatrix:" << (m1.isSubmatrix()?"true":"false") << std::endl;
  // データは空か？
  std::cout << "empty:" << (m1.empty()?"true":"false") << std::endl;
  return 0;
}

int mmult9( float *mres, float *m0, float *m1 )
{
  mres[0] = m0[0]*m1[0] + m0[1]*m1[3] + m0[2]*m1[6];
  mres[1] = m0[0]*m1[1] + m0[1]*m1[4] + m0[2]*m1[7];
  mres[2] = m0[0]*m1[2] + m0[1]*m1[5] + m0[2]*m1[8];
  mres[3] = m0[3]*m1[0] + m0[4]*m1[3] + m0[5]*m1[6];
  mres[4] = m0[3]*m1[1] + m0[4]*m1[4] + m0[5]*m1[7];
  mres[5] = m0[3]*m1[2] + m0[4]*m1[5] + m0[5]*m1[8];
  mres[6] = m0[6]*m1[0] + m0[7]*m1[3] + m0[8]*m1[6];
  mres[7] = m0[6]*m1[1] + m0[7]*m1[4] + m0[8]*m1[7];
  mres[8] = m0[6]*m1[2] + m0[7]*m1[5] + m0[8]*m1[8];
  return 0;
}

int writePCD( string fname, int vcnt, double *vx, double *vy, double *vz )
{
  int ret=0;

  std::string src_root_path = ros::package::getPath("T2_robot_vision");
  std::string pfname = src_root_path + "/data/debug/" + fname;

  std::ofstream pcd_file(pfname.c_str(), std::ios::out);
  pcd_file << "#.PCD v.7 - Point Cloud Data file format" << std::endl;
  pcd_file << "VERSION .7" << std::endl;
  pcd_file << "FIELDS x y z" << std::endl;
  pcd_file << "SIZE 4 4 4" << std::endl;
  pcd_file << "TYPE F F F" << std::endl;
  pcd_file << "COUNT 1 1 1" << std::endl;
  pcd_file << "WIDTH " << vcnt << std::endl;
  pcd_file << "HEIGHT " << 1 << std::endl;
  pcd_file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
  pcd_file << "POINTS " << vcnt << std::endl;
  pcd_file << "DATA ascii" << std::endl;
  for(unsigned int i=0; i<vcnt; i++){
    pcd_file << vx[i] << " " << vy[i] << " " << vz[i] << std::endl;
  }
  pcd_file.close();

  return ret;
}

int writePCD( string fname, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
  int ret=0;

  std::string src_root_path = ros::package::getPath("T2_robot_vision");
  std::string pfname = src_root_path + "/data/debug/" + fname;
  //printf("%s\n", pfname.c_str());
  std::ofstream pcd_file(pfname.c_str(), std::ios::out);
  pcd_file << "#.PCD v.7 - Point Cloud Data file format" << std::endl;
  pcd_file << "VERSION .7" << std::endl;
  pcd_file << "FIELDS x y z" << std::endl;
  pcd_file << "SIZE 4 4 4" << std::endl;
  pcd_file << "TYPE F F F" << std::endl;
  pcd_file << "COUNT 1 1 1" << std::endl;
  pcd_file << "WIDTH " << cloud->width << std::endl;
  pcd_file << "HEIGHT " << cloud->height << std::endl;
  pcd_file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
  pcd_file << "POINTS " << cloud->width << std::endl;
  pcd_file << "DATA ascii" << std::endl;
  for(unsigned int i=0; i<cloud->width; i++){
    pcd_file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
  }
  pcd_file.close();
  return ret;
}

int robot_vision_unification::convert( int camid, float x0, float y0, float d0, float *x1, float *y1, int flg )
{
  double cx,cy,cz, dx,dy,dz;
  double cfx,cfy, ccx,ccy, dfx,dfy, dcx,dcy, *cdmat=NULL,*dcmat=NULL;

  if( camid < 0 || MAX_CAM <= camid || d0==0.0 ){
    return -1;
  }
  cfx=cp_cfx[camid]; cfy=cp_cfy[camid]; ccx=cp_ccx[camid]; ccy=cp_ccy[camid];
  dfx=cp_dfx[camid]; dfy=cp_dfy[camid]; dcx=cp_dcx[camid]; dcy=cp_dcy[camid];
  cdmat = cp_cdmat[camid]; dcmat = cp_dcmat[camid];

  if( flg==0 ){ // 0:カラー座標→距離座標, 1:距離座標→カラー座標
    // color画像基準の3次元座標値
    cx = (x0-ccx)*d0/cfx;
    cy = (y0-ccy)*d0/cfy;
    cz = d0;
    //ROS_DEBUG_STREAM( "cx=" << cx << " cy=" << cy << " cz=" << cz );
    dx = dcmat[0]*cx + dcmat[4]*cy + dcmat[8]*cz + dcmat[12];
    dy = dcmat[1]*cx + dcmat[5]*cy + dcmat[9]*cz + dcmat[13];
    dz = dcmat[2]*cx + dcmat[6]*cy + dcmat[10]*cz + dcmat[14];
    //ROS_DEBUG_STREAM( "dx=" << dx << " dy=" << dy << " dz=" << dz );      
    *x1 = dx*dfx/dz+dcx;
    *y1 = dy*dfy/dz+dcy;
  } else {
    // depth画像基準の3次元座標値
    dx = (x0-dcx)*d0/dfx;
    dy = (y0-dcy)*d0/dfy;
    dz = d0;
    //ROS_DEBUG_STREAM( "dx=" << dx << " dy=" << dy << " dz=" << dz );      
    cx = cdmat[0]*dx + cdmat[4]*dy + cdmat[8]*dz + cdmat[12];
    cy = cdmat[1]*dx + cdmat[5]*dy + cdmat[9]*dz + cdmat[13];
    cz = cdmat[2]*dx + cdmat[6]*dy + cdmat[10]*dz + cdmat[14];
    //ROS_DEBUG_STREAM( "cx=" << cx << " cy=" << cy << " cz=" << cz );
    *x1 = cx*cfx/cz+ccx;
    *y1 = cy*cfy/cz+ccy;
  }
  return 0;
}


robot_vision_unification::robot_vision_unification()
{
}

/*
 * Nodelet Destructor.
 */
robot_vision_unification::~robot_vision_unification()
{
}

static int loadObj( std::string fname,
             pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr,
             //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_ptr,
             pcl::PointCloud<pcl::Normal>::Ptr normals_ptr,
             int idx )
{
  int i,j,ret=0;
  int pcnt0=0, pcnt1=0, pcnt2=0;
  int ncnt0=0, ncnt1=0;
  int fcnt0=0, fcnt1=0;
  int i01[MAX_MVCNT], i10[MAX_MVCNT];
  std::string buffer;
  double *nx=NULL, *ny=NULL, *nz=NULL;
  int *pncnt=NULL, *pnlist=NULL;

  for(i=0; i<MAX_MVCNT; i++){ i01[i] = i10[i] = -1; }

  std::ifstream file( fname.c_str(), std::ios::in );
  printf( "filename: %s\n", fname.c_str() );

  if(file==NULL){
    ret = -1; goto end;
  }

  while( !file.eof() ){
    std::getline(file, buffer);
    if( buffer.length() < 2 ){
      continue;
    }
    if( buffer.c_str()[0]=='v' && buffer.c_str()[1]==' ' ){
      float xx,yy,zz;
      std::stringstream buf(buffer);
      std::string v, x,y,z;
      buf >> v >> x >> y >> z;
      xx = atof(x.c_str());
      yy = atof(y.c_str());
      zz = atof(z.c_str());
      if( idx==6 && zz<0.02 ){
        pcnt2++;
        continue;
      }
      i01[pcnt2] = pcnt0;
      i10[pcnt0] = pcnt2;
      pcnt0++;
      pcnt2++;
    }
    if( buffer.c_str()[0]=='v' && buffer.c_str()[1]=='n' ){
      ncnt0++;
    }
    if( buffer.c_str()[0]=='f' && buffer.c_str()[1]==' ' ){
      fcnt0++;
    }
  }
  printf( "pcnt0 = %d, ncnt0 = %d, fcnt0 = %d\n", pcnt0, ncnt0, fcnt0 );
  
  pointcloud_ptr->width    = pcnt0;
  pointcloud_ptr->height   = 1;
  pointcloud_ptr->is_dense = false;
  pointcloud_ptr->points.resize (pointcloud_ptr->width * pointcloud_ptr->height);
  normals_ptr->width    = pcnt0;
  normals_ptr->height   = 1;
  normals_ptr->is_dense = false;
  normals_ptr->points.resize (normals_ptr->width * normals_ptr->height);

  nx = new double[ncnt0];
  ny = new double[ncnt0];
  nz = new double[ncnt0];
  pncnt = new int[pcnt0];
  pnlist = new int[pcnt0*10];
  memset( pncnt, 0, sizeof(int)*pcnt0 );

  file.clear();
  file.seekg(0,std::ios_base::beg);
  pcnt2=0; // reset
  while( !file.eof() ){
    std::getline(file, buffer);
    if( buffer.length() < 2 ){
      continue;
    }
    if( buffer.c_str()[0]=='v' && buffer.c_str()[1]==' ' ){
      float xx,yy,zz;
      if( i01[pcnt2]<0 ){
        pcnt2++;
        continue;
      }
      std::stringstream buf(buffer);
      std::string v, x,y,z;
      buf >> v >> x >> y >> z;
      pointcloud_ptr->points[pcnt1].x = xx = atof(x.c_str());
      pointcloud_ptr->points[pcnt1].y = yy = atof(y.c_str());
      pointcloud_ptr->points[pcnt1].z = zz = atof(z.c_str());
      normals_ptr->points[pcnt1].normal_x = 1.0;
      normals_ptr->points[pcnt1].normal_y = 0.0;
      normals_ptr->points[pcnt1].normal_z = 0.0;
      pcnt1++;
      pcnt2++;
    }
    if( buffer.c_str()[0]=='v' && buffer.c_str()[1]=='n' ){
      std::stringstream buf(buffer);
      std::string v, x,y,z;
      buf >> v >> x >> y >> z;
      nx[ncnt1] = atof(x.c_str());
      ny[ncnt1] = atof(y.c_str());
      nz[ncnt1] = atof(z.c_str());
      ncnt1++;
    }
    if( buffer.c_str()[0]=='f' && buffer.c_str()[1]==' ' ){
      std::stringstream buf(buffer);
      std::string v, x,y,z, item;
      int pid, nid;

      buf >> v >> x >> y >> z;      
      //printf("buf=%s\n", buf.str().c_str());
      //printf("x=%s, y=%s, z=%s\n", x.c_str(), y.c_str(), z.c_str() );
      
      std::istringstream streamx(x);
      if( getline(streamx,item,'/') ){ pid = atoi(item.c_str()); pid=i01[pid];}
      if( getline(streamx,item,'/') ){ }
      if( getline(streamx,item,'/') ){ nid = atoi(item.c_str()); }
      //printf("pid=%d, nid=%d, pncnt=%d\n", pid, nid, pncnt[pid]);
      if( 0<pid && pid <= pcnt0 && pncnt[pid-1]<10 && 0< nid && nid <= ncnt0 ){
        pnlist[(pid-1)*10+pncnt[pid-1]] = nid-1;
        pncnt[pid-1]++;
      }

      std::istringstream streamy(y);
      if( getline(streamy,item,'/') ){ pid = atoi(item.c_str()); pid=i01[pid]; }
      if( getline(streamy,item,'/') ){ }
      if( getline(streamy,item,'/') ){ nid = atoi(item.c_str()); }
      //printf("pid=%d, nid=%d, pncnt=%d\n", pid, nid, pncnt[pid]);
      if( 0<pid && pid <= pcnt0 && pncnt[pid-1]<10 && 0< nid && nid <= ncnt0 ){
        pnlist[(pid-1)*10+pncnt[pid-1]] = nid-1;
        pncnt[pid-1]++;
      }    

      std::istringstream streamz(z);
      if( getline(streamz,item,'/') ){ pid = atoi(item.c_str()); pid=i01[pid]; }
      if( getline(streamz,item,'/') ){ }
      if( getline(streamz,item,'/') ){ nid = atoi(item.c_str()); }
      //printf("pid=%d, nid=%d, pncnt=%d\n", pid, nid, pncnt[pid]);
      if( 0<pid && pid <= pcnt0 && pncnt[pid-1]<10 && 0< nid && nid <= ncnt0 ){
        pnlist[(pid-1)*10+pncnt[pid-1]] = nid-1;
        pncnt[pid-1]++;
      }
    }
  }
  file.close();

  for( i=0; i<pcnt1; i++ ){
    double tmpx=0.0, tmpy=0.0, tmpz=0.0, len=0.0;
    for( j=0; j<pncnt[i]; j++ ){
      int idx = pnlist[i*10+j];
      tmpx += nx[idx];
      tmpy += ny[idx];
      tmpz += nz[idx];
    }
    len = sqrt( tmpx*tmpx + tmpy*tmpy + tmpz*tmpz );
    normals_ptr->points[i].normal_x = tmpx / len;
    normals_ptr->points[i].normal_y = tmpy / len;
    normals_ptr->points[i].normal_z = tmpz / len;
    //if(i%50==0){
    //  printf("%f,%f,%f\n", normals_ptr->points[i].normal_x,normals_ptr->points[i].normal_y,normals_ptr->points[i].normal_z);
    //}
  }
end:
  return ret;
}

/*
 * Initialize the nodelet.
 */
void robot_vision_unification::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  resetproc();

  server_ = nh.advertiseService("conv_srv", &robot_vision_unification::conv, this);
  server_IC_ = nh.advertiseService("itemcad_srv", &robot_vision_unification::Item2Cad, this);
  server_CI_ = nh.advertiseService("caditem_srv", &robot_vision_unification::Cad2Item, this);
  unification_sub_lmod_ = nh.subscribe("lineMOD_results", 10, &robot_vision_unification::resultLineMODCallback, this);
  unification_sub_yolo_ = nh.subscribe("yolov2_results", 10, &robot_vision_unification::resultYOLOv2Callback, this);
  unification_sub_akaze_ = nh.subscribe("AKAZE_results", 10, &robot_vision_unification::resultAKAZECallback, this);
  unification_sub_ = nh.subscribe("recognized_item", 10, &robot_vision_unification::recognizedItemCallback, this);
  unification_pub_ = nh.advertise<T2_robot_vision::RecognizedResult>("recognized_result", 10);

  // 認識パスの使用／不使用を指定（ファイル読み込み）
  {
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string file_name = src_root_path + "/data/uni_prm.txt";
    ROS_INFO_STREAM( "filename: " << file_name.c_str() );
    std::ifstream file( file_name.c_str(), std::ios::in );
    std::string buffer;

    for( int i=0; i<MAX_PATH; i++ ){
      rcgpath[i] = 0;
    }
    while( !file.eof() && std::getline(file, buffer) ){
      int index, onoff;
      std::string item;
      istringstream stream(buffer);
      if( buffer.c_str()[0]=='#' ){
        continue;
      }
      // index number
      if( getline(stream,item,',')==NULL ){
        continue;
      }
      index = atoi(item.c_str());
      if( index < 0 || MAX_PATH <= index ){
        continue;
      }
      // ON/OFF
      if( getline(stream,item,',')==NULL ){
        continue;
      }
      rcgpath[index] = atoi(item.c_str());
    }
    file.close();
    for(int i=0; i<MAX_PATH; i++){
      printf( "rcgpath[%d] = %d\n", i, rcgpath[i] );
    }
  }
  for( int i=0; i<MAX_CAM; i++ ){
    cp_cfx[i] = cp_cfy[i] = cp_ccx[i] = cp_ccy[i] = 0;
    cp_dfx[i] = cp_dfy[i] = cp_dcx[i] = cp_dcy[i] = 0;
    cp_cdmat[i][0]=1.0;  cp_cdmat[i][1]=0.0;  cp_cdmat[i][2]=0.0;  cp_cdmat[i][3]=0.0;
    cp_cdmat[i][4]=0.0;  cp_cdmat[i][5]=1.0;  cp_cdmat[i][6]=0.0;  cp_cdmat[i][7]=0.0;
    cp_cdmat[i][8]=0.0;  cp_cdmat[i][9]=0.0;  cp_cdmat[i][10]=1.0; cp_cdmat[i][11]=0.0;
    cp_cdmat[i][12]=0.0; cp_cdmat[i][13]=0.0; cp_cdmat[i][14]=0.0; cp_cdmat[i][15]=1.0;
    cp_dcmat[i][0]=1.0;  cp_dcmat[i][1]=0.0;  cp_dcmat[i][2]=0.0;  cp_dcmat[i][3]=0.0;
    cp_dcmat[i][4]=0.0;  cp_dcmat[i][5]=1.0;  cp_dcmat[i][6]=0.0;  cp_dcmat[i][7]=0.0;
    cp_dcmat[i][8]=0.0;  cp_dcmat[i][9]=0.0;  cp_dcmat[i][10]=1.0; cp_dcmat[i][11]=0.0;
    cp_dcmat[i][12]=0.0; cp_dcmat[i][13]=0.0; cp_dcmat[i][14]=0.0; cp_dcmat[i][15]=1.0;
  }
  for (int i = 0; i < MAX_CAM; i++)
  {
    std::ostringstream oss;
    oss << i;
    std::string base_name = "/calibration" + oss.str();
    std::string serial_number_ = base_name + "/serial_number";
    std::string rgb_camera_matrix_ = base_name + "/rgb_camera_matrix";
    //std::string rgb_distCoeffs_ = base_name + "/rgb_distCoeffs";
    std::string depth_camera_matrix_ = base_name + "/depth_camera_matrix";
    //std::string depth_distCoeffs_ = base_name + "/depth_distCoeffs";
    std::string camera_rotation_ = base_name + "/camera_rotation";
    std::string camera_translation_ = base_name + "/camera_translation";
    std::string serial;
    std::string cam_serial;
    ros::NodeHandle &nh = getNodeHandle();
    std::vector<double> vec;
    std::vector<int> invec;
    cv::Mat matrix, mat16, matinv;
    mat16 = cv::Mat(4, 4, CV_64FC1);
    nh.getParam(serial_number_, serial);
    if( serial.length() > 0 ){
      nh.getParam(rgb_camera_matrix_, vec);
      matrix = cv::Mat(3, 3, CV_64FC1, vec.data());
      cp_cfx[i] = matrix.at<double>(0,0);
      cp_cfy[i] = matrix.at<double>(1,1);
      cp_ccx[i] = matrix.at<double>(0,2);
      cp_ccy[i] = matrix.at<double>(1,2);
      vec.clear();
      nh.getParam(depth_camera_matrix_, vec);
      matrix = cv::Mat(3, 3, CV_64FC1, vec.data());
      cp_dfx[i] = matrix.at<double>(0,0);
      cp_dfy[i] = matrix.at<double>(1,1);
      cp_dcx[i] = matrix.at<double>(0,2);
      cp_dcy[i] = matrix.at<double>(1,2);
      vec.clear();
      nh.getParam(camera_rotation_, vec);
      matrix = cv::Mat(3, 3, CV_64FC1, vec.data());
      mat16.at<double>(0,0) = cp_dcmat[i][0] = matrix.at<double>(0,0);
      mat16.at<double>(0,1) = cp_dcmat[i][1] = -matrix.at<double>(0,1);
      mat16.at<double>(0,2) = cp_dcmat[i][2] = -matrix.at<double>(0,2);
      mat16.at<double>(0,3) = cp_dcmat[i][3] = 0.0;
      mat16.at<double>(1,0) = cp_dcmat[i][4] = -matrix.at<double>(1,0);
      mat16.at<double>(1,1) = cp_dcmat[i][5] = matrix.at<double>(1,1);
      mat16.at<double>(1,2) = cp_dcmat[i][6] = matrix.at<double>(1,2);
      mat16.at<double>(1,3) = cp_dcmat[i][7] = 0.0;
      mat16.at<double>(2,0) = cp_dcmat[i][8] = -matrix.at<double>(2,0);
      mat16.at<double>(2,1) = cp_dcmat[i][9] = matrix.at<double>(2,1);
      mat16.at<double>(2,2) = cp_dcmat[i][10] = matrix.at<double>(2,2);
      mat16.at<double>(2,3) = cp_dcmat[i][11] = 0.0;
      vec.clear();

      nh.getParam(camera_translation_, vec);
      matrix = cv::Mat(3, 1, CV_64FC1, vec.data());
      mat16.at<double>(3,0) = cp_dcmat[i][12] = -matrix.at<double>(0,0)*1000;
      mat16.at<double>(3,1) = cp_dcmat[i][13] = matrix.at<double>(0,1)*1000;
      mat16.at<double>(3,2) = cp_dcmat[i][14] = matrix.at<double>(0,2)*1000;
      mat16.at<double>(3,3) = cp_dcmat[i][15] = 1.0;
      vec.clear();

      matinv = mat16.inv();
      cp_cdmat[i][0] = matinv.at<double>(0,0);
      cp_cdmat[i][1] = matinv.at<double>(0,1);
      cp_cdmat[i][2] = matinv.at<double>(0,2);
      cp_cdmat[i][3] = matinv.at<double>(0,3);
      cp_cdmat[i][4] = matinv.at<double>(1,0);
      cp_cdmat[i][5] = matinv.at<double>(1,1);
      cp_cdmat[i][6] = matinv.at<double>(1,2);
      cp_cdmat[i][7] = matinv.at<double>(1,3);
      cp_cdmat[i][8] = matinv.at<double>(2,0);
      cp_cdmat[i][9] = matinv.at<double>(2,1);
      cp_cdmat[i][10]= matinv.at<double>(2,2);
      cp_cdmat[i][11]= matinv.at<double>(2,3);
      cp_cdmat[i][12]= matinv.at<double>(3,0);
      cp_cdmat[i][13]= matinv.at<double>(3,1);
      cp_cdmat[i][14]= matinv.at<double>(3,2);
      cp_cdmat[i][15]= matinv.at<double>(3,3);
    } //if    
  } //for

  for( int i=0; i<MAX_CAM; i++ ){
    if( cp_cfx[i]==0 ){
      continue;
    }
    printf("camera %02d\n", i);
    printf(" color: fx=%0.3f, fy=%0.3f, cx=%0.3f, cy=%0.3f\n", cp_cfx[i],cp_cfy[i],cp_ccx[i],cp_ccy[i]);
    printf(" depth: fx=%0.3f, fy=%0.3f, cx=%0.3f, cy=%0.3f\n", cp_dfx[i],cp_dfy[i],cp_dcx[i],cp_dcy[i]);
    printf(" cdmat: \n");
    printf("  %0.3f, %0.3f, %0.3f, %0.3f\n", cp_cdmat[i][0], cp_cdmat[i][1], cp_cdmat[i][2], cp_cdmat[i][3]);
    printf("  %0.3f, %0.3f, %0.3f, %0.3f\n", cp_cdmat[i][4], cp_cdmat[i][5], cp_cdmat[i][6], cp_cdmat[i][7]);
    printf("  %0.3f, %0.3f, %0.3f, %0.3f\n", cp_cdmat[i][8], cp_cdmat[i][9], cp_cdmat[i][10],cp_cdmat[i][11]);
    printf("  %0.3f, %0.3f, %0.3f, %0.3f\n", cp_cdmat[i][12],cp_cdmat[i][13],cp_cdmat[i][14],cp_cdmat[i][15]);
    printf(" dcmat: \n");
    printf("  %0.3f, %0.3f, %0.3f, %0.3f\n", cp_dcmat[i][0], cp_dcmat[i][1], cp_dcmat[i][2], cp_dcmat[i][3]);
    printf("  %0.3f, %0.3f, %0.3f, %0.3f\n", cp_dcmat[i][4], cp_dcmat[i][5], cp_dcmat[i][6], cp_dcmat[i][7]);
    printf("  %0.3f, %0.3f, %0.3f, %0.3f\n", cp_dcmat[i][8], cp_dcmat[i][9], cp_dcmat[i][10],cp_dcmat[i][11]);
    printf("  %0.3f, %0.3f, %0.3f, %0.3f\n", cp_dcmat[i][12],cp_dcmat[i][13],cp_dcmat[i][14],cp_dcmat[i][15]);
  }

  {
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string file_name = src_root_path + "/data/cadid.csv";
    ROS_INFO_STREAM( "filename: " << file_name.c_str() );
    std::ifstream file( file_name.c_str(), std::ios::in );
    std::string buffer;

    for( int i=0; i<MAX_MCNT; i++ ){
      table_cadid[i] = -1;
    }
    while( !file.eof() && std::getline(file, buffer) ){
      int itemno, j;
      std::string item;
      istringstream stream(buffer);

      if( buffer.c_str()[0]=='#' ){
        continue;
      }

      // index number
      if( getline(stream,item,',')==NULL ){
        continue;
      }
      itemno = atoi(item.c_str());
      if( itemno < 0 || MAX_MCNT <= itemno ){
        continue;
      }

      // cad id
      if( getline(stream,item,',')==NULL ){
        continue;
      }
      table_cadid[itemno] = atoi(item.c_str());

      // table_procpriority
      if( getline(stream,item,',')==NULL ){
        continue;
      }
      int pri=MAX_PATH-1;
      memset( table_procpriority[itemno], 0, sizeof(int)*MAX_PATH );
      cout << "itemno=" << itemno << ":";
      for(int j=0; j<item.length(); j++){
        int idx = (int)(item[j] - '0');
        table_procpriority[itemno][idx] = pri;
        pri--;
      }

      // matrix
      for( j=0; j<9; j++ ){
        if( getline(stream,item,',')==NULL ){
          break;
        }
        table_matrix[itemno][j] = atof(item.c_str());
      }
      if( j<9 ){
        continue;
      }

      // recognized category -> item name
      // std::string table_name[MAX_MCNT];
      if( getline(stream,item,',')==NULL ){
        continue;
      }
      table_name[itemno] = item;

      if( table_name[itemno].find("##") != std::string::npos ){
        table_type[itemno] = 2; // 2: transparent
      } else if( table_name[itemno].find("#") != std::string::npos ){
        table_type[itemno] = 1; // 1: deformable
      } else {
        table_type[itemno] = 0; // 0: regular items
      }
    }
    file.close();
    table_cadid[0] = 9999;
    table_procpriority[0][9] = 1;
    table_name[0] = "unknown";
  }

  //ROS_INFO_STREAM( "tabel_size=" << table_size );
  for( int i=0; i<MAX_MCNT; i++ ){
    if(table_cadid[i]<0 || table_cadid[i]==9999){
      continue;
    }
    ROS_INFO_STREAM( "item_no:" << i << ", cad_id:" << table_cadid[i] << ", priority:"
    << table_procpriority[i][0] << "," << table_procpriority[i][1] << "," << table_procpriority[i][2] << ","
    << table_procpriority[i][3] << "," << table_procpriority[i][4] << "," << table_procpriority[i][5] << ","
    << table_procpriority[i][6] << "," << table_procpriority[i][7] << "," << table_procpriority[i][8] << ","
    << table_procpriority[i][9] << "," << table_procpriority[i][10] << "," << table_procpriority[i][11] );
    ROS_INFO_STREAM( " type: " << table_type[i] << ", item_name:" << table_name[i] );
    ROS_INFO_STREAM( "  mat: "
    << table_matrix[i][0] << ", " << table_matrix[i][1] << ", " << table_matrix[i][2] << ",  "
    << table_matrix[i][3] << ", " << table_matrix[i][4] << ", " << table_matrix[i][5] << ",  "
    << table_matrix[i][6] << ", " << table_matrix[i][7] << ", " << table_matrix[i][8] );
  }
  
  for( int i=0; i<MAX_MCNT; i++ ){
    int pcnt0=0, pcnt1=0;
    int cid = table_cadid[i];
    if( cid<0 || cid==9999 ){
      continue;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string debug_path = src_root_path + "/data/model/";
    std::stringstream ss;
    std::string buffer;
    ss << debug_path << setfill('0') << setw(2) << right << i << ".obj";
    
    int ret = loadObj( ss.str().c_str(), cloud, normals, i );
    if( ret > -1){
      cloud_model[i] = cloud;
      normal_model[i] = normals;
    } else {
      cloud_model[i] = NULL;
      normal_model[i] = NULL;
    }
  }

  memset( gp_type, 0, sizeof(int)*MAX_MCNT );
  for( int i=0; i<MAX_MCNT; i++ ){
    cv::Mat mat16, matinv;
    mat16 = cv::Mat::eye(4,4,CV_64FC1);
    int cid = table_cadid[i];
    if( cid<0 || cid==9999 ){
      //ROS_INFO_STREAM( "continue: item_no:" << i << ", cad_id:" << table_cadid[i] << " );
      continue;
    }

    XmlRpc::XmlRpcValue matParam, infoParam;
    std::ostringstream oss;
    oss << cid;
    std::string grasp_list = "t2_database/grasp_info_list/cad" + oss.str();
    int ret, rep, idx=-1;
    //double mat[16];
    ret = nh.getParam(grasp_list.c_str(), infoParam);

    rep = (int)infoParam["represent_gp"];
    for( int j=0; j<(int)infoParam["gp"].size(); j++ ){
      int gp_num = (int32_t)((int)infoParam["gp"][j]["gp_number"]);
      if( gp_num==rep ){
        idx = j;
        break;
      }
    }
    if( idx>-1 ){ // breakされた
      std::string gp_type_str = (std::string)infoParam["gp"][idx]["type"];
      if( gp_type_str == "suction" ){
        gp_type[i] = 1;
      } else {
        gp_type[i] = 0;
      }
      for( int j=0; j<16; j++ ){
        mat16.at<double>(j/4,j%4) = (double)infoParam["gp"][idx]["mat4x4"][j];
        //gp_mat[i][j] = (double)infoParam["gp"][idx]["mat4x4"][j];
      }
      ROS_INFO_STREAM( "cid:" << cid << ", rep:" << rep << ", idx: " << idx << ", type:" << gp_type[i] );
      ROS_INFO_STREAM( "mat4x4:"
         << mat16.at<double>(0,0) << ", " << mat16.at<double>(0,1) << ", " << mat16.at<double>(0,2) << ", " << mat16.at<double>(0,3) << ",  "
         << mat16.at<double>(1,0) << ", " << mat16.at<double>(1,1) << ", " << mat16.at<double>(1,2) << ", " << mat16.at<double>(1,3) << ",  "
         << mat16.at<double>(2,0) << ", " << mat16.at<double>(2,1) << ", " << mat16.at<double>(2,2) << ", " << mat16.at<double>(2,3) << ",  "
         << mat16.at<double>(3,0) << ", " << mat16.at<double>(3,1) << ", " << mat16.at<double>(3,2) << ", " << mat16.at<double>(3,3) );
      matinv = mat16.inv();
      for( int j=0; j<16; j++ ){
        gp_mat[i][j] = matinv.at<double>(j/4,j%4);
      }
    }

  }

  ROS_INFO_STREAM( "end" );
}

bool robot_vision_unification::conv(T2_robot_vision::ConvRequest &req, T2_robot_vision::ConvResponse &res)
{
  ros::NodeHandle nh = getMTNodeHandle();
  ros::NodeHandle pnh = getMTPrivateNodeHandle();

  res.job_no = req.job_no;
  int sts = robot_vision_unification::convert( req.cam_id, req.x0, req.y0, req.d0, &(res.x1), &(res.y1), req.flg );
  if( sts==0 ){
    return true;
  } else {
    return false;
  }
}
bool robot_vision_unification::Cad2Item(T2_robot_vision::CadItemRequest &req, T2_robot_vision::CadItemResponse &res)
{
  ros::NodeHandle nh = getMTNodeHandle();
  ros::NodeHandle pnh = getMTPrivateNodeHandle();

  //int table_cadid[MAX_MCNT]; // recognized category -> cad id
  if( req.cad_id < 0 ){
    res.category_id = -1;
    return false;
  }
  for( int i=0; i<MAX_MCNT; i++ ){
    if( table_cadid[i] == req.cad_id ){
      res.category_id = i;
      return true;
    }
  }
  res.category_id = -1;
  return false;
}

bool robot_vision_unification::Item2Cad(T2_robot_vision::ItemCadRequest &req, T2_robot_vision::ItemCadResponse &res)
{
  if( req.category_id < 0 || MAX_MCNT <= req.category_id ){
    res.cad_id = -1;
    return false;
  }
  res.cad_id = table_cadid[req.category_id];
  if( res.cad_id > 0 ){
    return true;
  } else {
    return false;
  }
}

double and_or( int sx0, int ex0, int sy0, int ey0, int sx1, int ex1, int sy1, int ey1 )
{
  double ret;
  double maxsx, minex, maxsy, miney, area0, area1, area_and, w,h;
  maxsx = (double)(sx0 > sx1 ? sx0 : sx1);
  maxsy = (double)(sy0 > sy1 ? sy0 : sy1);
  minex = (double)(ex0 < ex1 ? ex0 : ex1);
  miney = (double)(ey0 < ey1 ? ey0 : ey1);
  w = minex-maxsx;
  h = miney-maxsy;
  //printf("    msx:%.2f, msy:%.2f, mex:%.2f, mey:%.2f, w:%.2f, h:%.2f\n", maxsx,maxsy,minex,miney,w,h);
  if( w<=0 || h<=0 ){
    return 0.0;
  }
  area_and = w*h;
  area0 = (ex0-sx0)*(ey0-sy0);
  area1 = (ex1-sx1)*(ey1-sy1);
  ret = area_and / (area0 + area1 - area_and);
  //printf("    and:%.2f, area0:%.2f, area1:%.2f, ret:%.2f\n", area_and, area0, area1, ret);
  return ret;
}

int robot_vision_unification::storedata(const T2_robot_vision::RecognizedItemConstPtr &notice)
{
  int ret=0;
  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("job_no:" << jn << " itemized_data.size: " << notice->itemized_data.size());
  float thscore = 80.0;

  int idx = notice->rcg_module_index;
  printf("rcg_module_index = %d\n", idx);
  ROS_INFO_STREAM( "job_no:" << jn << " rcg_module_index = " << notice->rcg_module_index );
  if( idx < 0 || MAX_PATH <= idx ){
    ret=-1; goto end; 
  }
  rcgpath_proc[idx] = 1;
  if( rcgpath[idx]==0 ){
    goto end;
  }

  if( job_no_proc==-1 ){
    job_no_proc = jn; // job_no設定
    ROS_INFO_STREAM("start proccessing job_no:" << jn );
#ifdef DUMP_UNILOG
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string debug_path = src_root_path + "/data/debug/InputCheck_";
    std::stringstream ss;
    ss << debug_path << "jobNo_" << notice->recog_target.job_no << "_camID_" << notice->recog_target.data[0].cam_id;
    std::string log_name = ss.str()+"_log.txt";
    if( exportlog ){
      exportlog.close();
    }
    exportlog.open(log_name.c_str(), std::ios::out);
    //ROS_INFO_STREAM(" file open " << log_name.c_str() );    
    if( exportlog ){
      //ROS_INFO_STREAM(" file open " << log_name.c_str() << "success" ); 
      // cad番号出力
      exportlog << "Item,CAD," << endl;
      for(int i=0; i<notice->recog_target.cad_id.size(); i++){
        exportlog << notice->recog_target.category_id[i] << "," << notice->recog_target.cad_id[i] << ","
                  << table_name[notice->recog_target.category_id[i]] << endl;
      }
      exportlog << endl;
    }

#endif
  } else if( job_no_proc != jn ){
    ROS_INFO_STREAM("job_no_proc:" << job_no_proc << " != jn:" << jn );
  }

#ifdef DUMP_UNILOG
  if(exportlog){
    exportlog << endl << "rcg_module_index: " << notice->rcg_module_index << endl;
    switch( notice->rcg_module_index ){
      case 1: exportlog << "----- sekiyaseg + LineMOD -----" << endl << endl; break;
      case 2: exportlog << "----- sekiyaseg + LineMOD + plane -----" << endl << endl; break;
      case 3: exportlog << "----- YOLO(futeikei) -----" << endl << endl; break;
      case 4: exportlog << "----- YOLO + LineMOD -----" << endl << endl; break;
      case 5: exportlog << "----- YOLO + distlearn -----" << endl << endl; break;
      case 6: exportlog << "----- YOLO + plane -----" << endl << endl; break;
      case 7: exportlog << "----- AKAZE -----" << endl << endl; break;
      case 8: exportlog << "----- AKAZE + plane -----" << endl << endl; break;
      case 9: exportlog << "----- sekiyaseg + plane -----" << endl << endl; break;
      case 10: exportlog << "----- tottori -----" << endl << endl; break;
    }
  }
#endif

  for( int i=0; i<notice->itemized_data.size(); i++ ){
    if( cnt_res >= MAX_RES ){
      break;
    }
    ROS_INFO_STREAM("job_no:" << jn << " itemized_data[" << i << "].module_index: " << notice->itemized_data[i].module_index);
    ROS_INFO_STREAM("job_no:" << jn << " itemized_data[" << i << "].category: " << notice->itemized_data[i].category);
    ROS_INFO_STREAM("job_no:" << jn << " itemized_data[" << i << "].pitch: " << notice->itemized_data[i].pitch);
    ROS_INFO_STREAM("job_no:" << jn << " itemized_data[" << i << "].yaw: " << notice->itemized_data[i].yaw);
    ROS_INFO_STREAM("job_no:" << jn << " itemized_data[" << i << "].roll: " << notice->itemized_data[i].roll);
    if( notice->itemized_data[i].module_index == 7 ){
      ROS_INFO_STREAM("job_no:" << jn << " itemized_data[" << i << "].homo: "
        << notice->itemized_data[i].homo[0] << ", " << notice->itemized_data[i].homo[1] << ", " << notice->itemized_data[i].homo[2] << ", "
        << notice->itemized_data[i].homo[3] << ", " << notice->itemized_data[i].homo[4] << ", " << notice->itemized_data[i].homo[5] << ", "
        << notice->itemized_data[i].homo[6] << ", " << notice->itemized_data[i].homo[7] << ", " << notice->itemized_data[i].homo[8] );
    }
    ROS_INFO_STREAM("job_no:" << jn << " itemized_data[" << i << "].score: " << notice->itemized_data[i].score);
    ROS_INFO_STREAM("job_no:" << jn << " itemized_data[" << i << "].seg_id: " << notice->itemized_data[i].seg_id);
    int si = notice->itemized_data[i].seg_id;
    ROS_INFO_STREAM("job_no:" << jn << " segmented_data:[" << si << "].module_index: " << notice->segmented_data[si].module_index);
    ROS_INFO_STREAM("job_no:" << jn << " segmented_data:[" << si << "].sx: " << notice->segmented_data[si].sx);
    ROS_INFO_STREAM("job_no:" << jn << " segmented_data:[" << si << "].ex: " << notice->segmented_data[si].ex);
    ROS_INFO_STREAM("job_no:" << jn << " segmented_data:[" << si << "].sy: " << notice->segmented_data[si].sy);
    ROS_INFO_STREAM("job_no:" << jn << " segmented_data:[" << si << "].ey: " << notice->segmented_data[si].ey);
    ROS_INFO_STREAM("job_no:" << jn << " segmented_data:[" << si << "].img_idx: " << notice->segmented_data[si].img_idx);
    ROS_INFO_STREAM("job_no:" << jn << " segmented_data:[" << si << "].img_type: " << notice->segmented_data[si].img_type);
#ifdef DUMP_UNILOG
    if(exportlog){
      exportlog << "itemized_data[" << i << "].module_index: " << notice->itemized_data[i].module_index << endl;
      exportlog << "itemized_data[" << i << "].category: " << notice->itemized_data[i].category
                << "(" << table_name[notice->itemized_data[i].category]  << ")" << endl;
      exportlog << "itemized_data[" << i << "].pitch: " << notice->itemized_data[i].pitch << endl;
      exportlog << "itemized_data[" << i << "].yaw: " << notice->itemized_data[i].yaw << endl;
      exportlog << "itemized_data[" << i << "].roll: " << notice->itemized_data[i].roll << endl;
      if( notice->itemized_data[i].module_index == 7 ){
        exportlog << " itemized_data[" << i << "].homo: "
          << notice->itemized_data[i].homo[0] << ", " << notice->itemized_data[i].homo[1] << ", " << notice->itemized_data[i].homo[2] << ", "
          << notice->itemized_data[i].homo[3] << ", " << notice->itemized_data[i].homo[4] << ", " << notice->itemized_data[i].homo[5] << ", "
          << notice->itemized_data[i].homo[6] << ", " << notice->itemized_data[i].homo[7] << ", " << notice->itemized_data[i].homo[8] << endl;
      }
      exportlog << "itemized_data[" << i << "].score: " << notice->itemized_data[i].score << endl;
      exportlog << "itemized_data[" << i << "].seg_id: " << notice->itemized_data[i].seg_id << endl;
      exportlog << "segmented_data[" << si << "].module_index: " << notice->segmented_data[si].module_index << endl;
      exportlog << "segmented_data[" << si << "].sx: " << notice->segmented_data[si].sx << endl;
      exportlog << "segmented_data[" << si << "].ex: " << notice->segmented_data[si].ex << endl;
      exportlog << "segmented_data[" << si << "].sy: " << notice->segmented_data[si].sy << endl;
      exportlog << "segmented_data[" << si << "].ey: " << notice->segmented_data[si].ey << endl;
      exportlog << "segmented_data[" << si << "].img_idx: " << notice->segmented_data[si].img_idx << endl;
      exportlog << "segmented_data[" << si << "].img_type: " << notice->segmented_data[si].img_type << endl << endl;
    }
#endif
    int cat = notice->itemized_data[i].category;
    if( table_procpriority[cat][ notice->itemized_data[i].module_index % 10 ] == 0 ){
      continue;
    }

    if( notice->itemized_data[i].score < thscore ){
      int ii=0;
      int cid = notice->itemized_data[i].category;
      if( 0 < cid && cid <= MAX_MCNT-1 ){
        for( ii=0; ii<notice->recog_target.cad_id.size(); ii++ ){
          int t_cid = notice->recog_target.cad_id[ii];
          if( table_cadid[cid] == t_cid ){
#ifdef DUMP_UNILOG
            if(exportlog){
              exportlog << "rcg_res: table_cadid[" << cid << "]=" << table_cadid[cid]
                        << ", target_cid=" << t_cid << endl << endl;
            }
#endif
            break;
          }
        }
        if( ii == notice->recog_target.cad_id.size() ){
#ifdef DUMP_UNILOG
          if(exportlog){
            exportlog << "rcg_res: table_cadid[" << cid << "]=" << table_cadid[cid]
                      << ", no target_cid" << endl << endl;
          }
#endif
          continue; // i
        }
      }
    }

    res_recog_module[cnt_res] = notice->itemized_data[i].module_index;
    res_category[cnt_res] = notice->itemized_data[i].category;
    res_pitch[cnt_res]    = notice->itemized_data[i].pitch;
    res_yaw[cnt_res]      = notice->itemized_data[i].yaw;
    res_roll[cnt_res]     = notice->itemized_data[i].roll;
    res_posx[cnt_res]     = notice->itemized_data[i].posx;
    res_posy[cnt_res]     = notice->itemized_data[i].posy;
    res_posz[cnt_res]     = notice->itemized_data[i].posz;
    if( res_recog_module[cnt_res]==7 ){ // AKAZE
      for( int j=0; j<9; j++ ){
        res_homo[cnt_res][j] = notice->itemized_data[i].homo[j];
      }
    }
    if( res_recog_module[cnt_res]==9 ){ // seg+plane
      res_score[cnt_res]    = 11;
    } else {
      res_score[cnt_res]    = notice->itemized_data[i].score;
    }
    res_seg_module[cnt_res] = notice->segmented_data[si].module_index;
    res_seg_sx[cnt_res]   = notice->segmented_data[si].sx;
    res_seg_ex[cnt_res]   = notice->segmented_data[si].ex;
    res_seg_sy[cnt_res]   = notice->segmented_data[si].sy;
    res_seg_ey[cnt_res]   = notice->segmented_data[si].ey;
    res_seg_imgidx[cnt_res] = notice->segmented_data[si].img_idx;
    res_seg_imgtype[cnt_res] = notice->segmented_data[si].img_type;
    res_seg_mask[cnt_res] = notice->segmented_data[si].mask;
    cnt_res++;

#ifdef DUMP_UNILOG
    if(exportlog){ exportlog << "result cnt: " << cnt_res << endl << endl; }
#endif
  }

end:
#ifdef DUMP_UNILOG
  if( ret < 0 ){
    //if( exportlog ){ exportlog.close(); }
  }
#endif
  return ret;
}

int robot_vision_unification::resetproc()
{
  job_no_proc = -1;
  for( int i=0; i<MAX_PATH; i++ ){ rcgpath_proc[i]=0; }
  cnt_msg = cnt_res = 0; // reset counter
#ifdef DUMP_UNILOG
  if( exportlog ){
    exportlog.close();
  }  
#endif
}

int robot_vision_unification::integrate(const T2_robot_vision::RecognizedItemConstPtr &notice)
{
  int ret=0; // -1:異常, 0:処理すべきパスが未受信， 1:終了
  int res_seg_sx0[MAX_RES];
  int res_seg_ex0[MAX_RES];
  int res_seg_sy0[MAX_RES];
  int res_seg_ey0[MAX_RES];

  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("[START] job_no:" << jn << " integrate");
  {
    int *r=rcgpath, *p=rcgpath_proc;
    ROS_INFO_STREAM("job_no: " << jn << " rcgpath: "
      << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << " " << r[4] << " "  
      << r[5] << " " << r[6] << " " << r[7] << " " << r[8] << " " << r[9] << " "  
      << r[10]<< " " << r[11] << " " << r[12] << " " << r[13] << " " << r[14]
      <<  " procpath: "
      << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " " << p[4] << " "  
      << p[5] << " " << p[6] << " " << p[7] << " " << p[8] << " " << p[9] << " "  
      << p[10]<< " " << p[11]<< " " << p[12]<< " " << p[13]<< " " << p[14] );
  }
  for( int i=0; i<MAX_PATH; i++ ){
    if( rcgpath[i]>0 && rcgpath_proc[i]==0 ){ // 処理すべきパスが未受信
      ROS_INFO_STREAM("[END] job_no:" << jn << " integrate");
      return 0;
    }
  }

  printf(" cnt_res = %d\n", cnt_res );

#ifdef DUMP_UNILOG
  if(exportlog){ exportlog << "----- integrate -----" << endl << endl; }
#endif

  if( cnt_res==0 ){
    ROS_INFO_STREAM("job_no:" << jn << " 0 result from recognition node");
    resetproc();
    T2_robot_vision::RecognizedResultPtr data(new T2_robot_vision::RecognizedResult);
    data->obstacle_points = notice->calibrated_points[0];
    unification_pub_.publish(data);
    ROS_INFO_STREAM("cnt_res==0");
    ROS_INFO_STREAM("[END] job_no:" << jn << " integrate");
    return 1;
  }

  // debug
  for( int i=0; i<cnt_res; i++ ){
    ROS_DEBUG_STREAM("res_category[" << i << "]: " << res_category[i]);
    ROS_DEBUG_STREAM("res_pitch[" << i << "]: " << res_pitch[i]);
    ROS_DEBUG_STREAM("res_yaw[" << i << "]: " << res_yaw[i]);
    ROS_DEBUG_STREAM("res_roll[" << i << "]: " << res_roll[i]);
    ROS_DEBUG_STREAM("res_score[" << i << "]: " << res_score[i]);
  }

  T2_robot_vision::RecognizedResultPtr data(new T2_robot_vision::RecognizedResult);

  // convert depth image
  const sensor_msgs::Image depth = notice->recog_target.data[0].depth;
  const sensor_msgs::Image depth1 = notice->recog_target.data[0].depth_1;
  cv_bridge::CvImagePtr cv_depth_ptr;
  cv_bridge::CvImagePtr cv_depth1_ptr;
  cv::Mat dMat, d1Mat;  
  ROS_DEBUG_STREAM("sensor_msgs::Image -> cvMat (depth)");
  try{
    if(depth.encoding == "mono16"){
      cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
    } else if (depth.encoding == "16UC1"){
      sensor_msgs::Image imgd = depth;
      imgd.encoding = "mono16";
      cv_depth_ptr = cv_bridge::toCvCopy(imgd, sensor_msgs::image_encodings::MONO16);
    } else{
      //error
    }
    if(depth1.encoding == "mono16"){
      cv_depth1_ptr = cv_bridge::toCvCopy(depth1, sensor_msgs::image_encodings::MONO16);
    } else if (depth1.encoding == "16UC1"){
      sensor_msgs::Image imgd = depth1;
      imgd.encoding = "mono16";
      cv_depth1_ptr = cv_bridge::toCvCopy(imgd, sensor_msgs::image_encodings::MONO16);
    } else{
      //error
    }
  } catch (cv_bridge::Exception& e){
    ROS_ERROR("T2_robot_vision:cv_bridge exception:%s", e.what());
  }
  dMat = cv_depth_ptr->image;
  d1Mat = cv_depth1_ptr->image;

  for( int i=0; i<cnt_res; i++ ){
    res_seg_sx0[i] = res_seg_ex0[i] = res_seg_sy0[i] = res_seg_ey0[i] = 0;
    if( res_score[i]==0 ){
      continue;
    }
    //printf("res_seg_imgtype[%d]=%d\n", i, res_seg_imgtype[i]);
    ROS_DEBUG_STREAM("BEFORE ");
    ROS_DEBUG_STREAM("res_seg_sx[" << i << "]: " << res_seg_sx[i]);
    ROS_DEBUG_STREAM("res_seg_ex[" << i << "]: " << res_seg_ex[i]);
    ROS_DEBUG_STREAM("res_seg_sy[" << i << "]: " << res_seg_sy[i]);
    ROS_DEBUG_STREAM("res_seg_ey[" << i << "]: " << res_seg_ey[i]);

    if( res_seg_imgtype[i]==1 ){
      int sx0,sy0,ex0,ey0, dcnt=0;
      float sx1,sy1,ex1,ey1, depth=0.0;
      res_seg_sx0[i] = sx0 = res_seg_sx[i];
      res_seg_ex0[i] = ex0 = res_seg_ex[i];
      res_seg_sy0[i] = sy0 = res_seg_sy[i];
      res_seg_ey0[i] = ey0 = res_seg_ey[i];
      for( int jj=sy0; jj<ey0; jj++ ){
        for( int ii=sx0; ii<ex0; ii++ ){
          int d = (int) d1Mat.at<unsigned short>(jj, ii);
          if( d>0 ){
            depth += (float)d;
            dcnt++;
          }
        }
      }
      if( dcnt>0 ){
        depth /= (float)dcnt;
        ROS_DEBUG_STREAM( "depth=" << depth << " dcnt=" << dcnt );
      } else {
        depth = 500;
      }
      int camid = notice->recog_target.data[0].cam_id;
      ROS_DEBUG_STREAM( "cam_id=" << camid );
      convert( camid, (float)sx0, (float)sy0, depth, &sx1, &sy1, 0 ); // 0: color -> depth
      convert( camid, (float)ex0, (float)ey0, depth, &ex1, &ey1, 0 );
      res_seg_sx[i] = (int)sx1;
      res_seg_sy[i] = (int)sy1;
      res_seg_ex[i] = (int)ex1;
      res_seg_ey[i] = (int)ey1;
      if( res_seg_sx[i]<0 ){ res_seg_sx[i]=0; }
      if( res_seg_sy[i]<0 ){ res_seg_sy[i]=0; }
      if( res_seg_ex[i]<0 ){ res_seg_ex[i]=0; }
      if( res_seg_ey[i]<0 ){ res_seg_ey[i]=0; }
      if( res_seg_sx[i]>=dMat.cols ){ res_seg_sx[i]=dMat.cols-1; }
      if( res_seg_sy[i]>=dMat.rows ){ res_seg_sy[i]=dMat.rows-1; }
      if( res_seg_ex[i]>=dMat.cols ){ res_seg_ex[i]=dMat.cols-1; }
      if( res_seg_ey[i]>=dMat.rows ){ res_seg_ey[i]=dMat.rows-1; }
    }
    ROS_DEBUG_STREAM("AFTER ");
    ROS_DEBUG_STREAM("res_seg_sx[" << i << "]: " << res_seg_sx[i]);
    ROS_DEBUG_STREAM("res_seg_ex[" << i << "]: " << res_seg_ex[i]);
    ROS_DEBUG_STREAM("res_seg_sy[" << i << "]: " << res_seg_sy[i]);
    ROS_DEBUG_STREAM("res_seg_ey[" << i << "]: " << res_seg_ey[i]);
  }

  //
  // integration process
  //
  // choose result & set score=0
  for( int j=cnt_res-1; j>0; j-- ){
    for( int i=0; i<j; i++ ){
      if( res_category[i] == res_category[j]
          && res_category[i] != 9999
          &&(and_or(res_seg_sx[i],res_seg_ex[i],res_seg_sy[i],res_seg_ey[i],
            res_seg_sx[j],res_seg_ex[j],res_seg_sy[j],res_seg_ey[j]) > -0.1))
      {
        printf("compare result %d and %d\n", i,j );
        int *pp = table_procpriority[res_category[i]];
        int rmodi = res_recog_module[i];
        int rmodj = res_recog_module[j];
        if( pp[rmodi%10]>0 && pp[rmodi%10] > pp[rmodj%10] ){
          res_score[j] = 0; // choose i
        } else if( pp[rmodj%10]>0 && pp[rmodj%10] > pp[rmodi%10] ){
          res_score[i] = 0; // choose j
        } else if( pp[rmodi%10]>0 && pp[rmodi%10] == pp[rmodj%10] ){
          if( res_score[i] >= res_score[j] ){
            res_score[j] = 0;
          } else {
            res_score[i] = 0;
          }
        }

#ifdef DUMP_UNILOG
        if(exportlog){
          exportlog << "compare result " << i << "(cat:" << res_category[i] << ", mod:" << rmodi << ", pp:" << pp[rmodi]
                    << ") and " << j << "(cat:" << res_category[j] << ", mod:" << rmodj << ", pp:" << pp[rmodj]
                    << "), score_i:" << res_score[i] << ", score_j:" << res_score[j] << endl;
        }
#endif
      } // if
    }
  }

  // sort by score
  int idxa[MAX_RES], scr[MAX_RES], jj, cntidx=0;
  for( int i=0; i<MAX_RES; i++ ){ idxa[i]=-1; scr[i]=0; }
  idxa[0] = 0;
  scr[0] = res_score[0];
  cntidx++;
  for( int i=1; i<cnt_res; i++ ){
    // find position to insert
    for( jj=0; jj<cntidx; jj++ ){
      if( res_score[i] > scr[jj] ){
        break;
      }
    }
    // insert
    for( int k=cntidx; k>jj; k-- ){
      idxa[k] = idxa[k-1];
      scr[k] = scr[k-1];
    }
    idxa[jj] = i;
    scr[jj] = res_score[i];
    cntidx++;
  } // i


#ifdef DUMP_UNILOG
  if(exportlog){
    exportlog << endl;
    for( int k=0; k<cntidx; k++ ){
      int idx = idxa[k];
      exportlog << "idxa[" << k << "]=" << idx << ", score[" << idx << "]=" << res_score[idx] << endl;
    }
    exportlog << endl;
  }
#endif

  //
  // process all data
  //
  data->obstacle_points = notice->calibrated_points[0]; // deep copy
  for( int k=0; k<cntidx; k++ ){

  unsigned int single_gp = 0;
  int idx = idxa[k];
  if( idx<0 || res_score[idx]==0 ){
    continue; // k
  }
  int mid = res_category[idx];

  ROS_INFO_STREAM( "k = " << k << ", idx = " << idx << ", mid = " << mid );

  if( data->items.size() > 0 && res_recog_module[idx]==9 ){
    continue; // k
  }

  ROS_DEBUG_STREAM("sensor_msgs::Image -> cvMat (mask)");
  const sensor_msgs::Image mask = res_seg_mask[idx];
  int msk_w, msk_h;
  cv_bridge::CvImagePtr cv_mask_ptr;
  cv::Mat mMat;
  if( res_recog_module[idx]==3 || res_recog_module[idx]==5 || res_recog_module[idx]==10){ // 3:YOLO, 5:YOLO+distlearn,10tottori
    ROS_INFO_STREAM( "res_recog_module[" << idx << "] = " << res_recog_module[idx] << " -> make mask" );

    int sx0,sy0,ex0,ey0, dhist[256], dthres=0, dcnt=0;
    int mind=200, maxd=1000;
    sx0 = res_seg_sx[idx];
    ex0 = res_seg_ex[idx];
    sy0 = res_seg_sy[idx];
    ey0 = res_seg_ey[idx];
    msk_w = ex0-sx0;
    msk_h = ey0-sy0;
    mMat = cv::Mat::zeros(msk_h,msk_w,CV_8UC1);
    memset( dhist, 0, sizeof(int)*256 );
    for( int jj=sy0; jj<ey0; jj++ ){
      for( int ii=sx0; ii<ex0; ii++ ){
        int d = (int) dMat.at<unsigned short>(jj, ii);
        int hd = (int)( (double)(d-mind)/(double)(maxd-mind)*255.0 );
        if( 0<=hd && hd<256 ){
          dhist[hd]++; dcnt++;
        }
      }
    }
    dthres = dcnt*0.8; dcnt=0;
    for( int i=0; i<256; i++ ){
      dcnt += dhist[i];
      if( dthres <= dcnt ){
        dthres = (int)( (double)i*(double)(maxd-mind)/255.0 +mind );
        break;
      }
    }
    for( int jj=sy0; jj<ey0; jj++ ){
      for( int ii=sx0; ii<ex0; ii++ ){
        int p=ii-sx0, q=jj-sy0;
        int d = (int) dMat.at<unsigned short>(jj, ii);
        if( 0<d && d<dthres ){
          mMat.at<unsigned char>(q,p) = 1;
        }
      }
    } 
  } else {
    ROS_INFO_STREAM( "else -> cv_bridge::toCvCopy" );

    try{
      if( mask.width>0 && mask.height>0 ){
        cv_mask_ptr = cv_bridge::toCvCopy(mask, sensor_msgs::image_encodings::MONO8);
        mMat = cv_mask_ptr->image;
      }
    } catch (cv_bridge::Exception& e){
      ROS_ERROR("T2_robot_vision:cv_bridge exception:%s", e.what());
    }
    msk_w = mask.width;
    msk_h = mask.height;
    if( res_seg_imgtype[idx]==1 ){
      ROS_INFO_STREAM( "  res_seg_imgtype[" << idx << "]==1 -> change mask size" );
      cv::Mat mMat0 = mMat;
      msk_w = res_seg_ex[idx] - res_seg_sx[idx];
      if(msk_w <=0) {msk_w=1;}
      msk_h = res_seg_ey[idx] - res_seg_sy[idx];
      if(msk_h <=0) {msk_h=1;}
      mMat = cv::Mat::zeros(msk_h,msk_w,CV_8UC1);
      double di=0.0, dj=0.0;
      double dx=(double)mMat0.cols/(double)msk_w;
      double dy=(double)mMat0.rows/(double)msk_h;
      for(int jj=0; jj<msk_h; jj++){
        int sj = floor(dj+0.5);
        if( sj >= mMat0.rows ){ sj = mMat0.rows-1; }
        if( sj < 0 ){ sj = 0; }
        di=0.0;
        for(int ii=0; ii<msk_w; ii++){
          int si = floor(di+0.5);
          if( si >= mMat0.cols ){ si = mMat0.cols-1; }
          if( si < 0 ){ si = 0; }
          mMat.at<unsigned char>(jj,ii) = mMat0.at<unsigned char>(sj,si);
          di+=dx;
        }
        dj+=dy;
      }
    }
  }

  ROS_DEBUG_STREAM("PointCloud -> cv::Mat");
  const sensor_msgs::PointCloud2 pc2 = notice->recog_target.data[0].points;
  uint32_t h = pc2.height;
  uint32_t w = pc2.width;
  uint32_t row_step = pc2.row_step;
    
  if (h == 1){
    h = cv_depth_ptr->image.rows;
    w = cv_depth_ptr->image.cols;
    row_step = w*pc2.point_step;
  }
  cv::Mat pMat(h, w, CV_32FC3);
  for (size_t v = 0; v < h; v++){
    for (size_t u = 0; u < w; u++){
      for (uint32_t channel = 0; channel < 3; channel++){
	memcpy(&pMat.at<cv::Vec3f>(v, u)[channel],
	 &pc2.data[u*pc2.point_step + v*row_step + pc2.fields[channel].offset], sizeof(float));
      } //channel
    } //w
  } //h
  ROS_DEBUG_STREAM("point cloud w=" << w << ", h=" << h );

  // calculate average depth, x, y, z of the segment
  int i,j, sx,sy,ex,ey, aved=0, cnt=0;
  float avex=0.0, avey=0.0, avez=0.0, nrmx, nrmy, nrmz, len;
  sx = res_seg_sx[idx];
  sy = res_seg_sy[idx];
  ex = res_seg_ex[idx];
  ey = res_seg_ey[idx];

#if 0
  if( res_recog_module[idx]==10 ){
    avex = (double)res_posx[i]/100.0; // mm -> m
    avey = (double)res_posy[i]/100.0;
    avez = (double)res_posz[i]/100.0;
  } else {
#else
  {
#endif
    ROS_DEBUG_STREAM("depth image step=" << dMat.step << ", elemsize=" << dMat.elemSize() );
    ROS_DEBUG_STREAM("point cloud step=" << pMat.step << ", elemsize=" << pMat.elemSize() );
    ROS_DEBUG_STREAM("sx=" << sx << ", sy=" << sy << ", ex=" << ex << ", ey=" << ey );
    for( j=sy; j<ey; j++ ){
      for( i=sx; i<ex; i++ ){
        int m = 1;
        if( msk_w>0 && msk_h>0 ){
          m = (int) (mMat.data[ (j-sy)*mMat.step + (i-sx)*mMat.elemSize() ]);
        }
        int d = (int) dMat.at<unsigned short>(j, i);
        float x = (float) pMat.at<cv::Vec3f>(j,i)[0];
        float y = (float) pMat.at<cv::Vec3f>(j,i)[1];
        float z = (float) pMat.at<cv::Vec3f>(j,i)[2];
        if( m>0 && !(x==0 && y==0 && z==0)){
          aved += d;
          avex += x;
          avey += y;
          avez += z;
          cnt++;
        }
      }
    }

    if(cnt>0){
      aved /= cnt;
      avex /= cnt;  //avex = -avex;
      avey /= cnt;
      avez /= cnt;
    }else{
      ROS_INFO_STREAM( "calc average -> cntg==0" );
      continue; // for(k)
    }
  } // if/else

  len = sqrt( avex*avex + avey*avey + avez*avez );
  nrmx = avex/len;
  nrmy = avey/len;
  nrmz = avez/len;
  ROS_DEBUG_STREAM("aved=" << aved << ", sx=" << sx << ", sy=" << sy << ", ex=" << ex << ", ey=" << ey );
  ROS_DEBUG_STREAM("avex=" << avex*100 << ", avey=" << avey*100 << ", avez=" << avez*100 );

  float rm[16];
  if( res_recog_module[idx]==3 || res_recog_module[idx]==9 // YOLO, seg+plane
      || table_type[mid]==1 ){ // deformable
    ROS_INFO_STREAM( "res_recog_module[" << idx << "]=" << res_recog_module[idx]
                     << ", table_type[" << mid << "] -> unit mat in world coordinate system" );
    double a=-50.0/180.0*M_PI;
    rm[0] = 0.0;     rm[1] = -1.0; rm[2] = 0.0;      rm[3] = avex;
    rm[4] = sin(a);  rm[5] = 0.0;  rm[6] = -cos(a);  rm[7] = avey;
    rm[8] = cos(a);  rm[9] = 0.0;  rm[10]= sin(a);   rm[11]= avez;
    rm[12]= 0.0;     rm[13]= 0.0;  rm[14]= 0.0;      rm[15]= 1.0;
#if 1
  } else if( res_recog_module[idx]==2 || res_recog_module[idx]==6 || res_recog_module[idx]==8 // plane
            || table_type[mid]==2 ){ // transparent
    if( gp_type[mid]==1 ){
      ROS_INFO_STREAM( "res_recog_module[" << idx << "]=" << res_recog_module[idx]
                       << ", gp_type[" << mid << "] = " << gp_type[mid]
                       <<  "-> detect plane" );
      double *x = new double[cnt];
      double *y = new double[cnt];
      double *z = new double[cnt];
      double cen[3], eval[3], evec[16];
      int cnt0=0;
      for( j=sy; j<ey && cnt0<cnt; j++ ){
        for( i=sx; i<ex && cnt0<cnt; i++ ){
          int m = 1;
          if( msk_w>0 && msk_h>0 ){
            m = (int) (mMat.data[ (j-sy)*mMat.step + (i-sx)*mMat.elemSize() ]);
          }
          double x0 = (double) pMat.at<cv::Vec3f>(j,i)[0];
          double y0 = (double) pMat.at<cv::Vec3f>(j,i)[1];
          double z0 = (double) pMat.at<cv::Vec3f>(j,i)[2];
          if( m>0 && !(x0==0 && y0==0 && z0==0)){
            x[cnt0] = x0;
            y[cnt0] = y0;
            z[cnt0] = z0;
            cnt0++;
          }
        }
      }
      cen[0] = avex;
      cen[1] = avey;
      cen[2] = avez;
      int sts = PCA3( x, y, z, cnt, cen, eval, evec);

      double tmpeval[3], tmpevec[16];
      memcpy( tmpevec, evec, sizeof(double)*16 );
      memcpy( tmpeval, eval, sizeof(double)*3 );
      eval[0] = tmpeval[1];  eval[1] = tmpeval[0]; // eval[2] = tmpeval[2];
      evec[0] = tmpevec[4];  evec[1] = tmpevec[5];  evec[2] = tmpevec[6]; // X <- Y
      evec[4] = -tmpevec[0];  evec[5] = -tmpevec[1];  evec[6] = -tmpevec[2]; // Y <- -X

      std::ostringstream fname;
      fname.str("");
      fname << "plane" << k << ".pcd";    writePCD( fname.str(), cnt, x, y, z );

      delete[] x;  x = NULL;
      delete[] y;  y = NULL;
      delete[] z;  z = NULL;

      if( evec[10]>0 ){
        rm[0] = evec[0];  rm[1] = evec[4];  rm[2] = evec[8];  rm[3] = avex;
        rm[4] = evec[1];  rm[5] = evec[5];  rm[6] = evec[9];  rm[7] = avey;
        rm[8] = evec[2];  rm[9] = evec[6];  rm[10]= evec[10]; rm[11]= avez;
      } else {
        rm[0] = -evec[0]; rm[1] = -evec[4]; rm[2] = -evec[8]; rm[3] = avex;
        rm[4] = -evec[1]; rm[5] = -evec[5]; rm[6] = -evec[9]; rm[7] = avey;
        rm[8] = -evec[2]; rm[9] = -evec[6]; rm[10]=-evec[10]; rm[11]= avez;
      }
      rm[12]= 0.0;      rm[13]= 0.0;      rm[14]= 0.0;      rm[15]= 1.0;

    } else {
      ROS_INFO_STREAM( "res_recog_module[" << idx << "]=" << res_recog_module[idx]
                       << ", gp_type[" << mid << "] = " << gp_type[mid]
                       << "-> unit mat in world coordinate system" );
      double a=-50.0/180.0*M_PI;
      rm[0] = 1.0;  rm[1] = 0.0;     rm[2] = 0.0;      rm[3] = avex;
      rm[4] = 0.0;  rm[5] = cos(a);  rm[6] = -sin(a);  rm[7] = avey;
      rm[8] = 0.0;  rm[9] = sin(a);  rm[10]= cos(a);   rm[11]= avez;
      rm[12]= 0.0;  rm[13]= 0.0;     rm[14]= 0.0;      rm[15]= 1.0;
    }

    ROS_DEBUG_STREAM("rm0 = " << rm[0] << " rm1 = " << rm[1] << " rm2 = " << rm[2] << " rm3 = " << rm[3] );
    ROS_DEBUG_STREAM("rm4 = " << rm[4] << " rm5 = " << rm[5] << " rm6 = " << rm[6] << " rm7 = " << rm[7] );
    ROS_DEBUG_STREAM("rm8 = " << rm[8] << " rm9 = " << rm[9] << " rm10= " << rm[10]<< " rm11= " << rm[11] );
    ROS_DEBUG_STREAM("rm12= " << rm[12]<< " rm13= " << rm[13]<< " rm14= " << rm[14]<< " rm15= " << rm[15] );

    float rm0[16];
    double *tm=gp_mat[mid];

    ROS_DEBUG_STREAM("tm0 = " << tm[0] << " tm1 = " << tm[1] << " tm2 = " << tm[2] << " tm3 = " << tm[3] );
    ROS_DEBUG_STREAM("tm4 = " << tm[4] << " tm5 = " << tm[5] << " tm6 = " << tm[6] << " tm7 = " << tm[7] );
    ROS_DEBUG_STREAM("tm8 = " << tm[8] << " tm9 = " << tm[9] << " tm10= " << tm[10]<< " tm11= " << tm[11] );
    ROS_DEBUG_STREAM("tm12= " << tm[12]<< " tm13= " << tm[13]<< " tm14= " << tm[14]<< " tm15= " << tm[15] );

    rm0[0] = rm[0]*tm[0] + rm[1]*tm[4] + rm[2]*tm[8] + rm[3]*tm[12];
    rm0[1] = rm[0]*tm[1] + rm[1]*tm[5] + rm[2]*tm[9] + rm[3]*tm[13];
    rm0[2] = rm[0]*tm[2] + rm[1]*tm[6] + rm[2]*tm[10]+ rm[3]*tm[14];
    rm0[3] = rm[0]*tm[3] + rm[1]*tm[7] + rm[2]*tm[11]+ rm[3]*tm[15];
    rm0[4] = rm[4]*tm[0] + rm[5]*tm[4] + rm[6]*tm[8] + rm[7]*tm[12];
    rm0[5] = rm[4]*tm[1] + rm[5]*tm[5] + rm[6]*tm[9] + rm[7]*tm[13];
    rm0[6] = rm[4]*tm[2] + rm[5]*tm[6] + rm[6]*tm[10]+ rm[7]*tm[14];
    rm0[7] = rm[4]*tm[3] + rm[5]*tm[7] + rm[6]*tm[11]+ rm[7]*tm[15];
    rm0[8] = rm[8]*tm[0] + rm[9]*tm[4] + rm[10]*tm[8] + rm[11]*tm[12];
    rm0[9] = rm[8]*tm[1] + rm[9]*tm[5] + rm[10]*tm[9] + rm[11]*tm[13];
    rm0[10]= rm[8]*tm[2] + rm[9]*tm[6] + rm[10]*tm[10]+ rm[11]*tm[14];
    rm0[11]= rm[8]*tm[3] + rm[9]*tm[7] + rm[10]*tm[11]+ rm[11]*tm[15];
    rm0[12]= rm[12]*tm[0]+ rm[13]*tm[4]+ rm[14]*tm[8] + rm[15]*tm[12];
    rm0[13]= rm[12]*tm[1]+ rm[13]*tm[5]+ rm[14]*tm[9] + rm[15]*tm[13];
    rm0[14]= rm[12]*tm[2]+ rm[13]*tm[6]+ rm[14]*tm[10]+ rm[15]*tm[14];
    rm0[15]= rm[12]*tm[3]+ rm[13]*tm[7]+ rm[14]*tm[11]+ rm[15]*tm[15];
    memcpy(rm,rm0,sizeof(float)*16);
    single_gp = 1;

    ROS_DEBUG_STREAM("AFTER *gp_mat" );
    ROS_DEBUG_STREAM("rm0 = " << rm[0] << " rm1 = " << rm[1] << " rm2 = " << rm[2] << " rm3 = " << rm[3] );
    ROS_DEBUG_STREAM("rm4 = " << rm[4] << " rm5 = " << rm[5] << " rm6 = " << rm[6] << " rm7 = " << rm[7] );
    ROS_DEBUG_STREAM("rm8 = " << rm[8] << " rm9 = " << rm[9] << " rm10= " << rm[10]<< " rm11= " << rm[11] );
    ROS_DEBUG_STREAM("rm12= " << rm[12]<< " rm13= " << rm[13]<< " rm14= " << rm[14]<< " rm15= " << rm[15] );
#endif
  } else {
    ROS_INFO_STREAM( "else ->  roll,pitch,yaw -> rotation mat -> ICP" );
    int mid = res_category[idx];
    float rmat[9];
    if( 0 ){
    } else {
      float ryaw, rpch, rrol, siny, cosy, sinp, cosp, sinr, cosr;
      float myaw[9], mpch[9], mrol[9], moc[9], tmp0[9], tmp1[9], tmp2[9], ocrd[9];

      ryaw = (float)(-res_yaw[idx]) *M_PI/180.0;  siny = sin(ryaw);  cosy = cos(ryaw);
      myaw[0] = cosy;  myaw[1] =-siny;  myaw[2] = 0;
      myaw[3] = siny;  myaw[4] = cosy;  myaw[5] = 0;
      myaw[6] = 0;     myaw[7] = 0;     myaw[8] = 1;
      ROS_DEBUG_STREAM("res_yaw = " << res_yaw[idx] << ", ryaw = " << ryaw );
      ROS_DEBUG_STREAM("myaw0 = " << myaw[0] << " myaw1 = " << myaw[1] << " myaw2 = " << myaw[2] );
      ROS_DEBUG_STREAM("myaw3 = " << myaw[3] << " myaw4 = " << myaw[4] << " myaw5 = " << myaw[5] );
      ROS_DEBUG_STREAM("myaw6 = " << myaw[6] << " myaw7 = " << myaw[7] << " myaw8 = " << myaw[8] );

      rpch = (float)(res_pitch[idx])*M_PI/180.0;  sinp = sin(rpch);  cosp = cos(rpch);
      mpch[0] = 1;  mpch[1] = 0;     mpch[2] = 0;
      mpch[3] = 0;  mpch[4] = cosp;  mpch[5] =-sinp;
      mpch[6] = 0;  mpch[7] = sinp;  mpch[8] = cosp;
      ROS_DEBUG_STREAM("res_pitch = " << res_pitch[idx] << ", rpch = " << rpch );
      ROS_DEBUG_STREAM("mpch0 = " << mpch[0] << " mpch1 = " << mpch[1] << " mpch2 = " << mpch[2] );
      ROS_DEBUG_STREAM("mpch3 = " << mpch[3] << " mpch4 = " << mpch[4] << " mpch5 = " << mpch[5] );
      ROS_DEBUG_STREAM("mpch6 = " << mpch[6] << " mpch7 = " << mpch[7] << " mpch8 = " << mpch[8] );

      rrol = (float)(-res_roll[idx]) *M_PI/180.0;
      sinr = sin(rrol);  cosr = cos(rrol);
      mrol[0] = cosr;  mrol[1] = 0;  mrol[2] = sinr;
      mrol[3] = 0;     mrol[4] = 1;  mrol[5] = 0;
      mrol[6] =-sinr;  mrol[7] = 0;  mrol[8] = cosr;
      ROS_DEBUG_STREAM("res_roll = " << res_roll[idx] << ", rrol = " << rrol );
      ROS_DEBUG_STREAM("mrol0 = " << mrol[0] << " mrol1 = " << mrol[1] << " mrol2 = " << mrol[2] );
      ROS_DEBUG_STREAM("mrol3 = " << mrol[3] << " mrol4 = " << mrol[4] << " mrol5 = " << mrol[5] );
      ROS_DEBUG_STREAM("mrol6 = " << mrol[6] << " mrol7 = " << mrol[7] << " mrol8 = " << mrol[8] );
  
      // object -> camera (x-right, y-down)
      moc[0] = 1;  moc[1] = 0;  moc[2] = 0;
      moc[3] = 0;  moc[4] = 0;  moc[5] =-1;
      moc[6] = 0;  moc[7] = 1;  moc[8] = 0;
      ROS_DEBUG_STREAM("moc0 = " << moc[0] << " moc1 = " << moc[1] << " moc2 = " << moc[2] );
      ROS_DEBUG_STREAM("moc3 = " << moc[3] << " moc4 = " << moc[4] << " moc5 = " << moc[5] );
      ROS_DEBUG_STREAM("moc6 = " << moc[6] << " moc7 = " << moc[7] << " moc8 = " << moc[8] );

      mmult9( tmp0, mpch, myaw );
      ROS_DEBUG_STREAM("tmp00 = " << tmp0[0] << " tmp01 = " << tmp0[1] << " tmp02 = " << tmp0[2] );
      ROS_DEBUG_STREAM("tmp03 = " << tmp0[3] << " tmp04 = " << tmp0[4] << " tmp05 = " << tmp0[5] );
      ROS_DEBUG_STREAM("tmp06 = " << tmp0[6] << " tmp07 = " << tmp0[7] << " tmp08 = " << tmp0[8] );
      mmult9( tmp1, mrol, tmp0 );
      ROS_DEBUG_STREAM("tmp10 = " << tmp1[0] << " tmp11 = " << tmp1[1] << " tmp12 = " << tmp1[2] );
      ROS_DEBUG_STREAM("tmp13 = " << tmp1[3] << " tmp14 = " << tmp1[4] << " tmp15 = " << tmp1[5] );
      ROS_DEBUG_STREAM("tmp16 = " << tmp1[6] << " tmp17 = " << tmp1[7] << " tmp18 = " << tmp1[8] );
      double *ocrd0 = table_matrix[res_category[idx]];
      for( int i=0; i<9; i++ ){ ocrd[i] = ocrd0[i]; }
      ROS_DEBUG_STREAM("ocrd0 = " << ocrd[0] << " ocrd1 = " << ocrd[1] << " ocrd2 = " << ocrd[2] );
      ROS_DEBUG_STREAM("ocrd3 = " << ocrd[3] << " ocrd4 = " << ocrd[4] << " ocrd5 = " << ocrd[5] );
      ROS_DEBUG_STREAM("ocrd6 = " << ocrd[6] << " ocrd7 = " << ocrd[7] << " ocrd8 = " << ocrd[8] );
      mmult9( tmp2, tmp1, ocrd );
      ROS_DEBUG_STREAM("tmp20 = " << tmp2[0] << " tmp21 = " << tmp2[1] << " tmp22 = " << tmp2[2] );
      ROS_DEBUG_STREAM("tmp23 = " << tmp2[3] << " tmp24 = " << tmp2[4] << " tmp25 = " << tmp2[5] );
      ROS_DEBUG_STREAM("tmp26 = " << tmp2[6] << " tmp27 = " << tmp2[7] << " tmp28 = " << tmp2[8] );
      mmult9( rmat, moc, tmp2 );
      ROS_DEBUG_STREAM("rmat0 = " << rmat[0] << " rmat1 = " << rmat[1] << " rmat2 = " << rmat[2] );
      ROS_DEBUG_STREAM("rmat3 = " << rmat[3] << " rmat4 = " << rmat[4] << " rmat5 = " << rmat[5] );
      ROS_DEBUG_STREAM("rmat6 = " << rmat[6] << " rmat7 = " << rmat[7] << " rmat8 = " << rmat[8] );
    }
    
  float qx,qy,qz,qtheta,sinq2,cosq2, q[4], qmat[9];
  qx = 0*nrmz - nrmz*nrmy;
  qy = nrmz*nrmx - 0*nrmz;
  qz = 0*nrmy - 0*nrmx;
  len = sqrt(qx*qx+qy*qy+qz*qz); qx/=len; qy/=len; qz/=len;
  qtheta = acos(nrmx*0 + nrmy*0 + nrmz*1);
  ROS_DEBUG_STREAM("mrnz = " << nrmz << " qtheta(deg) = " << qtheta*180.0/M_PI );
  sinq2 = sin(qtheta*0.5);
  cosq2 = cos(qtheta*0.5);
  q[0] = sinq2*qx;
  q[1] = sinq2*qy;
  q[2] = sinq2*qz;
  q[3] = cosq2;
  qmat[0] = 1.0-2.0*(q[1]*q[1]+q[2]*q[2]);
  qmat[1] = 2.0*(q[0]*q[1]-q[2]*q[3]);
  qmat[2] = 2.0*(q[2]*q[0]+q[1]*q[3]);
  qmat[3] = 2.0*(q[0]*q[1]+q[2]*q[3]);
  qmat[4] = 1.0-2.0*(q[2]*q[2]+q[0]*q[0]);
  qmat[5] = 2.0*(q[1]*q[2]-q[0]*q[3]);
  qmat[6] = 2.0*(q[2]*q[0]-q[1]*q[3]);
  qmat[7] = 2.0*(q[1]*q[2]+q[0]*q[3]);
  qmat[8] = 1.0-2.0*(q[0]*q[0]+q[1]*q[1]);
  ROS_DEBUG_STREAM("qx = " << qx << " qy = " << qy << " qz = " << qz << " qt = " << qtheta*180/M_PI );
  ROS_DEBUG_STREAM("qm0 = " << qmat[0] << " qm1 = " << qmat[1] << " qm2 = " << qmat[2] );
  ROS_DEBUG_STREAM("qm3 = " << qmat[3] << " qm4 = " << qmat[4] << " qm5 = " << qmat[5] );
  ROS_DEBUG_STREAM("qm6 = " << qmat[6] << " qm7 = " << qmat[7] << " qm8 = " << qmat[8] );
  rm[0] = qmat[0]*rmat[0] + qmat[1]*rmat[3] + qmat[2]*rmat[6];
  rm[1] = qmat[0]*rmat[1] + qmat[1]*rmat[4] + qmat[2]*rmat[7];
  rm[2] = qmat[0]*rmat[2] + qmat[1]*rmat[5] + qmat[2]*rmat[8];
  rm[3] = avex;
  rm[4] = qmat[3]*rmat[0] + qmat[4]*rmat[3] + qmat[5]*rmat[6];
  rm[5] = qmat[3]*rmat[1] + qmat[4]*rmat[4] + qmat[5]*rmat[7];
  rm[6] = qmat[3]*rmat[2] + qmat[4]*rmat[5] + qmat[5]*rmat[8];
  rm[7] = avey;
  rm[8] = qmat[6]*rmat[0] + qmat[7]*rmat[3] + qmat[8]*rmat[6];
  rm[9] = qmat[6]*rmat[1] + qmat[7]*rmat[4] + qmat[8]*rmat[7];
  rm[10]= qmat[6]*rmat[2] + qmat[7]*rmat[5] + qmat[8]*rmat[8];
  rm[11]= avez;
  rm[12]= 0.0;
  rm[13]= 0.0;
  rm[14]= 0.0;
  rm[15]= 1.0;
  ROS_DEBUG_STREAM("rm0 = " << rm[0] << " rm1 = " << rm[1] << " rm2 = " << rm[2] << " rm3 = " << rm[3] );
  ROS_DEBUG_STREAM("rm4 = " << rm[4] << " rm5 = " << rm[5] << " rm6 = " << rm[6] << " rm7 = " << rm[7] );
  ROS_DEBUG_STREAM("rm8 = " << rm[8] << " rm9 = " << rm[9] << " rm10= " << rm[10]<< " rm11= " << rm[11] );
  ROS_DEBUG_STREAM("rm12= " << rm[12]<< " rm13= " << rm[13]<< " rm14= " << rm[14]<< " rm15= " << rm[15] );

#if ICP_MODE == 2
  if( cloud_model[mid]!=NULL || normal_model[mid]!=NULL ){

  const sensor_msgs::PointCloud2 pc3 = notice->calibrated_points[0];
  uint32_t h3 = pc3.height;
  uint32_t w3 = pc3.width;
  uint32_t row_step3 = pc3.row_step;

  cv::Mat pMat3(h3, w3, CV_32FC3);
  for(size_t v = 0; v < h3; v++){
    for(size_t u = 0; u < w3; u++){
      for(uint32_t channel = 0; channel < 3; channel++){
	memcpy(&pMat3.at<cv::Vec3f>(v, u)[channel],
         &pc3.data[u*pc3.point_step + v*row_step3 + pc3.fields[channel].offset], sizeof(float));
      } //channel
    } //w
  } //h

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int pcnt0=0, pcnt1=0;
  for( j=sy; j<ey; j++ ){
    for( i=sx; i<ex; i++ ){
      int m = 1;
      //if( mask.width>0 && mask.height>0 ){
      if( msk_w>0 && msk_h>0 ){
        m = (int) (mMat.data[ (j-sy)*mMat.step + (i-sx)*mMat.elemSize() ]);
      }
      if( m>0 ){ pcnt0++; }
    }
  }
 
  cloud->width    = pcnt0;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for( j=sy; j<ey; j++ ){
    for( i=sx; i<ex; i++ ){
      int m = 1;
      //if( mask.width>0 && mask.height>0 ){
      if( msk_w>0 && msk_h>0 ){
        m = (int) (mMat.data[ (j-sy)*mMat.step + (i-sx)*mMat.elemSize() ]);
      }
      if( m>0 ){
        cloud->points[pcnt1].x = pMat3.at<cv::Vec3f>(j,i)[0];
        cloud->points[pcnt1].y = pMat3.at<cv::Vec3f>(j,i)[1];
        cloud->points[pcnt1].z = pMat3.at<cv::Vec3f>(j,i)[2];
        pcnt1++;
      }
    }
  }
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
  int size = cloud_model[mid]->size();
  int *flg = new int[size];  memset( flg, 0, sizeof(int)*size );  
  int mvcnt0=0;
  for( i=0; i<size; i++ ){
    float nx0 = normal_model[mid]->points[i].normal_x;
    float ny0 = normal_model[mid]->points[i].normal_y;
    float nz0 = normal_model[mid]->points[i].normal_z;
    float nx1 = rm[0]*nx0 + rm[1]*ny0 + rm[2]*nz0;
    float ny1 = rm[4]*nx0 + rm[5]*ny0 + rm[6]*nz0;
    float nz1 = rm[8]*nx0 + rm[9]*ny0 + rm[10]*nz0;
    if( nz1 < 0 ){
      flg[i] = 1;
      mvcnt0++;
    }
  }
  model->width    = mvcnt0;
  model->height   = 1;
  model->is_dense = false;
  model->points.resize (model->width * model->height);
  int mvcnt1=0;
  for( i=0; i<size; i++ ){
    if( flg[i]==0 ){
      continue;
    }
    float x = cloud_model[mid]->points[i].x;
    float y = cloud_model[mid]->points[i].y;
    float z = cloud_model[mid]->points[i].z;
    model->points[mvcnt1].x = rm[0]*x + rm[1]*y + rm[2]*z + rm[3];
    model->points[mvcnt1].y = rm[4]*x + rm[5]*y + rm[6]*z + rm[7];
    model->points[mvcnt1].z = rm[8]*x + rm[9]*y + rm[10]*z + rm[11];
    mvcnt1++;
  }
  if( flg ){ delete[] flg; flg=NULL; }

  std::string src_root_path = ros::package::getPath("T2_robot_vision");
  std::string debug_path = src_root_path + "/data/debug/";
  std::ostringstream fname;
  fname.str(""); fname << "target" << k << ".pcd"; writePCD( fname.str(), cloud );
  fname.str(""); fname << "model" << k << ".pcd"; writePCD( fname.str(), model );

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations (20);
  icp.setInputSource (model);
  icp.setInputTarget (cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);  
  icp.align (*Final);
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  fname.str(""); fname << "final" << k << ".pcd"; writePCD( fname.str(), Final );

  Eigen::Matrix4f tm = icp.getFinalTransformation();

  float rm0[16];
  rm0[0] = tm(0,0)*rm[0] + tm(0,1)*rm[4] + tm(0,2)*rm[8] + tm(0,3)*rm[12];
  rm0[1] = tm(0,0)*rm[1] + tm(0,1)*rm[5] + tm(0,2)*rm[9] + tm(0,3)*rm[13];
  rm0[2] = tm(0,0)*rm[2] + tm(0,1)*rm[6] + tm(0,2)*rm[10]+ tm(0,3)*rm[14];
  rm0[3] = tm(0,0)*rm[3] + tm(0,1)*rm[7] + tm(0,2)*rm[11]+ tm(0,3)*rm[15];
  rm0[4] = tm(1,0)*rm[0] + tm(1,1)*rm[4] + tm(1,2)*rm[8] + tm(1,3)*rm[12];
  rm0[5] = tm(1,0)*rm[1] + tm(1,1)*rm[5] + tm(1,2)*rm[9] + tm(1,3)*rm[13];
  rm0[6] = tm(1,0)*rm[2] + tm(1,1)*rm[6] + tm(1,2)*rm[10]+ tm(1,3)*rm[14];
  rm0[7] = tm(1,0)*rm[3] + tm(1,1)*rm[7] + tm(1,2)*rm[11]+ tm(1,3)*rm[15];
  rm0[8] = tm(2,0)*rm[0] + tm(2,1)*rm[4] + tm(2,2)*rm[8] + tm(2,3)*rm[12];
  rm0[9] = tm(2,0)*rm[1] + tm(2,1)*rm[5] + tm(2,2)*rm[9] + tm(2,3)*rm[13];
  rm0[10]= tm(2,0)*rm[2] + tm(2,1)*rm[6] + tm(2,2)*rm[10]+ tm(2,3)*rm[14];
  rm0[11]= tm(2,0)*rm[3] + tm(2,1)*rm[7] + tm(2,2)*rm[11]+ tm(2,3)*rm[15];
  rm0[12]= tm(3,0)*rm[0] + tm(3,1)*rm[4] + tm(3,2)*rm[8] + tm(3,3)*rm[12];
  rm0[13]= tm(3,0)*rm[1] + tm(3,1)*rm[5] + tm(3,2)*rm[9] + tm(3,3)*rm[13];
  rm0[14]= tm(3,0)*rm[2] + tm(3,1)*rm[6] + tm(3,2)*rm[10]+ tm(3,3)*rm[14];
  rm0[15]= tm(3,0)*rm[3] + tm(3,1)*rm[7] + tm(3,2)*rm[11]+ tm(3,3)*rm[15];
  memcpy(rm,rm0,sizeof(float)*16);
  }
  }

  cnt_msg = cnt_res = 0;  // reset counter
  catch_robot_msgs::ItemData item;

  if( res_category[idx] < 0 || MAX_MCNT <= res_category[idx] ){
    item.cad_id = 0;
  } else {
    item.cad_id = table_cadid[ res_category[idx] ];
  }

  item.category_id = res_category[idx];
  item.scale.x = 1.0;
  item.scale.y = 1.0;
  item.scale.z = 1.0;
  item.probability = (float)res_score[idx]*0.01;
  item.pose.m[0] = rm[0];  item.pose.m[1] = rm[1];  item.pose.m[2] = rm[2];  item.pose.m[3] = rm[3];
  item.pose.m[4] = rm[4];  item.pose.m[5] = rm[5];  item.pose.m[6] = rm[6];  item.pose.m[7] = rm[7];
  item.pose.m[8] = rm[8];  item.pose.m[9] = rm[9];  item.pose.m[10]= rm[10]; item.pose.m[11]= rm[11];
  item.pose.m[12]= rm[12]; item.pose.m[13]= rm[13]; item.pose.m[14]= rm[14]; item.pose.m[15]= rm[15];
  item.single_gp = single_gp;
  data->items.push_back(item);

  ROS_INFO_STREAM("job_no:" << jn << " publish recognized result");
  ROS_INFO_STREAM("job_no:" << jn << " item.cad_id: " << item.cad_id );
  ROS_INFO_STREAM("job_no:" << jn << " item.category_id: " << item.category_id );
  ROS_INFO_STREAM("job_no:" << jn << " item.scale.x: " << item.scale.x << "y: " << item.scale.y << "z: " << item.scale.z );
  ROS_INFO_STREAM("job_no:" << jn << " item.probability: " << item.probability );
  ROS_INFO_STREAM("job_no:" << jn << " item.pose.m: " );
  ROS_INFO_STREAM(" " << item.pose.m[0] << " " << item.pose.m[1] << " " << item.pose.m[2] << " " << item.pose.m[3] );
  ROS_INFO_STREAM(" " << item.pose.m[4] << " " << item.pose.m[5] << " " << item.pose.m[6] << " " << item.pose.m[7] );
  ROS_INFO_STREAM(" " << item.pose.m[8] << " " << item.pose.m[9] << " " << item.pose.m[10] << " " << item.pose.m[11] );
  ROS_INFO_STREAM(" " << item.pose.m[12] << " " << item.pose.m[13] << " " << item.pose.m[14] << " " << item.pose.m[15] );
  ROS_INFO_STREAM("job_no:" << jn << " single_gp: " << item.single_gp );
#ifdef DUMP_UNILOG
  if(exportlog){
    exportlog << endl << "RESULT[" << k << "]:" << endl;
    switch( res_recog_module[idx] ){
      case 1: exportlog << " method: sekiyaseg + LineMOD" << endl; break;
      case 2: exportlog << " method: sekiyaseg + LineMOD + plane" << endl; break;
      case 3: exportlog << " method: YOLO(futeikei)" << endl; break;
      case 4: exportlog << " method: YOLO + LineMOD" << endl; break;
      case 5: exportlog << " method: YOLO + distlearn" << endl; break;
      case 6: exportlog << " method: YOLO + plane" << endl; break;
      case 7: exportlog << " method: AKAZE" << endl; break;
      case 8: exportlog << " method: AKAZE + plane" << endl; break;
      case 9: exportlog << " method: sekiyaseg + plane" << endl; break;
      case 10: exportlog << " method: tottori" << endl; break;
    }
    exportlog << " item.cad_id: " << item.cad_id  << endl;
    exportlog << " item.category_id: " << item.category_id
              << "(" << table_name[item.category_id]  << ")" << endl;
    //exportlog << " item.scale.x: " << item.scale.x << ", y: " << item.scale.y << ", z: " << item.scale.z << endl;
    exportlog << " item.probability: " << item.probability << endl;
    exportlog << " item.pose.m: " << endl;
    exportlog << "  " << item.pose.m[0] << " " << item.pose.m[1] << " " << item.pose.m[2] << " " << item.pose.m[3] << endl;
    exportlog << "  " << item.pose.m[4] << " " << item.pose.m[5] << " " << item.pose.m[6] << " " << item.pose.m[7] << endl;
    exportlog << "  " << item.pose.m[8] << " " << item.pose.m[9] << " " << item.pose.m[10] << " " << item.pose.m[11] << endl;
    exportlog << "  " << item.pose.m[12] << " " << item.pose.m[13] << " " << item.pose.m[14] << " " << item.pose.m[15] << endl;
    exportlog << " single_gp: " << item.single_gp << endl;
  }
#endif

  for( int j=sy; j<ey; j++ ){
    for( int i=sx; i<ex; i++ ){
      int m = 1;
      if( msk_w>0 && msk_h>0 ){
        m = (int) (mMat.data[ (j-sy)*mMat.step + (i-sx)*mMat.elemSize() ]);
      }
      if( m>0 ){
        for (uint32_t channel = 0; channel < 3; channel++){
          memset( &data->obstacle_points.data[ i*data->obstacle_points.point_step
                                               +j*data->obstacle_points.row_step
                                               +data->obstacle_points.fields[channel].offset],
                                               0, sizeof(float));
        } //channel
      }
    }
  }

#ifdef DUMP_UNILOG
  log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  if(1){
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    std::string debug_path = src_root_path + "/data/debug/Result_";
    std::stringstream ss;
    ss << debug_path << "jobNo_" << notice->recog_target.job_no << "_camID_" << notice->recog_target.data[0].cam_id;
    std::string log_name = ss.str()+"_log.txt";
    std::ofstream log_file(log_name.c_str(), std::ios::out);
    
    log_file << "fname,cad,ctg,pro,mat" << std::endl;
    for( int j=0; j<data->items.size(); j++ ){
      log_file << log_name.c_str() << "," << data->items[j].cad_id << "," << data->items[j].category_id << "," << data->items[j].probability << ",";
      for( int i=0; i<16; i++ ){ log_file << data->items[j].pose.m[i] << ","; }
    log_file << std::endl;
    }
    log_file.close();
  }
#endif

  } // k

  unification_pub_.publish(data);

end:
  resetproc();
  ROS_INFO_STREAM("[END] job_no:" << jn << " integrate");
  return 1;
}

void robot_vision_unification::recognizedItemCallback(const T2_robot_vision::RecognizedItemConstPtr &notice)
{
  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("[START] job_no:" << jn << " recognizedItemCallback [" << cnt_msg << "]");

  cnt_msg++;
  storedata( notice );

  ROS_INFO_STREAM("[END] job_no:" << jn << " recognizedItemCallback [" << cnt_msg-1 << "]");

  integrate( notice );
}

void robot_vision_unification::resultYOLOv2Callback(const T2_robot_vision::RecognizedItemConstPtr &notice)
{
  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("[START] job_no:" << jn << " resultYOLOv2Callback [" << cnt_msg << "]");

  cnt_msg++;
  storedata( notice );

  ROS_INFO_STREAM("[END] job_no:" << jn << " resultYOLOv2Callback [" << cnt_msg-1 << "]");

  integrate( notice );
}

void robot_vision_unification::resultAKAZECallback(const T2_robot_vision::RecognizedItemConstPtr &notice)
{
  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("[START] job_no:" << jn << " resultAKAZECallback [" << cnt_msg << "]");

  cnt_msg++;
  storedata( notice );

  ROS_INFO_STREAM("[END] job_no:" << jn << " resultAKAZECallback [" << cnt_msg-1 << "]");

  integrate( notice );
}

void robot_vision_unification::resultLineMODCallback(const T2_robot_vision::RecognizedItemConstPtr &notice)
{
  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("[START] job_no:" << jn << " resultLineMODCallback [" << cnt_msg << "]");

  cnt_msg++;
  storedata( notice );

  ROS_INFO_STREAM("[END] job_no:" << jn << " resultLineMODCallback [" << cnt_msg-1 << "]");

  integrate( notice );
}

} // namespace T2_robot_vision
