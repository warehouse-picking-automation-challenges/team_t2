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
#include "robot_vision_occupancy.h"

// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_occupancy, nodelet::Nodelet);

typedef struct MATRIX {
   double _11, _12, _13;
   double _21, _22, _23;
   double _31, _32, _33;
}MATRIX;

namespace T2_robot_vision
{

robot_vision_occupancy::robot_vision_occupancy()
{
}

/*
 * Nodelet Destructor.
 */
robot_vision_occupancy::~robot_vision_occupancy()
{
}

/*
 * Initialize the nodelet.
 */
void robot_vision_occupancy::onInit()
{
  NODELET_INFO("Occupancy Init start");
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();
  occupancy_target_sub_ = nh.subscribe("occupancy_target", 10, &robot_vision_occupancy::occupancyTargetCallback, this);
  occupancy_res_pub_ = nh.advertise<t2_msgs::OccupancyRes>("occupancy_res", 10);
  pnh.param("resolution", resolution, 0.03);
  pnh.param("interval", interval, 0.03);
  pnh.param("remove_area", remove_area, 0.05);
  pnh.param("save_bag", save_bag, 0);
  for (int i = 0; i <= 20; ++i)
  {
    std::ostringstream oss;
    oss << i;
    std::string calib_name = "/calibration" + oss.str();
    std::string ttw_name = calib_name + "/transform_to_world";
    std::string range_name =  calib_name + "/fill_up_range_z";
    if (nh.hasParam(ttw_name))
    {
      std::vector<double> vec;
      nh.getParam(ttw_name, vec);
      cv::Mat mat(4, 4, CV_64FC1, vec.data());
      matBin[i] = mat.clone();
    }else{
      NODELET_ERROR_STREAM("No parameter: " << ttw_name);
    }
    if(nh.hasParam(range_name)){
      double range;
      nh.getParam(range_name, range);
      fill_range_bin[i] = range;
      NODELET_INFO_STREAM(range_name << ": " << range);
    }else{
      NODELET_ERROR_STREAM("No parameter: " << range_name);
    }  
  }
  NODELET_INFO_STREAM("resolution      = " << resolution);
  NODELET_INFO_STREAM("interval        = " << interval);
  NODELET_INFO_STREAM("save_bag        = " << save_bag);
  NODELET_INFO("Occupancy Init end");
}

bool existDir(char* dirname){ 
	struct stat st;
	if(stat(dirname,&st) != 0){
		return false;
	}else{
		mode_t m = st.st_mode;
		if(S_ISDIR(m)){
			return true;
		}else{
			return false; // file
		}
	}
}

void print_query_info(octomap::point3d query, octomap::OcTreeNode* node) { 
  if (node != NULL) { 
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl; 
  }else{ 
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
  }
} 

int robot_vision_occupancy::fillUpDeadAngle(pcl::PointCloud<pcl::PointXYZ> &in_cloud,
                    						pcl::PointCloud<pcl::PointXYZ> &out_cloud,
                    						double fill_up_range_z)
{
  float fill_x, fill_y, fill_z=0;
  float ratio;
  double threshold = 0.30;
  pcl::PointCloud<pcl::PointXYZ> src_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  std::cout << "in_cloud.size()  = " << in_cloud.points.size() << std::endl;
  for(int i_point=0; i_point<in_cloud.points.size(); i_point++){
    if(in_cloud.points[i_point].z < 1e-10)continue;
    src_cloud.push_back(in_cloud.points[i_point]);
  }
  std::cout << "0remove_cloud.size()  = " << src_cloud.points.size() << std::endl;
  sor.setInputCloud(src_cloud.makeShared());
  sor.setMeanK(100);
  sor.setStddevMulThresh(remove_area);
  sor.setNegative(false);
  sor.filter(*dst_cloud);
  copyPointCloud(*dst_cloud,in_cloud);

  for(int i_point=0; i_point<in_cloud.points.size(); i_point++){
    ratio = interval / in_cloud.points[i_point].z;
    out_cloud.push_back(in_cloud.points[i_point]);
    
    double delta_x, delta_y, delta_z;
    delta_x = in_cloud.points[i_point].x * ratio;
    delta_y = in_cloud.points[i_point].y * ratio;
    delta_z = in_cloud.points[i_point].z * ratio;
    fill_x = in_cloud.points[i_point].x;
    fill_y = in_cloud.points[i_point].y;
    fill_z = in_cloud.points[i_point].z;
    
    while(fill_z<fill_up_range_z){
      fill_x += delta_x;
      fill_y += delta_y;
      fill_z += delta_z;
      out_cloud.push_back(pcl::PointXYZ(fill_x, fill_y, fill_z));
    }
  }
  return 0;
}


void robot_vision_occupancy::occupancyTargetCallback(const T2_robot_vision::OccupancyTargetConstPtr &notice)
{
  NODELET_INFO("Occupancy Target Callback start");
  
  t2_msgs::OccupancyResPtr data(new t2_msgs::OccupancyRes);
  data->seq_no = notice->seq_no;
  data->job_no = notice->job_no;
  octomap_msgs::OctomapWithPose oct_pos;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(notice->points[0], cloud);
  
  int base_cam_id = notice->base_cam_id;
  std::map<uint32_t, double>::iterator iter_range = fill_range_bin.find(base_cam_id);
  double fill_up_range_z;
  if (iter_range == fill_range_bin.end())
  {
    NODELET_ERROR_STREAM("Not found BIN Caera fill_up_range_z. cam_id:" << base_cam_id);
  }else
  {
    if(interval > 1e-10){
      fill_up_range_z = iter_range->second;
      NODELET_INFO_STREAM("fillUpDeadAngle start : fill_up_range_z = " << fill_up_range_z);
      pcl::PointCloud<pcl::PointXYZ> fill_cloud;
      fillUpDeadAngle(cloud, fill_cloud, fill_up_range_z);
      cloud.swap(fill_cloud);
      ROS_INFO("fillUpDeadAngle end");
    }else{
      ROS_INFO("intervel=0. Don't fill up");
    }
  }
  
  ROS_INFO("OcTree create start");
  octomap::OcTree tree(resolution);
  
  float x,y,z;
  for(int i=0; i<cloud.points.size(); i++){
    x = cloud.points[i].x;
    y = cloud.points[i].y;
    z = cloud.points[i].z;
    if(fabs(x)<1e-10 && fabs(y)<1e-10 && fabs(z)<1e-10)continue;
    octomap::point3d endpoint(x, y, z);
    tree.updateNode(endpoint, true);
  }
  ROS_INFO("OcTree create end");
 
  // pose
  std::map<uint32_t, cv::Mat>::iterator it = matBin.find(base_cam_id);
  geometry_msgs::Pose pose_msg;
  if (it == matBin.end())
  {
    NODELET_ERROR_STREAM("Not found BIN Camera pose matrix. cam_id:" << base_cam_id);
  }else
  {
    catch_robot_msgs::Matrix4x4 mat_msg;
    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        mat_msg.m[j*4+i] = it->second.at<double>(j,i);
      }
    }
    pose_msg = catch_robot_msgs::matrix4x4ToPoseMsg(mat_msg);
  }

  oct_pos.header = notice->points[0].header;
  oct_pos.origin = pose_msg;

  oct_pos.octomap.header = notice->points[0].header;
  oct_pos.octomap.header.frame_id = "/map";
  oct_pos.octomap.header.stamp = ros::Time::now();
  oct_pos.octomap.binary = true;
  oct_pos.octomap.id = tree.getTreeType();
  oct_pos.octomap.resolution = resolution;
  
  // OcTree serialization
  ROS_INFO("OcTree serialization start");
  if(oct_pos.octomap.binary){
    octomap_msgs::binaryMapToMsg(tree, oct_pos.octomap);
  }else{
    octomap_msgs::fullMapToMsg(tree, oct_pos.octomap);
  }
  ROS_INFO("OcTree serialization end");
  

  data->octomap_pose = oct_pos;
  
  if(save_bag==1){
    rosbag::Bag bag;
    std::string src_root_path = ros::package::getPath("T2_robot_vision");
    char tmpfn[MAX_FILENAME];
    sprintf(tmpfn,"/data/oct_debug/OctomapWithPose_jobNo%d_camId%d.bag", notice->job_no, notice->base_cam_id);
    std::string tmpfn2 = tmpfn;
    char tmppath[MAX_FILENAME];
    sprintf(tmppath, "%s/data/oct_debug", src_root_path.c_str());
    if(existDir(tmppath)){
      std::string bag_fname = src_root_path + tmpfn2;
      bag.open(bag_fname, rosbag::bagmode::Write);
      bag.write("octomap_pose", ros::Time::now(), oct_pos);
      bag.close();
      NODELET_INFO("seve OctomapWithPose bag file : %s", bag_fname.c_str()); 
    }else{
      NODELET_INFO_STREAM("not exist dir: " << tmppath);
      NODELET_INFO("can not save OctomapWithPose bag file");
    }
  }
  
  ROS_INFO("wait subscriber ...");
  ros::Rate rate(10); /* 10 Hz */
  while(ros::ok() && occupancy_res_pub_.getNumSubscribers() == 0) {
    rate.sleep();
  } 
  
  occupancy_res_pub_.publish(data);
  NODELET_INFO("Occupancy Target Callback end");
}

} // namespace T2_robot_vision
