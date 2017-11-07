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

#ifndef T2_PLANNING_SCENE_UPDATER_T2_PLANNING_SCENE_UPDATER_H
#define T2_PLANNING_SCENE_UPDATER_T2_PLANNING_SCENE_UPDATER_H

#include <t2_planning_scene_updater/capability_names.h>

#include <sys/stat.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Empty.h>

#include <moveit/move_group/capability_names.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <t2_msgs/AddCollisionObject.h>
#include <t2_msgs/RemoveCollisionObject.h>
#include <t2_msgs/AttachObject.h>
#include <t2_msgs/DetachObject.h>
#include <t2_msgs/SetACM.h>
#include <t2_msgs/ClearAttachedItem.h>

#include <t2_msgs/AddItemsToPlanningScene.h>
#include <t2_msgs/SetPickItemToPlanningScene.h>
#include <t2_msgs/SetContainerObjectToPlanningScene.h>
#include <t2_msgs/GetAttachedItemFromPlanningScene.h>

#include <catch_robot_msgs/conversions.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <regex>

namespace t2_planning_scene_updater
{
const std::string POINT_CLOUD_TOPIC_NAME = "/rgbd_camera/depth/points";
const std::string CONTAINER_INFO_PATH = "/t2_database/container_info";
const std::string BOX_SETTINGS_PATH = "/t2_database/box_settings";

class PlanningSceneUpdater
{
public:
  PlanningSceneUpdater();
  ~PlanningSceneUpdater();
  void init();
  bool publishSceneFromTextFile(const std::string& scene_file);

  bool addCollisionObject(const std::string& id, const std::string& frame_id, const std::string& mesh_file,
                          const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& scale,
                          const std_msgs::ColorRGBA& color);

  bool getCollisionObject(const std::string& id, moveit_msgs::CollisionObject& collision_object);

  bool removeCollisionObject(const std::string& id);
  bool attachObject(const std::string& id, const std::string& link_name,
                    const std::vector<std::string>& allowed_touch_links);
  bool attachObject(const std::string& id, const std::string& link_name,
                    const std::vector<std::string>& allowed_touch_links, const geometry_msgs::Pose& grasp_pose);
  bool detachObject(const std::string& id);
  bool setACM(const std::string& id1, const std::string& id2, const bool& allowed);
  // bool setPCL(const sensor_msgs::PointCloud2& pcl);
  // bool clearPCL();

  bool getStlFileNameFromCadId(const int& cad_id, std::string& file_name);

  bool getContainerPosition(const std::string& name, geometry_msgs::Point& position,
                            geometry_msgs::Point& inside_position);

  bool addBoxObject();
  bool addToteObject();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  planning_scene::PlanningScenePtr ps_;

  // Service Server
  ros::ServiceServer add_collision_object_server_;
  ros::ServiceServer remove_collision_object_server_;
  ros::ServiceServer attach_object_server_;
  ros::ServiceServer detach_object_server_;
  ros::ServiceServer set_acm_server_;
  ros::ServiceServer add_items_;
  ros::ServiceServer set_pick_item_;
  ros::ServiceServer set_container_object_server_;
  ros::ServiceServer clear_attached_item_server_;
  ros::ServiceServer get_attached_item_info_server_;

  // Service Client
  ros::ServiceClient get_planning_scene_client_;
  ros::ServiceClient apply_planning_scene_client_;
  ros::ServiceClient clear_octomap_client_;

  // Publisher
  ros::Publisher pcl_publisher_;

private:
  // Service callback
  bool addCollisionObjectCallback(t2_msgs::AddCollisionObject::Request& req,
                                  t2_msgs::AddCollisionObject::Response& res);
  bool removeCollisionObjectCallback(t2_msgs::RemoveCollisionObject::Request& req,
                                     t2_msgs::RemoveCollisionObject::Response& res);
  bool attachObjectCallback(t2_msgs::AttachObject::Request& req, t2_msgs::AttachObject::Response& res);
  bool detachObjectCallback(t2_msgs::DetachObject::Request& req, t2_msgs::DetachObject::Response& res);
  bool setACMCallback(t2_msgs::SetACM::Request& req, t2_msgs::SetACM::Response& res);

  bool addItemsCallback(t2_msgs::AddItemsToPlanningScene::Request& req,
                        t2_msgs::AddItemsToPlanningScene::Response& res);
  bool setPickItemCallback(t2_msgs::SetPickItemToPlanningScene::Request& req,
                           t2_msgs::SetPickItemToPlanningScene::Response& res);

  bool getAttachedItemFromPlanningSceneCallback(t2_msgs::GetAttachedItemFromPlanningScene::Request& req,
                                                t2_msgs::GetAttachedItemFromPlanningScene::Response& res);

  bool getContainerItems(const int& place_id, std::vector<std::string>& id);

  bool clearItems(const int& place_id);

  bool clearAttachedItemCallback(t2_msgs::ClearAttachedItem::Request& req, t2_msgs::ClearAttachedItem::Response& res);

  bool setContainerObjectCallback(t2_msgs::SetContainerObjectToPlanningScene::Request& req,
                                  t2_msgs::SetContainerObjectToPlanningScene::Response& res);

  bool clearContainers();

  std::string getItemName(const int& place_id, const int& cad_id, const int& job_no, const int& recog_id);

  std::string package_path_;
  std::string config_path_;
  std::string data_path_;
  std::string item_mesh_path_;
  std::string container_mesh_path_;
  std::vector<std::string> pinch_approach_acm_link_;

  bool approach_gp_storage_other_items_collision_check_;
  bool attach_item_pose_gp_;

  std::map<std::string, t2_msgs::ItemData> items_info_;
  t2_msgs::ItemData attached_item_info_;
  geometry_msgs::Pose attached_item_pose_;
};  // class

}  // namespace t2_planning_scene_updater

#endif  // T2_PLANNING_SCENE_UPDATER_T2_PLANNING_SCENE_UPDATER_H
