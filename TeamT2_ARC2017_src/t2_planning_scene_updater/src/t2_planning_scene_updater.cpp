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

#include <t2_planning_scene_updater/t2_planning_scene_updater.h>

namespace t2_planning_scene_updater
{
PlanningSceneUpdater::PlanningSceneUpdater() : nh_(), private_nh_("~")
{
}

void PlanningSceneUpdater::init()
{
  ROS_INFO("PlanningSceneUpdater init.");

  // Load Motion Planner configs from parameter server
  ros::NodeHandle private_config_nh("~/planning_scene_updater_configs");

  package_path_ = ros::package::getPath("t2_planning_scene_updater");
  config_path_ = package_path_ + "/config/";
  data_path_ = package_path_ + "/data/";

  robot_model_loader::RobotModelLoader::Options opt;
  opt.robot_description_ = "robot_description";
  opt.load_kinematics_solvers_ = false;
  robot_model_loader::RobotModelLoaderConstPtr rml(new robot_model_loader::RobotModelLoader(opt));
  ps_.reset(new planning_scene::PlanningScene(rml->getModel()));

  get_planning_scene_client_ =
      nh_.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
  apply_planning_scene_client_ =
      nh_.serviceClient<moveit_msgs::ApplyPlanningScene>(move_group::APPLY_PLANNING_SCENE_SERVICE_NAME);

  clear_octomap_client_ = nh_.serviceClient<std_srvs::Empty::Request>(move_group::CLEAR_OCTOMAP_SERVICE_NAME);

  apply_planning_scene_client_.waitForExistence();

  if (private_nh_.hasParam("scene"))
  {
    std::string scene_file;
    private_nh_.getParam("scene", scene_file);
    publishSceneFromTextFile(scene_file);
  }

  if (!private_nh_.getParam("item_mesh_path", item_mesh_path_))
  {
    ROS_ERROR("Failed to get item_mesh_path from parameter server.");
  }

  if (!private_nh_.getParam("container_mesh_path", container_mesh_path_))
  {
    ROS_ERROR("Failed to get container_mesh_path from parameter server.");
  }

  if (!private_nh_.getParam("pinch_approach_acm_link", pinch_approach_acm_link_))
  {
    ROS_ERROR("Failed to get pinch_approach_acm_link from parameter server.");
  }

  // cartesian_path_approach
  private_config_nh.param<bool>("approach_gp_storage_other_items_collision_check",
                                approach_gp_storage_other_items_collision_check_, false);

  // attach_item_pose_gp
  private_config_nh.param<bool>("attach_item_pose_gp", attach_item_pose_gp_, true);

  // publisher
  pcl_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC_NAME, 1);
  // Service
  add_collision_object_server_ = private_nh_.advertiseService(ADD_COLLISION_OBJECT_SERVICE_NAME,
                                                              &PlanningSceneUpdater::addCollisionObjectCallback, this);
  remove_collision_object_server_ = private_nh_.advertiseService(
      REMOVE_COLLISION_OBJECT_SERVICE_NAME, &PlanningSceneUpdater::removeCollisionObjectCallback, this);
  attach_object_server_ =
      private_nh_.advertiseService(ATTACH_OBJECT_SERVICE_NAME, &PlanningSceneUpdater::attachObjectCallback, this);
  detach_object_server_ =
      private_nh_.advertiseService(DETACH_OBJECT_SERVICE_NAME, &PlanningSceneUpdater::detachObjectCallback, this);
  set_acm_server_ = private_nh_.advertiseService(SET_ACM_SERVICE_NAME, &PlanningSceneUpdater::setACMCallback, this);

  add_items_ = private_nh_.advertiseService(ADD_ITEMS_SERVICE_NAME, &PlanningSceneUpdater::addItemsCallback, this);
  set_pick_item_ =
      private_nh_.advertiseService(SET_PICK_ITEM_SERVICE_NAME, &PlanningSceneUpdater::setPickItemCallback, this);

  set_container_object_server_ = private_nh_.advertiseService(SET_CONTAINER_OBJECT_SERVICE_NAME,
                                                              &PlanningSceneUpdater::setContainerObjectCallback, this);

  clear_attached_item_server_ = private_nh_.advertiseService(CLEAR_ATTACHED_ITEM_SERVICE_NAME,
                                                             &PlanningSceneUpdater::clearAttachedItemCallback, this);

  get_attached_item_info_server_ = private_nh_.advertiseService(
      GET_ATTACHED_ITEM_SERVICE_NAME, &PlanningSceneUpdater::getAttachedItemFromPlanningSceneCallback, this);
}

bool PlanningSceneUpdater::publishSceneFromTextFile(const std::string& scene_file)
{
  ROS_INFO("publishSceneFromTextFile()");

  std::ifstream f(scene_file);

  if (!f.good() || f.eof())
  {
    ROS_ERROR("Unable to load scene file.");
    return false;
  }

  moveit_msgs::ApplyPlanningScene asrv;
  planning_scene::PlanningScenePtr ps_diff(ps_->diff());
  ps_diff->loadGeometryFromStream(f);
  f.close();
  ps_diff->getPlanningSceneDiffMsg(asrv.request.scene);
  asrv.request.scene.is_diff = true;
  apply_planning_scene_client_.call(asrv);

  return true;
}

bool PlanningSceneUpdater::addCollisionObject(const std::string& id, const std::string& frame_id,
                                              const std::string& mesh_file, const geometry_msgs::Pose& pose,
                                              const geometry_msgs::Vector3& scale, const std_msgs::ColorRGBA& color)
{
  ROS_INFO("addCollisionObject id = %s, frame_id = %s, mesh_file = %s", id.c_str(), frame_id.c_str(),
           mesh_file.c_str());

  moveit_msgs::GetPlanningScene gsrv;
  gsrv.request.components.components = moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

  if (!get_planning_scene_client_.call(gsrv))
  {
    ROS_ERROR("Failed to call get_planning_scene service.");
    return false;
  }

  moveit_msgs::CollisionObject co;
  co.header.frame_id = frame_id.empty() ? "world" : frame_id;
  co.id = id;
  co.mesh_poses.push_back(pose);
  shapes::ShapeMsg shape_msg;

  Eigen::Vector3d eigen_scale(1.0, 1.0, 1.0);
  if (scale.x != 0 && scale.y != 0 && scale.z != 0)
  {
    eigen_scale = Eigen::Vector3d(scale.x, scale.y, scale.z);
  }

  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_file, eigen_scale);

  if (mesh == NULL)
  {
    ROS_ERROR("Failed to createMeshFromResource() %s", mesh_file.c_str());
    return false;
  }

  shapes::constructMsgFromShape(mesh, shape_msg);
  co.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
  co.operation = moveit_msgs::CollisionObject::ADD;
  moveit_msgs::ObjectColor oc;
  oc.id = id;
  oc.color = color;
  moveit_msgs::ApplyPlanningScene asrv;
  asrv.request.scene.world.collision_objects.push_back(co);
  asrv.request.scene.object_colors = gsrv.response.scene.object_colors;
  asrv.request.scene.object_colors.push_back(oc);
  asrv.request.scene.is_diff = true;

  if (!apply_planning_scene_client_.call(asrv))
  {
    ROS_ERROR("Failed to call apply_planning_scene service.");
    return false;
  }

  return true;
}

bool PlanningSceneUpdater::getCollisionObject(const std::string& id, moveit_msgs::CollisionObject& collision_object)
{
  moveit_msgs::GetPlanningScene gsrv;

  gsrv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
  if (!get_planning_scene_client_.call(gsrv))
  {
    ROS_ERROR("Failed to call get_planning_scene service.");
    return false;
  }

  moveit_msgs::CollisionObject target_object;

  std::vector<moveit_msgs::CollisionObject>& co = gsrv.response.scene.world.collision_objects;

  for (std::size_t i = 0; i < co.size(); i++)
  {
    if (co[i].id == id)
    {
      collision_object = co[i];
      return true;
    }
  }

  return false;
}

bool PlanningSceneUpdater::removeCollisionObject(const std::string& id)
{
  ROS_INFO("removeCollisionObject id = %s", id.c_str());

  moveit_msgs::CollisionObject co;
  co.id = id;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  moveit_msgs::ApplyPlanningScene asrv;
  asrv.request.scene.world.collision_objects.push_back(co);
  asrv.request.scene.is_diff = true;
  if (!apply_planning_scene_client_.call(asrv))
  {
    ROS_ERROR("Failed to call apply_planning_scene service.");
    return false;
  }

  return true;
}

bool PlanningSceneUpdater::attachObject(const std::string& id, const std::string& link_name,
                                        const std::vector<std::string>& allowed_touch_links)
{
  ROS_INFO("attachObject id = %s, link_name = %s", id.c_str(), link_name.c_str());

  moveit_msgs::AttachedCollisionObject aco;
  aco.object.id = id;
  aco.link_name = link_name;

  if (allowed_touch_links.empty())
  {
    aco.touch_links.push_back(aco.link_name);
  }
  else
  {
    aco.touch_links = allowed_touch_links;
  }

  aco.object.operation = moveit_msgs::CollisionObject::ADD;

  moveit_msgs::ApplyPlanningScene asrv;
  asrv.request.scene.robot_state.attached_collision_objects.push_back(aco);
  asrv.request.scene.robot_state.is_diff = true;
  asrv.request.scene.is_diff = true;
  if (!apply_planning_scene_client_.call(asrv))
  {
    ROS_ERROR("Failed to call apply_planning_scene service.");
    return false;
  }

  return true;
}

bool PlanningSceneUpdater::attachObject(const std::string& id, const std::string& link_name,
                                        const std::vector<std::string>& allowed_touch_links,
                                        const geometry_msgs::Pose& grasp_pose)
{
  ROS_INFO("attachObject id = %s, link_name = %s", id.c_str(), link_name.c_str());

  moveit_msgs::CollisionObject target_object;

  if (!getCollisionObject(id, target_object))
  {
    ROS_ERROR("Failed to call getCollisionObject");
    return false;
  }

  // remove collision object
  moveit_msgs::ApplyPlanningScene asrv;
  moveit_msgs::CollisionObject remove_object;
  remove_object.header = target_object.header;
  remove_object.id = target_object.id;
  remove_object.operation = moveit_msgs::CollisionObject::REMOVE;
  asrv.request.scene.world.collision_objects.push_back(remove_object);

  // add attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object = target_object;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;
  aco.object.header.frame_id = link_name;
  aco.link_name = link_name;

  if (allowed_touch_links.empty())
  {
    aco.touch_links.push_back(aco.link_name);
  }
  else
  {
    aco.touch_links = allowed_touch_links;
  }

  if (aco.object.mesh_poses.size() != 1)
  {
    ROS_ERROR("Invalid mesh_poses size.");
    return false;
  }

  Eigen::Affine3d eigen_g_center_pos, eigen_grasp_pose;
  geometry_msgs::Pose attach_pose;
  tf::poseMsgToEigen(aco.object.mesh_poses[0], eigen_g_center_pos);
  tf::poseMsgToEigen(grasp_pose, eigen_grasp_pose);
  tf::poseEigenToMsg(eigen_grasp_pose.inverse() * eigen_g_center_pos, attach_pose);

  attached_item_info_ = items_info_[id];
  attached_item_pose_ = attach_pose;

  aco.object.mesh_poses.clear();
  aco.object.mesh_poses.push_back(attach_pose);

  asrv.request.scene.robot_state.attached_collision_objects.push_back(aco);
  asrv.request.scene.robot_state.is_diff = true;
  asrv.request.scene.is_diff = true;
  if (!apply_planning_scene_client_.call(asrv))
  {
    ROS_ERROR("Failed to call apply_planning_scene service.");
    return false;
  }

  return true;
}

bool PlanningSceneUpdater::detachObject(const std::string& id)
{
  ROS_INFO("detachObject id = %s", id.c_str());

  moveit_msgs::AttachedCollisionObject aco;
  aco.object.id = id;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

  moveit_msgs::ApplyPlanningScene asrv;
  asrv.request.scene.robot_state.attached_collision_objects.push_back(aco);
  asrv.request.scene.robot_state.is_diff = true;
  asrv.request.scene.is_diff = true;
  if (!apply_planning_scene_client_.call(asrv))
  {
    ROS_ERROR("Failed to call apply_planning_scene service.");
    return false;
  }

  return true;
}

bool PlanningSceneUpdater::setACM(const std::string& id1, const std::string& id2, const bool& allowed)
{
  ROS_INFO("setACM id1 = %s, id2 = %s, allowed = %s", id1.c_str(), id2.c_str(), (allowed ? "true" : "false"));

  moveit_msgs::GetPlanningScene gsrv;
  moveit_msgs::ApplyPlanningScene asrv;
  collision_detection::AllowedCollisionMatrixPtr acm;

  gsrv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  if (!get_planning_scene_client_.call(gsrv))
  {
    ROS_ERROR("Failed to call get_planning_scene service.");
    return false;
  }

  acm.reset(new collision_detection::AllowedCollisionMatrix(gsrv.response.scene.allowed_collision_matrix));

  if (id2.empty())
  {
    acm->setEntry(id1, allowed);
  }
  else
  {
    acm->setEntry(id1, id2, allowed);
  }

  acm->getMessage(asrv.request.scene.allowed_collision_matrix);
  asrv.request.scene.is_diff = true;

  if (!apply_planning_scene_client_.call(asrv))
  {
    ROS_ERROR("Failed to call apply_planning_scene service.");
    return false;
  }
  return true;
}

bool PlanningSceneUpdater::clearAttachedItemCallback(t2_msgs::ClearAttachedItem::Request& req,
                                                     t2_msgs::ClearAttachedItem::Response& res)
{
  ROS_WARN("clearAttachedItemCallback()");

  moveit_msgs::GetPlanningScene gsrv;

  gsrv.request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
  if (!get_planning_scene_client_.call(gsrv))
  {
    ROS_ERROR("Failed to call get_planning_scene service.");
    return false;
  }

  std::vector<moveit_msgs::AttachedCollisionObject>& aco = gsrv.response.scene.robot_state.attached_collision_objects;

  for (std::size_t i = 0; i < aco.size(); i++)
  {
    std::string& id = aco[i].object.id;
    detachObject(id);
    removeCollisionObject(id);
  }

  return true;
}
/*
bool PlanningSceneUpdater::setPCL(const sensor_msgs::PointCloud2& pcl)
{
  pcl_publisher_.publish(pcl);
  return true;
}

bool PlanningSceneUpdater::clearPCL()
{
  std_srvs::Empty srv;
  clear_octomap_client_.call(srv);
  return true;
}
*/

bool PlanningSceneUpdater::addCollisionObjectCallback(t2_msgs::AddCollisionObject::Request& req,
                                                      t2_msgs::AddCollisionObject::Response& res)
{
  if (addCollisionObject(req.id, req.frame_id, req.mesh_file, req.pose, req.scale, req.color))
  {
    res.result = res.SUCCESS;
  }
  else
  {
    ROS_ERROR("addCollisionObject failed.");
    res.result = res.FAILED;
  }
  return true;
}

bool PlanningSceneUpdater::removeCollisionObjectCallback(t2_msgs::RemoveCollisionObject::Request& req,
                                                         t2_msgs::RemoveCollisionObject::Response& res)
{
  if (removeCollisionObject(req.id))
  {
    res.result = res.SUCCESS;
  }
  else
  {
    ROS_ERROR("removeCollisionObject failed.");
    res.result = res.FAILED;
  }
  return true;
}

bool PlanningSceneUpdater::attachObjectCallback(t2_msgs::AttachObject::Request& req,
                                                t2_msgs::AttachObject::Response& res)
{
  if (attachObject(req.id, req.link_name, req.allowed_touch_links))
  {
    res.result = res.SUCCESS;
  }
  else
  {
    ROS_ERROR("attachObject failed.");
    res.result = res.FAILED;
  }
  return true;
}

bool PlanningSceneUpdater::detachObjectCallback(t2_msgs::DetachObject::Request& req,
                                                t2_msgs::DetachObject::Response& res)
{
  if (detachObject(req.id))
  {
    res.result = res.SUCCESS;
  }
  else
  {
    ROS_ERROR("detachObject failed.");
    res.result = res.FAILED;
  }
  return true;
}

bool PlanningSceneUpdater::setACMCallback(t2_msgs::SetACM::Request& req, t2_msgs::SetACM::Response& res)
{
  if (setACM(req.id1, req.id2, req.allowed))
  {
    res.result = res.SUCCESS;
  }
  else
  {
    ROS_ERROR("setACM failed.");
    res.result = res.FAILED;
  }
  return true;
}

bool PlanningSceneUpdater::addItemsCallback(t2_msgs::AddItemsToPlanningScene::Request& req,
                                            t2_msgs::AddItemsToPlanningScene::Response& res)
{
  ROS_INFO("addItemsCallback");

  clearItems(req.place_id);

  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;

  for (std::size_t i = 0; i < req.items.size(); i++)
  {
    const t2_msgs::ItemData& item = req.items[i];
    const std::string item_name = getItemName(req.place_id, item.cad_id, req.job_no, i);
    std::string file_name;

    if (!getStlFileNameFromCadId(item.cad_id, file_name))
    {
      ROS_WARN("Ignored unknown cad_id = %d", item.cad_id);
      continue;
    }

    if (!addCollisionObject(item_name, "", file_name, item.pose, item.scale, color))
    {
      ROS_ERROR("Failed to add %s", item_name.c_str());
    }

    items_info_[item_name] = item;
  }

  res.result = res.SUCCESS;
  return true;
}

bool PlanningSceneUpdater::setPickItemCallback(t2_msgs::SetPickItemToPlanningScene::Request& req,
                                               t2_msgs::SetPickItemToPlanningScene::Response& res)
{
  ROS_INFO("setPickItemCallback");

  const std::string suction_tip_frame = "suction_nozzle_tip_frame";
  const std::string pinch_tool_frame = "gripper_tool_frame";

  const std::vector<std::string> suction_links({ "rs20n_eef1", "rs20n_eef2" });
  const std::vector<std::string> pinch_links({ "gripper_upper_finger_link", "gripper_lower_finger_link" });

  const std::string item_name = getItemName(req.place_id, req.cad_id, req.job_no, req.recog_id);

  ROS_INFO("item_name = %s", item_name.c_str());

  const std::string& grasp_pattern = req.grasp_point.grasp_pattern;

  if (!(grasp_pattern == "pinch" || grasp_pattern == "suction"))
  {
    ROS_ERROR("Invalid grasp_pattern.");
    res.result = res.FAILED;
    return false;
  }

  if (req.operation == req.APPROACH)
  {
    ROS_INFO("setPickItemCallback APPROACH");

    for (std::size_t i = 0; i < suction_links.size(); i++)
    {
      if (!setACM(item_name, suction_links[i], true))
      {
        res.result = res.FAILED;
      }
    }

    if (grasp_pattern == "pinch")
    {
      for (std::size_t i = 0; i < pinch_links.size(); i++)
      {
        if (!setACM(item_name, pinch_links[i], true))
        {
          res.result = res.FAILED;
        }

        for (std::size_t j = 0; j < pinch_approach_acm_link_.size(); j++)
        {
          if (!setACM(pinch_approach_acm_link_[j], pinch_links[i], true))
          {
            res.result = res.FAILED;
          }
        }
      }
    }
  }
  if (req.operation == req.APPROACH_GP)
  {
    ROS_INFO("setPickItemCallback APPROACH_GP");

    if (!approach_gp_storage_other_items_collision_check_)
    {
      std::vector<std::string> id;
      getContainerItems(req.place_id, id);

      for (std::size_t i = 0; i < id.size(); i++)
      {
        const std::string& tmp_item_name = id[i];

        for (std::size_t i = 0; i < suction_links.size(); i++)
        {
          if (!setACM(tmp_item_name, suction_links[i], true))
          {
            res.result = res.FAILED;
          }
        }

        if (grasp_pattern == "pinch")
        {
          for (std::size_t i = 0; i < pinch_links.size(); i++)
          {
            if (!setACM(tmp_item_name, pinch_links[i], true))
            {
              res.result = res.FAILED;
            }

            for (std::size_t j = 0; j < pinch_approach_acm_link_.size(); j++)
            {
              if (!setACM(pinch_approach_acm_link_[j], pinch_links[i], true))
              {
                res.result = res.FAILED;
              }
            }
          }
        }
      }
    }
  }
  else if (req.operation == req.ATTACH)
  {
    ROS_INFO("setPickItemCallback ATTACH");

    std::string link_name;

    if (grasp_pattern == "suction")
    {
      link_name = suction_tip_frame;
    }
    else if (grasp_pattern == "pinch")
    {
      link_name = pinch_tool_frame;
    }

    if (attach_item_pose_gp_)
    {
      if (!attachObject(item_name, link_name, std::vector<std::string>(), req.grasp_point.grasp_point_item))
      {
        res.result = res.FAILED;
      }
    }
    else
    {
      if (!attachObject(item_name, link_name, std::vector<std::string>()))
      {
        res.result = res.FAILED;
      }
    }

    if (!setACM(item_name, "", true))
    {
      res.result = res.FAILED;
    }
  }
  if (req.operation == req.FAR_MOVE_GP)
  {
    ROS_INFO("setPickItemCallback FAR_MOVE_GP");

    std::vector<std::string> id;
    getContainerItems(req.place_id, id);

    for (std::size_t i = 0; i < id.size(); i++)
    {
      const std::string& tmp_item_name = id[i];

      for (std::size_t i = 0; i < suction_links.size(); i++)
      {
        if (!setACM(tmp_item_name, suction_links[i], true))
        {
          res.result = res.FAILED;
        }
      }

      if (grasp_pattern == "pinch")
      {
        for (std::size_t i = 0; i < pinch_links.size(); i++)
        {
          if (!setACM(tmp_item_name, pinch_links[i], true))
          {
            res.result = res.FAILED;
          }

          for (std::size_t j = 0; j < pinch_approach_acm_link_.size(); j++)
          {
            if (!setACM(pinch_approach_acm_link_[j], pinch_links[i], true))
            {
              res.result = res.FAILED;
            }
          }
        }
      }
    }
  }
  else if (req.operation == req.FAR_MOVE)
  {
    ROS_INFO("setPickItemCallback FAR_MOVE");
    if (!setACM(item_name, "", false))
    {
      res.result = res.FAILED;
    }

    for (std::size_t i = 0; i < pinch_links.size(); i++)
    {
      for (std::size_t j = 0; j < pinch_approach_acm_link_.size(); j++)
      {
        if (!setACM(pinch_approach_acm_link_[j], pinch_links[i], false))
        {
          res.result = res.FAILED;
        }
      }
    }
  }
  else if (req.operation == req.PLAN_END)
  {
    ROS_INFO("setPickItemCallback PLAN_END");
    if (!setACM(item_name, "", true))
    {
      res.result = res.FAILED;
    }

    for (std::size_t i = 0; i < pinch_links.size(); i++)
    {
      for (std::size_t j = 0; j < pinch_approach_acm_link_.size(); j++)
      {
        if (!setACM(pinch_approach_acm_link_[j], pinch_links[i], false))
        {
          res.result = res.FAILED;
        }
      }
    }

    if (!approach_gp_storage_other_items_collision_check_)
    {
      std::vector<std::string> id;
      getContainerItems(req.place_id, id);

      for (std::size_t i = 0; i < id.size(); i++)
      {
        const std::string& tmp_item_name = id[i];

        for (std::size_t i = 0; i < suction_links.size(); i++)
        {
          if (!setACM(tmp_item_name, suction_links[i], false))
          {
            res.result = res.FAILED;
          }
        }

        if (grasp_pattern == "pinch")
        {
          for (std::size_t i = 0; i < pinch_links.size(); i++)
          {
            if (!setACM(tmp_item_name, pinch_links[i], false))
            {
              res.result = res.FAILED;
            }

            for (std::size_t j = 0; j < pinch_approach_acm_link_.size(); j++)
            {
              if (!setACM(pinch_approach_acm_link_[j], pinch_links[i], false))
              {
                res.result = res.FAILED;
              }
            }
          }
        }
      }
    }
  }
  else if (req.operation == req.DETACH)
  {
    ROS_INFO("setPickItemCallback DETACH");

    if (!detachObject(item_name))
    {
      res.result = res.FAILED;
    }
  }
  else if (req.operation == req.REMOVE)
  {
    ROS_INFO("setPickItemCallback REMOVE");

    if (!removeCollisionObject(item_name))
    {
      res.result = res.FAILED;
    }
  }

  res.result = res.SUCCESS;

  return true;
}

bool PlanningSceneUpdater::setContainerObjectCallback(t2_msgs::SetContainerObjectToPlanningScene::Request& req,
                                                      t2_msgs::SetContainerObjectToPlanningScene::Response& res)
{
  clearContainers();

  switch (req.object)
  {
    case req.NONE_OBJECT:
      break;
    case req.BOX_OBJECT:
      addBoxObject();
      break;
    case req.TOTE_OBJECT:
      addToteObject();
      break;
    default:
      break;
  }
  return true;
}

bool PlanningSceneUpdater::getAttachedItemFromPlanningSceneCallback(
    t2_msgs::GetAttachedItemFromPlanningScene::Request& req, t2_msgs::GetAttachedItemFromPlanningScene::Response& res)
{
  res.item = attached_item_info_;
  res.pose = attached_item_pose_;
  getStlFileNameFromCadId(attached_item_info_.cad_id, res.mesh_resource);

  return true;
}

bool PlanningSceneUpdater::getStlFileNameFromCadId(const int& cad_id, std::string& file_name)
{
  if (!nh_.getParam("t2_database/grasp_info_list/cad" + std::to_string(cad_id) + "/mesh_file", file_name))
  {
    ROS_ERROR("Failed to get mesh_file.");
    return false;
  }

  file_name = item_mesh_path_ + file_name;

  return true;
}

bool PlanningSceneUpdater::getContainerItems(const int& place_id, std::vector<std::string>& id)
{
  ROS_INFO("clearItems place_id = %d", place_id);

  moveit_msgs::GetPlanningScene gsrv;

  gsrv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
  if (!get_planning_scene_client_.call(gsrv))
  {
    ROS_ERROR("Failed to call get_planning_scene service.");
    return false;
  }

  std::vector<moveit_msgs::CollisionObject>& co = gsrv.response.scene.world.collision_objects;

  std::regex re("item_" + std::to_string(place_id) + "_.*");
  std::smatch match;

  for (std::size_t i = 0; i < co.size(); i++)
  {
    if (std::regex_match(co[i].id, match, re))
    {
      id.push_back(co[i].id);
    }
  }

  return true;
}

bool PlanningSceneUpdater::clearItems(const int& place_id)
{
  ROS_INFO("clearItems place_id = %d", place_id);

  moveit_msgs::ApplyPlanningScene asrv;
  std::vector<std::string> id;

  if (!getContainerItems(place_id, id))
  {
    ROS_ERROR("Failed to call getContainerItems().");
    return false;
  }

  asrv.request.scene.is_diff = true;
  std::vector<moveit_msgs::CollisionObject>& co = asrv.request.scene.world.collision_objects;
  co.resize(id.size());

  for (std::size_t i = 0; i < id.size(); i++)
  {
    co[i].id = id[i];
    co[i].operation = moveit_msgs::CollisionObject::REMOVE;
  }

  if (!apply_planning_scene_client_.call(asrv))
  {
    ROS_ERROR("Failed to call apply_planning_scene service.");
    return false;
  }

  return true;
}

std::string PlanningSceneUpdater::getItemName(const int& place_id, const int& cad_id, const int& job_no,
                                              const int& recog_id)
{
  return "item_" + std::to_string(place_id) + "_" + std::to_string(cad_id) + "_" + std::to_string(job_no) + "_" +
         std::to_string(recog_id);
}

bool PlanningSceneUpdater::getContainerPosition(const std::string& name, geometry_msgs::Point& position,
                                                geometry_msgs::Point& inside_position)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue container_info_array;
  if (!nh.getParam(CONTAINER_INFO_PATH, container_info_array))
  {
    ROS_ERROR("getParam() error: %s", CONTAINER_INFO_PATH.c_str());
    return false;
  }
  ROS_ASSERT(container_info_array.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (std::size_t i = 0; i < container_info_array.size(); i++)
  {
    ROS_ASSERT(container_info_array[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string container_name = static_cast<std::string>(container_info_array[i]["name"]);

    if (name == container_name)
    {
      ROS_ASSERT(container_info_array[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      XmlRpc::XmlRpcValue& position_value = container_info_array[i]["position"];

      position.x = static_cast<double>(position_value[0]);
      position.y = static_cast<double>(position_value[1]);
      position.z = static_cast<double>(position_value[2]);

      ROS_ASSERT(container_info_array[i]["inside_position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      XmlRpc::XmlRpcValue& inside_position_value = container_info_array[i]["inside_position"];

      inside_position.x = static_cast<double>(inside_position_value[0]);
      inside_position.y = static_cast<double>(inside_position_value[1]);
      inside_position.z = static_cast<double>(inside_position_value[2]);
      return true;
    }
  }

  return false;
}

bool PlanningSceneUpdater::addBoxObject()
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue box_settings_array;
  if (!nh.getParam(BOX_SETTINGS_PATH, box_settings_array))
  {
    ROS_ERROR("getParam() error. %s", BOX_SETTINGS_PATH.c_str());
    return false;
  }

  for (std::size_t i = 0; i < box_settings_array.size(); i++)
  {
    std::string name = static_cast<std::string>(box_settings_array[i]["name"]);
    std::string size_id = static_cast<std::string>(box_settings_array[i]["size_id"]);

    geometry_msgs::Point pos, in_pos;
    getContainerPosition(name, pos, in_pos);

    std::transform(size_id.begin(), size_id.end(), size_id.begin(), ::tolower);

    std::string id = "cardboard_" + size_id;
    std::string mesh_file = container_mesh_path_ + id + ".stl";

    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

    geometry_msgs::Pose pose;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose.position = pos;

    geometry_msgs::Vector3 scale;
    scale.x = 1.0;
    scale.y = 1.0;
    scale.z = 1.0;

    std_msgs::ColorRGBA color;
    color.r = 0.76;
    color.g = 0.56;
    color.b = 0.26;
    color.a = 0.5;

    addCollisionObject(id, "world", mesh_file, pose, scale, color);
  }

  return true;
}

bool PlanningSceneUpdater::addToteObject()
{
  std::vector<std::string> totes = { "TOTE1", "TOTE2" };

  for (std::size_t i = 0; i < totes.size(); i++)
  {
    std::string name = totes[i];
    geometry_msgs::Point pos, in_pos;
    getContainerPosition(name, pos, in_pos);

    std::string mesh_file = container_mesh_path_ + "tote.stl";

    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

    geometry_msgs::Pose pose;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose.position = pos;

    geometry_msgs::Vector3 scale;
    scale.x = 1.0;
    scale.y = 1.0;
    scale.z = 1.0;

    std_msgs::ColorRGBA color;
    color.r = 0.8;
    color.g = 0.2;
    color.b = 0.2;
    color.a = 0.5;

    addCollisionObject(name, "world", mesh_file, pose, scale, color);
  }

  return true;
}

bool PlanningSceneUpdater::clearContainers()
{
  moveit_msgs::GetPlanningScene gsrv;
  moveit_msgs::ApplyPlanningScene asrv;

  gsrv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
  if (!get_planning_scene_client_.call(gsrv))
  {
    ROS_ERROR("Failed to call get_planning_scene service.");
    return false;
  }

  std::vector<moveit_msgs::CollisionObject>& co = gsrv.response.scene.world.collision_objects;

  std::regex re("(TOTE.*|cardboard_.*)");
  std::smatch match;

  asrv.request.scene.is_diff = true;

  for (std::size_t i = 0; i < co.size(); i++)
  {
    if (std::regex_match(co[i].id, match, re))
    {
      co[i].operation = moveit_msgs::CollisionObject::REMOVE;
      asrv.request.scene.world.collision_objects.push_back(co[i]);
      items_info_.erase(co[i].id);
    }
  }

  if (!apply_planning_scene_client_.call(asrv))
  {
    ROS_ERROR("Failed to call apply_planning_scene service.");
    return false;
  }

  return true;
}

PlanningSceneUpdater::~PlanningSceneUpdater()
{
}

}  // namespace t2_planning_scene_updater
