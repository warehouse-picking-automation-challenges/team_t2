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

#include <t2_motion_planner/t2_motion_planner.h>

namespace t2_motion_planner
{
MotionPlanner::MotionPlanner()
  : container_region_names_({ CONTAINER_REGION_BIN, CONTAINER_REGION_TOTE, CONTAINER_REGION_BOX, CONTAINER_REGION_SS })
  , nh_()
  , private_nh_("~")
  , suction_state_flag_(0)
  , pad_contact_state_flag_(0)
  , gripper_suction_flag_(false)
  , arm_approach_item_flag_(false)
  , arm_controller_feedback_flag_(false)
  , gripper_finger_contact_state_flag_(0)
  , gripper_is_gripped_flag_(0)
  , plan_groups_({ PLAN_GROUP_ARM, PLAN_GROUP_SUCTION, PLAN_GROUP_PINCH })
  , default_plan_group_(PLAN_GROUP_ARM)
{
}

void MotionPlanner::init()
{
  robot_model_loader::RobotModelLoader rml(ROBOT_DESCRIPTION);
  robot_model::RobotModelConstPtr robot_model = rml.getModel();

  if (!robot_model)
  {
    ROS_ERROR("Failed to load robotmodel");
    return;
  }

  // Load Motion Planner configs from parameter server
  ros::NodeHandle private_config_nh("~/motion_planner_configs");

  // plan_group
  private_config_nh.getParam("plan_group", plan_groups_);
  if (plan_groups_.size() < 1)
  {
    ROS_ERROR("Failed to load plan_group");
    return;
  }

  // Initialize Move Groups
  for (std::size_t i = 0; i < plan_groups_.size(); i++)
  {
    const std::string& plan_group = plan_groups_[i];
    move_groups_[plan_group] = MoveGroupPtr(new moveit::planning_interface::MoveGroup(plan_group));
    moveit::core::RobotModelConstPtr robot_model = move_groups_[plan_group]->getRobotModel();
    t2_robot_models_[plan_group] = T2RobotModelPtr(new T2RobotModel(robot_model->getURDF(), robot_model->getSRDF()));
  }

  // planner id
  std::string plannner_id;
  private_config_nh.param<std::string>("planner_id", plannner_id, "RRTConnectkConfigDefault");

  // planning time
  private_config_nh.param<double>("planning_time_default", planning_time_default_, 1.0);
  private_config_nh.param<double>("planning_time_pick", planning_time_pick_, 1.0);

  // plannning retry
  private_config_nh.param<int>("planning_retry_count_default", planning_retry_count_default_, 5);
  private_config_nh.param<int>("planning_retry_count_pick", planning_retry_count_pick_, 5);

  // planning attempts
  private_config_nh.param<int>("planning_attempts_default", planning_attempts_default_, 1);
  private_config_nh.param<int>("planning_attempts_pick", planning_attempts_pick_, 1);

  // workspace
  std::vector<double> ws;
  private_config_nh.getParam("workspace", ws);

  // end_effector_position_constraint_marekr
  private_config_nh.param<bool>("end_effector_position_constraint_marekr", end_effector_position_constraint_marekr_,
                                false);

  // end_effector_position_constraint_box
  private_config_nh.getParam("end_effector_position_constraint_box", end_effector_position_constraint_box_);
  if (end_effector_position_constraint_box_.size() != 6)
  {
    ROS_ERROR("Failed to load end_effector_position_constraint_box");
    return;
  }

  // end_effector_position_constraint_midair_scale
  private_config_nh.param<double>("end_effector_position_constraint_midair_scale",
                                  end_effector_position_constraint_midair_scale_, 1.2);

  // end_effector_position_constraint_container_mergin
  private_config_nh.param<double>("end_effector_position_constraint_container_mergin",
                                  end_effector_position_constraint_container_mergin_, 0.05);

  // cartesian_path_approach
  private_config_nh.param<bool>("cartesian_path_approach", cartesian_path_approach_, true);

  // cartesian_path_eef_step
  private_config_nh.param<double>("cartesian_path_eef_step", cartesian_path_eef_step_, 0.01);

  // cartesian_path_jump_threshold
  private_config_nh.param<double>("cartesian_path_jump_threshold", cartesian_path_jump_threshold_, 0.05);

  // cartesian_path_approach_fraction_threshold
  private_config_nh.param<double>("cartesian_path_approach_fraction_threshold",
                                  cartesian_path_approach_fraction_threshold_, 1.0);

  // cartesian_path_error_recover_by_normal_plan
  private_config_nh.param<bool>("cartesian_path_error_recover_by_normal_plan",
                                cartesian_path_error_recover_by_normal_plan_, true);

  // max_valid_path_length:
  private_config_nh.param<double>("max_valid_path_length_threshold_near", max_valid_path_length_threshold_near_, 0.1);
  private_config_nh.param<double>("max_valid_path_length_threshold_far", max_valid_path_length_threshold_far_, 1.0);

  private_config_nh.param<double>("max_valid_path_length_near_strict", max_valid_path_length_near_strict_, 1.0);
  private_config_nh.param<double>("max_valid_path_length_near_lax", max_valid_path_length_near_lax_, 2.0);

  private_config_nh.param<double>("max_valid_path_length_middle_strict", max_valid_path_length_middle_strict_, 3.0);
  private_config_nh.param<double>("max_valid_path_length_middle_lax", max_valid_path_length_middle_lax_, 15.0);

  private_config_nh.param<double>("max_valid_path_length_far_strict", max_valid_path_length_far_strict_, 6.0);
  private_config_nh.param<double>("max_valid_path_length_far_lax", max_valid_path_length_far_lax_, 20.0);

  // max_valid_path_smoothness: 0.1
  private_config_nh.param<double>("max_valid_path_smoothness", max_valid_path_smoothness_, 0.1);

  // n_points: 5
  private_config_nh.param<int>("n_points", n_points_, 5);

  // delete_trajectory_connection_points
  private_config_nh.param<bool>("delete_trajectory_connection_points", delete_trajectory_connection_points_, true);

  // path_evaluate_joints
  private_config_nh.getParam("path_evaluate_joints", path_evaluate_joints_);

  // path_length_joint_weight
  private_config_nh.getParam("path_length_joint_weight", path_length_joint_weight_);

  // path_smooth_joint_weight
  private_config_nh.getParam("path_smooth_joint_weight", path_smooth_joint_weight_);

  // set params to planning interface
  for (std::size_t i = 0; i < plan_groups_.size(); i++)
  {
    const std::string& plan_group = plan_groups_[i];
    move_groups_[plan_group] = MoveGroupPtr(new moveit::planning_interface::MoveGroup(plan_group));
    move_groups_[plan_group]->setPlannerId(plannner_id);
    if (ws.size() == 6)
    {
      configureWorkspace(move_groups_[plan_group], ws[0], ws[1], ws[2], ws[3], ws[4], ws[5]);
    }
  }

  default_move_group_ = move_groups_[default_plan_group_];

  // suction_sensor_waiting_time: 1.0
  private_config_nh.param<double>("suction_sensor_waiting_time", suction_sensor_waiting_time_, 1.0);

  // Load Gripper configs from parameter server
  ros::NodeHandle gripper_config_nh(t2_pinching_gripper::NODE_NAME);
  gripper_config_nh.param<double>("FingertipSens_detect_extend_pos", gripper_detect_extend_pos_, 0.1);

  gripper_extender_retract_min_ = 0;
  gripper_pinch_close_min_ = 0;

  gripper_butting_joint_positions_ = { gripper_detect_extend_pos_, gripper_pinch_close_min_ };
  gripper_retract_joint_positions_ = { gripper_extender_retract_min_, gripper_pinch_close_min_ };

  // trajectory_collison_check
  private_config_nh.getParam("trajectory_collision_check_area", trajectory_collision_check_area_);

  // position_feedback_marker
  private_config_nh.param<bool>("position_feedback_marker", position_feedback_marker_, false);

  // position_feedback_bin_midair_height
  private_config_nh.param<double>("position_feedback_bin_midair_height",
                                  position_feedback_midair_height_[CONTAINER_REGION_BIN], 0.9);

  // position_feedback_tote_midair_height_
  private_config_nh.param<double>("position_feedback_tote_midair_height",
                                  position_feedback_midair_height_[CONTAINER_REGION_TOTE], 0.9);

  // position_feedback_box_midair_height_;
  private_config_nh.param<double>("position_feedback_box_midair_height",
                                  position_feedback_midair_height_[CONTAINER_REGION_BOX], 0.9);

  // position_feedback_ss_midair_height_;
  private_config_nh.param<double>("position_feedback_ss_midair_height",
                                  position_feedback_midair_height_[CONTAINER_REGION_SS], 1.5);

  // goal_joint_tolerance
  private_config_nh.param<double>("goal_joint_tolerance", goal_joint_tolerance_, 0.01);

  // goal_orientation_tolerance
  private_config_nh.param<double>("goal_orientation_tolerance", goal_orientation_tolerance_, 0.01);

  // goal_position_tolerance
  private_config_nh.param<double>("goal_position_tolerance", goal_position_tolerance_, 0.001);

  // weight_scale_timeout_time
  private_config_nh.param<double>("weight_scale_timeout_time", weight_scale_timeout_time_, 5.0);

  // weight_scale_no_item_weight_tolerance
  private_config_nh.param<double>("weight_scale_no_item_weight_tolerance", weight_scale_no_item_weight_tolerance_,
                                  0.002);

  // weight_scale_item_weight_tolerance
  private_config_nh.param<double>("weight_scale_item_weight_tolerance", weight_scale_item_weight_tolerance_, 0.1);

  // disable_weight_check
  private_config_nh.param<bool>("disable_weight_check", disable_weight_check_, false);

  // compare_joint_tolerance
  private_config_nh.param<double>("compare_joint_tolerance", compare_joint_tolerance_, goal_joint_tolerance_);

  // Topic
  suction_state_ = nh_.subscribe(GRIPPER_SUCTION_SENSOR_TOPIC_NAME, 10, &MotionPlanner::cbSuctionState, this);

  pad_contact_state_ = nh_.subscribe(GRIPPER_PAD_CONTACT_STATE_TOPIC_NAME, 10, &MotionPlanner::cbPadContactState, this);

  gripper_finger_contact_state_ =
      nh_.subscribe(t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::BUTTING_STATE_TOPIC_NAME, 10,
                    &MotionPlanner::cbFingerContactState, this);

  gripping_state_ = nh_.subscribe(t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::GRIPPING_STATE_TOPIC_NAME,
                                  10, &MotionPlanner::cbGrippingState, this);

#ifdef USE_MOVE_GROUP_FOR_EXECUTE
  arm_controller_feedback_ =
      nh_.subscribe(ARM_CONTROLLER_FEEDBACK_TOPIC_NAME, 10, &MotionPlanner::cbArmControllerFeedback, this);
#endif

  // Action Server
  arm_execute_action_server_.reset(
      new ArmExecuteActionServer(private_nh_, t2_motion_planner::ARM_EXECUTE_ACTION_NAME,
                                 boost::bind(&MotionPlanner::cbArmExecuteActionServerGoal, this, _1), false));
  arm_execute_action_server_->start();

  arm_move_action_server_.reset(
      new ArmMoveActionServer(private_nh_, t2_motion_planner::ARM_MOVE_ACTION_NAME,
                              boost::bind(&MotionPlanner::cbArmMoveActionServerGoal, this, _1), false));
  arm_move_action_server_->start();

  arm_plan_action_server_.reset(
      new ArmPlanActionServer(private_nh_, t2_motion_planner::ARM_PLAN_ACTION_NAME,
                              boost::bind(&MotionPlanner::cbArmPlanActionServerGoal, this, _1), false));
  arm_plan_action_server_->start();

  arm_pick_plan_action_server_.reset(
      new ArmPickPlanActionServer(private_nh_, t2_motion_planner::ARM_PICK_PLAN_ACTION_NAME,
                                  boost::bind(&MotionPlanner::cbArmPickPlanActionServerGoal, this, _1), false));
  arm_pick_plan_action_server_->start();

  arm_pick_execute_action_server_.reset(
      new ArmPickExecuteActionServer(private_nh_, t2_motion_planner::ARM_PICK_EXECUTE_ACTION_NAME,
                                     boost::bind(&MotionPlanner::cbArmPickExecuteActionServerGoal, this, _1), false));
  arm_pick_execute_action_server_->start();

  arm_place_move_action_server_.reset(
      new ArmPlaceMoveActionServer(private_nh_, t2_motion_planner::ARM_PLACE_MOVE_ACTION_NAME,
                                   boost::bind(&MotionPlanner::cbArmPlaceMoveActionServerGoal, this, _1), false));
  arm_place_move_action_server_->start();

  // Service Server
  clear_arm_plan_service_ = private_nh_.advertiseService(t2_motion_planner::CLEAR_ARM_PLAN_SERVICE_NAME,
                                                         &MotionPlanner::cbClearArmPlanService, this);
  get_arm_pose_service_ = private_nh_.advertiseService(t2_motion_planner::GET_ARM_POSE_SERVICE_NAME,
                                                       &MotionPlanner::cbGetArmPoseService, this);
  get_arm_group_state_pose_service_ = private_nh_.advertiseService(
      t2_motion_planner::GET_ARM_GROUP_STATE_POSE_SERVICE_NAME, &MotionPlanner::cbGetArmGroupStatePoseService, this);
  save_arm_plan_service_ = private_nh_.advertiseService(t2_motion_planner::SAVE_ARM_PLAN_SERVICE_NAME,
                                                        &MotionPlanner::cbSaveArmPlanService, this);
  get_planned_arm_trajectory_service_ =
      private_nh_.advertiseService(t2_motion_planner::GET_PLANNED_ARM_TRAJECTRY_SERVICE_NAME,
                                   &MotionPlanner::cbGetPlannedArmTrajectoryService, this);

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(RVIZ_VISUALIZATION_MARKER_TOPIC_NAME, 10);

  set_pick_item_to_planning_scene_client_ = nh_.serviceClient<t2_msgs::SetPickItemToPlanningScene>(
      t2_planning_scene_updater::NODE_NAME + "/" + t2_planning_scene_updater::SET_PICK_ITEM_SERVICE_NAME);

  clear_attached_item_client_ = nh_.serviceClient<t2_msgs::ClearAttachedItem>(
      t2_planning_scene_updater::NODE_NAME + "/" + t2_planning_scene_updater::CLEAR_ATTACHED_ITEM_SERVICE_NAME);

  get_attached_item_client_ = nh_.serviceClient<t2_msgs::GetAttachedItemFromPlanningScene>(
      t2_planning_scene_updater::NODE_NAME + "/" + t2_planning_scene_updater::GET_ATTACHED_ITEM_SERVICE_NAME);
}

#ifdef USE_MOVE_GROUP_FOR_EXECUTE
void MotionPlanner::cbArmControllerFeedback(const control_msgs::FollowJointTrajectoryActionFeedbackConstPtr& feedback)
#else
void MotionPlanner::cbArmControllerFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
#endif
{
#ifdef USE_MOVE_GROUP_FOR_EXECUTE
  last_arm_controller_feedback_ = feedback->feedback;
#else
  last_arm_controller_feedback_ = *feedback;
#endif
  arm_controller_feedback_flag_ = true;

  if (arm_pick_execute_action_server_->isActive() && arm_approach_item_flag_)
  {
    t2_msgs::GraspPoint& grasp_point = pick_execute_plan_->plan_goal.grasp_point;

    if (grasp_point.grasp_pattern == GRASP_PATTERN_SUCTION && gripper_suction_flag_ == t2_msgs::GraspPoint::SUCTION_OFF)
    {
      std::string region;

      if (getContainerRegionFromPlaceId(pick_execute_plan_->plan_goal.place_id, region))
      {
        std::vector<std::string> regions = { region };
        std::vector<bool> collision_flags;

        endEffectorRegionCollisionCheck(pick_execute_plan_->plan_group, last_arm_controller_feedback_, regions,
                                        collision_flags);

        if (collision_flags[0])
        {
          ROS_INFO("set gripper suction on by end-effector enter bin or tote");

          // suction on
          if (grasp_point.suction_strength == t2_msgs::GraspPoint::SUCTION_OFF)
          {
            ROS_WARN("Force to use suction_strength HIGH because it was OFF");
            grasp_point.suction_strength = t2_msgs::GraspPoint::SUCTION_HIGH;
          }

          if (!setGripperSuction(grasp_point.suction_strength, grasp_point.threshold_of_vacuum_for_suction))
          {
            ROS_ERROR("Failed to setGripperSuction()");
          }
        }
      }
      else
      {
        ROS_ERROR("Invalid place_id.");
      }
    }
  }

  if (arm_place_move_action_server_->isActive() &&
      (!feedback_leave_container_flag_ || !feedback_leave_container_midair_flag_))
  {
    std::string region;

    if (getContainerRegionFromPlaceId(feedback_place_id_, region))
    {
      std::vector<std::string> regions;
      std::vector<bool> collision_flags;

      if (region == CONTAINER_REGION_BIN)
      {
        regions = { region, CONTAINER_REGION_SS };
      }
      else
      {
        regions = { region };
      }

      endEffectorRegionCollisionCheck(pick_execute_plan_->plan_group, last_arm_controller_feedback_, regions,
                                      collision_flags);

      // end-effector leave bin or tote
      if (!feedback_leave_container_flag_ && !collision_flags[0])
      {
        ROS_INFO("end-effector leave bin or tote");

        feedback_leave_container_flag_ = true;

        // feedback to task planner for capture storage
        t2_msgs::ArmPlaceMoveFeedback feedback;
        feedback.place_id = feedback_place_id_;
        feedback.event = FEEDBACK_EVENT_LEAVE_CONTAINER;
        arm_place_move_action_server_->publishFeedback(feedback);

        // weight scale
        weightScaleExecute(feedback_place_id_, false);
      }
    }
    else
    {
      ROS_ERROR("Invalid place_id.");
    }
  }
}

bool MotionPlanner::getJointNames(const std::string& plan_group, std::vector<std::string>& joint_names)
{
  MoveGroupPtr move_group;

  if (!getMoveGroup(plan_group, move_group))
  {
    return false;
  }

  joint_names = move_group->getCurrentState()->getJointModelGroup(move_group->getName())->getJointModelNames();

  return true;
}

bool MotionPlanner::getArmJointPositions(std::vector<std::string>& joint_names, std::vector<double>& joint_positions)
{
  MoveGroupPtr move_group = default_move_group_;

  // set JointValue form group_state_name
  robot_state::RobotState current_state = *move_group->getCurrentState();

  // set JointValue
  joint_names = current_state.getJointModelGroup(move_group->getName())->getJointModelNames();

  joint_positions.resize(joint_names.size());

  const bool arm_controller_feedback_updated = arm_controller_feedback_flag_;

  trajectory_msgs::JointTrajectoryPoint* last_joint_trajectory_point;

  if (arm_controller_feedback_updated)
  {
    if (!last_arm_controller_feedback_.actual.positions.empty())
    {
      last_joint_trajectory_point = &last_arm_controller_feedback_.actual;
    }
    else if (!last_arm_controller_feedback_.desired.positions.empty())
    {
      last_joint_trajectory_point = &last_arm_controller_feedback_.desired;
    }
    else
    {
      ROS_ERROR("Arm controller feedback value error.");
      return false;
    }
  }

  for (std::size_t i = 0; i < joint_positions.size(); ++i)
  {
    auto ite = std::find(last_arm_controller_feedback_.joint_names.begin(),
                         last_arm_controller_feedback_.joint_names.end(), joint_names[i]);

    if (!arm_controller_feedback_updated || ite == last_arm_controller_feedback_.joint_names.end())
    {
      joint_positions[i] = *current_state.getJointPositions(joint_names[i]);
    }
    else
    {
      size_t j = std::distance(last_arm_controller_feedback_.joint_names.begin(), ite);
      joint_positions[i] = last_joint_trajectory_point->positions[j];
    }
  }

  return true;
}

bool MotionPlanner::isEqualJointPositions(std::vector<std::string>& joint_names1, std::vector<double>& joint_positions1,
                                          std::vector<std::string>& joint_names2, std::vector<double>& joint_positions2,
                                          double range)
{
  for (std::size_t i = 0; i < joint_names1.size(); i++)
  {
    auto ite = std::find(joint_names2.begin(), joint_names2.end(), joint_names1[i]);

    if (ite == joint_names2.end())
    {
      ROS_ERROR("Invalid joint_name = %s", joint_names1[i].c_str());
      return false;
    }

    size_t j = std::distance(joint_names2.begin(), ite);

    if (fabs(joint_positions1[i] - joint_positions2[j]) > range)
    {
      // ROS_INFO("isEqualJointPositions() false");
      return false;
    }
  }
  return true;
}

bool MotionPlanner::trajectorySamePointFilter(const std::vector<trajectory_msgs::JointTrajectoryPoint>& points_in,
                                              std::vector<trajectory_msgs::JointTrajectoryPoint>& points_out)
{
  points_out.clear();

  for (std::size_t i = 0; i < points_in.size(); i++)
  {
    if (i == 0)
    {
      points_out.push_back(points_in[i]);
      continue;
    }

    if (points_in[i - 1].positions != points_in[i].positions)
    {
      points_out.push_back(points_in[i]);
    }
  }
  return true;
}

bool MotionPlanner::trajectoryNPointFilter(const std::vector<trajectory_msgs::JointTrajectoryPoint>& points_in,
                                           std::vector<trajectory_msgs::JointTrajectoryPoint>& points_out, int n_points)
{
  points_out.clear();

  if (n_points >= 2 && points_in.size() > static_cast<std::size_t>(n_points))
  {
    std::vector<bool> adopt_flags(points_in.size(), false);

    int inc = static_cast<int>(ceil(static_cast<double>(points_in.size()) / static_cast<double>(n_points - 1)));

    for (std::size_t i = 0; i < points_in.size(); i += inc)
    {
      adopt_flags[i] = true;
    }

    adopt_flags[points_in.size() - 1] = true;  // end point

    points_out.resize(std::count(adopt_flags.begin(), adopt_flags.end(), true));

    std::size_t out_index = 0;
    for (std::size_t i = 0; i < points_in.size(); i++)
    {
      if (adopt_flags[i])
      {
        points_out[out_index] = points_in[i];
        out_index++;
      }
    }
  }
  else
  {
    points_out = points_in;
  }

  ROS_INFO("trajectoryNPointFilter %lu -> %lu", points_in.size(), points_out.size());

  return true;
}

bool MotionPlanner::trajectoryJointNameFilter(const trajectory_msgs::JointTrajectory& trajectory_in,
                                              trajectory_msgs::JointTrajectory& trajectory_out,
                                              const std::vector<std::string>& valid_joint_names)
{
  trajectory_out.header = trajectory_in.header;
  trajectory_out.joint_names.clear();
  trajectory_out.points.resize(trajectory_in.points.size());
  std::vector<std::size_t> valid_joint_num;

  for (std::size_t i = 0; i < trajectory_in.joint_names.size(); i++)
  {
    if (std::find(valid_joint_names.begin(), valid_joint_names.end(), trajectory_in.joint_names[i]) !=
        valid_joint_names.end())
    {
      trajectory_out.joint_names.push_back(trajectory_in.joint_names[i]);
      valid_joint_num.push_back(i);
    }
  }

  if (valid_joint_num.size() == trajectory_in.joint_names.size())
  {
    // no need to filter
    trajectory_out = trajectory_in;
    return true;
  }

  for (std::size_t i = 0; i < trajectory_in.points.size(); i++)
  {
    const trajectory_msgs::JointTrajectoryPoint& in_point = trajectory_in.points[i];
    trajectory_msgs::JointTrajectoryPoint& out_point = trajectory_out.points[i];

    out_point.time_from_start = in_point.time_from_start;
    out_point.effort = in_point.effort;

    for (std::size_t j = 0; j < valid_joint_num.size(); j++)
    {
      const size_t& num = valid_joint_num[j];

      if (!in_point.accelerations.empty())
      {
        out_point.accelerations.push_back(in_point.accelerations[num]);
      }

      if (!in_point.positions.empty())
      {
        out_point.positions.push_back(in_point.positions[num]);
      }

      if (!in_point.velocities.empty())
      {
        out_point.velocities.push_back(in_point.velocities[num]);
      }
    }
  }

  return true;
}

bool MotionPlanner::trajectoryPointsAddVelocity(std::vector<trajectory_msgs::JointTrajectoryPoint>& points,
                                                const double& velocity)
{
  std::vector<double> velocities(points[0].positions.size(), velocity);

  for (std::size_t i = 0; i < points.size(); i++)
  {
    points[i].velocities = velocities;
  }

  return true;
}

bool MotionPlanner::combinePlanTrajectory(const std::vector<int>& plan_id, const std::vector<float>& velocity,
                                          trajectory_msgs::JointTrajectory& trajectory)
{
  std::lock_guard<std::mutex> lock(combine_plan_mutex_);

  for (std::size_t i = 0; i < plan_id.size(); i++)
  {
    ArmPlanMap::iterator ite = arm_plans_.find(plan_id[i]);

    if (ite == arm_plans_.end())
    {
      ROS_ERROR("invalid plan_id = %d", plan_id[i]);
      return false;
    }

    trajectory_msgs::JointTrajectory& trj = ite->second.plan.trajectory_.joint_trajectory;

    trajectoryPointsAddVelocity(trj.points, velocity[i]);

    if (i == 0)
    {
      trajectory.header = trj.header;
      trajectory.joint_names = trj.joint_names;
      trajectory.points = trj.points;
    }
    else
    {
      if (delete_trajectory_connection_points_ && trajectory.points.size() >= static_cast<std::size_t>(n_points_))
      {
        // delete waypoint
        trajectory.points.pop_back();
      }
      trajectory.points.insert(trajectory.points.end(), trj.points.begin() + 1, trj.points.end());
    }
  }

  return true;
}

bool MotionPlanner::combinePickPlanTrajectory(const int plan_id, const float velocity, const int pick_plan_id,
                                              trajectory_msgs::JointTrajectory& trajectory)
{
  std::lock_guard<std::mutex> lock(combine_plan_mutex_);

  ArmPlanMap::iterator plan_ite = arm_plans_.find(plan_id);
  ArmPickPlanMap::iterator pick_plan_ite = arm_pick_plans_.find(pick_plan_id);

  if (plan_ite == arm_plans_.end())
  {
    ROS_ERROR("invalid plan_id = %d", plan_id);
    return false;
  }

  if (pick_plan_ite == arm_pick_plans_.end())
  {
    ROS_ERROR("invalid pick_plan_id = %d", pick_plan_id);
    return false;
  }

  trajectory = plan_ite->second.plan.trajectory_.joint_trajectory;

  trajectoryPointsAddVelocity(trajectory.points, velocity);

  const trajectory_msgs::JointTrajectory& pick_plan_trj = pick_plan_ite->second.plan.trajectory_.joint_trajectory;

  if (delete_trajectory_connection_points_ && trajectory.points.size() >= static_cast<std::size_t>(n_points_))
  {
    // delete waypoint
    trajectory.points.pop_back();
  }

  trajectory.points.insert(trajectory.points.end(), pick_plan_trj.points.begin() + 1, pick_plan_trj.points.end());

  return true;
}

double MotionPlanner::computeDirectDistance(MoveGroupPtr& move_group,
                                            const trajectory_msgs::JointTrajectory& trajectory)
{
  robot_trajectory::RobotTrajectory p(move_group->getRobotModel(), move_group->getName());
  p.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

  Eigen::Translation3d start_pos(
      p.getFirstWayPoint().getGlobalLinkTransform(move_group->getEndEffectorLink()).translation());
  Eigen::Translation3d end_pos(
      p.getLastWayPoint().getGlobalLinkTransform(move_group->getEndEffectorLink()).translation());

  double dx = end_pos.x() - start_pos.x();
  double dy = end_pos.y() - start_pos.y();
  double dz = end_pos.z() - start_pos.z();

  return sqrt(dx * dx + dy * dy + dz * dz);
}

double MotionPlanner::computePathLength(MoveGroupPtr& move_group, T2RobotModelPtr& t2_robot_model,
                                        const trajectory_msgs::JointTrajectory& trajectory,
                                        const std::vector<std::string>& joint_names, const std::vector<double>& weight)
{
  // From moveit_ros benchmarks (benchmark_execution.cpp)

  double length = 0;

  moveit::core::RobotModelConstPtr robot_model = move_group->getRobotModel();
  robot_trajectory::RobotTrajectory p(robot_model, move_group->getName());
  p.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

  for (std::size_t i = 1; i < p.getWayPointCount(); i++)
  {
    const double* state1 = p.getWayPoint(i - 1).getVariablePositions();
    const double* state2 = p.getWayPoint(i).getVariablePositions();

    length += t2_robot_model->weightingDistance(state1, state2, joint_names, weight);
  }

  return length;
}

double MotionPlanner::computePathSmoothness(MoveGroupPtr& move_group, T2RobotModelPtr& t2_robot_model,
                                            const trajectory_msgs::JointTrajectory& trajectory,
                                            const std::vector<std::string>& joint_names,
                                            const std::vector<double>& weight)
{
  // From moveit_ros benchmarks (benchmark_execution.cpp)

  double smoothness = 0;

  moveit::core::RobotModelConstPtr robot_model = move_group->getRobotModel();
  robot_trajectory::RobotTrajectory p(robot_model, move_group->getName());
  p.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

  if (p.getWayPointCount() > 2)
  {
    const double* state0 = p.getWayPoint(0).getVariablePositions();
    const double* state1 = p.getWayPoint(1).getVariablePositions();
    double a = t2_robot_model->weightingDistance(state0, state1, joint_names, weight);

    for (std::size_t k = 2; k < p.getWayPointCount(); k++)
    {
      const double* state_k0 = p.getWayPoint(k).getVariablePositions();
      const double* state_k1 = p.getWayPoint(k - 1).getVariablePositions();
      const double* state_k2 = p.getWayPoint(k - 2).getVariablePositions();

      // double b = p.getWayPoint(k - 1).distance(p.getWayPoint(k));
      double b = t2_robot_model->weightingDistance(state_k1, state_k0, joint_names, weight);
      // double cdist = p.getWayPoint(k - 2).distance(p.getWayPoint(k));
      double cdist = t2_robot_model->weightingDistance(state_k2, state_k0, joint_names, weight);

      double acosValue = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
      if (acosValue > -1.0 && acosValue < 1.0)
      {
        double angle = (M_PI - acos(acosValue));
        double u = 2.0 * angle;
        smoothness += u * u;
      }
      a = b;
    }
    smoothness /= static_cast<double>(p.getWayPointCount());
  }

  return smoothness;
}

bool MotionPlanner::trajectoryCollisionCheck(const std::string& plan_group,
                                             const trajectory_msgs::JointTrajectory& trajectory, double x_max,
                                             double x_min, double y_max, double y_min, double z)
{
  MoveGroupPtr move_group;

  if (!getMoveGroup(plan_group, move_group))
  {
    return false;
  }

  robot_trajectory::RobotTrajectory p(move_group->getRobotModel(), move_group->getName());
  p.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

  for (std::size_t i = 0; i < p.getWayPointCount(); i++)
  {
    geometry_msgs::Pose pose;
    const Eigen::Affine3d& eigen_pose = p.getWayPoint(i).getGlobalLinkTransform(move_group->getEndEffectorLink());
    tf::poseEigenToMsg(eigen_pose, pose);

    if ((x_min <= pose.position.x && pose.position.x <= x_max) &&
        (y_min <= pose.position.y && pose.position.y <= y_max) && pose.position.z <= z)
    {
      return false;
    }
  }

  return true;
}

bool MotionPlanner::getContainerRegionFromPlaceId(const uint32_t& place_id, std::string& region)
{
  for (auto ite = container_region_map_.begin(); ite != container_region_map_.end(); ++ite)
  {
    std::vector<uint32_t>& place_ids = ite->second.place_ids;

    if (std::find(place_ids.begin(), place_ids.end(), place_id) != place_ids.end())
    {
      region = ite->first;
      return true;
    }
  }

  return false;
}

bool MotionPlanner::endEffectorRegionCollisionCheck(const std::string& plan_group,
                                                    const control_msgs::FollowJointTrajectoryFeedback& feedback,
                                                    const std::vector<std::string>& regions,
                                                    std::vector<bool>& collision_flags)
{
  MoveGroupPtr move_group;
  bool maker_flag = false;

  collision_flags = std::vector<bool>(regions.size(), false);

  if (!getMoveGroup(plan_group, move_group))
  {
    return false;
  }

  for (std::size_t i = 0; i < regions.size(); i++)
  {
    if (collision_flags[i])
    {
      continue;
    }

    const geometry_msgs::Point& position = container_region_map_[regions[i]].position;
    const geometry_msgs::Vector3& dimensions = container_region_map_[regions[i]].dimensions;

    const double x_min = position.x;
    const double y_min = position.y;
    const double z_min = position.z;
    const double x_max = position.x + dimensions.x;
    const double y_max = position.y + dimensions.y;
    const double z_max = position.z + dimensions.z;

    robot_state::RobotState current_state = *move_group->getCurrentState();

    // set joint position
    for (std::size_t j = 0; j < feedback.joint_names.size(); j++)
    {
      current_state.setJointPositions(feedback.joint_names[j], &feedback.actual.positions[j]);
    }

    geometry_msgs::Pose pose;
    const Eigen::Affine3d& eigen_pose = current_state.getGlobalLinkTransform(move_group->getEndEffectorLink());
    tf::poseEigenToMsg(eigen_pose, pose);

    if ((x_min <= pose.position.x && pose.position.x <= x_max) &&
        (y_min <= pose.position.y && pose.position.y <= y_max) &&
        (z_min <= pose.position.z && pose.position.z <= z_max))
    {
      collision_flags[i] = true;
      maker_flag = true;
    }
  }

  if (maker_flag && position_feedback_marker_)
  {
    publishContainerRegionMarker(regions);
  }

  return true;
}

MotionPlannerRet MotionPlanner::configureWorkspace(MoveGroupPtr& move_group, const double& center_x,
                                                   const double& center_y, const double& center_z, const double& size_x,
                                                   const double& size_y, const double& size_z)
{
  if (!move_group)
  {
    ROS_ERROR("move_group is null");
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }

  double minx, miny, minz, maxx, maxy, maxz;
  minx = center_x - size_x / 2.0;
  maxx = center_x + size_x / 2.0;
  miny = center_y - size_y / 2.0;
  maxy = center_y + size_y / 2.0;
  minz = center_z - size_z / 2.0;
  maxz = center_z + size_z / 2.0;

  move_group->setWorkspace(minx, miny, minz, maxx, maxy, maxz);

  return MotionPlannerRet::MOTION_PLANNER_SUCCESS;
}

MotionPlannerRet MotionPlanner::Plan(const t2_msgs::ArmPlanGoalConstPtr& goal, t2_msgs::ArmPlanResult& result)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);

  ros::Time start_time = ros::Time::now();

  result.result = t2_msgs::ArmPlanResult::FAILED;
  result.plan_id = goal->plan_id;

  MotionPlannerPlan current_plan;
  MoveGroupPtr move_group;
  T2RobotModelPtr t2_robot_model;

  double planning_time = planning_time_default_;
  int planning_retry_count = planning_retry_count_default_;
  int planning_attempts = planning_attempts_default_;

  ROS_INFO("%s  Plan start. plan_group = %s", goal->plan_title.c_str(), goal->plan_group.c_str());

  if (goal->plan_type == t2_msgs::ArmPlanGoal::PLAN_PICK)
  {
    ROS_INFO("plan_type = PLAN_PICK");
    planning_time = planning_time_pick_;
    planning_retry_count = planning_retry_count_pick_;
    planning_attempts = planning_attempts_pick_;
  }
  else
  {
    ROS_INFO("plan_type = PLAN_DEFAULT");
  }

  ROS_INFO("planning_time = %f, planning_retry_count = %d, planning_attempts = %d", planning_time, planning_retry_count,
           planning_attempts);

  if (!getMoveGroup(goal->plan_group, move_group))
  {
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }

  if (!getT2RobotModel(goal->plan_group, t2_robot_model))
  {
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }

  move_group->setPlanningTime(planning_time);
  move_group->setNumPlanningAttempts(planning_attempts);

  move_group->setGoalJointTolerance(goal_joint_tolerance_);
  move_group->setGoalOrientationTolerance(goal_orientation_tolerance_);
  move_group->setGoalPositionTolerance(goal_position_tolerance_);

  robot_state::RobotState start_state = *move_group->getCurrentState();
  robot_state::RobotState goal_state = *move_group->getCurrentState();

  // feedback state
  std::vector<double> gripper_joint_positions = gripper_joint_positions_;

  if (goal->grasp_pattern == GRASP_PATTERN_PINCH)
  {
    if (goal->gripper_state == t2_msgs::ArmPlanGoal::BUTTING_STATE)
    {
      // butting pose for grasp plan
      gripper_joint_positions = gripper_butting_joint_positions_;
    }
  }
  else if (goal->grasp_pattern == GRASP_PATTERN_SUCTION)
  {
    // suction pose for grasp plan
    gripper_joint_positions = gripper_retract_joint_positions_;
  }
  else
  {
    ROS_WARN("gripper_joint_positions use feedback value");
  }

  for (std::size_t i = 0; i < gripper_joint_names_.size(); i++)
  {
    // current pose
    start_state.setJointPositions(gripper_joint_names_[i], &gripper_joint_positions[i]);
    goal_state.setJointPositions(gripper_joint_names_[i], &gripper_joint_positions[i]);
  }

  if (goal->start.type.type == t2_msgs::ArmPoseType::CURRENT_POSITION)
  {
    // set current state
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;

    getArmJointPositions(joint_names, joint_positions);

    for (std::size_t i = 0; i < joint_names.size(); i++)
    {
      start_state.setJointPositions(joint_names[i], &joint_positions[i]);
    }

    move_group->setStartState(start_state);
  }
  else if (goal->start.type.type == t2_msgs::ArmPoseType::CARTESIAN_POSITION)
  {
    // set end effector pose
    ROS_ERROR("Not supported start_type = CARTESIAN_POSITION");
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }
  else if (goal->start.type.type == t2_msgs::ArmPoseType::JOINT_POSITION)
  {
    // set joint position

    for (std::size_t i = 0; i < goal->start.joint_names.size(); i++)
    {
      start_state.setJointPositions(goal->start.joint_names[i], &goal->start.joint_positions[i]);
    }
    move_group->setStartState(start_state);
  }
  else if (goal->start.type.type == t2_msgs::ArmPoseType::GROUP_STATE)
  {
    // set JointValue form group_state_name
    start_state.setToDefaultValues(start_state.getJointModelGroup(PLAN_GROUP_SUCTION), goal->start.group_state);
    move_group->setStartState(start_state);
  }

  move_group->clearPathConstraints();
  moveit_msgs::Constraints path_constraints = goal->path_constraints;

  if (goal->stay_level && goal->plan_group == PLAN_GROUP_SUCTION)
  {
    // end-effector stay level
    geometry_msgs::QuaternionStamped quaternion;
    quaternion.header.frame_id = "rs20n_eef2";
    quaternion.quaternion.w = 1.0;
    path_constraints = kinematic_constraints::mergeConstraints(
        path_constraints, kinematic_constraints::constructGoalConstraints("rs20n_eef2", quaternion));
  }

  if (!kinematic_constraints::isEmpty(path_constraints))
  {
    move_group->setPathConstraints(path_constraints);
  }

  bool normal_plan_flag = false;
  bool cartesian_path_flag = false;

  if (goal->waypoints.size() > 0)
  {
    // catresian path
    normal_plan_flag = false;
    cartesian_path_flag = true;
  }
  else
  {
    // normal plan
    normal_plan_flag = true;
    cartesian_path_flag = false;
  }

  if (cartesian_path_flag)
  {
    ROS_INFO("computeCartesianPath");

    publishAxisMarker(goal->waypoints.back());

    moveit_msgs::RobotTrajectory trajectory;

    moveit_msgs::MoveItErrorCodes error_code;

    double ret;
    double fraction_threshold = 1.0;

    if (goal->cartesian_path_fraction_threshold > 0)
    {
      fraction_threshold = goal->cartesian_path_fraction_threshold;
    }

    ROS_INFO("fraction_threshold = %f", fraction_threshold);

    result.planning_attempt = 1;

    if ((ret =
             move_group->computeCartesianPath(goal->waypoints, cartesian_path_eef_step_, cartesian_path_jump_threshold_,
                                              trajectory, path_constraints, true, &error_code)) >= fraction_threshold)

    {
      ROS_INFO("Plan(computeCartesianPath) success. trajectory_points = %lu",
               trajectory.joint_trajectory.points.size());
      current_plan.plan.trajectory_ = trajectory;
    }
    else if (goal->cartesian_path_error_recover_by_normal_plan && cartesian_path_error_recover_by_normal_plan_)
    {
      ROS_WARN("Plan(computeCartesianPath) error. ret = %f Try normal plan.", ret);
      normal_plan_flag = true;
    }
    else
    {
      ROS_ERROR("Plan(computeCartesianPath) error. ret = %f", ret);
      return MotionPlannerRet::MOTION_PLANNER_FAILED;
    }
  }

  if (normal_plan_flag)
  {
    ROS_INFO("normal plan");

    if (goal->goal.type.type == t2_msgs::ArmPoseType::CURRENT_POSITION)
    {
      ROS_INFO("Plan goal_type = CURRENT_POSITION");
      ROS_ERROR("Not supported goal_type = CURRENT_POSITION");
      return MotionPlannerRet::MOTION_PLANNER_FAILED;
    }
    else if (goal->goal.type.type == t2_msgs::ArmPoseType::CARTESIAN_POSITION)
    {
      // set end effector pose
      ROS_INFO("Plan goal_type = CARTESIAN_POSITION");
      geometry_msgs::Pose pose = goal->goal.pose;

      publishAxisMarker(pose);

      move_group->setPoseTarget(pose);
    }
    else if (goal->goal.type.type == t2_msgs::ArmPoseType::JOINT_POSITION)
    {
      ROS_INFO("Plan goal_type = JOINT_POSITION");
      // set joint position

      for (std::size_t i = 0; i < goal->goal.joint_names.size(); i++)
      {
        goal_state.setJointPositions(goal->goal.joint_names[i], &goal->goal.joint_positions[i]);
      }

      move_group->setJointValueTarget(goal_state);
    }
    else if (goal->goal.type.type == t2_msgs::ArmPoseType::GROUP_STATE)
    {
      ROS_INFO("Plan goal_type = GROUP_STATE, name = %s", goal->goal.group_state.c_str());

      goal_state.setToDefaultValues(goal_state.getJointModelGroup(PLAN_GROUP_SUCTION), goal->goal.group_state);
      move_group->setJointValueTarget(goal_state);
    }

    bool plan_flag = false;
    std::string plan_quality;

    double max_valid_path_length_strict, max_valid_path_length_lax;

    for (int n = 0; n <= planning_retry_count; n++)
    {
      MotionPlannerPlan tmp_plan;

      moveit::planning_interface::MoveItErrorCode error_code;

      result.planning_attempt = n + 1;

      error_code = move_group->plan(tmp_plan.plan);

      if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        const trajectory_msgs::JointTrajectory& trajectory = tmp_plan.plan.trajectory_.joint_trajectory;
        double direct_distance = computeDirectDistance(move_group, trajectory);
        tmp_plan.path_length =
            computePathLength(move_group, t2_robot_model, trajectory, path_evaluate_joints_, path_length_joint_weight_);
        tmp_plan.smoothness = computePathSmoothness(move_group, t2_robot_model, trajectory, path_evaluate_joints_,
                                                    path_smooth_joint_weight_);

        if (direct_distance < max_valid_path_length_threshold_near_)
        {
          // near
          max_valid_path_length_strict = max_valid_path_length_near_strict_;
          max_valid_path_length_lax = max_valid_path_length_near_lax_;
        }
        else if (direct_distance < max_valid_path_length_threshold_far_)
        {
          // middle
          max_valid_path_length_strict = max_valid_path_length_middle_strict_;
          max_valid_path_length_lax = max_valid_path_length_middle_lax_;
        }
        else
        {
          // far
          max_valid_path_length_strict = max_valid_path_length_far_strict_;
          max_valid_path_length_lax = max_valid_path_length_far_lax_;
        }

        ROS_INFO("Plan %.3f[s], trajectory_points = %lu, path_length = %.3f, smoothness = %.3f, direct_distance = "
                 "%.3f, max_valid_path_length_strict = %.3f, max_valid_path_length_lax = %.3f",
                 tmp_plan.plan.planning_time_, trajectory.points.size(), tmp_plan.path_length, tmp_plan.smoothness,
                 direct_distance, max_valid_path_length_strict, max_valid_path_length_lax);

        if ((tmp_plan.path_length < max_valid_path_length_strict) && (tmp_plan.smoothness < max_valid_path_smoothness_))
        {
          // strict pat
          ROS_INFO("%s  Good plan(normal). Retry(%d/%d)", goal->plan_title.c_str(), n, planning_retry_count);
          plan_flag = true;
          plan_quality = "Good";
          current_plan = tmp_plan;
          break;
        }
        else if ((tmp_plan.path_length < max_valid_path_length_lax) &&
                 (tmp_plan.smoothness < max_valid_path_smoothness_))
        {
          // lax
          if (!plan_flag || current_plan.path_length > tmp_plan.path_length)
          {
            ROS_WARN("%s  Keep plan(normal). Retry(%d/%d)", goal->plan_title.c_str(), n, planning_retry_count);
            plan_flag = true;
            plan_quality = "Keep";
            current_plan = tmp_plan;
          }
          else
          {
            ROS_WARN("%s  Ignore plan(normal). Retry(%d/%d)", goal->plan_title.c_str(), n, planning_retry_count);
          }
        }
        else
        {
          // bad plan
          ROS_WARN("%s  Bad plan(normal). Retry(%d/%d)", goal->plan_title.c_str(), n, planning_retry_count);
          plan_quality = "Bad";
          result.error_msg = "Bad plan.";
        }
      }
      else if (error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN)
      {
        // invalid motion plan
        ROS_WARN("%s  Invalid plan(normal). Retry(%d/%d)", goal->plan_title.c_str(), n, planning_retry_count);
        plan_quality = "Invalid";
        result.error_msg = "Invalid plan.";
      }
      else
      {
        // no plan
        ROS_ERROR("%s  Plan(normal) failed. %s. Retry(%d/%d)", goal->plan_title.c_str(),
                  getMoveItErrorCodeStr(error_code.val).c_str(), n, planning_retry_count);
        plan_quality = "Non";
        result.error_msg = getMoveItErrorCodeStr(error_code.val);
        break;
      }
    }  // end of for loop

    if (plan_flag)
    {
      ROS_INFO("%s  Plan(normal) success. Quality = %s", goal->plan_title.c_str(), plan_quality.c_str());
    }
    else
    {
      ROS_INFO("Plan(normal) error.");

      return MotionPlannerRet::MOTION_PLANNER_FAILED;
    }
  }

  // trajectory filter
  std::vector<trajectory_msgs::JointTrajectoryPoint> filterd_points1, filterd_points2;
  trajectorySamePointFilter(current_plan.plan.trajectory_.joint_trajectory.points, filterd_points1);
  trajectoryNPointFilter(filterd_points1, filterd_points2, n_points_);
  current_plan.plan.trajectory_.joint_trajectory.points = filterd_points2;
  trajectory_msgs::JointTrajectory filtered_joint_trajectory;
  std::vector<std::string> valid_joint_names;
  if (!getJointNames(PLAN_GROUP_SUCTION, valid_joint_names))
  {
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }

  trajectoryJointNameFilter(current_plan.plan.trajectory_.joint_trajectory, filtered_joint_trajectory,
                            valid_joint_names);

  current_plan.plan.trajectory_.joint_trajectory = filtered_joint_trajectory;

  if (trajectory_collision_check_area_.size() == 5)
  {
    // trajectoryCollisionCheck
    if (!trajectoryCollisionCheck(goal->plan_group, current_plan.plan.trajectory_.joint_trajectory,
                                  trajectory_collision_check_area_[0], trajectory_collision_check_area_[1],
                                  trajectory_collision_check_area_[2], trajectory_collision_check_area_[3],
                                  trajectory_collision_check_area_[4]))
    {
      ROS_ERROR("trajectoryCollisionCheck() detected collision!!");
      ROS_INFO_STREAM(current_plan.plan.trajectory_.joint_trajectory);
      return MotionPlannerRet::MOTION_PLANNER_FAILED;
    }
  }

  // set the last joint position of trajectory
  result.goal.type.type = t2_msgs::ArmPoseType::JOINT_POSITION;
  result.goal.joint_names = current_plan.plan.trajectory_.joint_trajectory.joint_names;
  result.goal.joint_positions = current_plan.plan.trajectory_.joint_trajectory.points.back().positions;
  result.result = t2_msgs::ArmPlanResult::SUCCESS;

  current_plan.plan_group = goal->plan_group;
  arm_plans_[result.plan_id] = current_plan;

  ros::Duration past = ros::Time::now() - start_time;
  result.planning_time = past.toSec();

  return MotionPlannerRet::MOTION_PLANNER_SUCCESS;
}

MotionPlannerRet MotionPlanner::Execute(const t2_msgs::ArmExecuteGoalConstPtr& goal, t2_msgs::ArmExecuteResult& result)
{
  result.result = t2_msgs::ArmExecuteResult::FAILED;
  MotionPlannerPlan* mp_plan;

  MoveGroupPtr move_group;

  ROS_INFO("plan_id = %d", goal->plan_id);
  ArmPlanMap::iterator ite = arm_plans_.find(goal->plan_id);

  if (ite == arm_plans_.end())
  {
    ROS_ERROR("invalid plan_id = %d", goal->plan_id);
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }

  mp_plan = &ite->second;

  if (!getMoveGroup(mp_plan->plan_group, move_group))
  {
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }

  // set velocity
  trajectoryPointsAddVelocity(mp_plan->plan.trajectory_.joint_trajectory.points, goal->velocity);

#if 0
  // for DEBUG
  for (std::size_t i = 0; i < filterd_points.size(); i++)
  {
    std::string position_str = "";
    std::vector<double>& positions = filterd_points[i].positions;
    for (std::size_t j = 0; j < positions.size(); j++)
    {
      position_str += std::to_string(positions[j]) + ",";
    }
    ROS_INFO("%s", position_str.c_str());
  }
#endif

  publishTrajectoryMarker(mp_plan->plan_group, mp_plan->plan.trajectory_.joint_trajectory);

  moveit_msgs::MoveItErrorCodes error_code = executeTrajectory(move_group, mp_plan->plan);

  if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("move_group->execute() SUCCESS");

    ros::Duration(0.5).sleep();

    result.result = t2_msgs::ArmExecuteResult::SUCCESS;
    return MotionPlannerRet::MOTION_PLANNER_SUCCESS;
  }
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("move_group_->execute() PREEMPTED");

    ros::Duration(1.0).sleep();

    result.result = t2_msgs::ArmExecuteResult::SUCCESS;
    return MotionPlannerRet::MOTION_PLANNER_SUCCESS;
  }
  else
  {
    ROS_ERROR("move_group_->execute() ERROR");
  }

  return MotionPlannerRet::MOTION_PLANNER_FAILED;
}

moveit_msgs::MoveItErrorCodes MotionPlanner::executeTrajectory(MoveGroupPtr& move_group,
                                                               const moveit::planning_interface::MoveGroup::Plan& plan)
{
  moveit_msgs::MoveItErrorCodes error_code;

#ifdef USE_MOVE_GROUP_FOR_EXECUTE
  error_code = move_group->execute(plan);
#else

  follow_joint_trajectry_client_.reset(
      new FollowJointTrajectoryActionClient(ARM_CONTROLLER_FOLLOW_JOINT_TRAJECTORY_ACTION_NAME, true));

  if (!follow_joint_trajectry_client_->waitForServer(ros::Duration(1.0)))
  {
    ROS_WARN("Failed to connect ArmController.");
    error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;

    return error_code;
  }
  else
  {
    control_msgs::FollowJointTrajectoryGoal follow_joint_trajectory_goal;
    follow_joint_trajectory_goal.trajectory = plan.trajectory_.joint_trajectory;

    follow_joint_trajectry_client_->sendGoal(follow_joint_trajectory_goal, NULL, NULL,
                                             boost::bind(&MotionPlanner::cbArmControllerFeedback, this, _1));

    if (!follow_joint_trajectry_client_->waitForResult(ros::Duration(60.0)))
    {
      error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
      return error_code;
    }

    switch (follow_joint_trajectry_client_->getState().state_)
    {
      case actionlib::SimpleClientGoalState::SUCCEEDED:
        error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        break;
      case actionlib::SimpleClientGoalState::PREEMPTED:
        error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
        break;
      default:
        error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        break;
    }
  }

#endif

  return error_code;
}

bool MotionPlanner::stopTrajectory(MoveGroupPtr& move_group)
{
#ifdef USE_MOVE_GROUP_FOR_EXECUTE
  move_group->stop();
#else
  follow_joint_trajectry_client_->cancelAllGoals();
#endif
  return true;
}

MotionPlannerRet MotionPlanner::Move(const t2_msgs::ArmMoveGoalConstPtr& goal, t2_msgs::ArmMoveResult& result)
{
  ROS_INFO("##### PlanArm::Move #####");

  result.result = t2_msgs::ArmMoveResult::FAILED;

  // Plan
  t2_msgs::ArmPlanGoalPtr plan_goal(new t2_msgs::ArmPlanGoal());
  t2_msgs::ArmPlanResultPtr plan_result(new t2_msgs::ArmPlanResult());

  if (goal->start.type.type != t2_msgs::ArmPoseType::CURRENT_POSITION)
  {
    ROS_WARN("ArmMove supported only start_type = CURRENT_POSITION. Use CURRENT_POSITION.");
  }

  ROS_INFO("Move plan_group = %s", goal->plan_group.c_str());

  plan_goal->plan_title = goal->plan_title;
  plan_goal->plan_id = 0;
  plan_goal->plan_group = goal->plan_group;
  plan_goal->start = goal->start;
  plan_goal->start.type.type = t2_msgs::ArmPoseType::CURRENT_POSITION;
  plan_goal->goal = goal->goal;
  plan_goal->waypoints = goal->waypoints;
  plan_goal->cartesian_path_fraction_threshold = goal->cartesian_path_fraction_threshold;
  plan_goal->cartesian_path_error_recover_by_normal_plan = goal->cartesian_path_error_recover_by_normal_plan;
  plan_goal->stay_level = goal->stay_level;
  plan_goal->path_constraints = goal->path_constraints;
  plan_goal->goal_constraints = goal->goal_constraints;

  if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
  {
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }

  // Execute
  t2_msgs::ArmExecuteGoalPtr execute_goal(new t2_msgs::ArmExecuteGoal());
  t2_msgs::ArmExecuteResultPtr execute_result(new t2_msgs::ArmExecuteResult());
  execute_goal->plan_id = 0;
  execute_goal->velocity = goal->velocity;

  if (Execute(execute_goal, *execute_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
  {
    return MotionPlannerRet::MOTION_PLANNER_FAILED;
  }

  result.result = t2_msgs::ArmMoveResult::SUCCESS;

  return MotionPlannerRet::MOTION_PLANNER_SUCCESS;
}

std::string MotionPlanner::getMoveItErrorCodeStr(const int32_t& error_val)
{
  std::string error_str;
  switch (error_val)
  {
    case moveit::planning_interface::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE:
      error_str = "COLLISION_CHECKING_UNAVAILABLE";
      break;
    case moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED:
      error_str = "CONTROL_FAILED";
      break;
    case moveit::planning_interface::MoveItErrorCode::FAILURE:
      error_str = "FAILURE";
      break;
    case moveit::planning_interface::MoveItErrorCode::FRAME_TRANSFORM_FAILURE:
      error_str = "FRAME_TRANSFORM_FAILURE";
      break;
    case moveit::planning_interface::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
      error_str = "GOAL_CONSTRAINTS_VIOLATED";
      break;
    case moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION:
      error_str = "GOAL_IN_COLLISION";
      break;
    case moveit::planning_interface::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
      error_str = "GOAL_VIOLATES_PATH_CONSTRAINTS";
      break;
    case moveit::planning_interface::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
      error_str = "INVALID_GOAL_CONSTRAINTS";
      break;
    case moveit::planning_interface::MoveItErrorCode::INVALID_GROUP_NAME:
      error_str = "INVALID_GROUP_NAME";
      break;
    case moveit::planning_interface::MoveItErrorCode::INVALID_LINK_NAME:
      error_str = "INVALID_LINK_NAME";
      break;
    case moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN:
      error_str = "INVALID_MOTION_PLAN";
      break;
    case moveit::planning_interface::MoveItErrorCode::INVALID_OBJECT_NAME:
      error_str = "INVALID_OBJECT_NAME";
      break;
    case moveit::planning_interface::MoveItErrorCode::INVALID_ROBOT_STATE:
      error_str = "INVALID_ROBOT_STATE";
      break;
    case moveit::planning_interface::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      error_str = "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
      break;
    case moveit::planning_interface::MoveItErrorCode::NO_IK_SOLUTION:
      error_str = "NO_IK_SOLUTION";
      break;
    case moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED:
      error_str = "PLANNING_FAILED";
      break;
    case moveit::planning_interface::MoveItErrorCode::PREEMPTED:
      error_str = "PREEMPTED";
      break;
    case moveit::planning_interface::MoveItErrorCode::ROBOT_STATE_STALE:
      error_str = "ROBOT_STATE_STALE";
      break;
    case moveit::planning_interface::MoveItErrorCode::SENSOR_INFO_STALE:
      error_str = "SENSOR_INFO_STALE";
      break;
    case moveit::planning_interface::MoveItErrorCode::START_STATE_IN_COLLISION:
      error_str = "START_STATE_IN_COLLISION";
      break;
    case moveit::planning_interface::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      error_str = "START_STATE_VIOLATES_PATH_CONSTRAINTS";
      break;
    case moveit::planning_interface::MoveItErrorCode::SUCCESS:
      error_str = "SUCCESS";
      break;
    case moveit::planning_interface::MoveItErrorCode::TIMED_OUT:
      error_str = "TIMED_OUT";
      break;
    case moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
      error_str = "UNABLE_TO_AQUIRE_SENSOR_DATA";
      break;
    default:
      error_str = "UNKNOWN";
      break;
  }
  return error_str;
}

bool MotionPlanner::setGripperSuction(const int32_t suction, const double threshold)
{
  ros::ServiceClient gripper_suction_client = nh_.serviceClient<t2_msgs::GripperSuction>(GRIPPER_SUCTION_SERVICE_NAME);

  gripper_suction_flag_ = suction;

  t2_msgs::GripperSuction gripper_suction_srv;

  gripper_suction_srv.request.suction = suction;
  gripper_suction_srv.request.threshold_of_vaccum = threshold;

  if (suction == t2_msgs::GripperSuctionRequest::SUCTION_OFF)
  {
    // Clear attached item
    t2_msgs::ClearAttachedItem cai_srv;
    if (!clear_attached_item_client_.call(cai_srv))
    {
      ROS_ERROR("Failed to call clear_attached_item_client");
    }

    publishSuctionStateMarker(false);
  }
  else
  {
    publishSuctionStateMarker(true);
  }

  if (!gripper_suction_client.exists())
  {
    ROS_ERROR("gripper_suction service is not exist.");
    return false;
  }

  if (!gripper_suction_client.call(gripper_suction_srv))
  {
    ROS_ERROR("Failed to call gripper_suction service.");
    return false;
  }

  if (gripper_suction_srv.response.result != t2_msgs::GripperSuctionResponse::SUCCESS)
  {
    ROS_ERROR("gripper_suction service return error.");
    return false;
  }

  return true;
}

void MotionPlanner::cbArmExecuteActionServerGoal(const t2_msgs::ArmExecuteGoalConstPtr& goal)
{
  ROS_INFO("ArmExecuteAction Start.");

  t2_msgs::ArmExecuteResult result;

  if (Execute(goal, result) == MotionPlannerRet::MOTION_PLANNER_SUCCESS)
  {
    arm_execute_action_server_->setSucceeded(result);
    ROS_INFO("ArmExecuteAction Succeeded.");
    return;
  }
  else
  {
    arm_execute_action_server_->setAborted(result);
    ROS_INFO("ArmExecuteAction Aborted.");
    return;
  }
}

void MotionPlanner::cbArmMoveActionServerGoal(const t2_msgs::ArmMoveGoalConstPtr& goal)
{
  ROS_INFO("ArmMoveAction Start.");

  t2_msgs::ArmMoveResult result;

  if (Move(goal, result) == MotionPlannerRet::MOTION_PLANNER_SUCCESS)
  {
    arm_move_action_server_->setSucceeded(result);
    ROS_INFO("ArmMoveAction Succeeded.");
  }
  else
  {
    arm_move_action_server_->setAborted(result);
    ROS_INFO("ArmMoveAction Aborted.");
  }
}

void MotionPlanner::cbArmPlanActionServerGoal(const t2_msgs::ArmPlanGoalConstPtr& goal)
{
  ROS_INFO("ArmPlanAction Start.");

  t2_msgs::ArmPlanResult result;
  result.result = t2_msgs::ArmPlanResult::FAILED;

  if (goal->plan_id == 0)
  {
    ROS_ERROR("plan_id should start numbering with the number 1.");
    arm_plan_action_server_->setAborted(result);
  }

  if (Plan(goal, result) == MotionPlannerRet::MOTION_PLANNER_SUCCESS)
  {
    arm_plan_action_server_->setSucceeded(result);
  }
  else
  {
    arm_plan_action_server_->setAborted(result);
  }
}

void MotionPlanner::cbArmPickPlanActionServerGoal(const t2_msgs::ArmPickPlanGoalConstPtr& goal)
{
  ROS_INFO("cbArmPickPlanActionServerGoal");

  ros::Time start_time = ros::Time::now();

  t2_msgs::ArmPickPlanResult result;
  result.result = t2_msgs::ArmPickPlanResult::FAILED;
  result.planning_attempt = 0;

  std::string plan_group;
  MoveGroupPtr move_group;

  if (goal->grasp_point.grasp_pattern == GRASP_PATTERN_SUCTION)
  {
    plan_group = PLAN_GROUP_SUCTION;
  }
  else if (goal->grasp_point.grasp_pattern == GRASP_PATTERN_PINCH)
  {
    plan_group = PLAN_GROUP_PINCH;
  }

  if (!getMoveGroup(plan_group, move_group))
  {
    return;
  }

  // Seq plan mutex
  {
    std::lock_guard<std::mutex> lock(seq_plan_mutex_);

    PickPlan pick_plan;

    // ****************************************
    //  Plan (GAP1 -> GAP2 -> GP)
    // ****************************************
    {
      t2_msgs::SetPickItemToPlanningScene sbp_srv;
      sbp_srv.request.place_id = goal->place_id;
      sbp_srv.request.cad_id = goal->cad_id;
      sbp_srv.request.job_no = goal->job_no;
      sbp_srv.request.recog_id = goal->recog_id;
      sbp_srv.request.grasp_point = goal->grasp_point;

      t2_msgs::ArmPlanGoalPtr plan_goal(new t2_msgs::ArmPlanGoal());
      t2_msgs::ArmPlanResultPtr plan_result(new t2_msgs::ArmPlanResult());

      // plan type
      plan_goal->plan_type = t2_msgs::ArmPlanGoal::PLAN_PICK;

      // path constraints (position)
      moveit_msgs::PositionConstraint position_constraint;
      std::vector<uint32_t> place_id = { goal->place_id };
      getPositionConstraints(move_group->getEndEffectorLink(), place_id, 1.0, position_constraint);
      plan_goal->path_constraints.position_constraints.push_back(position_constraint);

      if (goal->grasp_point.grasp_pattern == GRASP_PATTERN_PINCH)
      {
        plan_goal->gripper_state = t2_msgs::ArmPlanGoal::BUTTING_STATE;
      }

      // Plan GAP1 -> GAP2
      plan_goal->plan_title = "### Plan GAP1 -> GAP2";
      plan_goal->plan_id = 1001;
      plan_goal->plan_group = plan_group;
      plan_goal->grasp_pattern = goal->grasp_point.grasp_pattern;

      // set GAP1
      plan_goal->start = goal->goal;

      // set GAP2
      plan_goal->goal.type.type = t2_msgs::ArmPoseType::CARTESIAN_POSITION;
      plan_goal->goal.pose = goal->grasp_point.approach_point_item;

      plan_goal->stay_level = false;

      // set ACM
      sbp_srv.request.operation = sbp_srv.request.APPROACH;
      if (!set_pick_item_to_planning_scene_client_.call(sbp_srv))
      {
        ROS_ERROR("Failed to set ACM");
        result.error_msg = "Failed to set ACM(APPROACH)";
        arm_pick_plan_action_server_->setAborted(result);
        return;
      }

      if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
      {
        ROS_INFO("ArmPickPlanAction plan GAP1 -> GAP2 ERROR");
        result.error_msg = "GAP1 -> GAP2 ERROR (" + plan_result->error_msg + ")";
        arm_pick_plan_action_server_->setAborted(result);
        return;
      }

      result.planning_attempt += plan_result->planning_attempt;
      plan_goal->goal_constraints = moveit_msgs::Constraints();

      // Plan GAP2 -> GP
      plan_goal->plan_title = "### Plan GAP2 -> GP";
      plan_goal->plan_id = 1002;

      plan_goal->start = plan_result->goal;
      plan_goal->goal.type.type = t2_msgs::ArmPoseType::CARTESIAN_POSITION;
      plan_goal->goal.pose = goal->grasp_point.grasp_point_item;

      plan_goal->stay_level = false;

      // set ACM
      sbp_srv.request.operation = sbp_srv.request.APPROACH_GP;
      if (!set_pick_item_to_planning_scene_client_.call(sbp_srv))
      {
        ROS_ERROR("Failed to set ACM");
        result.error_msg = "Failed to set ACM(APPROACH_GP)";
        arm_pick_plan_action_server_->setAborted(result);
        return;
      }

      if (cartesian_path_approach_)
      {
        // Cartesian path
        plan_goal->waypoints.clear();
        plan_goal->waypoints.push_back(plan_goal->goal.pose);  // GP
        plan_goal->cartesian_path_fraction_threshold = cartesian_path_approach_fraction_threshold_;
        plan_goal->cartesian_path_error_recover_by_normal_plan = false;
      }

      if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
      {
        ROS_INFO("ArmPickPlanAction plan GAP2 -> GP ERROR");
        result.error_msg = "GAP2 -> GP ERROR (" + plan_result->error_msg + ")";
        arm_pick_plan_action_server_->setAborted(result);
        return;
      }

      result.planning_attempt += plan_result->planning_attempt;

      plan_goal->waypoints.clear();
      plan_goal->goal_constraints = moveit_msgs::Constraints();

      // set ACM
      sbp_srv.request.operation = sbp_srv.request.PLAN_END;
      if (!set_pick_item_to_planning_scene_client_.call(sbp_srv))
      {
        ROS_ERROR("Failed to set ACM");
        result.error_msg = "Failed to set ACM(PLAN_END)";
        arm_pick_plan_action_server_->setAborted(result);
        return;
      }
    }

    // **************************************************
    //  Combine plans (GAP1 -> GAP2 -> GP)
    // **************************************************
    {
      std::vector<int> plan_id = { 1001, 1002 };
      std::vector<float> velocity = { goal->gap2_velocity, goal->gp_velocity };

      pick_plan.plan_goal = *goal;
      pick_plan.plan_group = plan_group;

      combinePlanTrajectory(plan_id, velocity, pick_plan.plan.trajectory_.joint_trajectory);
    }

    // **************************************************
    //  Back plan (GAP2 -> GAP1) : For error recovery
    // **************************************************
    {
      int plan_id = 1001;

      ArmPlanMap::iterator ite = arm_plans_.find(plan_id);

      if (ite == arm_plans_.end())
      {
        ROS_ERROR("invalid plan_id = %d", plan_id);
        result.error_msg = "Back plan : invalid plan_id";
        arm_pick_plan_action_server_->setAborted(result);
        return;
      }

      trajectory_msgs::JointTrajectory back_plan_trajectory = ite->second.plan.trajectory_.joint_trajectory;

      // reverse trajectory
      std::reverse(back_plan_trajectory.points.begin(), back_plan_trajectory.points.end());

      trajectoryPointsAddVelocity(back_plan_trajectory.points, goal->back_gap1_velocity);

      pick_plan.back_plan.trajectory_.joint_trajectory = back_plan_trajectory;
    }

    arm_pick_plans_[goal->plan_id] = pick_plan;
  }

  result.result = t2_msgs::ArmPickPlanResult::SUCCESS;

  ros::Duration past = ros::Time::now() - start_time;
  result.planning_time = past.toSec();

  arm_pick_plan_action_server_->setSucceeded(result);

  ROS_INFO("End of ArmPickPlan()");
}

void MotionPlanner::cbArmPickExecuteActionServerGoal(const t2_msgs::ArmPickExecuteGoalConstPtr& goal)
{
  ROS_INFO("cbArmPickExecuteActionServerGoal");

  t2_msgs::ArmPickExecuteResult result;
  result.result = t2_msgs::ArmPickExecuteResult::FAILED;

  MoveGroupPtr move_group;

  ROS_INFO("plan_id = %d", goal->plan_id);
  ArmPickPlanMap::iterator ite = arm_pick_plans_.find(goal->plan_id);

  if (ite == arm_pick_plans_.end())
  {
    ROS_ERROR("invalid plan_id = %d", goal->plan_id);
    result.error_msg = "invalid plan_id";
    arm_pick_execute_action_server_->setAborted(result);
    return;
  }

  pick_execute_plan_ = &ite->second;

  if (!getMoveGroup(pick_execute_plan_->plan_group, move_group))
  {
    result.error_msg = "getMoveGroup error";
    arm_pick_execute_action_server_->setAborted(result);
    return;
  }

  t2_msgs::SetPickItemToPlanningScene sbp_srv;
  sbp_srv.request.place_id = pick_execute_plan_->plan_goal.place_id;
  sbp_srv.request.cad_id = pick_execute_plan_->plan_goal.cad_id;
  sbp_srv.request.job_no = pick_execute_plan_->plan_goal.job_no;
  sbp_srv.request.recog_id = pick_execute_plan_->plan_goal.recog_id;
  sbp_srv.request.grasp_point = pick_execute_plan_->plan_goal.grasp_point;

  moveit::planning_interface::MoveGroup::Plan plan;

  // Seq plan mutex
  {
    std::lock_guard<std::mutex> lock(seq_plan_mutex_);

    t2_msgs::ArmPlanGoalPtr plan_goal(new t2_msgs::ArmPlanGoal());
    t2_msgs::ArmPlanResultPtr plan_result(new t2_msgs::ArmPlanResult());

    plan_goal->plan_group = pick_execute_plan_->plan_group;

    // ****************************************
    //  Check current position
    // ****************************************

    bool plan_gap1_flag = true;

    if (pick_execute_plan_->plan_goal.goal.type.type == t2_msgs::ArmPoseType::GROUP_STATE)
    {
      t2_msgs::GetArmPose get_arm_pose;
      get_arm_pose.request.plan_group = pick_execute_plan_->plan_group;
      cbGetArmPoseService(get_arm_pose.request, get_arm_pose.response);

      t2_msgs::GetArmGroupStatePose get_arm_group_state_pose;
      get_arm_group_state_pose.request.plan_group = PLAN_GROUP_SUCTION;
      get_arm_group_state_pose.request.group_state = pick_execute_plan_->plan_goal.goal.group_state;
      cbGetArmGroupStatePoseService(get_arm_group_state_pose.request, get_arm_group_state_pose.response);

      if (isEqualJointPositions(get_arm_pose.response.pose.joint_names, get_arm_pose.response.pose.joint_positions,
                                get_arm_group_state_pose.response.pose.joint_names,
                                get_arm_group_state_pose.response.pose.joint_positions, compare_joint_tolerance_))
      {
        ROS_WARN("current position is GAP1. Skip current -> GAP1 plan.");
        plan_gap1_flag = false;
      }
    }

    if (plan_gap1_flag)
    {
      // ****************************************
      //  Plan (start -> GAP1)
      // ****************************************

      // path constraints (position)
      moveit_msgs::PositionConstraint position_constraint;
      std::vector<uint32_t> place_id = { goal->place_id };
      getPositionConstraints(move_group->getEndEffectorLink(), place_id, 1.0, position_constraint);
      plan_goal->path_constraints.position_constraints.push_back(position_constraint);

      // Plan start pos -> GAP1
      plan_goal->plan_title = "### Plan start pos -> GAP1";
      plan_goal->plan_id = 1000;
      plan_goal->start = goal->start;
      plan_goal->goal = pick_execute_plan_->plan_goal.goal;

      plan_goal->stay_level = goal->stay_level;

      // set ACM
      sbp_srv.request.operation = sbp_srv.request.FAR_MOVE;
      set_pick_item_to_planning_scene_client_.call(sbp_srv);

      if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
      {
        ROS_INFO("ArmPickPlanAction plan current pos -> GAP1 ERROR");
        result.error_msg = "current pos -> GAP1 ERROR (" + plan_result->error_msg + ")";
        arm_pick_execute_action_server_->setAborted(result);
        return;
      }

      // result.planning_attempt += plan_result->planning_attempt;

      // set ACM
      sbp_srv.request.operation = sbp_srv.request.PLAN_END;
      if (!set_pick_item_to_planning_scene_client_.call(sbp_srv))
      {
        ROS_ERROR("Failed to set ACM");
        result.error_msg = "Failed to set ACM(PLAN_END)";
        arm_pick_execute_action_server_->setAborted(result);
        return;
      }

      // **************************************************
      //  Combine plans (start -> GAP1 -> GAP2 -> GP)
      // **************************************************

      if (!combinePickPlanTrajectory(plan_goal->plan_id, goal->gap1_velocity, goal->plan_id,
                                     plan.trajectory_.joint_trajectory))
      {
        ROS_ERROR("Failed to call combinePickPlanTrajectory");
        result.error_msg = "Failed to call combinePickPlanTrajectory";
        arm_pick_execute_action_server_->setAborted(result);
        return;
      }
    }
    else
    {
      // **************************************************
      //  Get pick plan (GAP1 -> GAP2 -> GP)
      // **************************************************

      ArmPickPlanMap::iterator pick_plan_ite = arm_pick_plans_.find(goal->plan_id);

      if (pick_plan_ite == arm_pick_plans_.end())
      {
        ROS_ERROR("invalid pick_plan_id = %d", goal->plan_id);
        result.error_msg = "invalid pick_plan_id";
        arm_pick_execute_action_server_->setAborted(result);
        return;
      }

      plan = pick_plan_ite->second.plan;
    }
  }

  // **************************************************
  //  Execute plans (current -> GAP1 -> GAP2 -> GP)
  // **************************************************

  publishTrajectoryMarker(pick_execute_plan_->plan_group, plan.trajectory_.joint_trajectory, 1);

  updateChekingRegion();

#ifdef USE_WEIGHT_SCALE
  if (weightScaleExecute(goal->place_id))
  {
    pick_execute_plan_->weight_scale_flag = true;
  }
  else
  {
    ROS_WARN("Failed to weightScaleExecute");

    pick_execute_plan_->weight_scale_flag = false;
  }
#else
  pick_execute_plan_->weight_scale_flag = false;
#endif

  // flag reset
  gripper_finger_contact_state_flag_ = 0;
  pad_contact_state_flag_ = 0;
  suction_state_flag_ = 0;

  // Clear attached item
  t2_msgs::ClearAttachedItem cai_srv;
  if (!clear_attached_item_client_.call(cai_srv))
  {
    ROS_ERROR("Failed to call clear_attached_item_client");
  }

  // execute
  if (pick_execute_plan_->plan_goal.grasp_point.grasp_pattern == GRASP_PATTERN_SUCTION)
  {
    gripper_retract_client_.reset(new GripperRetractActionClient(
        t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::RETRACT_ACTION_NAME, true));
    if (!gripper_retract_client_->waitForServer(ros::Duration(1.0)))
    {
      ROS_WARN("Failed to connect t2_pinching_gripper. GripperRetract");
      result.error_msg = "Failed to connect t2_pinching_gripper. GripperRetract";
      arm_pick_execute_action_server_->setAborted(result);
    }
    else
    {
      t2_msgs::GripperRetractGoal gripper_retract_goal;  // empty
      gripper_retract_client_->sendGoalAndWait(gripper_retract_goal);

      // set gripper joint
      gripper_joint_positions_ = gripper_retract_joint_positions_;
    }
  }
  else if (pick_execute_plan_->plan_goal.grasp_point.grasp_pattern == GRASP_PATTERN_PINCH)
  {
    // Gripper extend
    gripper_extend_client_.reset(new GripperExtendActionClient(
        t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::EXTEND_ACTION_NAME, true));

    if (!gripper_extend_client_->waitForServer(ros::Duration(1.0)))
    {
      ROS_WARN("Failed to connect t2_pinching_gripper. GripperExtend");
      result.error_msg = "Failed to connect t2_pinching_gripper. GripperExtend";
      arm_pick_execute_action_server_->setAborted(result);
      return;
    }
    else
    {
      t2_msgs::GripperExtendGoal gripper_extend_goal;  // empty
      gripper_extend_client_->sendGoal(gripper_extend_goal);

      // set gripper joint
      gripper_joint_positions_ = gripper_butting_joint_positions_;
    }
  }

  arm_approach_item_flag_ = true;
  moveit_msgs::MoveItErrorCodes error_code = executeTrajectory(move_group, plan);
  arm_approach_item_flag_ = false;

  if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("move_group->execute() SUCCESS");
  }
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("move_group_->execute() PREEMPTED");
  }
  else
  {
    ROS_ERROR("move_group_->execute() ERROR");
    setGripperSuction(t2_msgs::GripperSuctionRequest::SUCTION_OFF);
  }

  if (pick_execute_plan_->plan_goal.grasp_point.grasp_pattern == GRASP_PATTERN_SUCTION)
  {
    // Wait gripper suction sensor

    ros::Duration suction_timeout(suction_sensor_waiting_time_);
    ros::Time rc_result_begin(0);

    ROS_INFO("Waiting gripper suction sensor until timeout(%.3f[s]).", suction_sensor_waiting_time_);

    rc_result_begin = ros::Time::now();

    while (ros::ok())
    {
      if (arm_pick_execute_action_server_->isPreemptRequested())
      {
        ROS_INFO("ArmPickExecuteAction suction setPreempted()");
        arm_pick_execute_action_server_->setPreempted();
        return;
      }

      if (!arm_pick_execute_action_server_->isActive())
      {
        ROS_INFO("ArmPickExecuteAction suction Finished.");
        // setGripperSuction(t2_msgs::GripperSuctionRequest::SUCTION_OFF);
        return;
      }

      if (suction_state_flag_ >= 1)
      {
        // suction sensor detect after arm moved
        // result.result = t2_msgs::ArmPickResult::SUCCESS;
        // arm_pick_move_action_server_->setSucceeded(result);
        ROS_INFO("ArmPickExecuteAction suction Succeeded.");
        pick_execute_plan_->pick_state = true;
        break;
      }

      if (ros::Time::now() - rc_result_begin > suction_timeout)
      {
        // arm trajectory is finished without suction sensor detection

        pick_execute_plan_->pick_state = false;

        ROS_INFO("ArmPickExecuteAction suction timeout. setGripperSuction(OFF)");
        result.error_msg = "suction timeout.";

        if (!setGripperSuction(t2_msgs::GripperSuctionRequest::SUCTION_OFF))
        {
          result.error_msg = "suction timeout. setGripperSuction(OFF) error";
        }

        goto ERROR_RECOVERY;
      }

      ros::Duration(0.1).sleep();
    }  // while
  }
  else if (pick_execute_plan_->plan_goal.grasp_point.grasp_pattern == GRASP_PATTERN_PINCH)
  {
    // Gripper pinch
    gripper_pinch_client_.reset(new GripperPinchActionClient(
        t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::PINCH_ACTION_NAME, true));
    t2_msgs::GripperPinchGoal gripper_pinch_goal;
    gripper_pinch_goal.opening_width = pick_execute_plan_->plan_goal.grasp_point.width_between_finger_for_pinch;
    gripper_pinch_goal.extrusion =
        gripper_detect_extend_pos_ + pick_execute_plan_->plan_goal.grasp_point.finger_intrusion_for_pinch;
    gripper_pinch_goal.max_effort = pick_execute_plan_->plan_goal.grasp_point.max_effort_for_pinch;

    if (!gripper_pinch_client_->waitForServer(ros::Duration(1.0)))
    {
      ROS_WARN("Failed to connect t2_pinching_gripper. GripperPinch");
      result.error_msg = "Failed to connect t2_pinching_gripper. GripperPinch";
      goto ERROR_RECOVERY;
    }
    else
    {
      gripper_pinch_client_->sendGoal(gripper_pinch_goal, NULL, NULL,
                                      boost::bind(&MotionPlanner::cbGripperPinchFeedback, this, _1));

      if (!gripper_pinch_client_->waitForResult(ros::Duration(10.0)))
      {
        result.error_msg = "GripperPinch result timeout.";
        goto ERROR_RECOVERY;
      }

      if (gripper_pinch_client_->getResult()->result != t2_msgs::GripperPinchResult::SUCCESS)
      {
        result.error_msg = "GripperPinch result error.";
        goto ERROR_RECOVERY;
      }
    }
  }

  // SUCCESS:
  // set ACM
  sbp_srv.request.operation = sbp_srv.request.ATTACH;
  set_pick_item_to_planning_scene_client_.call(sbp_srv);

  result.result = t2_msgs::ArmPickExecuteResult::SUCCESS;
  arm_pick_execute_action_server_->setSucceeded(result);
  ROS_INFO("End of ArmPickExecute() Success");
  return;

ERROR_RECOVERY:
  // back to GAP4
  ROS_WARN("PickExecute ERROR_RECOVERY: Failed to Pick. Back to GAP4. (GAP2 -> GAP4(GAP1))");

  moveit_msgs::MoveItErrorCodes execute_error_code;

  // ***************************************************************************
  //  Execute plans (GAP2 -> GAP4(GAP1))
  // ***************************************************************************

  moveit::planning_interface::MoveGroup::Plan back_plan = pick_execute_plan_->back_plan;

  // insert GP' to back plan
  t2_msgs::GetArmPoseRequest arm_pose_req;
  t2_msgs::GetArmPoseResponse arm_pose_res;
  arm_pose_req.plan_group = pick_execute_plan_->plan_group;
  cbGetArmPoseService(arm_pose_req, arm_pose_res);
  std::vector<trajectory_msgs::JointTrajectoryPoint>& trj_point = back_plan.trajectory_.joint_trajectory.points;
  trajectory_msgs::JointTrajectoryPoint gp_dash_point;
  gp_dash_point.positions = arm_pose_res.pose.joint_positions;
  gp_dash_point.velocities = trj_point.front().velocities;
  trj_point.insert(trj_point.begin(), gp_dash_point);

  publishTrajectoryMarker(pick_execute_plan_->plan_group, back_plan.trajectory_.joint_trajectory);

  execute_error_code = executeTrajectory(move_group, back_plan);

  if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("move_group->execute() SUCCESS");
  }
  else if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("move_group_->execute() PREEMPTED");
  }
  else
  {
    ROS_ERROR("move_group_->execute() ERROR");
  }

  arm_pick_execute_action_server_->setAborted(result);

  ROS_INFO("End of ArmPickExecute() Aborted");
  return;
}

void MotionPlanner::cbArmPlaceMoveActionServerGoal(const t2_msgs::ArmPlaceMoveGoalConstPtr& goal)
{
  ROS_INFO("cbArmPlaceMoveActionServerGoal");

  ros::Time start_time = ros::Time::now();

  t2_msgs::ArmPlaceMoveResult result;
  result.result = t2_msgs::ArmPlaceMoveResult::PLAN_ERROR;
  result.planning_attempt = 0;

  double item_weight, actual_weight;
  std::string item_name;

  bool use_weight = true;
  bool pick_result = pick_execute_plan_->pick_state;
  bool invalid_item = false;

  std::string plan_group;
  MoveGroupPtr move_group;

  t2_msgs::GraspPoint& grasp_point = pick_execute_plan_->plan_goal.grasp_point;
  std::string& grasp_pattern = grasp_point.grasp_pattern;

  if (grasp_pattern == GRASP_PATTERN_SUCTION)
  {
    plan_group = PLAN_GROUP_SUCTION;
  }
  else if (grasp_pattern == GRASP_PATTERN_PINCH)
  {
    plan_group = PLAN_GROUP_PINCH;
  }

  if (!getMoveGroup(plan_group, move_group))
  {
    result.result = t2_msgs::ArmPlaceMoveResult::PLAN_ERROR;
    result.error_msg = "getMoveGroup error";
    arm_place_move_action_server_->setAborted(result);
    return;
  }

  t2_msgs::ArmPose current_pose;

  moveit_msgs::MoveItErrorCodes execute_error_code;
  moveit::planning_interface::MoveGroup::Plan plan_rp, plan_rap4, back_plan_gap3, back_plan_gap4;

  t2_msgs::ArmPlanGoalPtr plan_goal(new t2_msgs::ArmPlanGoal());
  t2_msgs::ArmPlanResultPtr plan_result(new t2_msgs::ArmPlanResult());

  t2_msgs::SetPickItemToPlanningScene sbp_srv;
  sbp_srv.request.place_id = pick_execute_plan_->plan_goal.place_id;
  sbp_srv.request.cad_id = pick_execute_plan_->plan_goal.cad_id;
  sbp_srv.request.job_no = pick_execute_plan_->plan_goal.job_no;
  sbp_srv.request.recog_id = pick_execute_plan_->plan_goal.recog_id;
  sbp_srv.request.grasp_point = pick_execute_plan_->plan_goal.grasp_point;

  // clear item release pose
  {
    visualization_msgs::Marker mesh_resource;
    mesh_resource.header.frame_id = BASE_FRAME;
    mesh_resource.header.stamp = ros::Time::now();
    mesh_resource.ns = "place_item_mesh";
    mesh_resource.action = visualization_msgs::Marker::DELETE;
    mesh_resource.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_resource.id = 1;
    marker_pub_.publish(mesh_resource);

    for (int i = 0; i < 3; i++)
    {
      mesh_resource.id = i;
      marker_pub_.publish(mesh_resource);
    }
  }

  // publish item release pose
  t2_msgs::GetAttachedItemFromPlanningScene gap_srv;
  if (get_attached_item_client_.call(gap_srv))
  {
    Eigen::Affine3d eigen_attach_pose, eigen_rp_pose;
    geometry_msgs::Pose mesh_pose;
    tf::poseMsgToEigen(gap_srv.response.pose, eigen_attach_pose);
    tf::poseMsgToEigen(goal->rp_pose, eigen_rp_pose);
    tf::poseEigenToMsg(eigen_rp_pose * eigen_attach_pose, mesh_pose);
    visualization_msgs::Marker mesh_resource;
    mesh_resource.header.frame_id = BASE_FRAME;
    mesh_resource.header.stamp = ros::Time::now();
    mesh_resource.ns = "place_item_mesh";
    mesh_resource.id = 0;
    mesh_resource.action = visualization_msgs::Marker::ADD;
    mesh_resource.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_resource.mesh_resource = gap_srv.response.mesh_resource;
    mesh_resource.scale.x = mesh_resource.scale.y = mesh_resource.scale.z = 1.0;
    mesh_resource.pose = mesh_pose;
    marker_pub_.publish(mesh_resource);

    if (grasp_pattern == GRASP_PATTERN_PINCH)
    {
      tf::StampedTransform transform_gripper_lower, transform_gripper_upper;
      tf_listener_.lookupTransform("gripper_tool_frame", "gripper_lower_finger_link", ros::Time(0),
                                   transform_gripper_lower);
      tf_listener_.lookupTransform("gripper_tool_frame", "gripper_upper_finger_link", ros::Time(0),
                                   transform_gripper_upper);

      geometry_msgs::Pose gripper_lower_pose, gripper_upper_pose;
      Eigen::Affine3d eigen_transform_gripper_lower, eigen_transform_gripper_upper;
      tf::transformTFToEigen(transform_gripper_lower, eigen_transform_gripper_lower);
      tf::transformTFToEigen(transform_gripper_upper, eigen_transform_gripper_upper);
      tf::poseEigenToMsg(eigen_rp_pose * eigen_transform_gripper_lower, gripper_lower_pose);
      tf::poseEigenToMsg(eigen_rp_pose * eigen_transform_gripper_upper, gripper_upper_pose);

      mesh_resource.scale.x = mesh_resource.scale.y = mesh_resource.scale.z = 0.001;

      mesh_resource.id = 1;
      mesh_resource.mesh_resource = "package://t2_description/mesh/visual/eef4_gripper_lower.stl";
      mesh_resource.pose = gripper_lower_pose;
      marker_pub_.publish(mesh_resource);

      mesh_resource.id = 2;
      mesh_resource.mesh_resource = "package://t2_description/mesh/visual/eef4_gripper_upper.stl";
      mesh_resource.pose = gripper_upper_pose;
      marker_pub_.publish(mesh_resource);
    }
    else if (grasp_pattern == GRASP_PATTERN_SUCTION)
    {
      tf::StampedTransform transform_eef2;
      tf_listener_.lookupTransform("suction_nozzle_tip_frame", "rs20n_eef2", ros::Time(0), transform_eef2);

      geometry_msgs::Pose eef2_pose;
      Eigen::Affine3d eigen_transform_eef2;
      tf::transformTFToEigen(transform_eef2, eigen_transform_eef2);
      tf::poseEigenToMsg(eigen_rp_pose * eigen_transform_eef2 * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()),
                         eef2_pose);

      mesh_resource.scale.x = mesh_resource.scale.y = mesh_resource.scale.z = 0.001;

      mesh_resource.id = 1;
      mesh_resource.mesh_resource = "package://t2_description/mesh/visual/eef4_suction_pad.stl";
      mesh_resource.pose = eef2_pose;
      marker_pub_.publish(mesh_resource);
    }
  }

  // Seq plan mutex
  {
    std::lock_guard<std::mutex> lock(seq_plan_mutex_);

    plan_goal->plan_group = plan_group;

    // ******************************************************************************************
    //  Plan (current(GP') -> GAP3 -> GAP4(GAP1) -> RAP1 -> RAP2 -> RP -> RAP3 -> RAP4(RAP1))
    // ******************************************************************************************

    // path constraints (position)
    moveit_msgs::PositionConstraint position_constraint;
    std::vector<uint32_t> place_id = { pick_execute_plan_->plan_goal.place_id, goal->goal_place_id };
    getPositionConstraints(move_group->getEndEffectorLink(), place_id, 1.0, position_constraint);
    plan_goal->path_constraints.position_constraints.push_back(position_constraint);

    // set ACM
    sbp_srv.request.operation = sbp_srv.request.FAR_MOVE_GP;
    if (!set_pick_item_to_planning_scene_client_.call(sbp_srv))
    {
      ROS_ERROR("Failed to set ACM");
      result.error_msg = "Failed to set ACM(FAR_MOVE_GP)";
      goto ERROR_RECOVERY;
    }

    if (grasp_pattern == GRASP_PATTERN_PINCH)
    {
      plan_goal->gripper_state = t2_msgs::ArmPlanGoal::CURRENT_STATE;
    }

    // Plan current(GP') -> GAP3
    plan_goal->plan_title = "### Plan current(GP') -> GAP3";
    plan_goal->plan_id = 1003;

    t2_msgs::GetArmPoseRequest arm_pose_req;
    t2_msgs::GetArmPoseResponse arm_pose_res;
    arm_pose_req.plan_group = plan_group;
    cbGetArmPoseService(arm_pose_req, arm_pose_res);

    current_pose = arm_pose_res.pose;

    plan_goal->start = arm_pose_res.pose;
    plan_goal->goal.type.type = t2_msgs::ArmPoseType::CARTESIAN_POSITION;
    plan_goal->goal.pose = arm_pose_res.pose.pose;
    plan_goal->goal.pose.position.z += goal->gp_lift_up_length;
    plan_goal->stay_level = false;

    if (cartesian_path_approach_)
    {
      // Cartesian path
      plan_goal->waypoints.clear();
      plan_goal->waypoints.push_back(plan_goal->goal.pose);  // GAP3
      plan_goal->cartesian_path_fraction_threshold = 1.0;
      plan_goal->cartesian_path_error_recover_by_normal_plan = true;
    }

    if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
    {
      ROS_INFO("ArmPlaceMoveAction plan GP' -> GAP3 ERROR");
      result.error_msg = "GP' -> GAP3 ERROR (" + plan_result->error_msg + ")";
      goto ERROR_RECOVERY;
    }

    result.planning_attempt += plan_result->planning_attempt;

    plan_goal->waypoints.clear();

    // Plan GAP3 -> GAP4(GAP1)
    plan_goal->plan_title = "### Plan GAP3 -> GAP4(GPA1)";
    plan_goal->plan_id = 1004;

    plan_goal->start = plan_result->goal;
    plan_goal->goal = goal->gap4_goal;

    plan_goal->stay_level = false;

    if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
    {
      ROS_INFO("ArmPlaceMoveAction plan GAP3 -> GAP4(GAP1) ERROR");
      result.error_msg = "GAP3 -> GAP4(GAP1) ERROR (" + plan_result->error_msg + ")";
      goto ERROR_RECOVERY;
    }

    result.planning_attempt += plan_result->planning_attempt;

    // Plan GAP4(GAP1) -> RAP1
    plan_goal->plan_title = "### Plan GAP4(GAP1) -> RAP1";
    plan_goal->plan_id = 2000;
    plan_goal->start = plan_result->goal;
    plan_goal->goal = goal->rap1_goal;

    plan_goal->stay_level = goal->stay_level;

    // set ACM
    sbp_srv.request.operation = sbp_srv.request.FAR_MOVE;
    if (!set_pick_item_to_planning_scene_client_.call(sbp_srv))
    {
      ROS_ERROR("Failed to set ACM");
      goto ERROR_RECOVERY;
    }

    if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
    {
      ROS_INFO("ArmPlaceMoveAction plan current pos -> RAP1 ERROR");
      result.error_msg = "current pos -> RAP1 ERROR (" + plan_result->error_msg + ")";
      goto ERROR_RECOVERY;
    }

    result.planning_attempt += plan_result->planning_attempt;

    // Plan RAP1 -> RAP2
    plan_goal->plan_title = "### Plan RAP1 -> RAP2";
    plan_goal->plan_id = 2001;

    plan_goal->start = plan_result->goal;
    plan_goal->goal.type.type = t2_msgs::ArmPoseType::CARTESIAN_POSITION;
    plan_goal->goal.pose = goal->rap2_pose;

    plan_goal->stay_level = false;

    if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
    {
      ROS_INFO("ArmPlaceMoveAction plan RAP1 -> RAP2 ERROR");
      result.error_msg = "RAP1 -> RAP2 ERROR (" + plan_result->error_msg + ")";
      goto ERROR_RECOVERY;
    }

    result.planning_attempt += plan_result->planning_attempt;

    // Plan RAP2 -> RP
    plan_goal->plan_title = "### Plan RAP2 -> RP";
    plan_goal->plan_id = 2002;

    plan_goal->start = plan_result->goal;
    plan_goal->goal.type.type = t2_msgs::ArmPoseType::CARTESIAN_POSITION;
    plan_goal->goal.pose = goal->rp_pose;

    plan_goal->stay_level = false;

    if (Plan(plan_goal, *plan_result) != MotionPlannerRet::MOTION_PLANNER_SUCCESS)
    {
      ROS_INFO("ArmPlaceMoveAction plan RAP2 -> RP ERROR");
      result.error_msg = "RAP2 -> RP ERROR (" + plan_result->error_msg + ")";
      goto ERROR_RECOVERY;
    }

    result.planning_attempt += plan_result->planning_attempt;

    ros::Duration past = ros::Time::now() - start_time;
    result.planning_time = past.toSec();

    // set ACM
    sbp_srv.request.operation = sbp_srv.request.PLAN_END;
    if (!set_pick_item_to_planning_scene_client_.call(sbp_srv))
    {
      ROS_ERROR("Failed to set ACM");
      result.error_msg = "Failed to set ACM(PLAN_END)";
      goto ERROR_RECOVERY;
    }

    // ***************************************************************************
    //  Combine plans (current(GP') -> GAP3 -> GAP4(GAP1) -> RAP1 -> RAP2 -> RP)
    // ***************************************************************************
    {
      std::vector<int> plan_id = { 1003, 1004, 2000, 2001, 2002 };
      std::vector<float> velocity = { goal->gap3_velocity, goal->gap4_velocity, goal->rap1_velocity,
                                      goal->rap2_velocity, goal->rp_velocity };

      combinePlanTrajectory(plan_id, velocity, plan_rp.trajectory_.joint_trajectory);
    }

    // *********************************************
    //  Combine plans (RP -> RAP2 -> RAP4(RAP1))
    // *********************************************
    {
      std::vector<int> plan_id = { 2001, 2002 };
      std::vector<float> velocity = { goal->back_rap1_velocity, goal->back_rap2_velocity };

      combinePlanTrajectory(plan_id, velocity, plan_rap4.trajectory_.joint_trajectory);

      // reverse trajectory
      std::reverse(plan_rap4.trajectory_.joint_trajectory.points.begin(),
                   plan_rap4.trajectory_.joint_trajectory.points.end());
    }

    // ***************************************************************************
    //  Combine plans (RP -> RAP2 -> RAP4(RAP1) -> GAP4(GAP1) -> GAP3)
    // ***************************************************************************
    {
      std::vector<int> plan_id = { 1004, 2000, 2001, 2002 };
      std::vector<float> velocity = { goal->gap4_velocity, goal->rap1_velocity, goal->rap2_velocity,
                                      goal->rp_velocity };

      combinePlanTrajectory(plan_id, velocity, back_plan_gap3.trajectory_.joint_trajectory);

      // reverse trajectory
      std::reverse(back_plan_gap3.trajectory_.joint_trajectory.points.begin(),
                   back_plan_gap3.trajectory_.joint_trajectory.points.end());
    }

    // ***************************************************************************
    //  Combine plans (GAP3 -> GAP4(GAP1))
    // ***************************************************************************
    {
      std::vector<int> plan_id = { 1004 };
      std::vector<float> velocity = { goal->gap4_velocity };

      combinePlanTrajectory(plan_id, velocity, back_plan_gap4.trajectory_.joint_trajectory);
    }
  }

  // ***************************************************************************
  //  Execute plans (current(GP') -> GAP3 -> GAP4(GAP1) -> RAP1 -> RAP2 -> RP)
  // ***************************************************************************

  publishTrajectoryMarker(plan_group, plan_rp.trajectory_.joint_trajectory, 2);

  feedback_place_id_ = pick_execute_plan_->plan_goal.place_id;

  feedback_leave_container_flag_ = false;
  feedback_leave_container_midair_flag_ = false;

  execute_error_code = executeTrajectory(move_group, plan_rp);

  if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("move_group->execute() SUCCESS");
  }
  else if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("move_group_->execute() PREEMPTED");
  }
  else
  {
    ROS_ERROR("move_group_->execute() ERROR");
    goto ERROR_RECOVERY;
  }

  if (pick_execute_plan_->weight_scale_flag)
  {
    ROS_INFO("##### Check item weight #####");

    // check weight
    if (!getItemWeightFromCadId(pick_execute_plan_->plan_goal.cad_id, item_weight, item_name))
    {
      ROS_ERROR("Invalid cad_id.");
      use_weight = false;
    }
    else
    {
      ROS_INFO("item_database: %s  %f[g]", item_name.c_str(), item_weight * 1000);
    }

    if (use_weight && waitForWeightScaleExecuteResult(pick_execute_plan_->plan_goal.place_id, actual_weight))
    {
      weightscale_weight_ = weightscale_weight_base_line_ - actual_weight;

      ROS_WARN("weight: %f[g] (%f - %f)", static_cast<double>(weightscale_weight_) * 1000,
               static_cast<double>(weightscale_weight_base_line_) * 1000, actual_weight * 1000);

      // picked item check
      if (disable_weight_check_)
      {
        ROS_WARN("Disable weight check mode. Always picked item is correct.");
        pick_result = true;
        invalid_item = false;
      }
      else if (std::abs(weightscale_weight_) <= weight_scale_no_item_weight_tolerance_)
      {
        // 0 +- tolerance
        ROS_WARN("Picked item check: 0 +- tolerance");
        result.error_msg = "Picked item check: no item, item_database = " + std::to_string(item_weight * 1000) +
                           "[g], weight = " + std::to_string(weightscale_weight_ * 1000) + "[g]";
        pick_result = false;
        invalid_item = false;

        // Clear attached item for simulator
        t2_msgs::ClearAttachedItem cai_srv;
        if (!clear_attached_item_client_.call(cai_srv))
        {
          ROS_ERROR("Failed to call clear_attached_item_client");
        }
      }
      else if (std::abs(weightscale_weight_ - item_weight) <= (item_weight * weight_scale_item_weight_tolerance_))
      {
        // item weight +- tolerance
        ROS_INFO("Picked item check: item weight +- tolerance");
        pick_result = true;
        invalid_item = false;
      }
      else
      {
        // other
        ROS_WARN("Picked item check: invalid item");
        result.error_msg = "Picked item check: invalid item, item_database = " + std::to_string(item_weight * 1000) +
                           "[g], weight = " + std::to_string(weightscale_weight_ * 1000) + "[g]";
        pick_result = false;
        invalid_item = true;
      }
    }
    else
    {
      ROS_WARN("Not use weight for picked item check.");
    }
  }
  else
  {
    ROS_WARN("Not use weight for picked item check.");
  }

  if (invalid_item)
  {
    // back to GAP3

    // ***************************************************************************
    //  Execute plans (RP -> RAP2 -> RAP4(RAP1) -> GAP4(GAP1) -> GAP3)
    // ***************************************************************************

    publishTrajectoryMarker(plan_group, back_plan_gap3.trajectory_.joint_trajectory);

    execute_error_code = executeTrajectory(move_group, back_plan_gap3);

    if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("move_group->execute() SUCCESS");
    }
    else if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    {
      ROS_INFO("move_group_->execute() PREEMPTED");
    }
    else
    {
      ROS_ERROR("move_group_->execute() ERROR");
    }
  }

  // ***************************************************************************
  //  item release
  // ***************************************************************************

  if (grasp_pattern == GRASP_PATTERN_SUCTION)
  {
    if (!setGripperSuction(t2_msgs::GripperSuctionRequest::SUCTION_OFF))
    {
      result.error_msg = "setGripperSuction(OFF) error";
      arm_place_move_action_server_->setAborted(result);
      return;
    }
  }
  else if (grasp_pattern == GRASP_PATTERN_PINCH)
  {
    gripper_release_client_.reset(new GripperReleaseActionClient(
        t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::RELEASE_ACTION_NAME, true));
    t2_msgs::GripperReleaseGoal gripper_release_goal;
    gripper_release_goal.width = grasp_point.width_between_finger_for_release;
    gripper_release_goal.max_effort = 0;

    if (!gripper_release_client_->waitForServer(ros::Duration(1.0)))
    {
      ROS_WARN("Failed to connect t2_pinching_gripper. GripperRelease");
      result.error_msg = "Failed to connect t2_pinching_gripper. GripperRelease";
      arm_place_move_action_server_->setAborted(result);
      return;
    }
    else
    {
      // Clear attached item
      t2_msgs::ClearAttachedItem cai_srv;
      if (!clear_attached_item_client_.call(cai_srv))
      {
        ROS_ERROR("Failed to call clear_attached_item_client");
      }

      gripper_release_client_->sendGoal(gripper_release_goal, NULL, NULL,
                                        boost::bind(&MotionPlanner::cbGripperReleaseFeedback, this, _1));

      if (!gripper_release_client_->waitForResult(ros::Duration(10.0)))
      {
        result.error_msg = "GripperRelease timeout.";
        arm_place_move_action_server_->setAborted(result);
        return;
      }

      if (gripper_release_client_->getResult()->result != t2_msgs::GripperReleaseResult::SUCCESS)
      {
        result.error_msg = "GripperRelease failed.";
        arm_place_move_action_server_->setAborted(result);
        return;
      }
    }
  }

  if (!invalid_item)
  {
    // back to RAP4(RAP1)

    // *********************************************
    //  Execute plans (RP -> RAP3 -> RAP4(RAP1))
    // *********************************************

    publishTrajectoryMarker(plan_group, plan_rap4.trajectory_.joint_trajectory);

    execute_error_code = executeTrajectory(move_group, plan_rap4);

    if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("move_group->execute() SUCCESS");
    }
    else if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    {
      ROS_INFO("move_group_->execute() PREEMPTED");
    }
    else
    {
      ROS_ERROR("move_group_->execute() ERROR");
    }
  }

  if (invalid_item)
  {
    // back to GAP4(GAP1)

    // ***************************************************************************
    //  Execute plans (GAP3 -> GAP4(GAP1))
    // ***************************************************************************

    publishTrajectoryMarker(plan_group, back_plan_gap4.trajectory_.joint_trajectory);

    execute_error_code = executeTrajectory(move_group, back_plan_gap4);

    if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("move_group->execute() SUCCESS");
    }
    else if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    {
      ROS_INFO("move_group_->execute() PREEMPTED");
    }
    else
    {
      ROS_ERROR("move_group_->execute() ERROR");
    }
  }

  // SUCCESS:
  if (pick_result && !invalid_item)
  {
    result.result = t2_msgs::ArmPlaceMoveResult::SUCCESS;
    arm_place_move_action_server_->setSucceeded(result);

    ROS_INFO("End of ArmPlaceMove() Success");
  }
  else if (invalid_item)
  {
    result.result = t2_msgs::ArmPlaceMoveResult::ITEM_ERROR;
    arm_place_move_action_server_->setAborted(result);

    ROS_INFO("End of ArmPlaceMove() setAborted");
  }
  else
  {
    result.result = t2_msgs::ArmPlaceMoveResult::PICK_ERROR;
    arm_place_move_action_server_->setAborted(result);

    ROS_INFO("End of ArmPlaceMove() Aborted");
  }
  return;

ERROR_RECOVERY:
  // plan error: back to GAP4
  ROS_WARN("PlaceMove ERROR_RECOVERY: Failed to Place. Back to GAP4. (GAP2 -> GAP4(GAP1))");

  // ***************************************************************************
  //  Execute plans (GAP2 -> GAP4(GAP1))
  // ***************************************************************************

  moveit::planning_interface::MoveGroup::Plan& back_plan = pick_execute_plan_->back_plan;

  // insert GP' to back plan
  std::vector<trajectory_msgs::JointTrajectoryPoint>& trj_point = back_plan.trajectory_.joint_trajectory.points;
  trajectory_msgs::JointTrajectoryPoint gp_dash_point;
  gp_dash_point.positions = current_pose.joint_positions;
  trj_point.insert(trj_point.begin(), gp_dash_point);

  publishTrajectoryMarker(plan_group, back_plan.trajectory_.joint_trajectory);

  if (grasp_pattern == GRASP_PATTERN_SUCTION)
  {
    if (!setGripperSuction(t2_msgs::GripperSuctionRequest::SUCTION_OFF))
    {
      result.error_msg = "setGripperSuction(OFF) error";
    }
  }
  else if (grasp_pattern == GRASP_PATTERN_PINCH)
  {
    gripper_release_client_.reset(new GripperReleaseActionClient(
        t2_pinching_gripper::NAMESPACE + "/" + t2_pinching_gripper::RELEASE_ACTION_NAME, true));
    t2_msgs::GripperReleaseGoal gripper_release_goal;
    gripper_release_goal.width = grasp_point.width_between_finger_for_release;
    gripper_release_goal.max_effort = 0;

    if (!gripper_release_client_->waitForServer(ros::Duration(1.0)))
    {
      ROS_WARN("Failed to connect t2_pinching_gripper. GripperRelease");
      result.error_msg = "Failed to connect t2_pinching_gripper. GripperRelease";
    }
    else
    {
      // Clear attached item
      t2_msgs::ClearAttachedItem cai_srv;
      if (!clear_attached_item_client_.call(cai_srv))
      {
        ROS_ERROR("Failed to call clear_attached_item_client");
      }

      gripper_release_client_->sendGoal(gripper_release_goal, NULL, NULL,
                                        boost::bind(&MotionPlanner::cbGripperReleaseFeedback, this, _1));

      if (!gripper_release_client_->waitForResult(ros::Duration(10.0)))
      {
        result.error_msg = "GripperRelease timeout.";
      }

      if (gripper_release_client_->getResult()->result != t2_msgs::GripperReleaseResult::SUCCESS)
      {
        result.error_msg = "GripperRelease failed.";
      }
    }
  }

  execute_error_code = executeTrajectory(move_group, back_plan);

  if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("move_group->execute() SUCCESS");
  }
  else if (execute_error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("move_group_->execute() PREEMPTED");
  }
  else
  {
    ROS_ERROR("move_group_->execute() ERROR");
  }

  result.result = t2_msgs::ArmPlaceMoveResult::PLAN_ERROR;
  arm_place_move_action_server_->setAborted(result);

  ROS_INFO("End of ArmPlaceMove() Aborted");
  return;
}

bool MotionPlanner::cbClearArmPlanService(t2_msgs::ClearArmPlanRequest& req, t2_msgs::ClearArmPlanResponse& res)
{
  arm_plans_.clear();
  res.result = t2_msgs::ClearArmPlanResponse::SUCCESS;
  return true;
}

bool MotionPlanner::cbGetArmPoseService(t2_msgs::GetArmPoseRequest& req, t2_msgs::GetArmPoseResponse& res)
{
  ROS_INFO("GetArmPoseService plan_group = %s", req.plan_group.c_str());

  MoveGroupPtr move_group;

  if (!getMoveGroup(req.plan_group, move_group))
  {
    return false;
  }

  if (!move_group)
  {
    ROS_ERROR("move_group is null");
    return false;
  }

  // set joint position
  res.pose.type.type = t2_msgs::ArmPoseType::JOINT_POSITION;
  getArmJointPositions(res.pose.joint_names, res.pose.joint_positions);

  // set pose
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(move_group->getRobotModel()));

  for (std::size_t i = 0; i < res.pose.joint_names.size(); i++)
  {
    kinematic_state->setJointPositions(res.pose.joint_names[i], &res.pose.joint_positions[i]);
  }

  tf::poseEigenToMsg(kinematic_state->getGlobalLinkTransform(move_group->getEndEffectorLink()), res.pose.pose);

  res.result = t2_msgs::GetArmPoseResponse::SUCCESS;

  // ROS_INFO_STREAM(res);

  return true;
}

bool MotionPlanner::cbGetArmGroupStatePoseService(t2_msgs::GetArmGroupStatePoseRequest& req,
                                                  t2_msgs::GetArmGroupStatePoseResponse& res)
{
  ROS_INFO("GetArmGroupStatePoseService plan_group = %s, group_state = %s", req.plan_group.c_str(),
           req.group_state.c_str());

  MoveGroupPtr move_group;

  if (!getMoveGroup(req.plan_group, move_group))
  {
    return false;
  }

  if (!move_group)
  {
    ROS_ERROR("move_group is null");
    return false;
  }

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(default_move_group_->getRobotModel()));
  const robot_state::JointModelGroup* joint_model_group =
      kinematic_state->getJointModelGroup(default_move_group_->getName());
  const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();
  kinematic_state->setToDefaultValues(joint_model_group, req.group_state);
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  res.pose.joint_names = joint_names;
  res.pose.joint_positions = joint_values;

  robot_state::RobotStatePtr target_kinematic_state(new robot_state::RobotState(move_group->getRobotModel()));
  target_kinematic_state->setToDefaultValues();

  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    // ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    target_kinematic_state->setJointPositions(joint_names[i], &joint_values[i]);
  }

  std::vector<double> gripper_joint_positions = gripper_joint_positions_;

  if (req.plan_group == PLAN_GROUP_PINCH)
  {
    // butting pose for grasp plan
    gripper_joint_positions = gripper_butting_joint_positions_;
  }
  else if (req.plan_group == PLAN_GROUP_SUCTION)
  {
    // suction pose for grasp plan
    gripper_joint_positions = gripper_retract_joint_positions_;
  }
  else
  {
    ROS_ERROR("Invalid plan_group = %s", req.plan_group.c_str());
    return false;
  }

  for (std::size_t i = 0; i < gripper_joint_names_.size(); i++)
  {
    target_kinematic_state->setJointPositions(gripper_joint_names_[i], &gripper_joint_positions[i]);
  }

  tf::poseEigenToMsg(target_kinematic_state->getGlobalLinkTransform(move_group->getEndEffectorLink()), res.pose.pose);

  res.result = t2_msgs::GetArmPoseResponse::SUCCESS;

  return true;
}

bool MotionPlanner::cbSaveArmPlanService(t2_msgs::SaveArmPlanRequest& req, t2_msgs::SaveArmPlanResponse& res)
{
  // TODO:
  ROS_ERROR("SaveArmPlanService is not implemented.");
  res.result = t2_msgs::SaveArmPlanResponse::FAILED;
  return false;
}

bool MotionPlanner::cbGetPlannedArmTrajectoryService(t2_msgs::GetPlannedArmTrajectoryRequest& req,
                                                     t2_msgs::GetPlannedArmTrajectoryResponse& res)
{
  ROS_INFO("GetPlannedArmTrajectoryService");
  res.result = t2_msgs::GetPlannedArmTrajectoryResponse::FAILED;

  if (req.plan_name != "")
  {
    ROS_INFO("plan_name = %s", req.plan_name.c_str());

    // TODO:
    ROS_ERROR("Not implemented plan_name support.");
    return true;
  }

  ROS_INFO("plan_id = %d", req.plan_id);

  ArmPlanMap::iterator ite = arm_plans_.find(req.plan_id);

  if (ite == arm_plans_.end())
  {
    ROS_ERROR("invalid plan_id = %d", req.plan_id);
    return true;
  }

  res.planning_time = ite->second.plan.planning_time_;
  res.joint_trajectory = ite->second.plan.trajectory_.joint_trajectory;
  res.result = t2_msgs::GetPlannedArmTrajectoryResponse::SUCCESS;

  return true;
}

void MotionPlanner::cbSuctionState(const t2_msgs::GrippingStateConstPtr& msg)
{
  ROS_INFO("recieved topic SuctionState suction = %d", msg->gripping_state);

  suction_state_flag_ = msg->gripping_state;

  MoveGroupPtr move_group;

  if (!getMoveGroup(PLAN_GROUP_SUCTION, move_group))
  {
    return;
  }

  if ((suction_state_flag_ >= 1) && arm_pick_execute_action_server_->isActive() && arm_approach_item_flag_)
  {
    ROS_WARN("stop arm motion by hand suction sensor.");
    stopTrajectory(move_group);
    arm_approach_item_flag_ = false;
  }
}

void MotionPlanner::cbPadContactState(const t2_msgs::ContactStateConstPtr& msg)
{
  ROS_INFO("Received topic PadContactState %d", msg->contact_state);
  pad_contact_state_flag_ = msg->contact_state;

  MoveGroupPtr move_group;

  if (!getMoveGroup(PLAN_GROUP_SUCTION, move_group))
  {
    return;
  }

  if ((pad_contact_state_flag_ >= 1) && arm_pick_execute_action_server_->isActive() && arm_approach_item_flag_)
  {
    ROS_WARN("stop arm motion by PadContactState.");
    stopTrajectory(move_group);  // move_group->stop();
    arm_approach_item_flag_ = false;

    t2_msgs::GraspPoint& grasp_point = pick_execute_plan_->plan_goal.grasp_point;

    if (grasp_point.grasp_pattern == GRASP_PATTERN_SUCTION && suction_state_flag_ == 0)
    {
      // suction on
      ROS_INFO("set gripper suction on by cbPadContactState");

      if (!setGripperSuction(grasp_point.suction_strength, grasp_point.threshold_of_vacuum_for_suction))
      {
        ROS_ERROR("Failed to setGripperSuction()");
      }
    }
  }
}

void MotionPlanner::cbFingerContactState(const t2_msgs::ContactStateConstPtr& msg)
{
  ROS_INFO("Received topic FingerContactState %d", msg->contact_state);
  gripper_finger_contact_state_flag_ = msg->contact_state;

  MoveGroupPtr move_group;

  if (!getMoveGroup(PLAN_GROUP_SUCTION, move_group))
  {
    return;
  }

  if ((gripper_finger_contact_state_flag_ >= 1) && arm_pick_execute_action_server_->isActive() &&
      arm_approach_item_flag_)
  {
    ROS_WARN("stop arm motion by finger contact sensor.");
    stopTrajectory(move_group);  // move_group->stop();
    arm_approach_item_flag_ = false;
  }
}

void MotionPlanner::cbGrippingState(const t2_msgs::GrippingStateConstPtr& msg)
{
  ROS_INFO("Received topic GrippingState is_gripped = %d", msg->gripping_state);
  gripper_is_gripped_flag_ = msg->gripping_state;
}

bool MotionPlanner::publishAxisMarker(const geometry_msgs::Pose& pose)
{
  double length = 0.1;
  double radius = 0.01;

  std::vector<std_msgs::ColorRGBA> cylinder_color(3);
  std::vector<Eigen::Affine3d> eigen_cylinder_pose(3);

  Eigen::Affine3d eigen_pose;
  tf::poseMsgToEigen(pose, eigen_pose);

  eigen_cylinder_pose[0] =
      Eigen::Translation3d(length / 2.0, 0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
  eigen_cylinder_pose[1] =
      Eigen::Translation3d(0, length / 2.0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
  eigen_cylinder_pose[2] = Eigen::Translation3d(0, 0, length / 2.0) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

  cylinder_color[0].r = 1.0;
  cylinder_color[0].g = 0.0;
  cylinder_color[0].b = 0.0;
  cylinder_color[0].a = 1.0;
  cylinder_color[1].r = 0.0;
  cylinder_color[1].g = 1.0;
  cylinder_color[1].b = 0.0;
  cylinder_color[1].a = 1.0;
  cylinder_color[2].r = 0.0;
  cylinder_color[2].g = 0.0;
  cylinder_color[2].b = 1.0;
  cylinder_color[2].a = 1.0;

  for (std::size_t i = 0; i < eigen_cylinder_pose.size(); i++)
  {
    visualization_msgs::Marker cylinder;
    cylinder.header.frame_id = BASE_FRAME;
    cylinder.header.stamp = ros::Time::now();
    cylinder.ns = "pose_axis";
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.id = i;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.scale.x = cylinder.scale.y = radius;
    cylinder.scale.z = length;
    tf::poseEigenToMsg(eigen_pose * eigen_cylinder_pose[i], cylinder.pose);
    cylinder.color = cylinder_color[i];
    cylinder.lifetime = ros::Duration(planning_time_default_);
    marker_pub_.publish(cylinder);
  }

  geometry_msgs::Pose text_pose = pose;
  text_pose.position.x -= 0.05;
  text_pose.position.y -= 0.05;
  text_pose.position.z -= 0.05;

  visualization_msgs::Marker text;
  text.header.frame_id = BASE_FRAME;
  text.header.stamp = ros::Time::now();
  text.ns = "pose_axis_text";
  text.action = visualization_msgs::Marker::ADD;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.scale.x = text.scale.y = text.scale.z = 0.06;
  text.id = 0;
  text.pose = text_pose;
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.b = 1.0;
  text.color.a = 1.0;
  text.lifetime = ros::Duration(planning_time_default_);

  char ctext[256];
  sprintf(ctext, "(x,y,z) = (%f,%f,%f)", pose.position.x, pose.position.y, pose.position.z);

  text.text = ctext;
  marker_pub_.publish(text);

  return true;
}

bool MotionPlanner::publishSuctionStateMarker(bool flag)
{
  Eigen::Affine3d eigen_pose;
  eigen_pose = Eigen::Translation3d(0, 0, 0.045);

  geometry_msgs::Pose cylinder_pose;
  tf::poseEigenToMsg(eigen_pose, cylinder_pose);

  visualization_msgs::Marker cylinder;
  cylinder.header.frame_id = "rs20n_eef2";
  cylinder.header.stamp = ros::Time::now();
  cylinder.ns = "suction_state";
  cylinder.id = 0;

  cylinder.action = visualization_msgs::Marker::ADD;
  cylinder.type = visualization_msgs::Marker::CYLINDER;
  cylinder.pose = cylinder_pose;
  cylinder.scale.x = 0.05;
  cylinder.scale.y = 0.05;
  cylinder.scale.z = 0.05;

  if (flag)
  {
    cylinder.color.r = 1.0;
    cylinder.color.g = 0.5;
    cylinder.color.b = 0.0;
    cylinder.color.a = 0.5;
  }
  else
  {
    cylinder.color.r = 1.0;
    cylinder.color.g = 1.0;
    cylinder.color.b = 1.0;
    cylinder.color.a = 0.5;
  }

  marker_pub_.publish(cylinder);

  return true;
}

bool MotionPlanner::publishTrajectoryMarker(const std::string& plan_group,
                                            const trajectory_msgs::JointTrajectory& trajectory, int type)
{
  MoveGroupPtr move_group;

  if (!getMoveGroup(plan_group, move_group))
  {
    return false;
  }

  robot_trajectory::RobotTrajectory p(move_group->getRobotModel(), move_group->getName());
  p.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

  visualization_msgs::Marker points, line_strip, line_list;
  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = BASE_FRAME;
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;

  int id_base = type * 10;
  points.id = id_base + 0;
  line_strip.id = id_base + 1;
  line_list.id = id_base + 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  points.scale.x = points.scale.y = points.scale.z = 0.02;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.01;
  line_list.scale.x = 0.01;

  // Points are green
  points.color.g = 1.0;
  points.color.a = 0.5;

  if (type == 1)
  {
    // Line strip is red
    line_strip.color.r = 1.0;
    line_strip.color.a = 0.5;
  }
  else if (type == 2)
  {
    // Line strip is green
    line_strip.color.g = 1.0;
    line_strip.color.a = 0.5;
  }
  else
  {
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 0.5;
  }

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 0.5;

  for (std::size_t i = 0; i < p.getWayPointCount(); i++)
  {
    geometry_msgs::Pose pose;
    const Eigen::Affine3d& eigen_pose = p.getWayPoint(i).getGlobalLinkTransform(move_group->getEndEffectorLink());
    tf::poseEigenToMsg(eigen_pose, pose);
    points.points.push_back(pose.position);
    line_strip.points.push_back(pose.position);
    line_list.points.push_back(pose.position);
    pose.position.z += 0.05;
    line_list.points.push_back(pose.position);
    if (trajectory.points[i].velocities.empty())
    {
      ROS_INFO("Trajectory[%ld](x, y, z, velocity) = (%f, %f, %f, empty)", i, pose.position.x, pose.position.y,
               pose.position.z);
    }
    else
    {
      ROS_INFO("Trajectory[%ld](x, y, z, velocity) = (%f, %f, %f, %f)", i, pose.position.x, pose.position.y,
               pose.position.z, trajectory.points[i].velocities[0]);
    }
  }

  marker_pub_.publish(points);
  marker_pub_.publish(line_strip);
  marker_pub_.publish(line_list);

  return true;
}

bool MotionPlanner::publishContainerRegionMarker(const std::vector<std::string>& regions)
{
  // visualization marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = BASE_FRAME;
  marker.header.stamp = ros::Time::now();
  marker.ns = "container_region";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.2;
  marker.lifetime = ros::Duration(2.0);

  for (std::size_t i = 0; i < regions.size(); i++)
  {
    ContainerRegion& container_region = container_region_map_[regions[i]];

    marker.id = i;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = container_region.position.x + container_region.dimensions.x / 2;
    marker.pose.position.y = container_region.position.y + container_region.dimensions.y / 2;
    marker.pose.position.z = container_region.position.z + container_region.dimensions.z / 2;
    marker.scale = container_region.dimensions;
    marker_pub_.publish(marker);
  }

  return true;
}

bool MotionPlanner::getMoveGroup(const std::string& plan_group, MoveGroupPtr& move_group)
{
  if (std::find(plan_groups_.begin(), plan_groups_.end(), plan_group) != plan_groups_.end())
  {
    move_group = move_groups_[plan_group];
  }
  else
  {
    ROS_ERROR("Invalid plan_group = %s", plan_group.c_str());
    return false;
  }

  if (!move_group)
  {
    ROS_ERROR("move_group is null");
    return false;
  }
  return true;
}

bool MotionPlanner::getT2RobotModel(const std::string& plan_group, T2RobotModelPtr& robot_model)
{
  if (std::find(plan_groups_.begin(), plan_groups_.end(), plan_group) != plan_groups_.end())
  {
    robot_model = t2_robot_models_[plan_group];
  }
  else
  {
    ROS_ERROR("Invalid plan_group = %s", plan_group.c_str());
    return false;
  }

  if (!robot_model)
  {
    ROS_ERROR("robot_model is null");
    return false;
  }
  return true;
}

bool MotionPlanner::getPlaceIdsFromContainerType(const std::string& type, std::vector<uint32_t>& place_ids)
{
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue container_info_array;
  if (!nh.getParam(CONTAINER_INFO_PATH, container_info_array))
  {
    ROS_ERROR("getParam() error: %s", CONTAINER_INFO_PATH.c_str());
    return false;
  }
  ROS_ASSERT(container_info_array.getType() == XmlRpc::XmlRpcValue::TypeArray);

  place_ids.clear();

  for (int i = 0; i < container_info_array.size(); i++)
  {
    ROS_ASSERT(container_info_array[i]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string info_type = static_cast<std::string>(container_info_array[i]["type"]);

    if (info_type == type)
    {
      ROS_ASSERT(container_info_array[i]["place_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);

      int info_place_id = static_cast<int>(container_info_array[i]["place_id"]);

      place_ids.push_back(info_place_id);
    }
  }

  return true;
}

bool MotionPlanner::getContainerInfoFromPlaceId(const uint32_t& place_id, geometry_msgs::Point& inside_position,
                                                geometry_msgs::Vector3& inside_dimensions)
{
  if (place_id == 0)
  {
    return false;
  }

  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue container_info_array;
  if (!nh.getParam(CONTAINER_INFO_PATH, container_info_array))
  {
    ROS_ERROR("getParam() error: %s", CONTAINER_INFO_PATH.c_str());
    return false;
  }
  ROS_ASSERT(container_info_array.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int i = 0; i < container_info_array.size(); i++)
  {
    ROS_ASSERT(container_info_array[i]["place_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);

    int info_place_id = static_cast<int>(container_info_array[i]["place_id"]);

    if (place_id == static_cast<uint32_t>(info_place_id))
    {
      ROS_ASSERT(container_info_array[i]["inside_position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      XmlRpc::XmlRpcValue& inside_position_value = container_info_array[i]["inside_position"];

      inside_position.x = static_cast<double>(inside_position_value[0]);
      inside_position.y = static_cast<double>(inside_position_value[1]);
      inside_position.z = static_cast<double>(inside_position_value[2]);

      ROS_ASSERT(container_info_array[i]["inside_dimensions"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      XmlRpc::XmlRpcValue& inside_dimensions_value = container_info_array[i]["inside_dimensions"];

      inside_dimensions.x = static_cast<double>(inside_dimensions_value[0]);
      inside_dimensions.y = static_cast<double>(inside_dimensions_value[1]);
      inside_dimensions.z = static_cast<double>(inside_dimensions_value[2]);

      return true;
    }
  }

  return false;
}

bool MotionPlanner::addConstraintBoxRegion(moveit_msgs::PositionConstraint& constraint,
                                           const geometry_msgs::Point& position,
                                           const geometry_msgs::Vector3& dimensions)
{
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimensions.x;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimensions.y;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimensions.z;
  constraint.constraint_region.primitives.push_back(primitive);
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.position.x = dimensions.x / 2 + position.x;
  pose.position.y = dimensions.y / 2 + position.y;
  pose.position.z = dimensions.z / 2 + position.z;
  constraint.constraint_region.primitive_poses.push_back(pose);

  return true;
}

bool MotionPlanner::getPositionConstraints(const std::string& link_name, const std::vector<uint32_t>& place_id,
                                           const double& weight, moveit_msgs::PositionConstraint& constraint)
{
  constraint.header.frame_id = BASE_FRAME;
  constraint.header.stamp = ros::Time::now();
  constraint.link_name = link_name;
  constraint.weight = weight;

  std::size_t region = place_id.size() + 1;

  std::vector<geometry_msgs::Point> position(region);
  std::vector<geometry_msgs::Vector3> dimensions(region);

  position.back().x = end_effector_position_constraint_box_[0];
  position.back().y = end_effector_position_constraint_box_[1];
  position.back().z = end_effector_position_constraint_box_[2];
  dimensions.back().x = end_effector_position_constraint_box_[3];
  dimensions.back().y = end_effector_position_constraint_box_[4];
  dimensions.back().z = end_effector_position_constraint_box_[5];

  addConstraintBoxRegion(constraint, position.back(), dimensions.back());

  for (std::size_t i = 0; i < place_id.size(); i++)
  {
    if (place_id[i] > 0)
    {
      // get container info from parameter server
      getContainerInfoFromPlaceId(place_id[i], position[i], dimensions[i]);

      // transform corner of container
      position[i].x = position[i].x - dimensions[i].x - end_effector_position_constraint_container_mergin_;
      position[i].y = position[i].y - dimensions[i].y - end_effector_position_constraint_container_mergin_;
      position[i].z = position[i].z - end_effector_position_constraint_container_mergin_;
      dimensions[i].x = dimensions[i].x + end_effector_position_constraint_container_mergin_ * 2;
      dimensions[i].y = dimensions[i].y + end_effector_position_constraint_container_mergin_ * 2;
      dimensions[i].z = dimensions[i].z + end_effector_position_constraint_container_mergin_;

      // add inside region of container
      addConstraintBoxRegion(constraint, position[i], dimensions[i]);

      geometry_msgs::Point midair_position;
      geometry_msgs::Vector3 midair_dimensions;
      midair_position.x = position[i].x + dimensions[i].x * (1 - end_effector_position_constraint_midair_scale_) / 2;
      midair_position.y = position[i].y + dimensions[i].y * (1 - end_effector_position_constraint_midair_scale_) / 2;
      midair_position.z = position[i].z + dimensions[i].z;
      midair_dimensions.x = dimensions[i].x * end_effector_position_constraint_midair_scale_;
      midair_dimensions.y = dimensions[i].y * end_effector_position_constraint_midair_scale_;
      midair_dimensions.z = position.back().z - midair_position.z;

      // add midair region of container
      addConstraintBoxRegion(constraint, midair_position, midair_dimensions);
    }
  }

  if (end_effector_position_constraint_marekr_)
  {
    // visualization marker
    visualization_msgs::Marker marker;
    marker.header = constraint.header;
    marker.ns = "position_constraints";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1;
    marker.lifetime = ros::Duration(planning_time_default_);

    for (std::size_t i = 0; i < constraint.constraint_region.primitives.size(); i++)
    {
      const shape_msgs::SolidPrimitive& primitive = constraint.constraint_region.primitives[i];
      const geometry_msgs::Pose& pose = constraint.constraint_region.primitive_poses[i];

      if (primitive.type == shape_msgs::SolidPrimitive::BOX)
      {
        marker.id = i;
        marker.pose = pose;
        marker.scale.x = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X];
        marker.scale.y = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
        marker.scale.z = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
        marker_pub_.publish(marker);
      }
    }
  }

  return true;
}

void MotionPlanner::cbGripperPinchFeedback(const t2_msgs::GripperPinchFeedbackConstPtr& feedback)
{
  ROS_INFO("cbGripperPinchFeedback width = %f, extrusion = %f, effort = %f", feedback->width, feedback->extrusion,
           feedback->effort);

  gripper_joint_positions_[0] = feedback->extrusion;
  gripper_joint_positions_[1] = feedback->width;
}

void MotionPlanner::cbGripperReleaseFeedback(const t2_msgs::GripperReleaseFeedbackConstPtr& feedback)
{
  ROS_INFO("cbGripperReleaseFeedback width = %f, effort = %f", feedback->width, feedback->effort);

  gripper_joint_positions_[1] = feedback->width;
}

bool MotionPlanner::getCircumscribedCudeRegin(const std::vector<geometry_msgs::Point>& in_positions,
                                              const std::vector<geometry_msgs::Vector3>& in_dimensions,
                                              geometry_msgs::Point& out_position,
                                              geometry_msgs::Vector3& out_dimensions)
{
  geometry_msgs::Point max_position;

  for (std::size_t i = 0; i < in_positions.size(); i++)
  {
    if (i == 0)
    {
      out_position = in_positions[i];
      max_position.x = out_position.x + in_dimensions[i].x;
      max_position.y = out_position.y + in_dimensions[i].y;
      max_position.z = out_position.z + in_dimensions[i].z;
    }
    else
    {
      out_position.x = std::min(out_position.x, in_positions[i].x);
      out_position.y = std::min(out_position.y, in_positions[i].y);
      out_position.z = std::min(out_position.z, in_positions[i].z);
      max_position.x = std::max(max_position.x, in_positions[i].x + in_dimensions[i].x);
      max_position.y = std::max(max_position.y, in_positions[i].y + in_dimensions[i].y);
      max_position.z = std::max(max_position.z, in_positions[i].z + in_dimensions[i].z);
    }
  }

  out_dimensions.x = max_position.x - out_position.x;
  out_dimensions.y = max_position.y - out_position.y;
  out_dimensions.z = max_position.z - out_position.z;

  return true;
}

bool MotionPlanner::getContainerRegion(const std::string& type, std::vector<uint32_t>& place_ids,
                                       geometry_msgs::Point& out_position, geometry_msgs::Vector3& out_dimensions)
{
  std::vector<geometry_msgs::Point> positions;
  std::vector<geometry_msgs::Vector3> dimensions;

  getPlaceIdsFromContainerType(type, place_ids);

  positions.resize(place_ids.size());
  dimensions.resize(place_ids.size());

  for (std::size_t i = 0; i < place_ids.size(); i++)
  {
    getContainerInfoFromPlaceId(place_ids[i], positions[i], dimensions[i]);

    // transform corner of container
    positions[i].x = positions[i].x - dimensions[i].x;
    positions[i].y = positions[i].y - dimensions[i].y;
  }

  getCircumscribedCudeRegin(positions, dimensions, out_position, out_dimensions);

  return true;
}

bool MotionPlanner::updateChekingRegion()
{
  if (container_region_map_.size() == container_region_names_.size())
  {
    ROS_INFO("updateChekingRegion already updated.");
    return true;
  }

  container_region_map_.clear();

  for (std::size_t i = 0; i < container_region_names_.size(); i++)
  {
    const std::string& region = container_region_names_[i];
    ContainerRegion container_region;

    if (region == CONTAINER_REGION_SS)
    {
      container_region.position = container_region_map_["bin"].position;
      container_region.dimensions = container_region_map_["bin"].dimensions;
    }
    else
    {
      getContainerRegion(region, container_region.place_ids, container_region.position, container_region.dimensions);
    }

    container_region.dimensions.z = position_feedback_midair_height_[region] - container_region.position.z;
    container_region_map_[region] = container_region;
  }

  return true;
}

bool MotionPlanner::getWeightScaleIdFromPlaceId(const uint32_t& place_id, uint32_t& weight_scale_id)
{
  if (place_id == 0)
  {
    return false;
  }

  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue container_info_array;
  if (!nh.getParam(CONTAINER_INFO_PATH, container_info_array))
  {
    ROS_ERROR("getParam() error: %s", CONTAINER_INFO_PATH.c_str());
    return false;
  }
  ROS_ASSERT(container_info_array.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int i = 0; i < container_info_array.size(); i++)
  {
    ROS_ASSERT(container_info_array[i]["place_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);

    int info_place_id = static_cast<int>(container_info_array[i]["place_id"]);

    if (place_id == static_cast<uint32_t>(info_place_id))
    {
      ROS_ASSERT(container_info_array[i]["weight_scale_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);

      weight_scale_id = static_cast<int>(container_info_array[i]["weight_scale_id"]);

      return true;
    }
  }

  return false;
}

bool MotionPlanner::waitForWeightScaleExecuteResult(const uint32_t& place_id, double& weight)
{
  uint32_t weight_scale_id;

  if (!getWeightScaleIdFromPlaceId(place_id, weight_scale_id))
  {
    return false;
  }

  if (!weightscale_execute_clients_[weight_scale_id])
  {
    ROS_WARN("weightscale_execute_clients_ is null. call weightScaleExecute()");
    weightScaleExecute(place_id, false);
  }

  bool before_timeout =
      weightscale_execute_clients_[weight_scale_id]->waitForResult(ros::Duration(weight_scale_timeout_time_));

  if (before_timeout &&
      weightscale_execute_clients_[weight_scale_id]->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    weight = weightscale_execute_clients_[weight_scale_id]->getResult()->weight;
    weightscale_execute_clients_[weight_scale_id].reset();
    return true;
  }

  return false;
}

bool MotionPlanner::weightScaleExecute(const uint32_t& place_id, const bool base_line_flag)
{
  uint32_t weight_scale_id;

  if (!getWeightScaleIdFromPlaceId(place_id, weight_scale_id))
  {
    return false;
  }

  ROS_WARN("weightScaleExecute weight_scale_id = %u, baseline_flag = %d", weight_scale_id, base_line_flag);

  weightscale_execute_clients_[weight_scale_id].reset(new WeightScaleExecuteActionClient(
      WEIGHT_SCALE_NODE_BASE_NAME_NAME + std::to_string(weight_scale_id) + "/" + WEIGHT_SCALE_EXECUTE_ACTION_NAME));

  if (!weightscale_execute_clients_[weight_scale_id]->waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("weightscale_execute_clients_[%u] is not available", weight_scale_id);
    return false;
  }

  t2_msgs::WeightScaleExecuteGoal wse_goal;  // empty

  ROS_INFO("Waiting weight scale result.");

  if (base_line_flag)
  {
    weightscale_weight_base_line_id_ = weight_scale_id;
    weightscale_weight_base_line_ = 0;

    if (weightscale_execute_clients_[weight_scale_id]->sendGoalAndWait(
            wse_goal, ros::Duration(weight_scale_timeout_time_)) != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("weightscale_execute_clients_[%d] failed", weight_scale_id);
      return false;
    }

    weightscale_weight_base_line_ = weightscale_execute_clients_[weight_scale_id]->getResult()->weight;

    ROS_INFO("weightscale_weight_base_line_[%u] = %f", weight_scale_id,
             static_cast<double>(weightscale_weight_base_line_));

    weightscale_execute_clients_[weight_scale_id].reset();
  }
  else
  {
    weightscale_execute_clients_[weight_scale_id]->sendGoal(wse_goal);

    // weightscale_execute_clients_[weight_scale_id]->sendGoal(
    //    wse_goal, boost::bind(&MotionPlanner::cbWeightScaleExecute, this, weight_scale_id, _1, _2));
  }

  return true;
}

bool MotionPlanner::getItemWeightFromCadId(const uint32_t& cad_id, double& weight, std::string& name)
{
  if (cad_id == 0 || cad_id == 9999)
  {
    return false;
  }

  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue grasp_info;
  if (!nh.getParam(GRASP_INFO_PATH, grasp_info))
  {
    ROS_ERROR("Failed to get grasp_info");
    return false;
  }
  ROS_ASSERT(grasp_info.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  std::string cad_id_str = "cad" + std::to_string(cad_id);

  for (auto it = grasp_info.begin(); it != grasp_info.end(); ++it)
  {
    if (it->first == cad_id_str)
    {
      weight = static_cast<double>(grasp_info[it->first]["weight_of_item"]);
      name = static_cast<std::string>(grasp_info[it->first]["item_name"]);
      return true;
    }
  }
  return false;
}

MotionPlanner::~MotionPlanner()
{
}

}  // namespace t2_motion_planner
