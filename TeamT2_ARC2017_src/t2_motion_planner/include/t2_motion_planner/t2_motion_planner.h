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

#ifndef T2_MOTION_PLANNER_T2_MOTION_PLANNER_H
#define T2_MOTION_PLANNER_T2_MOTION_PLANNER_H

#define USE_WEIGHT_SCALE

//#define USE_MOVE_GROUP_FOR_EXECUTE // need to enable move_group allow_trajectory_execution

#include <t2_motion_planner/capability_names.h>
#include <t2_planning_scene_updater/capability_names.h>
#include <t2_pinching_gripper/capability_names.h>

#include <sys/stat.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <t2_motion_planner/t2_robot_model.h>

#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <t2_msgs/ArmExecuteAction.h>
#include <t2_msgs/ArmMoveAction.h>
#include <t2_msgs/ArmPlanAction.h>
#include <t2_msgs/ClearArmPlan.h>
#include <t2_msgs/GetArmPose.h>
#include <t2_msgs/GetArmGroupStatePose.h>
#include <t2_msgs/SaveArmPlan.h>
#include <t2_msgs/GetPlannedArmTrajectory.h>
#include <t2_msgs/GripperSuction.h>

#include <t2_msgs/ArmPickPlanAction.h>
#include <t2_msgs/ArmPickExecuteAction.h>

#include <t2_msgs/ArmPlaceMoveAction.h>

#include <t2_msgs/GripperExtendAction.h>
#include <t2_msgs/GripperPinchAction.h>
#include <t2_msgs/GripperReleaseAction.h>
#include <t2_msgs/GripperRetractAction.h>
#include <t2_msgs/ContactState.h>
#include <t2_msgs/GrippingState.h>

#include <t2_msgs/SetPickItemToPlanningScene.h>
#include <t2_msgs/ClearAttachedItem.h>
#include <t2_msgs/GetAttachedItemFromPlanningScene.h>

#include <t2_msgs/WeightScaleExecuteAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <atomic>
#include <mutex>

namespace t2_motion_planner
{
const std::string BASE_FRAME = "/world";

const std::string ROBOT_DESCRIPTION = "robot_description";

const std::string GRASP_PATTERN_SUCTION = "suction";
const std::string GRASP_PATTERN_PINCH = "pinch";

const std::string PLAN_GROUP_ARM = "arm";
const std::string PLAN_GROUP_SUCTION = "arm_suction";
const std::string PLAN_GROUP_PINCH = "arm_pinch";

const std::string CONTAINER_REGION_BIN = "bin";
const std::string CONTAINER_REGION_TOTE = "tote";
const std::string CONTAINER_REGION_BOX = "box";
const std::string CONTAINER_REGION_SS = "ss";

const std::string GRIPPER_SUCTION_SERVICE_NAME = "t2_gripper/suction";
const std::string GRIPPER_SUCTION_SENSOR_TOPIC_NAME = "t2_gripper/suction_state";
const std::string GRIPPER_PAD_CONTACT_STATE_TOPIC_NAME = "t2_gripper/pad_contact_state";

#ifdef USE_MOVE_GROUP_FOR_EXECUTE
const std::string ARM_CONTROLLER_FEEDBACK_TOPIC_NAME = "t2_arm_controller/follow_joint_trajectory_action/feedback";
#else
const std::string ARM_CONTROLLER_FOLLOW_JOINT_TRAJECTORY_ACTION_NAME = "t2_arm_controller/"
                                                                       "follow_joint_trajectory_action";
#endif

const std::string RVIZ_VISUALIZATION_MARKER_TOPIC_NAME = "visualization_marker";

const std::string CONTAINER_INFO_PATH = "/t2_database/container_info";
const std::string BOX_SETTINGS_PATH = "/t2_database/box_settings";
const std::string GRASP_INFO_PATH = "/t2_database/grasp_info_list";

const std::string WEIGHT_SCALE_NODE_BASE_NAME_NAME = "/t2_weight_scale_";
const std::string WEIGHT_SCALE_EXECUTE_ACTION_NAME = "weight_scale_execute";

typedef actionlib::SimpleActionServer<t2_msgs::ArmExecuteAction> ArmExecuteActionServer;
typedef actionlib::SimpleActionServer<t2_msgs::ArmMoveAction> ArmMoveActionServer;
typedef actionlib::SimpleActionServer<t2_msgs::ArmPlanAction> ArmPlanActionServer;

typedef actionlib::SimpleActionServer<t2_msgs::ArmPickPlanAction> ArmPickPlanActionServer;
typedef actionlib::SimpleActionServer<t2_msgs::ArmPickExecuteAction> ArmPickExecuteActionServer;

typedef actionlib::SimpleActionServer<t2_msgs::ArmPlaceMoveAction> ArmPlaceMoveActionServer;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionClient;

typedef actionlib::SimpleActionClient<t2_msgs::GripperExtendAction> GripperExtendActionClient;
typedef actionlib::SimpleActionClient<t2_msgs::GripperPinchAction> GripperPinchActionClient;
typedef actionlib::SimpleActionClient<t2_msgs::GripperReleaseAction> GripperReleaseActionClient;
typedef actionlib::SimpleActionClient<t2_msgs::GripperRetractAction> GripperRetractActionClient;

typedef actionlib::SimpleActionClient<t2_msgs::WeightScaleExecuteAction> WeightScaleExecuteActionClient;

typedef enum
{
  MOTION_PLANNER_SUCCESS,
  MOTION_PLANNER_FAILED,
  MOTION_PLANNER_INVALID_PARAM
} MotionPlannerRet;

typedef struct
{
  std::string plan_group;
  moveit::planning_interface::MoveGroup::Plan plan;
  double path_length;
  double smoothness;
} MotionPlannerPlan;

typedef struct
{
  t2_msgs::ArmPickPlanGoal plan_goal;
  std::string plan_group;
  moveit::planning_interface::MoveGroup::Plan plan;
  moveit::planning_interface::MoveGroup::Plan back_plan;  // GP' -> GAP2 -> GAP1
  bool pick_state;
  bool weight_scale_flag;
} PickPlan;

typedef struct
{
  std::vector<uint32_t> place_ids;
  geometry_msgs::Point position;
  geometry_msgs::Vector3 dimensions;
} ContainerRegion;

class MotionPlanner
{
public:
  MotionPlanner();
  ~MotionPlanner();
  void init();

protected:
  typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
  typedef boost::shared_ptr<moveit::planning_interface::MoveGroup::Plan> PlanPtr;
  typedef std::map<std::string, MoveGroupPtr> MoveGroupMap;
  typedef boost::shared_ptr<T2RobotModel> T2RobotModelPtr;
  typedef std::map<std::string, T2RobotModelPtr> T2RobotModelMap;
  typedef std::map<std::string, ContainerRegion> ContainerRegionMap;

  MoveGroupMap move_groups_;

  T2RobotModelMap t2_robot_models_;

  MoveGroupPtr default_move_group_;

  typedef std::map<int, MotionPlannerPlan> ArmPlanMap;

  typedef std::map<int, PickPlan> ArmPickPlanMap;

  ArmPlanMap arm_plans_;

  ArmPickPlanMap arm_pick_plans_;

  std::vector<std::string> container_region_names_;
  ContainerRegionMap container_region_map_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

private:
  bool getJointNames(const std::string& plan_group, std::vector<std::string>& joint_names);
  bool getArmJointPositions(std::vector<std::string>& joint_names, std::vector<double>& joint_positions);

  bool isEqualJointPositions(std::vector<std::string>& joint_names1, std::vector<double>& joint_positions1,
                             std::vector<std::string>& joint_names2, std::vector<double>& joint_positions2,
                             double range);

  bool trajectorySamePointFilter(const std::vector<trajectory_msgs::JointTrajectoryPoint>& points_in,
                                 std::vector<trajectory_msgs::JointTrajectoryPoint>& points_out);

  bool trajectoryNPointFilter(const std::vector<trajectory_msgs::JointTrajectoryPoint>& points_in,
                              std::vector<trajectory_msgs::JointTrajectoryPoint>& points_out, int n_points);

  bool trajectoryJointNameFilter(const trajectory_msgs::JointTrajectory& trajectory_in,
                                 trajectory_msgs::JointTrajectory& trajectory_out,
                                 const std::vector<std::string>& valid_joint_names);

  bool trajectoryPointsAddVelocity(std::vector<trajectory_msgs::JointTrajectoryPoint>& points, const double& velocity);

  bool combinePlanTrajectory(const std::vector<int>& plan_id, const std::vector<float>& velocity,
                             trajectory_msgs::JointTrajectory& trajectory);

  bool combinePickPlanTrajectory(const int plan_id, const float velocity, const int pick_plan_id,
                                 trajectory_msgs::JointTrajectory& trajectory);

  bool trajectoryCollisionCheck(const std::string& plan_group, const trajectory_msgs::JointTrajectory& trajectory,
                                double x_max, double x_min, double y_max, double y_min, double z);

  bool updateChekingRegion();

  bool getContainerRegionFromPlaceId(const uint32_t& place_id, std::string& region);

  bool endEffectorRegionCollisionCheck(const std::string& plan_group,
                                       const control_msgs::FollowJointTrajectoryFeedback& feedback,
                                       const std::vector<std::string>& regions, std::vector<bool>& collision_flags);

  MotionPlannerRet configureWorkspace(MoveGroupPtr& move_group, const double& center_x, const double& center_y,
                                      const double& center_z, const double& size_x, const double& size_y,
                                      const double& size_z);

  MotionPlannerRet Plan(const t2_msgs::ArmPlanGoalConstPtr& goal, t2_msgs::ArmPlanResult& result);

  MotionPlannerRet Execute(const t2_msgs::ArmExecuteGoalConstPtr& goal, t2_msgs::ArmExecuteResult& result);

  MotionPlannerRet Move(const t2_msgs::ArmMoveGoalConstPtr& goal, t2_msgs::ArmMoveResult& result);

  moveit_msgs::MoveItErrorCodes executeTrajectory(MoveGroupPtr& move_group,
                                                  const moveit::planning_interface::MoveGroup::Plan& plan);

  bool stopTrajectory(MoveGroupPtr& move_group);

  std::string getMoveItErrorCodeStr(const int32_t& error_val);

  bool setGripperSuction(const int32_t suction = t2_msgs::GripperSuctionRequest::SUCTION_OFF,
                         const double threshold = 0);

  void cbSuctionState(const t2_msgs::GrippingStateConstPtr& msg);

  void cbPadContactState(const t2_msgs::ContactStateConstPtr& msg);

  void cbFingerContactState(const t2_msgs::ContactStateConstPtr& msg);

  void cbGrippingState(const t2_msgs::GrippingStateConstPtr& msg);

  // Action Server callback
  void cbArmExecuteActionServerGoal(const t2_msgs::ArmExecuteGoalConstPtr& goal);

  void cbArmMoveActionServerGoal(const t2_msgs::ArmMoveGoalConstPtr& goal);

  void cbArmPlanActionServerGoal(const t2_msgs::ArmPlanGoalConstPtr& goal);

  void cbArmPickPlanActionServerGoal(const t2_msgs::ArmPickPlanGoalConstPtr& goal);

  void cbArmPickExecuteActionServerGoal(const t2_msgs::ArmPickExecuteGoalConstPtr& goal);

  void cbArmPlaceMoveActionServerGoal(const t2_msgs::ArmPlaceMoveGoalConstPtr& goal);

  // Service callback
  bool cbClearArmPlanService(t2_msgs::ClearArmPlanRequest& req, t2_msgs::ClearArmPlanResponse& res);
  bool cbGetArmPoseService(t2_msgs::GetArmPoseRequest& req, t2_msgs::GetArmPoseResponse& res);
  bool cbGetArmGroupStatePoseService(t2_msgs::GetArmGroupStatePoseRequest& req,
                                     t2_msgs::GetArmGroupStatePoseResponse& res);

  bool cbSaveArmPlanService(t2_msgs::SaveArmPlanRequest& req, t2_msgs::SaveArmPlanResponse& res);
  bool cbGetPlannedArmTrajectoryService(t2_msgs::GetPlannedArmTrajectoryRequest& req,
                                        t2_msgs::GetPlannedArmTrajectoryResponse& res);

  // Client callback
  void cbGripperPinchFeedback(const t2_msgs::GripperPinchFeedbackConstPtr& feedback);
  void cbGripperReleaseFeedback(const t2_msgs::GripperReleaseFeedbackConstPtr& feedback);

#ifdef USE_MOVE_GROUP_FOR_EXECUTE
  void cbArmControllerFeedback(const control_msgs::FollowJointTrajectoryActionFeedbackConstPtr& feedback);
#else
  void cbArmControllerFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
#endif

  // Subscriber
  ros::Subscriber suction_state_;
  ros::Subscriber pad_contact_state_;
  ros::Subscriber gripper_finger_contact_state_;
  ros::Subscriber gripping_state_;
#ifdef USE_MOVE_GROUP_FOR_EXECUTE
  ros::Subscriber arm_controller_feedback_;
#endif

  // Action Server
  boost::shared_ptr<ArmExecuteActionServer> arm_execute_action_server_;
  boost::shared_ptr<ArmMoveActionServer> arm_move_action_server_;
  boost::shared_ptr<ArmPlanActionServer> arm_plan_action_server_;

  boost::shared_ptr<ArmPickPlanActionServer> arm_pick_plan_action_server_;
  boost::shared_ptr<ArmPickExecuteActionServer> arm_pick_execute_action_server_;

  boost::shared_ptr<ArmPlaceMoveActionServer> arm_place_move_action_server_;

  // Service Server
  ros::ServiceServer clear_arm_plan_service_;
  ros::ServiceServer get_arm_pose_service_;
  ros::ServiceServer get_arm_group_state_pose_service_;
  ros::ServiceServer save_arm_plan_service_;
  ros::ServiceServer get_planned_arm_trajectory_service_;

  // Action Client
  boost::shared_ptr<GripperExtendActionClient> gripper_extend_client_;
  boost::shared_ptr<GripperPinchActionClient> gripper_pinch_client_;
  boost::shared_ptr<GripperReleaseActionClient> gripper_release_client_;
  boost::shared_ptr<GripperRetractActionClient> gripper_retract_client_;
  boost::shared_ptr<FollowJointTrajectoryActionClient> follow_joint_trajectry_client_;

  std::map<int, boost::shared_ptr<WeightScaleExecuteActionClient>> weightscale_execute_clients_;

  // Service Client
  ros::ServiceClient set_pick_item_to_planning_scene_client_;
  ros::ServiceClient clear_attached_item_client_;
  ros::ServiceClient get_attached_item_client_;

  double planning_time_default_;
  double planning_time_pick_;

  std::atomic<uint8_t> suction_state_flag_;
  std::atomic<uint8_t> pad_contact_state_flag_;
  std::atomic<bool> gripper_suction_flag_;
  std::atomic<bool> arm_approach_item_flag_;

  std::atomic<bool> arm_controller_feedback_flag_;
  std::atomic<uint8_t> gripper_finger_contact_state_flag_;
  std::atomic<uint8_t> gripper_is_gripped_flag_;

  std::atomic<bool> feedback_leave_container_flag_;
  std::atomic<bool> feedback_leave_container_midair_flag_;

  std::atomic<uint32_t> weightscale_weight_base_line_id_;
  std::atomic<double> weightscale_weight_base_line_;
  std::atomic<double> weightscale_weight_;
  bool disable_weight_check_;

  control_msgs::FollowJointTrajectoryFeedback last_arm_controller_feedback_;

  const std::vector<std::string> gripper_joint_names_ = { "gripper_slider_joint", "gripper_finger_joint" };
  std::vector<double> gripper_joint_positions_ = { 0, 0 };
  std::vector<double> gripper_butting_joint_positions_ = { 0, 0 };
  std::vector<double> gripper_retract_joint_positions_ = { 0, 0 };

  void jointStateCallback(const sensor_msgs::JointState& joint_state);

  double computeDirectDistance(MoveGroupPtr& move_group, const trajectory_msgs::JointTrajectory& trajectory);

  double computePathLength(MoveGroupPtr& move_group, T2RobotModelPtr& t2_robot_model,
                           const trajectory_msgs::JointTrajectory& trajectory,
                           const std::vector<std::string>& joint_names, const std::vector<double>& weight);

  double computePathSmoothness(MoveGroupPtr& move_group, T2RobotModelPtr& t2_robot_model,
                               const trajectory_msgs::JointTrajectory& trajectory,
                               const std::vector<std::string>& joint_names, const std::vector<double>& weight);

  bool solveIKForEndEffectorPose(MoveGroupPtr& move_group, const geometry_msgs::Pose& pose,
                                 const moveit_msgs::Constraints& constraints,
                                 const robot_state::RobotState& start_state, robot_state::RobotState& goal_state);

  ros::Publisher marker_pub_;

  bool publishAxisMarker(const geometry_msgs::Pose& pose);

  bool publishSuctionStateMarker(bool flag);

  bool publishTrajectoryMarker(const std::string& plan_group, const trajectory_msgs::JointTrajectory& trajectory,
                               int type = 0);

  bool publishContainerRegionMarker(const std::vector<std::string>& regions);

  std::vector<std::string> path_evaluate_joints_;
  std::vector<double> path_length_joint_weight_;
  std::vector<double> path_smooth_joint_weight_;

  int planning_retry_count_default_;
  int planning_retry_count_pick_;

  int planning_attempts_default_;
  int planning_attempts_pick_;

  double max_valid_path_length_threshold_near_;
  double max_valid_path_length_threshold_far_;

  double max_valid_path_length_near_strict_;
  double max_valid_path_length_near_lax_;
  double max_valid_path_length_middle_strict_;
  double max_valid_path_length_middle_lax_;
  double max_valid_path_length_far_strict_;
  double max_valid_path_length_far_lax_;

  double max_valid_path_smoothness_;

  double goal_joint_tolerance_;
  double goal_orientation_tolerance_;
  double goal_position_tolerance_;

  bool end_effector_position_constraint_marekr_;
  std::vector<double> end_effector_position_constraint_box_;
  double end_effector_position_constraint_midair_scale_;
  double end_effector_position_constraint_container_mergin_;

  bool cartesian_path_approach_;
  double cartesian_path_eef_step_;
  double cartesian_path_jump_threshold_;
  double cartesian_path_approach_fraction_threshold_;
  bool cartesian_path_error_recover_by_normal_plan_;

  int n_points_;
  bool delete_trajectory_connection_points_;

  std::vector<double> trajectory_collision_check_area_;

  std::vector<std::string> plan_groups_;

  std::string default_plan_group_;

  std::string getEndEffectorLink(const std::string& plan_group);

  bool getMoveGroup(const std::string& plan_group, MoveGroupPtr& move_group);

  bool getT2RobotModel(const std::string& plan_group, T2RobotModelPtr& robot_model);

  bool getCircumscribedCudeRegin(const std::vector<geometry_msgs::Point>& in_positions,
                                 const std::vector<geometry_msgs::Vector3>& in_dimensions,
                                 geometry_msgs::Point& out_position, geometry_msgs::Vector3& out_dimensions);
  bool getContainerRegion(const std::string& type, std::vector<uint32_t>& place_ids, geometry_msgs::Point& out_position,
                          geometry_msgs::Vector3& out_dimensions);

  bool getPlaceIdsFromContainerType(const std::string& type, std::vector<uint32_t>& place_ids);

  bool getContainerInfoFromPlaceId(const uint32_t& place_id, geometry_msgs::Point& inside_position,
                                   geometry_msgs::Vector3& inside_dimensions);

  bool addConstraintBoxRegion(moveit_msgs::PositionConstraint& constraint, const geometry_msgs::Point& box_position,
                              const geometry_msgs::Vector3& box_dimensions);

  bool getPositionConstraints(const std::string& link_name, const std::vector<uint32_t>& place_id, const double& weight,
                              moveit_msgs::PositionConstraint& constraint);

  bool getWeightScaleIdFromPlaceId(const uint32_t& place_id, uint32_t& weight_scale_id);

  bool waitForWeightScaleExecuteResult(const uint32_t& place_id, double& weight);

  bool weightScaleExecute(const uint32_t& place_id, const bool base_line_flag = true);

  bool getItemWeightFromCadId(const uint32_t& cad_id, double& weight, std::string& name);

  // gripper settings
  double gripper_detect_extend_pos_;
  double gripper_extender_retract_min_;
  double gripper_pinch_close_min_;

  double suction_sensor_waiting_time_;

  double weight_scale_timeout_time_;
  double weight_scale_no_item_weight_tolerance_;
  double weight_scale_item_weight_tolerance_;

  double compare_joint_tolerance_;

  bool position_feedback_marker_;
  std::map<std::string, double> position_feedback_midair_height_;

  PickPlan* pick_execute_plan_;

  uint32_t feedback_place_id_;

  std::mutex plan_mutex_;
  std::mutex seq_plan_mutex_;
  std::mutex combine_plan_mutex_;

  tf::TransformListener tf_listener_;

};  // class

}  // namespace t2_motion_planner

#endif  // T2_MOTION_PLANNER_T2_MOTION_PLANNER_H
