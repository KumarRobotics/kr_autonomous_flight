#pragma once

#include <action_trackers/action/run_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kr_mav_msgs/msg/position_command.hpp>
#include <kr_planning_msgs/msg/path.hpp>
#include <kr_planning_msgs/action/plan_two_point.hpp>
#include <kr_planning_msgs/msg/voxel_map.hpp>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <state_machine/action/replan.hpp>
#include <std_msgs/msg/int64.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "std_msgs/msg/string.hpp"

#include <traj_opt_ros/msg_traj.h>
#include <traj_opt_ros/ros_bridge.h>

#include <boost/timer/timer.hpp>
#include <memory>
#include <mutex>
#include <state_machine/intersect_utils.hpp>
#include <state_machine/traj_opt_utils.hpp>
#include <string>
#include <vector>

// Timer stuff
using boost::timer::cpu_timer;
using boost::timer::cpu_times;

class RePlanner : public rclcpp::Node {
 public:
  using Replan = state_machine::action::Replan;
  using GoalHandleReplan = rclcpp_action::ServerGoalHandle<Replan>;
  using RunTrajectory = action_trackers::action::RunTrajectory;
  using GoalHandleRunTrajectory =
      rclcpp_action::ClientGoalHandle<RunTrajectory>;
  using PlanTwoPoint = kr_planning_msgs::action::PlanTwoPoint;
  using GoalHandlePlanTwoPoint = rclcpp_action::ClientGoalHandle<PlanTwoPoint>;

  RePlanner();
  void init();
  int max_horizon_;
  bool active_{false};

  /**
   * @brief Set up replanner, get an initial plan and execute it
   * only run if the following three conditions are met:
   * if the planner is not active_
   * if ReplanGoalCb is already called
   * if local map callback is already called (path_crop needs local map)
   */
  void setup_replanner();

  /**
   * @brief Update status, where replan_server_ status is set as success if the
   * following two conditions are satisfied: (1)finished_replanning_ is true
   * (distance between user-given goal position and traj evaluated position is
   * less than 1.0) (2)the commanded position and traj end position is less than
   * 1e-3
   */
  void update_status();

 private:
  // Replan action server
  rclcpp_action::Server<Replan>::SharedPtr replan_server_;
  std::shared_ptr<GoalHandleReplan> replan_goal_handle_;

  // action clients
  rclcpp_action::Client<RunTrajectory>::SharedPtr run_client_;
  rclcpp_action::Client<PlanTwoPoint>::SharedPtr local_plan_client_;
  rclcpp_action::Client<PlanTwoPoint>::SharedPtr global_plan_client_;

  std::mutex mtx_;

  // Timing stuff
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr time_pub1;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr time_pub2;
  cpu_timer timer;
  double total_replan_time_ = 0;
  double timer_counter_ = 0;
  double average_time_ = 0;

  // tf_listener
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;

  // reference frame names
  std::string map_frame_;   // map frame
  std::string odom_frame_;  // odom frame

  // local cropped path pub for visualization
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr cropped_path_pub_;

  // transformed global path pub for visualization
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr
      global_path_wrt_map_pub_;

  geometry_msgs::msg::Pose pose_goal_;           // goal recorder
  geometry_msgs::msg::Pose pose_goal_wrt_odom_;  // goal recorder (transformed
                                                 // to account for odom drift)

  int cur_cb_waypoint_idx_{0};  // index of current goal in the array of goals
                                // (waypoints)
  std::vector<geometry_msgs::msg::Pose> pose_goals_;  // an array of goals

  bool finished_replanning_{
      true};  // one of termination conditions (the other is that commanded
              // position and traj end position is less than 1e-3)
  bool do_setup_{false};  // two conditions: if not yet received any goal, don't
                          // do setup; if already setup once, don't do again.

  traj_opt::VecD cmd_pos_{traj_opt::VecD::Zero(4, 1)};  // position command
  boost::shared_ptr<traj_opt::Trajectory>
      last_traj_;  // record of latest trajectory

  rclcpp::Subscription<kr_mav_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<kr_planning_msgs::msg::VoxelMap>::SharedPtr
      local_map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_trigger_sub_;
  kr_planning_msgs::msg::VoxelMap::ConstSharedPtr local_map_ptr_;

  // epoch is to record how many steps have been executed, the duration of one
  // epoch is the execution time, which is 1.0/local_replan_rate_
  int last_plan_epoch_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr epoch_sub_;

  // subscribe to global path
  rclcpp::Subscription<kr_planning_msgs::msg::Path>::SharedPtr global_path_sub_;

  // map update frequency sanity check
  double local_map_last_timestamp_ = -1.0;
  double avg_map_frequency_ = -1.0;
  int map_counter_ = 0;
  double total_map_time_ = -1;

  // local-global framework related params, which should be set in the goal sent
  // from the state_machine
  double local_replan_rate_;
  int global_replan_rate_factor_;

  vec_Vec3f global_path_;  // recorder of path planned by global action server
  double local_timeout_duration_;  // local planner timeout duration
  Vec3f prev_start_pos_;  // replanning records: previous replanning start
                          // position

  // maximum trials of local replan allowed
  int max_local_trials_;
  double failed_local_trials_ = 0;
  unsigned int global_plan_counter_ = 0;
  double crop_radius_;    // local path crop radius (local path length will be
                          // this value)
  double crop_radius_z_;  // local path crop radius along z axis
  double close_to_final_dist_;
  float final_waypoint_threshold_;
  float waypoint_threshold_;
  bool avoid_obstacle_{true};

  // Latest global/local planner results (set by result callbacks)
  kr_planning_msgs::action::PlanTwoPoint::Result::SharedPtr
      latest_global_result_;
  bool latest_global_result_ready_{false};
  kr_planning_msgs::action::PlanTwoPoint::Result::SharedPtr latest_local_result_;
  bool latest_local_result_ready_{false};

  // Latest run trajectory result (set by result callback)
  bool latest_run_result_ready_{false};
  rclcpp_action::ResultCode latest_run_result_code_{
      rclcpp_action::ResultCode::UNKNOWN};

  state_machine::action::Replan::Result critical_;  // create critical bug
                                                    // report

  /**
   * @brief Epoch callback function, triggered by epoch msg published by
   trajectory_tracker_upgraded, it will trigger the replan process (ONLY IF
   current epoch is different than the previously recorded one) by calling
   plan_trajectory and RunTrajectory functions

   */
  void EpochCb(const std_msgs::msg::Int64::ConstSharedPtr msg);

  /**
   * @brief Command callback function, setting cmd_pos_ to be the commanded
   * position
   */
  void CmdCb(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd);

  /**
   * @brief map callback, update local_map_ptr_
   */
  void LocalMapCb(
      const kr_planning_msgs::msg::VoxelMap::ConstSharedPtr msg);

  /**
   * @brief state trigger callback, this will execute high-level commands such
   * as remove next waypoint, or reset mission (remove all waypoints)
   */
  void StateTriggerCb(const std_msgs::msg::String::ConstSharedPtr msg);

  // rclcpp_action server callbacks for Replan
  rclcpp_action::GoalResponse handleReplanGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const Replan::Goal> goal);
  rclcpp_action::CancelResponse handleReplanCancel(
      const std::shared_ptr<GoalHandleReplan> goal_handle);
  void handleReplanAccepted(const std::shared_ptr<GoalHandleReplan> goal_handle);

  /**
   * @brief Global path subscription callback
   */
  void GlobalPathCb(const kr_planning_msgs::msg::Path::ConstSharedPtr path);

  /**
   * @brief Execute the planned trajectory, during which epoch will be published
   * by trajectory tracker and EpochCb will be triggered, which will trigger
   * replanning process
   */
  void RunTrajectoryFn();

  /**
   * @brief Plan trajectory: (1) call global planner, (2) crop global path to
   * get local goal, (3) call local planner
   */
  bool PlanTrajectory(int horizon);

  /**
   * @brief Crop global path for local planner by intersecting it with local map
   * boundaries
   */
  vec_Vec3f PathCropIntersect(const vec_Vec3f& path);

  /**
   * @brief Crop global path for local planner with a fixed distance
   */
  vec_Vec3f PathCropDist(const vec_Vec3f& path,
                         double crop_dist_xyz,
                         double crop_dist_z);

  /**
   * @brief Check if cropped path reaches the end of original path
   */
  bool CloseToFinal(const vec_Vec3f& original_path,
                    const vec_Vec3f& cropped_path,
                    double dist_threshold = 10.0);

  /**
   * @brief transform global path from odom frame to map frame
   */
  vec_Vec3f TransformGlobalPath(const vec_Vec3f& path_original);

  /**
   * @brief transform global path from map frame to odom frame, this is the key
   * step in two reference frame system setup
   */
  void TransformGlobalGoal();

  /**
   * @brief abort the replan process, exit with abort full mission, will transit
   * to hover
   */
  void AbortFullMission(void);

  /**
   * @brief abort the replan process, exit with critical error, will transit
   * to RetryWaypoints (i.e. re-enter the re-planner)
   */
  void AbortReplan(void);

  /**
   * @brief Helper: synchronously wait for a PlanTwoPoint result with a timeout.
   * Returns true if the result came back within the timeout.
   */
  bool SendAndWaitPlanTwoPoint(
      rclcpp_action::Client<PlanTwoPoint>::SharedPtr client,
      const PlanTwoPoint::Goal& goal, double timeout_sec,
      PlanTwoPoint::Result::SharedPtr* out_result);
};
