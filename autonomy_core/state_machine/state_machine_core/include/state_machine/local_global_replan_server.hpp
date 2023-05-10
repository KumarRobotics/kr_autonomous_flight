
#include <action_trackers/RunTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_planning_msgs/Path.h>
#include <kr_planning_msgs/PlanTwoPointAction.h>
#include <kr_planning_msgs/VoxelMap.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <state_machine/ReplanAction.h>
#include <std_msgs/Int64.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "std_msgs/String.h"

// #include <tf/transform_listener.h>
#include <traj_opt_ros/msg_traj.h>
#include <traj_opt_ros/ros_bridge.h>

#include <boost/timer/timer.hpp>
#include <memory>
#include <state_machine/intersect_utils.hpp>
#include <state_machine/traj_opt_utils.hpp>
#include <string>
#include <vector>

// Timer stuff
using boost::timer::cpu_timer;
using boost::timer::cpu_times;

class RePlanner {
 public:
  RePlanner();
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
  std::unique_ptr<actionlib::SimpleActionServer<state_machine::ReplanAction>>
      replan_server_;
  std::unique_ptr<
      actionlib::SimpleActionClient<action_trackers::RunTrajectoryAction>>
      run_client_;
  std::unique_ptr<
      actionlib::SimpleActionClient<kr_planning_msgs::PlanTwoPointAction>>
      local_plan_client_;  // local plan action server client
  std::unique_ptr<
      actionlib::SimpleActionClient<kr_planning_msgs::PlanTwoPointAction>>
      global_plan_client_;  // global plan action server client
  boost::mutex mtx_;

  // Timing stuff
  ros::Publisher time_pub1;
  ros::Publisher time_pub2;
  cpu_timer timer;
  double total_replan_time_ = 0;
  double timer_counter_ = 0;
  double average_time_ = 0;

  // tf_listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListener;

  // reference frame names
  std::string map_frame_;   // map frame
  std::string odom_frame_;  // odom frame

  // local cropped path pub for visualization
  ros::Publisher cropped_path_pub_;

  // transformed global path pub for visualization
  ros::Publisher global_path_wrt_map_pub_;

  geometry_msgs::Pose pose_goal_;           // goal recorder
  geometry_msgs::Pose pose_goal_wrt_odom_;  // goal recorder (transformed to
                                            // account for odom drift)

  int cur_cb_waypoint_idx_{0};  // index of current goal in the array of goals
                                // (waypoints)
  std::vector<geometry_msgs::Pose> pose_goals_;  // an array of goals

  bool finished_replanning_{
      true};  // one of termination conditions (the other is that commanded
              // position and traj end position is less than 1e-3)
  bool do_setup_{false};  // two conditions: if not yet received any goal, don't
                          // do setup; if already setup once, don't do again.

  traj_opt::VecD cmd_pos_{traj_opt::VecD::Zero(4, 1)};  // position command
  boost::shared_ptr<traj_opt::Trajectory>
      last_traj_;  // record of latest trajectory
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber local_map_sub_;
  ros::Subscriber state_trigger_sub_;
  kr_planning_msgs::VoxelMapConstPtr local_map_ptr_;

  // epoch is to record how many steps have been executed, the duration of one
  // epoch is the execution time, which is 1.0/local_replan_rate_
  int last_plan_epoch_;
  ros::Subscriber epoch_sub_;

  // subscribe to global path
  ros::Subscriber global_path_sub_;

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

  state_machine::ReplanResult critical_;  // create critical bug report

  /**
   * @brief Epoch callback function, triggered by epoch msg published by
   trajectory_tracker_upgraded, it will trigger the replan process (ONLY IF
   current epoch is different than the previously recorded one) by calling
   plan_trajectory and RunTrajectory functions

   */
  void EpochCb(const std_msgs::Int64& msg);

  /**
   * @brief Command callback function, setting cmd_pos_ to be the commanded
   * position
   */
  void CmdCb(const kr_mav_msgs::PositionCommand& cmd);

  /**
   * @brief map callback, update local_map_ptr_
   */
  void LocalMapCb(const kr_planning_msgs::VoxelMap::ConstPtr& msg);

  /**
   * @brief state trigger callback, this will execute high-level commands such
   * as remove next waypoint, or reset mission (remove all waypoints)
   */
  void StateTriggerCb(const std_msgs::String::ConstPtr& msg);

  /**
   * @brief Goal callback function
   */
  void ReplanGoalCb();

  /**
   * @brief Goal done callback function
   */
  void GlobalPathCb(const kr_planning_msgs::Path& path);

  /**
   * @brief Execute the planned trajectory, during which epoch will be published
   * by trajectory tracker and EpochCb will be triggered, which will trigger
   * replanning process
   */
  void RunTrajectory();

  /**
   * @brief Plan trajectory: (1) call global planner, (2) crop global path to
   * get local goal, (3) call local planner
   */
  bool PlanTrajectory(int horizon);

  /**
   * @brief Crop global path for local planner by intersecting it with local map
   * boundaries
   * @param path original path to crop
   * @return cropped path, which has at least 2 waypoints, the first element
   * will be the global path start, and the last element will be the
   * intersection between the global map and local voxel map (which is to be
   * used as local goal). The other waypoints are global path's waypoints that
   * lie between the global path's start and the intersection
   *
   */
  vec_Vec3f PathCropIntersect(const vec_Vec3f& path);

  /**
   * @brief Crop global path for local planner with a fixed distance
   * (note: this is not actively used in our stack at the moment)
   * @param path original path to crop
   * @param d length of the cropped path
   *
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
};
