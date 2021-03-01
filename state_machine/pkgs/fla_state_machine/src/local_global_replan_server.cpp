#include <action_planner/PlanPathAction.h>
#include <action_planner/PlanTwoPointAction.h>
#include <action_trackers/RunTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <fla_state_machine/ReplanAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <traj_opt_basic/msg_traj.h>
#include <traj_opt_ros/ros_bridge.h>

#include <fla_state_machine/traj_opt_utils.hpp>

class RePlanner {
 public:
  RePlanner();
  int max_horizon_{5};
  bool active_{false};

  /**
   * @brief Set up replanner, get an initial plan and execute it
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
  std::unique_ptr<
      actionlib::SimpleActionServer<fla_state_machine::ReplanAction>>
      replan_server_;
  std::unique_ptr<
      actionlib::SimpleActionClient<action_trackers::RunTrajectoryAction>>
      run_client_;
  std::unique_ptr<
      actionlib::SimpleActionClient<action_planner::PlanTwoPointAction>>
      local_plan_client_;  // local plan action server client
  std::unique_ptr<
      actionlib::SimpleActionClient<action_planner::PlanTwoPointAction>>
      global_plan_client_;  // global plan action server client
  boost::mutex mtx_;

  geometry_msgs::Pose pose_goal_;  // goal recorder
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

  // epoch is to record how many steps have been executed, the duration of one
  // epoch is the execution time, which is 1.0/replan_rate
  int last_plan_epoch_;
  ros::Subscriber epoch_sub_;

  // local-global framework related params
  double local_replan_rate_;  // should be set in the goal sent from the
                              // state_machine
  int global_replan_interval_{
      2};  // a global replan will be called once every x local replan calls,
           // where x = global_replan_interval_. TODO: maybe also specify global
           // replan rate in goal?
  int local_replan_counter_{
      0};  // counter of local replan calls from the last global replan call
  vec_Vec3f global_path_;  // recorder of path planned by global action server
  double global_timeout_duration_{2.0};  // global planner timeout duration
  double local_timeout_duration_{2.0};   // local planner timeout duration
  Vec3f prev_start_pos_;  // replanning records: previous replanning start
                          // position
  decimal_t executed_dist_{
      0.0};  // replanning records: accumulated executed distance along the path
  double crop_radius_;  // local path crop radius (local path length will be
                        // this value)
  double termination_distance_;
  bool avoid_obstacle_{true};

  /**
   * @brief Epoch callback function, triggered by epoch msg published by
   trajectory_tracker, it will trigger the replan process (ONLY IF current epoch
   is different than the previously recorded one) by calling plan_trajectory and
   run_trajectory functions

   */
  void epoch_cb(const std_msgs::Int64 &msg);

  /**
   * @brief Command callback function, setting cmd_pos_ to be the commanded
   * position
   */
  void cmd_cb(const kr_mav_msgs::PositionCommand &cmd);

  /**
   * @brief Goal callback function
   */
  void replan_goal_cb();

  /**
   * @brief Execute the planned trajectory, during which epoch will be published
   * by trajectory tracker and epoch_cb will be triggered, which will trigger
   * replanning process
   */
  void run_trajectory();

  /**
   * @brief Plan trajectory: (1) call global planner, (2) crop global path to
   * get local goal, (3) call local planner
   */
  bool plan_trajectory(int horizon);

  /**
   * @brief Crop global path for local planner
   * @param ps original path to crop
   * @param d length of the cropped path
   */
  vec_Vec3f path_crop(const vec_Vec3f &ps, decimal_t d);
};

void RePlanner::epoch_cb(const std_msgs::Int64 &msg) {
  boost::mutex::scoped_lock lock(mtx_);
  static int epoch_old = -1;  // keep a record of last epoch

  // replan ONLY IF current epoch is different than the previously recorded one
  if (epoch_old != msg.data) {
    int horizon;
    if (epoch_old == -1) {
      horizon = 1;
    } else {
      // msg.data = current_epoch_ + int(std::floor(duration/execution_time)),
      // where duration = (t_now - started_).toSec() horizon = 1 +
      // (current_plan_epoch - last_plan_epoch). Duration of one epoch is
      // execution_time, which is 1.0/replan_rate
      horizon = msg.data - last_plan_epoch_ + 1;
    }
    epoch_old = msg.data;  // keep a record of last epoch

    // trigger replan, set horizon
    if (!finished_replanning_) {
      if (plan_trajectory(horizon))
        // execute the replanned trajectory
        run_trajectory();
    }
  } else {
    ROS_WARN_STREAM("Epoch is the same as previous call, epoch = "
                    << msg.data << ", skip re-planning");
  }
  update_status();
}

void RePlanner::cmd_cb(const kr_mav_msgs::PositionCommand &cmd) {
  boost::mutex::scoped_lock lock(mtx_);
  cmd_pos_(0) = cmd.position.x;
  cmd_pos_(1) = cmd.position.y;
  cmd_pos_(2) = cmd.position.z;
  cmd_pos_(3) = cmd.yaw;
}

void RePlanner::replan_goal_cb() {
  boost::mutex::scoped_lock lock(mtx_);
  // accept new goal (ref:
  // http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a4964ef9e28f5620e87909c41f0458ecb)
  auto goal = replan_server_->acceptNewGoal();
  local_replan_rate_ =
      goal->replan_rate;  // set local planner replan_rate to be the same as
                          // specified in goal (assigned in state machine)
  pose_goal_ = goal->p_final;
  avoid_obstacle_ = goal->avoid_obstacles;  // obstacle avoidance mode (assigned
                                            // in state machine)
  // create critical bug report
  fla_state_machine::ReplanResult critical;
  critical.status = fla_state_machine::ReplanResult::CRITICAL_ERROR;

  // check cmd
  if (cmd_pos_.norm() == 0) {
    ROS_ERROR("RePlanner has not received position cmd, failing");
    active_ = false;
    replan_server_->setSucceeded(critical);
    return;
  }

  if (!active_)        // if not active, do setup again
    do_setup_ = true;  // only run setup_replanner function after replan_goal_cb
}

void RePlanner::setup_replanner() {
  if (!do_setup_ || active_) return;
  do_setup_ = false;  // only run setup_replanner once
  boost::mutex::scoped_lock lock(mtx_);

  ROS_INFO_STREAM("Setup replan");
  if (!avoid_obstacle_) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("Obstacle avoidance is disabled!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  }

  fla_state_machine::ReplanResult critical;
  critical.status = fla_state_machine::ReplanResult::CRITICAL_ERROR;

  // Initial plan step 1: global plan
  // ########################################################################################################
  // set goal
  action_planner::PlanTwoPointGoal global_tpgoal;
  global_tpgoal.p_final = pose_goal_;
  global_tpgoal.avoid_obstacles = avoid_obstacle_;
  // send goal to global plan action server
  global_plan_client_->sendGoal(
      global_tpgoal);  // only send goal, because global plan server is
                       // subscribing to odom and use that as start
  bool global_finished_before_timeout = global_plan_client_->waitForResult(
      ros::Duration(global_timeout_duration_));
  // check result of global plan
  if (!global_finished_before_timeout) {
    ROS_ERROR("initial global planner timed out");
    fla_state_machine::ReplanResult critical;
    critical.status = fla_state_machine::ReplanResult::CRITICAL_ERROR;
    active_ = false;
    if (replan_server_->isActive()) {
      replan_server_->setSucceeded(critical);
    }
    return;
  }
  auto global_result = global_plan_client_->getResult();
  if (global_result->success) {
    global_path_ = ros_to_path(
        global_result->path);  // extract the global path information
    ROS_WARN("initial global plan succeeded!");
  } else {
    ROS_WARN("initial global plan failed!");
    return;
  }

  //  Initial plan step 2: Crop global path to get local goal
  //  #################################################################################
  vec_Vec3f path_cropped = path_crop(global_path_, crop_radius_);

  // Initial plan step 3: local plan
  // ##########################################################################################################
  // set goal
  action_planner::PlanTwoPointGoal local_tpgoal;
  TrajOptUtils::vec_to_pose(
      cmd_pos_, &local_tpgoal.p_init);  // use current position command as the
                                        // start position for local planner
  // initialize prev_start_pos_ for replan purpose
  prev_start_pos_(0) = cmd_pos_(0);
  prev_start_pos_(1) = cmd_pos_(1);
  prev_start_pos_(2) = cmd_pos_(2);

  // set vars
  local_tpgoal.epoch = 1;
  local_tpgoal.avoid_obstacles = avoid_obstacle_;
  local_tpgoal.execution_time = ros::Duration(1.0 / local_replan_rate_);
  // set p_final to be path_cropped.back(), which is exactly at accumulated
  // distance d from the robot (unless path is shorter than d, crop_end will be
  // default as the end of path)
  Vec3f local_goal = path_cropped.back();
  local_tpgoal.p_final.position.x = local_goal(0);
  local_tpgoal.p_final.position.y = local_goal(1);
  // TODO(xu): temporarily setting start and goal to have same z value due to
  // local
  // motion primitive planner seems to be unable to handle 3D
  local_tpgoal.p_final.position.z = local_tpgoal.p_init.position.z;
  // local_tpgoal.p_final.position.z = local_goal(2);

  // send goal to local trajectory plan action server
  local_plan_client_->sendGoal(local_tpgoal);

  // wait for result
  bool local_finished_before_timeout =
      local_plan_client_->waitForResult(ros::Duration(local_timeout_duration_));
  // check result of local plan
  fla_state_machine::ReplanResult local_critical;
  local_critical.status = fla_state_machine::ReplanResult::CRITICAL_ERROR;
  if (!local_finished_before_timeout) {
    ROS_ERROR("Initial local planner timed out");
    active_ = false;
    if (replan_server_->isActive()) {
      replan_server_->setSucceeded(local_critical);
    }
    return;
  }
  auto local_result = local_plan_client_->getResult();
  if (!local_result->success) {
    ROS_ERROR("Failed to find a local trajectory!");
    active_ = false;
    replan_server_->setSucceeded(local_critical);
    return;
  }

  // get the result
  last_traj_ = boost::make_shared<traj_opt::MsgTrajectory>(
      TrajRosBridge::convert(local_result->traj));
  last_plan_epoch_ = local_result->epoch;

  // setup state vars
  finished_replanning_ = false;

  // Initial plan step 4: execute the planned trajectory, during which
  // replanning process will also be triggered
  ROS_INFO("Started replanning!");
  run_trajectory();
}

void RePlanner::run_trajectory() {
  // if(!replan_server_->isActive()) {
  //   return;
  // }

  // set up run goal
  action_trackers::RunTrajectoryGoal rungoal;
  rungoal.traj = TrajRosBridge::convert(last_traj_->serialize());
  rungoal.epoch = last_plan_epoch_;
  rungoal.replan_rate = local_replan_rate_;

  // call the trajectory tracker
  run_client_->sendGoal(rungoal);
  // ROS_INFO("Sent Trajectory!");
  bool tracker_finished_before_timeout =
      run_client_->waitForResult(ros::Duration(0.01));
}

bool RePlanner::plan_trajectory(int horizon) {
  // horizon = 1 + (current_plan_epoch - last_plan_epoch). Duration of one epoch
  // is execution_time, which is 1.0/replan_rate
  if (horizon > max_horizon_) {
    fla_state_machine::ReplanResult abort;
    abort.status = fla_state_machine::ReplanResult::DYNAMICALLY_INFEASIBLE;
    active_ = false;
    if (replan_server_->isActive()) replan_server_->setSucceeded(abort);
    return false;
  }

  // set global goal
  action_planner::PlanTwoPointGoal global_tpgoal;
  global_tpgoal.p_final = pose_goal_;
  global_tpgoal.avoid_obstacles = avoid_obstacle_;

  // set local goal
  action_planner::PlanTwoPointGoal local_tpgoal;
  double eval_time =
      double(horizon) /
      local_replan_rate_;  // calculate evaluation time, i.e., beginning of
                           // current plan epoch (in seconds)
  // make the end of last trajectory consistent with the start of current
  // trajectory
  TrajOptUtils::evaluate_to_msgs(last_traj_, eval_time, &local_tpgoal.p_init,
                                 &local_tpgoal.v_init, &local_tpgoal.a_init,
                                 &local_tpgoal.j_init);
  Vec3f start_pos;
  start_pos = pose_to_eigen(local_tpgoal.p_init);

  // Re-plan step 1: Global plan
  // ########################################################################################################
  local_replan_counter_++;  // only do global replan once every
                            // global_replan_interval_ local replans
  if (local_replan_counter_ == global_replan_interval_) {
    executed_dist_ = 0.0;       // reset the executed distance to be zero
    local_replan_counter_ = 0;  // reset the counter
    // send goal to global plan action server
    global_plan_client_->sendGoal(global_tpgoal);
    bool global_finished_before_timeout = global_plan_client_->waitForResult(
        ros::Duration(global_timeout_duration_));
    // check result of global plan
    if (!global_finished_before_timeout) {
      ROS_ERROR("Global planner timed out");
      fla_state_machine::ReplanResult critical;
      critical.status = fla_state_machine::ReplanResult::CRITICAL_ERROR;
      active_ = false;
      if (replan_server_->isActive()) {
        replan_server_->setSucceeded(critical);
      }
      return false;
    }

    auto global_result = global_plan_client_->getResult();
    if (global_result->success) {
      global_path_.clear();
      global_path_ = ros_to_path(
          global_result->path);  // extract the global path information
    } else {
      ROS_WARN("Global plan failed, trying replan");
      return false;
    }
  } else {
    // incrementally record executed_dist_
    executed_dist_ =
        executed_dist_ + (start_pos - prev_start_pos_)
                             .norm();  // straight line distance to approximate
                                       // executed portion of path
    prev_start_pos_ = start_pos;       // keep updating prev_start_pos_
    if (executed_dist_ > 5.0) {
      ROS_WARN_STREAM(
          "++++ executed distance is larger than 5 meters: " << executed_dist_);
    }
  }

  //  Re-plan step 2: Crop global path to get local goal
  //  #################################################################################
  // total crop distance
  decimal_t crop_dist = executed_dist_ + crop_radius_;
  // ROS_WARN_STREAM("++++ total_crop_dist = " << crop_dist);
  vec_Vec3f path_cropped = path_crop(global_path_, crop_dist);

  // Re-plan step 3: local plan
  // ##########################################################################################################
  // set vars
  local_tpgoal.avoid_obstacles = avoid_obstacle_;
  local_tpgoal.execution_time = ros::Duration(1.0 / local_replan_rate_);
  local_tpgoal.epoch = last_plan_epoch_ + horizon;
  // change p_final to be path_cropped.back(), which is exactly at accumulated
  // distance d from the robot (unless path is shorter than d, crop_end will be
  // default as the end of path)
  Vec3f local_goal = path_cropped.back();
  local_tpgoal.p_final.position.x = local_goal(0);
  local_tpgoal.p_final.position.y = local_goal(1);
  // TODO(xu): temporarily setting start and goal to have same z due to local
  // motion primitive planner seems to be unable to handle 3D
  local_tpgoal.p_final.position.z = local_tpgoal.p_init.position.z;
  // local_tpgoal.p_final.position.z = local_goal(2);

  // send goal to local trajectory plan action server
  local_plan_client_->sendGoal(local_tpgoal);

  // wait for result
  bool local_finished_before_timeout =
      local_plan_client_->waitForResult(ros::Duration(local_timeout_duration_));
  // check result of local plan
  fla_state_machine::ReplanResult local_critical;
  local_critical.status = fla_state_machine::ReplanResult::CRITICAL_ERROR;
  if (!local_finished_before_timeout) {
    ROS_ERROR("Local planner timed out");
    active_ = false;
    if (replan_server_->isActive()) {
      replan_server_->setSucceeded(local_critical);
    }
    return false;
  }
  auto local_result = local_plan_client_->getResult();
  if (local_result->success) {
    last_traj_ = boost::make_shared<traj_opt::MsgTrajectory>(
        TrajRosBridge::convert(local_result->traj));
    last_plan_epoch_ = local_result->epoch;
    // ROS_INFO_STREAM("Got local plan with epoch " << last_plan_epoch_);
    return true;
  } else if (local_result->policy_status < 0) {
    ROS_ERROR_STREAM("Local policy_status " << local_result->policy_status);
    active_ = false;
    if (replan_server_->isActive())
      replan_server_->setSucceeded(local_critical);
    return false;
  } else {
    ROS_WARN("Local plan failed, trying replan");
    return false;
  }
  return true;
}

void RePlanner::update_status() {
  // check for termination
  if (last_traj_ != NULL) {
    traj_opt::VecD pos_goal = traj_opt::VecD::Zero(4, 1);
    traj_opt::VecD pos_final, pos_finaln;
    pos_goal(0) = pose_goal_.position.x;
    pos_goal(1) = pose_goal_.position.y;

    // evaluate traj at 1.0/local_replan_rate_, output recorded to pos_finaln,
    // refer to msg_traj.cpp.
    // TODO(xu): maybe instead of doing 1.0/local_replan_rate_, we should use
    // last_traj_->getTotalTime() since this is for checking the termination?
    last_traj_->evaluate(1.0 / local_replan_rate_, 0,
                         pos_finaln);  // TODO(mike) {fix this}.

    pos_final = TrajOptUtils::make4d(pos_finaln);
    pos_final(2) = 0;
    pos_final(3) = 0;

    // finished_replanning_ is set as true if goal position and traj evaluated
    // position is less than a threshold
    if ((pos_goal - pos_final).norm() <= termination_distance_) {
      finished_replanning_ = true;
    }
  }

  // check for termination, evaluating trajectory and see if it's close to
  // pos_final
  if (last_traj_ != NULL && finished_replanning_) {
    ROS_INFO_STREAM_THROTTLE(1, "Waiting for traj termination");
    traj_opt::VecD pos_no_yaw = cmd_pos_;
    traj_opt::VecD pos_final, pos_finaln;
    // evaluate traj at last_traj_->getTotalTime(), output recorded to
    // pos_finaln,
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    pos_final = TrajOptUtils::make4d(pos_finaln);

    pos_no_yaw(2) = 0;
    pos_no_yaw(3) = 0;
    pos_final(2) = 0;
    pos_final(3) = 0;
    ROS_INFO_STREAM_THROTTLE(
        1, "Termination error: " << (pos_no_yaw - pos_final).norm());

    // replan_server_ set as success if the commanded position and traj end
    // position is less than 1e-3
    if ((pos_no_yaw - pos_final).norm() <= 1e-3) {
      fla_state_machine::ReplanResult success;
      success.status = fla_state_machine::ReplanResult::SUCCESS;
      active_ = false;
      if (replan_server_->isActive()) replan_server_->setSucceeded(success);
      ROS_INFO("RePlanner success!!");
      last_traj_ = boost::shared_ptr<traj_opt::Trajectory>();
      return;
    }
  }

  fla_state_machine::ReplanResult in_progress;
  in_progress.status = fla_state_machine::ReplanResult::IN_PROGRESS;

  if (replan_server_->isActive()) {
    active_ = true;
    // replan_server_->setSucceeded(in_progress);
  }
}

vec_Vec3f RePlanner::path_crop(const vec_Vec3f &ps, decimal_t d) {
  // return nonempth
  // precondition
  if (ps.size() < 2 || d < 0) return ps;

  vec_Vec3f path;
  Vec3f crop_end = ps.back();  // if path is shorter than d, crop_end will be
                               // default as the end of path
  decimal_t dist = 0;

  // add path segments until the accumulated length is farther than distance d
  // from the robot
  for (unsigned int i = 1; i < ps.size(); i++) {
    if (dist + (ps[i] - ps[i - 1]).norm() > d) {
      crop_end =
          ps[i - 1] +
          (d - dist) *
              (ps[i] - ps[i - 1])
                  .normalized();  // assign end to be exactly at
                                  // accumulated distance d from the robot
      path.push_back(ps[i - 1]);
      break;
    } else {
      dist += (ps[i] - ps[i - 1]).norm();
      path.push_back(ps[i - 1]);
    }
  }

  if ((path.back() - crop_end).norm() > 1e-1)
    path.push_back(crop_end);  // crop_end is exactly at accumulated distance d
                               // from the robot (unless path is shorter than d,
                               // crop_end will be default as the end of path)

  // post condition
  // CHECK(glog) path is not empty
  return path;
}

RePlanner::RePlanner() : nh_("~") {
  ros::NodeHandle priv_nh(nh_, "local_global_server");
  priv_nh.param("max_horizon", max_horizon_, 5);
  priv_nh.param("crop_radius", crop_radius_, 10.0);
  priv_nh.param("termination_distance", termination_distance_, 5.0);

  // replan action server
  replan_server_.reset(
      new actionlib::SimpleActionServer<fla_state_machine::ReplanAction>(
          nh_, "replan", false));

  // plan global path action client, auto spin option set to true
  global_plan_client_.reset(
      new actionlib::SimpleActionClient<action_planner::PlanTwoPointAction>(
          nh_, "plan_global_path", true));

  // plan local trajectory action client, auto spin option set to true
  local_plan_client_.reset(
      new actionlib::SimpleActionClient<action_planner::PlanTwoPointAction>(
          nh_, "plan_local_trajectory", true));

  // run trajectory action client
  run_client_.reset(
      new actionlib::SimpleActionClient<action_trackers::RunTrajectoryAction>(
          nh_, "execute_trajectory", true));

  // subscriber of position command
  // command callback: setting cmd_pos_ to be the commanded position
  cmd_sub_ = nh_.subscribe("position_cmd", 1, &RePlanner::cmd_cb, this);

  // subscriber of epoch command, epoch is published by trajectory tracker
  // epoch callback: trigger replan, set horizon
  epoch_sub_ = nh_.subscribe("epoch", 1, &RePlanner::epoch_cb, this);

  // Goal callback
  replan_server_->registerGoalCallback(
      boost::bind(&RePlanner::replan_goal_cb, this));

  replan_server_->start();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "replanner");
  ros::NodeHandle nh;
  RePlanner replanner;
  ros::Rate r(5);  // Should be at least at the rate of local_replan_rate_!
  while (nh.ok()) {
    r.sleep();
    replanner.setup_replanner();  // this function will only run AFTER the
                                  // replan_goal_cb, and will only run ONCE
    replanner.update_status();
    ros::spinOnce();
  }
}
