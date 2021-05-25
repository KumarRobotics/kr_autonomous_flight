#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <tf/transform_datatypes.h>

// necessary changes after refactoring
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_tracker_msgs/TrackerStatus.h>

// quad control stuff
#include <kr_trackers_manager/Tracker.h>

// planning ros msgs stuff
#include <planning_ros_msgs/SplineTrajectory.h>

// traj_opt stuff
#include <std_msgs/Empty.h>
#include <traj_opt_ros/msg_traj.h>
#include <traj_opt_ros/ros_bridge.h>
#include <traj_opt_ros/traj_to_quad_cmd.h>

// action stuff
#include <action_trackers/RunTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
// boost
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
// angles
#include <angles/angles.h>
#include <geometry_msgs/Vector3.h>

class ActionTrajectoryTracker : public kr_trackers_manager::Tracker {
 public:
  // Initialize, Activate, Deactivate, update functions are all called by
  // trackers_manager.cpp Goal will have information about trajectory
  // (goal->traj), epoch (goal->epoch), replan_rate (goal->replan_rate) and
  // bloack (goal->block)

  /**
   * @brief initialize tracker
   */
  void Initialize(const ros::NodeHandle &nh) override;

  /**
   * @brief activate tracker
   */
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd) override;

  /**
   * @brief deactivate tracker
   */
  void Deactivate(void) override;

  /**
   * @brief Update function:
   * Part1: check existance of current traj and next traj, check continuity in
   * position and velocity of current traj and new traj Part2: evaluate
   * trajectory and get the cmd
   * @param use_lambda_ if true and if trajectory dimension is not 9, will check
   * difference in position between odometry and trajectory is larger than
   * pos_err_max_, and (2) the position difference is along the trajectory
   * velocity direction (i.e. odom ahead of trajectory)
   * @param use_yaw_ if true and if trajectory dimension is not 9, will check
   * yaw. If both use_lambda_ and use_yaw_ set as false, will check nothing
   */
  kr_mav_msgs::PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg) override;

  /**
   * @brief Goal callback function.
   */
  void trajCB();

  void preemptCB();
  uint8_t status() const override;

 private:
  boost::shared_ptr<ros::NodeHandle> nh_;
  boost::shared_ptr<traj_opt::Trajectory> current_trajectory_;
  std::vector<boost::shared_ptr<traj_opt::Trajectory>> next_trajectory_;

  // vsion

  ros::Time started_;
  // sequencing
  double execution_time{-1};
  int current_epoch_{0};
  std::vector<int> traj_epoch;

  bool active_;
  bool done_{false};
  // gains
  double kx_[3], kv_[3];
  double pos_err_max_;
  traj_opt::Vec3 K_lambda, Limits_lamda, state_lambda;
  kr_mav_msgs::PositionCommand::Ptr cmd;

  kr_mav_msgs::PositionCommand::Ptr init_cmd_;

  // action lib
  std::unique_ptr<
      actionlib::SimpleActionServer<action_trackers::RunTrajectoryAction>>
      as_;
  bool prempted{false};
  double prempted_time;
  double yaw_, start_yaw_;
  double yaw_thr_;
  boost::mutex mtx_;
  bool use_yaw_, use_lambda_;
  bool ignore_yaw_;  // should we check trajectory yaw
  double yaw_speed_;
  ros::Publisher epoch_pub_, point_pub_, lambda_pub_;

  static traj_opt::VecD make4d(const traj_opt::VecD &vec) {
    traj_opt::VecD out = traj_opt::VecD::Zero(4);
    long int rows = std::min(vec.rows(), static_cast<long int>(4));
    out.block(0, 0, rows, 1) = vec.block(0, 0, rows, 1);
    return out;
  }
};

void ActionTrajectoryTracker::Initialize(const ros::NodeHandle &nh) {
  nh_ = boost::make_shared<ros::NodeHandle>(nh);
  nh_->param("gains/pos/x", kx_[0], 2.5);
  nh_->param("gains/pos/y", kx_[1], 2.5);
  nh_->param("gains/pos/z", kx_[2], 5.0);
  nh_->param("gains/vel/x", kv_[0], 2.2);
  nh_->param("gains/vel/y", kv_[1], 2.2);
  nh_->param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "trajectory_tracker");
  priv_nh.param("max_pos_error", pos_err_max_, 1.0);
  priv_nh.param("use_yaw", use_yaw_, false);
  priv_nh.param("yaw_thr", yaw_thr_, 3.14159);
  priv_nh.param("use_lambda", use_lambda_, true);  // pose error checking
  priv_nh.param("yaw_speed", yaw_speed_, 0.2);
  priv_nh.param("ignore_yaw", ignore_yaw_, false);

  epoch_pub_ = nh_->advertise<std_msgs::Int64>("epoch", 10);
  point_pub_ = nh_->advertise<geometry_msgs::PointStamped>("roi", 10);
  lambda_pub_ = nh_->advertise<geometry_msgs::Vector3>("lambda", 10);

  active_ = false;
  start_yaw_ = 0;
  if (ignore_yaw_) use_yaw_ = true;

  as_.reset(
      new actionlib::SimpleActionServer<action_trackers::RunTrajectoryAction>(
          nh, "execute_trajectory", false));

  // Register goal and preempt callbacks
  as_->registerGoalCallback(
      boost::bind(&ActionTrajectoryTracker::trajCB, this));
  as_->registerPreemptCallback(
      boost::bind(&ActionTrajectoryTracker::preemptCB, this));

  as_->start();
}

bool ActionTrajectoryTracker::Activate(
    const kr_mav_msgs::PositionCommand::ConstPtr &msg) {
  boost::mutex::scoped_lock lock(mtx_);

  if (next_trajectory_.size() == 0) {
    if (msg == NULL)
      return false;
    else {
      init_cmd_.reset(new kr_mav_msgs::PositionCommand(*(msg)));
      start_yaw_ = yaw_;
      active_ = true;
      return true;
    }
  } else {
    active_ = true;
    start_yaw_ = yaw_;
    if (msg != NULL) init_cmd_.reset(new kr_mav_msgs::PositionCommand(*(msg)));
    return true;
  }
}

void ActionTrajectoryTracker::Deactivate(void) {
  boost::mutex::scoped_lock lock(mtx_);
  active_ = false;
  init_cmd_ = kr_mav_msgs::PositionCommand::Ptr();
  current_trajectory_ = boost::shared_ptr<traj_opt::Trajectory>();
  next_trajectory_.clear();
  traj_epoch.clear();
  current_epoch_ = 0;
  execution_time = -1;
  started_ = ros::Time(0);
}

kr_mav_msgs::PositionCommand::ConstPtr ActionTrajectoryTracker::update(
    const nav_msgs::Odometry::ConstPtr &msg) {
  boost::mutex::scoped_lock lock(mtx_);

  if (!active_) {
    return kr_mav_msgs::PositionCommand::ConstPtr();
  }

  // ROS_ERROR_STREAM_THROTTLE(1,"Trajectory tracker active with
  // current_trajectory_" << (current_trajectory_ == NULL));
  // ROS_ERROR_STREAM_THROTTLE(1,"Trajectory tracker active with
  // next_trajectory_" << next_trajectory_.size() << " init_cmd " << (init_cmd_
  // == NULL) << " epoch size " << traj_epoch.size());

  // no current traj and next traj, return init_cmd_
  if (current_trajectory_ == NULL && next_trajectory_.size() == 0) {
    init_cmd_->header.stamp = ros::Time::now();
    return init_cmd_;
  }

  const ros::Time t_now = ros::Time::now();
  double duration = (t_now - started_).toSec();
  geometry_msgs::Vector3 lambda_v;
  lambda_v.x = state_lambda(0);
  lambda_v.y = state_lambda(1);
  lambda_v.z = state_lambda(2);
  lambda_pub_.publish(lambda_v);

  // no current traj but have next traj, check continuity first
  if (current_trajectory_ == NULL && next_trajectory_.size() > 0) {
    started_ = ros::Time::now();
    current_epoch_ = traj_epoch.front();
    duration = 0.0;
    // check for initial continuity
    traj_opt::VecD pos_old(4, 1), pos_new;
    pos_old << init_cmd_->position.x, init_cmd_->position.y,
        init_cmd_->position.z, init_cmd_->yaw;
    next_trajectory_.front()->evaluate(0.0, 0, pos_new);

    if (pos_new.rows() < 4) {
      pos_old.conservativeResize(pos_new.rows(), 1);
    } else if (pos_new.rows() == 9) {  // special sphere traj thing
      pos_old = pos_old.block(0, 0, 3, 1);
      pos_new = pos_new.block(0, 0, 3, 1);
    }

    if (pos_new.rows() >= 4) {
      // robust to wrapping by pi
      pos_old(3) = std::cos(pos_old(3));
      pos_new(3) = std::cos(pos_new(3));
    }

    // check continuity in position
    if ((pos_old - pos_new).norm() > 3.0) {
      // incontinuity is too large, abort
      ROS_ERROR_STREAM("Trajectories not continuous, aborting transition from "
                       << pos_old.transpose() << " to " << pos_new.transpose());
      next_trajectory_.erase(next_trajectory_.begin());
      traj_epoch.erase(traj_epoch.begin());
      init_cmd_->header.stamp = ros::Time::now();
      if (next_trajectory_.size() == 0) done_ = true;
      return init_cmd_;
    } else {
      // incontinuity is within threshold, accept next_trajectory_ as new
      // trajectory
      current_trajectory_ = next_trajectory_.front();
      next_trajectory_.erase(next_trajectory_.begin());
      current_epoch_ = traj_epoch.front();
      traj_epoch.erase(traj_epoch.begin());
      state_lambda(0) = 0;
      ROS_INFO_STREAM("Accepted new trajectory. Queue sizes: "
                      << next_trajectory_.size() << " and "
                      << traj_epoch.size());
    }
  }
  // both current traj and next traj exist
  else if (next_trajectory_.size() > 0) {
    int epoch_diff = traj_epoch.front() - current_epoch_;

    // check_time is the timestamp to stitch old traj and new traj together
    double check_time = execution_time * double(epoch_diff);
    if (duration >= check_time) {
      // only do something if trajectory is old
      traj_opt::VecD pos_old, pos_new;
      traj_opt::VecD vel_old, vel_new;

      // evaluate current traj at check_time
      current_trajectory_->evaluate(check_time, 0, pos_old);
      current_trajectory_->evaluate(check_time, 1, vel_old);

      // evaluate next traj at time=0
      next_trajectory_.front()->evaluate(0.0, 0, pos_new);
      next_trajectory_.front()->evaluate(0.0, 1, vel_new);

      if (pos_new.rows() < 4) {
        pos_old.conservativeResize(pos_new.rows(), 1);
        vel_old.conservativeResize(vel_new.rows(), 1);
      } else if (pos_new.rows() == 9) {  // special sphere traj thing
        pos_old = pos_old.block(0, 0, 3, 1);
        pos_new = pos_new.block(0, 0, 3, 1);
      }

      if (pos_new.rows() >= 4) {
        pos_old(3) = std::cos(pos_old(3));
        pos_new(3) = std::cos(pos_new(3));
      }

      // check for continuity
      if ((pos_old - pos_new).norm() > 1.0) {
        // incontinuity in position is too large, abort
        ROS_ERROR_STREAM("Replan not continuous from " << pos_old.transpose()
                                                       << " to "
                                                       << pos_new.transpose());
        ROS_ERROR_STREAM("current epoch: "
                         << current_epoch_
                         << " traj epoch: " << traj_epoch.front()
                         << " execution time: " << execution_time
                         << " check time " << check_time);
        next_trajectory_.erase(next_trajectory_.begin());
        traj_epoch.erase(traj_epoch.begin());

      } else if ((vel_old - vel_new).norm() > 1.0) {
        // incontinuity in velocity is too large, abort
        ROS_ERROR_STREAM("Replan velocity not continuous from "
                         << vel_old.transpose() << " to "
                         << vel_new.transpose());
        ROS_ERROR_STREAM("current epoch: "
                         << current_epoch_
                         << " traj epoch: " << traj_epoch.front()
                         << " execution time: " << execution_time);
        next_trajectory_.erase(next_trajectory_.begin());
        traj_epoch.erase(traj_epoch.begin());

      } else {
        // incontinuity in vel and position are both within threshold, accept
        // next_trajectory_ as new trajectory
        current_trajectory_ = next_trajectory_.front();
        next_trajectory_.erase(next_trajectory_.begin());
        current_epoch_ = traj_epoch.front();
        traj_epoch.erase(traj_epoch.begin());
        started_ = ros::Time::now();
        duration = 0.0;
        state_lambda(0) = 0;
        // ROS_INFO_STREAM("Accepted replan Queue sizes: "<<
        // next_trajectory_.size() << " and " << traj_epoch.size() );
      }
    }
  }

  // kr_mav_msgs::PositionCommand::Ptr cmd;

  // reset command
  cmd.reset(new kr_mav_msgs::PositionCommand);
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  //  if (duration < 1.0 || duration > 10.0)
  // ROS_ERROR_STREAM_THROTTLE(1, "Evaluating duration" << duration);

  // evaluate trajectory and get the cmd,  see traj_opt_ros/traj_to_quad_cmd.h
  if (use_lambda_) {
    // evaluatePos will return false if difference in position between odometry
    // and trajectory is larger than pos_err_max_
    if (!traj_opt::EvaluateTrajectoryPos(current_trajectory_, msg, pos_err_max_,
                                         duration, 0.01, cmd.get())) {
      // started_ = started_ + ros::Duration(0.01); // this slowing down
      // strategy is not working properly, commented out
      ROS_ERROR(
          "[Traj Tracker:] Max position error threshold violated!! Slow "
          "down the execution!!!");
    }
  } else {
    // no pos or yaw check
    traj_opt::EvaluateTrajectory(current_trajectory_, duration, cmd.get(), 2);
  }

  //  cmd->yaw = start_yaw_, cmd->yaw_dot = 0;
  init_cmd_ = cmd;

  // check if current trajectory is finished
  if (duration > current_trajectory_->getTotalTime()) {
    ROS_INFO_STREAM("Finished trajectories at time "
                    << duration << " out of "
                    << current_trajectory_->getTotalTime());
    ROS_INFO_STREAM("Null " << (next_trajectory_.size() == 0)
                            << " current epoch " << current_epoch_
                            << " traj epoch " << traj_epoch.front());
    init_cmd_->header.stamp = ros::Time::now();
    done_ = true;
    current_trajectory_ = boost::shared_ptr<traj_opt::Trajectory>();

    if (as_->isActive()) {
      action_trackers::RunTrajectoryResult result;
      as_->setSucceeded(result);
    }
  }

  // publish current epoch
  std_msgs::Int64 epoch_msg;
  epoch_msg.data = current_epoch_ + int(std::floor(duration / execution_time));
  epoch_pub_.publish(epoch_msg);

  return cmd;
}

void ActionTrajectoryTracker::trajCB() {
  // traj_opt::Timer t; // Timer initialization
  auto goal = as_->acceptNewGoal();
  boost::mutex::scoped_lock lock(mtx_);
  // double sec = t.toc();  // return time between Timer initialization and this
  // point if(sec > 0.2) ROS_ERROR_STREAM("[Traj Tracker:] thread blocked for "
  // << sec);

  done_ = false;

  if (traj_epoch.size() > 0 && (traj_epoch.front() > goal->epoch)) {
    ROS_ERROR_STREAM("Traj epoch > goal epoch. Reset queue!");
    next_trajectory_.clear();
    traj_epoch.clear();
  }

  while (traj_epoch.size() > 0 && traj_epoch.back() >= goal->epoch) {
    next_trajectory_.pop_back();
    traj_epoch.pop_back();
  }
  // extract trajectory msg from goal->traj
  planning_ros_msgs::SplineTrajectory msg = goal->traj;
  boost::shared_ptr<traj_opt::Trajectory> sp =
      boost::make_shared<traj_opt::MsgTrajectory>(
          traj_opt::TrajDataFromSplineTrajectory(msg));
  next_trajectory_.push_back(sp);
  prempted = false;
  // ROS_ERROR_STREAM("Done Callback with epoch " << goal->epoch);
  if (goal->replan_rate > 0 && goal->epoch <= 0) {
    ROS_ERROR("Replan rate and epoch must both be set! Rejecting Trajectory");
    return;
  }

  // set ignore_yaw according to the dimension of goal->traj
  if (msg.dimensions < 4) {
    ignore_yaw_ = true;
  }

  // change mode to either replanning or normal
  if (goal->replan_rate > 0) {
    execution_time = 1.0 / goal->replan_rate;
    // replanning mode will have non-zero traj_epoch (goal->epoch)
    traj_epoch.push_back(goal->epoch);
  } else {
    execution_time = -1;
    traj_epoch.push_back(0);
    current_epoch_ = 0;
  }

  // check for continuity
  traj_opt::TrajRosBridge::publish_msg(msg);

  action_trackers::RunTrajectoryResult result;
  if (as_->isActive() && !goal->block) as_->setSucceeded(result);

  ROS_INFO_STREAM("[Traj Tracker:] Current Epoch: " << current_epoch_);
  static uint prevSize = 0;
  if (next_trajectory_.size() > prevSize) {
    prevSize = next_trajectory_.size();
  }
}
void ActionTrajectoryTracker::preemptCB() {
  prempted_time = (ros::Time::now() - started_).toSec();
}
uint8_t ActionTrajectoryTracker::status() const { return done_; }

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ActionTrajectoryTracker, kr_trackers_manager::Tracker);
