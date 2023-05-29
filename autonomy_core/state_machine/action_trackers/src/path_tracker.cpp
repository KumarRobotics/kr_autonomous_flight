#include <action_trackers/RunPathAction.h>
#include <actionlib/server/simple_action_server.h>
#include <angles/angles.h>
#include <kr_trackers_manager/Tracker.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <action_trackers/projector.hpp>

using kr_mav_msgs::PositionCommand;

/*
 * This tracker is no longer used after switching to motion primitive planner.
 * This tracker calculates the velocity target's magnitude using vel_cur + dt *
 * max_acc, and direction by finding the intersection between the planned path
 * and a sphere around robot's current position.
 *
 */

class ActionPathTracker : public kr_trackers_manager::Tracker {
 public:
  void Initialize(const ros::NodeHandle& nh) override;
  bool Activate(const PositionCommand::ConstPtr& cmd) override;
  void Deactivate() override;

  PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr& msg);
  uint8_t status() const override;

  void pathCB();
  void preemptCB();
  void cloudCB(const sensor_msgs::PointCloud::ConstPtr& msg);
  void desMaxCB(const std_msgs::Float64MultiArray::ConstPtr& msg);

 private:
  void Decelerate(PositionCommand::Ptr* cmd);
  boost::shared_ptr<ros::NodeHandle> nh_;
  Projector proj_;

  bool active_;
  bool done_{false};
  boost::optional<ros::Time> started_time_;
  // gains
  double kx_[3], kv_[3];

  PositionCommand::Ptr init_cmd_;

  // action lib
  std::unique_ptr<actionlib::SimpleActionServer<action_trackers::RunPathAction>>
      as_;

  double v_max_;
  double a_max_;
  double v_min_;

  double theta_thr_;
  double yaw_thr_;
  double yaw_speed_;
  bool start_dec_{false};

  boost::optional<Vec3f> init_vel_;
  ros::Publisher project_goal_pub;
  ros::Subscriber cloud_sub;
  ros::Subscriber des_max_sub;
};

void ActionPathTracker::Initialize(const ros::NodeHandle& nh) {
  nh_ = boost::make_shared<ros::NodeHandle>(nh);
  nh_->param("gains/pos/x", kx_[0], 2.5);
  nh_->param("gains/pos/y", kx_[1], 2.5);
  nh_->param("gains/pos/z", kx_[2], 5.0);
  nh_->param("gains/vel/x", kv_[0], 2.2);
  nh_->param("gains/vel/y", kv_[1], 2.2);
  nh_->param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "path_tracker");
  priv_nh.param("theta_thr", theta_thr_, 1.57);
  priv_nh.param("yaw_thr", yaw_thr_, 0.5);
  priv_nh.param("yaw_speed", yaw_speed_, 0.2);
  priv_nh.param("v_max", v_max_, 1.0);
  priv_nh.param("a_max", a_max_, 1.0);
  priv_nh.param("v_min", v_min_, 0.2);
  double r_max, r_min, outer_r_max;
  priv_nh.param("r_max", r_max, 0.7);
  priv_nh.param("r_min", r_min, 0.1);
  priv_nh.param("outer_r_max", outer_r_max, 4.0);

  double robot_r;
  priv_nh.param("robot_r", robot_r, 0.3);
  proj_.set_shrink_distance(robot_r);
  proj_.set_r_max(r_max);
  proj_.set_r_min(r_min);

  cloud_sub = nh_->subscribe("cloud", 1, &ActionPathTracker::cloudCB, this);
  des_max_sub =
      nh_->subscribe("des_max", 1, &ActionPathTracker::desMaxCB, this);

  project_goal_pub = nh_->advertise<kr_planning_msgs::Path>("sg", 1, true);

  active_ = false;

  as_.reset(new actionlib::SimpleActionServer<action_trackers::RunPathAction>(
      nh, "execute_path", false));
  // Register goal and preempt callbacks
  as_->registerGoalCallback(boost::bind(&ActionPathTracker::pathCB, this));
  as_->registerPreemptCallback(
      boost::bind(&ActionPathTracker::preemptCB, this));
  as_->start();

  ROS_INFO("[ActionPathTracker]: v_max = %f", v_max_);
  ROS_INFO("[ActionPathTracker]: a_max = %f", a_max_);
  ROS_INFO("[ActionPathTracker]: v_min = %f", v_min_);
}

bool ActionPathTracker::Activate(const PositionCommand::ConstPtr& msg) {
  active_ = true;
  if (msg == NULL)
    return false;
  else {
    init_cmd_.reset(new PositionCommand(*(msg)));
    active_ = true;
    return true;
  }
}

void ActionPathTracker::Deactivate(void) {
  active_ = false;
  started_time_ = boost::none;
  init_cmd_ = PositionCommand::Ptr();
}

// Heuristic to go to zero velocity
void ActionPathTracker::Decelerate(PositionCommand::Ptr* cmd) {
  // ROS_WARN_THROTTLE(1, "[ActionPathTracker]: decelerate...");
  Eigen::Vector3d pos(
      init_cmd_->position.x, init_cmd_->position.y, init_cmd_->position.z);
  Eigen::Vector3d vel(
      init_cmd_->velocity.x, init_cmd_->velocity.y, init_cmd_->velocity.z);
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d jrk = Eigen::Vector3d::Zero();
  if (!init_vel_) {
    init_vel_ = vel;
    start_dec_ = true;
  }
  double a_max = init_vel_->norm() * 2;

  // Thresholds to cap the min and max acceleration
  if (a_max < 0.5)
    a_max = 0.5;
  else if (a_max > 4.0)
    a_max = 4.0;

  double dt = 0.01;
  if (vel.norm() > a_max * dt) {
    acc =
        -a_max *
        init_vel_->normalized();  // Compute acceleration to drive velocity to 0
    vel += dt * acc;              // Update the velocity
    pos += dt * vel;              // Integrate the position
  } else {
    // Set all to 0 (stop moving)
    jrk = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    init_vel_ = boost::none;
    start_dec_ = false;
  }

  // Set it as command
  (*cmd)->position.x = pos(0);
  (*cmd)->position.y = pos(1);
  (*cmd)->position.z = pos(2);
  (*cmd)->velocity.x = vel(0);
  (*cmd)->velocity.y = vel(1);
  (*cmd)->velocity.z = vel(2);
  (*cmd)->acceleration.x = acc(0);
  (*cmd)->acceleration.y = acc(1);
  (*cmd)->acceleration.z = acc(2);
  (*cmd)->jerk.x = jrk(0);
  (*cmd)->jerk.y = jrk(1);
  (*cmd)->jerk.z = jrk(2);
}

PositionCommand::ConstPtr ActionPathTracker::update(
    const nav_msgs::Odometry::ConstPtr& msg) {
  if (!active_ || !proj_.exist()) {
    return PositionCommand::ConstPtr();
  }

  static auto prev_time = msg->header.stamp;
  double dt = (msg->header.stamp - prev_time).toSec();

  // Temporary solution to very aggressive beheavior after switching from
  // another tracker to path_tracker (when short_range is clicked)
  if (dt > 0.1) {
    ROS_ERROR(
        "++++++++[ActionPathTracker]: error!!! Stopping the robot because "
        "two-time-stamp dt is too large, which will cause very dangerous "
        "behavior. PLEASE LAND THE ROBOT (either click LandHere or manually "
        "take-over)!!!++++++++");
    dt = 0.000001;
  }

  prev_time = msg->header.stamp;

  // ROS_WARN_THROTTLE(1, "dt: %f", dt);

  ros::Time t_now = ros::Time::now();
  if (!started_time_) started_time_ = ros::Time::now();
  double durration = (t_now - *started_time_).toSec();

  PositionCommand::Ptr cmd;
  cmd.reset(new PositionCommand);

  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;

  // Set the gains (only PD)
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  // Decelerate
  if (start_dec_) {
    Decelerate(&cmd);
    cmd->yaw = init_cmd_->yaw;
    cmd->yaw_dot = 0;
    init_cmd_ = cmd;
    return cmd;
  }

  // Get current pos, vel, acc
  const Vec3f curr_pos(
      init_cmd_->position.x, init_cmd_->position.y, init_cmd_->position.z);
  const Vec3f curr_vel(
      init_cmd_->velocity.x, init_cmd_->velocity.y, init_cmd_->velocity.z);
  const Vec3f curr_acc(init_cmd_->acceleration.x,
                       init_cmd_->acceleration.y,
                       init_cmd_->acceleration.z);

  // Eigen aligned ellipsoid3D
  // fast_flight/mpl_ros/DecompRos/DecompUtil/include/decomp_geometry
  vec_E<Ellipsoid3D> es;

  // Increasing a_max_ decreases dist_a to allow the robot accelerate and
  // decelerate gentally. (Don't tune outer_r_max or r_max, tune this!)
  double stop_acc = 0.5 * a_max_;
  double dist_a = curr_vel.norm() * curr_vel.norm() /
                  (2.0 * stop_acc);  // s = 1/2 (a*t^2) = 1/2 * (a*(v/a)^2) =
                                     // v^2 / 2a, where a is stop_acc

  if (dist_a < 2.0) dist_a = 2.0;
  // if (dist_a > 4.0)
  //   dist_a = 4.0;
  if (dist_a > 20.0) dist_a = 20.0;
  proj_.set_outer_r_max(dist_a);

  // robot's current position will be inserted to the path if it does not
  // already exist, see project function in projector.hpp for detail
  if (proj_.project(curr_pos)) {
    const Vec3f projected_goal = proj_.projected_goal();
    const Vec3f outer_projected_goal = proj_.outer_projected_goal();
    const Vec3f direction_goal = proj_.direction_goal();
    // visualize sg
    if (project_goal_pub.getNumSubscribers() > 0) {
      vec_Vec3f sg;
      sg.push_back(curr_pos);
      sg.push_back(projected_goal);
      // sg.push_back(direction_goal);
      sg.push_back(outer_projected_goal);

      kr_planning_msgs::Path path = path_to_ros(sg);
      path.header.frame_id = msg->header.frame_id;
      project_goal_pub.publish(path);
    }

    // initialize command as zero
    Vec3f des_vel = Vec3f::Zero();
    Vec3f des_acc = Vec3f::Zero();
    Vec3f des_jrk = Vec3f::Zero();

    // check yaw offset
    // Set the yaw to be towards the outer projected goal
    double des_yaw = init_cmd_->yaw;
    const Vec3f dpos_direction = direction_goal - curr_pos;
    bool projected_goal_on_ellipsoid = proj_.on_ellipsoid();
    if (dpos_direction.topRows<2>().norm() > 0.1 && projected_goal_on_ellipsoid)
      des_yaw = atan2(dpos_direction(1), dpos_direction(0));

    double dyaw = angles::shortest_angular_distance(init_cmd_->yaw, des_yaw);
    double yaw_ratio = 0;
    if (dyaw > 0.1)
      yaw_ratio = 1;
    else if (dyaw < -0.1)
      yaw_ratio = -1;

    double des_yaw_dot = yaw_speed_ * yaw_ratio;

    cmd->yaw = init_cmd_->yaw + des_yaw_dot * dt;
    cmd->yaw_dot = des_yaw_dot;

    bool start_moving =
        std::abs(dyaw) <= yaw_thr_ && dpos_direction.norm() != 0;

    // if yaw is aligned, start moving
    if (start_moving) {
      es = proj_.project_array(curr_pos, 10, 0.4);
      double r = (projected_goal - curr_pos).norm();
      if (r <= 0) ROS_ERROR("[ActionPathTracker]: error!!!!!");
      for (const auto& it : es) {
        if (it.C_(0, 0) < r) r = it.C_(0, 0);
      }

      double dist_scale = std::pow(r / proj_.r_max(), 1);
      dist_scale = std::min(dist_scale, 1.0);
      double outer_r =
          (outer_projected_goal - curr_pos).norm() / proj_.outer_r_max();
      dist_scale = std::min(outer_r, dist_scale);

      double yaw_scale = 1;
      Vec3f dpos_outer = outer_projected_goal - curr_pos;
      double yaw_outer = std::atan2(dpos_outer(1), dpos_outer(0));
      double yaw_vel = init_cmd_->yaw;
      double dyaw_outer = angles::shortest_angular_distance(yaw_vel, yaw_outer);
      yaw_scale = 1 - std::abs(dyaw_outer) / (M_PI / 2);
      yaw_scale = std::max(yaw_scale, 0.0);

      // Here is where all the desired pos, vel, acc are set
      Vec3f des_vel_ref = v_max_ * dist_scale * yaw_scale *
                          (projected_goal - curr_pos).normalized();
      if (des_vel_ref.norm() < v_min_ && projected_goal_on_ellipsoid)
        des_vel_ref = v_min_ * (projected_goal - curr_pos).normalized();

      if ((des_vel_ref - curr_vel).norm() > 0.5) {
        des_acc = a_max_ * (des_vel_ref - curr_vel).normalized();  // / v_max_;
        des_vel = curr_vel + des_acc * dt;
      } else {
        des_acc = a_max_ * (des_vel_ref - curr_vel);  // / v_max_;
        des_vel = curr_vel + des_acc * dt;
      }

      Vec3f des_pos = des_vel * dt + curr_pos;
      if (!projected_goal_on_ellipsoid &&
          (projected_goal - curr_pos).norm() < 0.1) {
        ROS_WARN_STREAM("Not on ellipsoid" << des_pos.transpose());
        des_pos = proj_.path().back();
      }
      cmd->position.x = des_pos[0];
      cmd->position.y = des_pos[1];
      cmd->position.z = des_pos[2];
      cmd->velocity.x = des_vel[0];
      cmd->velocity.y = des_vel[1];
      cmd->velocity.z = des_vel[2];
      cmd->acceleration.x = des_acc[0];
      cmd->acceleration.y = des_acc[1];
      cmd->acceleration.z = des_acc[2];
      cmd->jerk.x = des_jrk[0];
      cmd->jerk.y = des_jrk[1];
      cmd->jerk.z = des_jrk[2];
    } else
      Decelerate(&cmd);

  } else {
    ROS_ERROR("Fail to find a project point!");
    Decelerate(&cmd);
    init_cmd_->header.stamp = ros::Time::now();
    done_ = true;

    if (as_->isActive()) {
      action_trackers::RunPathResult result;
      as_->setSucceeded(result);
    }
  }

  init_cmd_ = cmd;
  return cmd;
}

void ActionPathTracker::pathCB() {
  auto goal = as_->acceptNewGoal();
  done_ = false;

  auto path = ros_to_path(goal->path);

  if (path.empty()) ROS_ERROR("[ActionPathTracker]: Get an empty path!");

  // robot's current position will be inserted to the path if it does not
  // already exist, see project function in projector.hpp for detail
  proj_.set_path(path);

  action_trackers::RunPathResult result;
  if (as_->isActive() && !goal->block) as_->setSucceeded(result);
}

void ActionPathTracker::desMaxCB(
    const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() == 3) {
    v_max_ = msg->data[0];
    a_max_ = msg->data[1];
    v_min_ = msg->data[2];
    ROS_WARN("[ActionPathTracker]: v_max: %f, a_max: %f, v_min: %f",
             v_max_,
             a_max_,
             v_min_);
  }
}

void ActionPathTracker::cloudCB(const sensor_msgs::PointCloud::ConstPtr& msg) {
  ROS_WARN_ONCE("[ActionPathTracker]: Connect to rgbd cloud");
  proj_.set_obs(cloud_to_vec(*msg));
}

void ActionPathTracker::preemptCB() {}

uint8_t ActionPathTracker::status() const { return done_; }

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ActionPathTracker, kr_trackers_manager::Tracker);
