#include <action_planner/ActionPlannerConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <data_conversions.h>  // setMap, getMap, etc
#include <eigen_conversions/eigen_msg.h>
#include <kr_planning_msgs/PlanTwoPointAction.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <mpl_basis/trajectory.h>
#include <mpl_collision/map_util.h>
#include <mpl_planner/map_planner.h>
#include <primitive_ros_utils.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <traj_opt_ros/ros_bridge.h>

#include <boost/range/irange.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

using boost::irange;

// Local planning server for Sikang's motion primitive planner
class LocalPlanServer {
 public:
  explicit LocalPlanServer(const ros::NodeHandle& nh);

  /**
   * @brief Call process_goal function, check planner timeout
   */
  void process_all();

  bool aborted_;

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber local_map_sub_;
  ros::Publisher traj_pub;

  // visualization messages pub
  ros::Publisher sg_pub;
  ros::Publisher expanded_cloud_pub;

  ros::Publisher local_map_cleared_pub;

  boost::mutex map_mtx, traj_mtx;  // TODO(xu): do we need this?

  // motion primitive planner util and its map util
  std::shared_ptr<MPL::VoxelMapPlanner> mp_planner_util_;
  std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;

  // motion primitive trajectory
  MPL::Trajectory3D traj_;

  // current local map
  kr_planning_msgs::VoxelMapConstPtr local_map_ptr_ = nullptr;

  // actionlib
  boost::shared_ptr<const kr_planning_msgs::PlanTwoPointGoal> goal_;
  boost::shared_ptr<kr_planning_msgs::PlanTwoPointResult> result_;
  // action lib
  std::unique_ptr<
      actionlib::SimpleActionServer<kr_planning_msgs::PlanTwoPointAction>>
      local_as_;

  bool debug_;
  bool verbose_;
  bool pub_cleared_map_ = false;

  // planner tolerance
  double tol_pos_, goal_tol_vel_, goal_tol_acc_;

  /**
   * @brief Goal callback function, prevent concurrent planner modes, call
   * process_all function
   */
  void goalCB();

  /**
   * @brief Call planner after setting planner start and goal and specify params
   * (use jrk, acc or vel)
   */
  void process_goal();

  /**
   * @brief Record result (trajectory, status, etc)
   */
  void process_result(const MPL::Trajectory3D& traj, bool solved);

  /**
   * @brief map callback, update local_map_ptr_
   */
  void localMapCB(const kr_planning_msgs::VoxelMap::ConstPtr& msg);

  /**
   * @brief Local planner warpper
   */
  bool local_plan_process(const MPL::Waypoint3D& start,
                          const MPL::Waypoint3D& goal,
                          const kr_planning_msgs::VoxelMap& map);

  /**
   * @brief Local planner clear footprint
   */
  kr_planning_msgs::VoxelMap clear_map_position(
      const kr_planning_msgs::VoxelMap& local_map, const Vec3f& start);

  /**
   * @brief Local planner check if outside map
   */
  bool is_outside_map(const Eigen::Vector3i& pn, const Eigen::Vector3i& dim);
};

// map callback, update local_map_ptr_
void LocalPlanServer::localMapCB(
    const kr_planning_msgs::VoxelMap::ConstPtr& msg) {
  ROS_WARN_ONCE("[Local planner:] Got the local voxel map!");
  local_map_ptr_ = msg;
}

LocalPlanServer::LocalPlanServer(const ros::NodeHandle& nh) : pnh_(nh) {
  traj_pub = pnh_.advertise<kr_planning_msgs::Trajectory>("traj", 1, true);
  local_map_sub_ =
      pnh_.subscribe("local_voxel_map", 2, &LocalPlanServer::localMapCB, this);
  local_as_ = std::make_unique<
      actionlib::SimpleActionServer<kr_planning_msgs::PlanTwoPointAction>>(
      pnh_, "plan_local_trajectory", false);

  // set up visualization publisher for mpl planner
  sg_pub = pnh_.advertise<kr_planning_msgs::Path>("start_goal", 1, true);
  expanded_cloud_pub =
      pnh_.advertise<sensor_msgs::PointCloud>("expanded_cloud", 1, true);

  local_map_cleared_pub = pnh_.advertise<kr_planning_msgs::VoxelMap>(
      "local_voxel_map_cleared", 1, true);

  // set up mpl planner
  mp_map_util_ = std::make_shared<MPL::VoxelMapUtil>();
  ros::NodeHandle traj_planner_nh(pnh_, "trajectory_planner");
  traj_planner_nh.param("debug", debug_, false);
  traj_planner_nh.param("verbose", verbose_, false);
  double v_max, a_max, j_max, u_max;
  traj_planner_nh.param("max_v", v_max, 2.0);
  traj_planner_nh.param("max_a", a_max, 1.0);
  traj_planner_nh.param("max_j", j_max, 1.0);
  traj_planner_nh.param("max_u", u_max, 1.0);

  bool use_3d_local_;
  traj_planner_nh.param("use_3d_local", use_3d_local_, false);
  double vz_max, az_max, jz_max, uz_max;
  traj_planner_nh.param("max_v_z", vz_max, 2.0);
  traj_planner_nh.param("max_a_z", az_max, 1.0);
  traj_planner_nh.param("max_j_z", jz_max, 1.0);
  traj_planner_nh.param("max_u_z", uz_max, 1.0);

  // Important: set motion primitive control inputs
  vec_E<VecDf> U;
  if (!use_3d_local_) {
    const decimal_t du = u_max;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du)
      for (decimal_t dy = -u_max; dy <= u_max; dy += du)
        U.push_back(Vec3f(dx, dy, 0));
  } else {
    const decimal_t du = u_max;
    const decimal_t du_z = uz_max;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du)
      for (decimal_t dy = -u_max; dy <= u_max; dy += du)
        for (decimal_t dz = -uz_max; dz <= uz_max; dz += du_z)
          U.push_back(Vec3f(dx, dy, dz));
  }

  double dt;
  double W, v_fov;
  int ndt, max_num;
  traj_planner_nh.param("tol_pos", tol_pos_, 0.5);
  traj_planner_nh.param("global_goal_tol_vel", goal_tol_vel_, 0.5);
  traj_planner_nh.param("global_goal_tol_acc", goal_tol_acc_, 0.5);
  /// Execution time for each primitive
  traj_planner_nh.param("dt", dt, 1.0);
  traj_planner_nh.param("ndt", ndt, -1);
  traj_planner_nh.param("max_num", max_num, -1);
  traj_planner_nh.param("heuristic_weight", W, 10.0);
  traj_planner_nh.param("vertical_semi_fov", v_fov, 0.392);

  // TODO(xu:) not differentiating between 2D and 3D, causing extra resource
  // usage for 2D case, this needed to be changed in both planner util as well
  // as map util, which requires the slicing map function
  mp_planner_util_.reset(new MPL::VoxelMapPlanner(verbose_));  // verbose
  mp_planner_util_->setMapUtil(
      mp_map_util_);                  // Set collision checking function
  mp_planner_util_->setEpsilon(1.0);  // Set greedy param (default equal to 1)

  mp_planner_util_->setVxy(v_max);  // Set max velocity along x and y
  mp_planner_util_->setVz(vz_max);  // Set max velocity along z

  mp_planner_util_->setAmax(a_max);  // Set max acceleration
  mp_planner_util_->setJmax(j_max);  // Set max jerk
  mp_planner_util_->setVfov(v_fov);  // Set vertical semi-fov
  // mp_planner_util_->setUmax(u_max); // Set max control input
  mp_planner_util_->setDt(dt);          // Set dt for each primitive
  mp_planner_util_->setTmax(ndt * dt);  // Set max time horizon of planning
  mp_planner_util_->setMaxNum(
      max_num);  // Set maximum allowed expansion, -1 means no limitation
  mp_planner_util_->setU(U);  // 2D discretization if false, 3D if true
  mp_planner_util_->setW(W);  // 2D discretization if false, 3D if true
  mp_planner_util_->setLPAstar(false);  // Use Astar

  // Register goal and preempt callbacks
  local_as_->registerGoalCallback(boost::bind(&LocalPlanServer::goalCB, this));
  local_as_->start();
}

void LocalPlanServer::process_all() {
  boost::mutex::scoped_lock lockm(map_mtx);

  if (goal_ == NULL) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("[LocalPlanServer:] Goal is null!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  } else if (local_map_ptr_ == nullptr) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("[LocalPlanServer:] local map is not received!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  } else {
    process_goal();
  }
}

void LocalPlanServer::process_result(const MPL::Trajectory3D& traj,
                                     bool solved) {
  result_ = boost::make_shared<kr_planning_msgs::PlanTwoPointResult>();
  result_->success = solved;  // set success status
  result_->policy_status = solved ? 1 : -1;
  if (solved) {
    // covert traj to a ros message
    kr_planning_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header.frame_id = local_map_ptr_->header.frame_id;
    traj_pub.publish(traj_msg);

    // record trajectory in result
    result_->traj = traj_opt::SplineTrajectoryFromTrajectory(traj_msg);
    result_->traj.header.frame_id = local_map_ptr_->header.frame_id;
    traj_opt::TrajRosBridge::publish_msg(result_->traj);

    // execution_time (set in replanner)
    // equals 1.0/local_replan_rate
    double endt = goal_->execution_time.toSec();

    // evaluate trajectory for 5 steps, each step duration equals
    // execution_time, get corresponding waypoints and record in result
    // (result_->p_stop etc.) (evaluate the whole traj if execution_time is not
    // set (i.e. not in replan mode))
    int num_goals = 5;
    if (endt <= 0) {
      endt = traj.getTotalTime();
      num_goals = 1;
    }

    for (int i = 0; i < num_goals; i++) {
      geometry_msgs::Pose p_fin;
      geometry_msgs::Twist v_fin, a_fin, j_fin;

      MPL::Waypoint3D pt_f = traj.evaluate(endt * static_cast<double>(i + 1));
      // check if evaluation is successful, if not, set result->success to be
      // false! (if failure case, a null Waypoint is returned)
      if ((pt_f.pos(0) == 0) && (pt_f.pos(1) == 0) && (pt_f.pos(2) == 0) &&
          (pt_f.vel(0) == 0) && (pt_f.vel(1) == 0) && (pt_f.vel(2) == 0)) {
        result_->success = 0;
        ROS_WARN_STREAM(
            "waypoint evaluation failed, set result->success to be false");
        ROS_WARN_STREAM("trajectory total time:" << traj.total_t_);
        ROS_WARN_STREAM("evaluating at:" << endt * static_cast<double>(i + 1));
      }

      p_fin.position.x = pt_f.pos(0), p_fin.position.y = pt_f.pos(1),
      p_fin.position.z = pt_f.pos(2);
      p_fin.orientation.w = 1, p_fin.orientation.z = 0;
      v_fin.linear.x = pt_f.vel(0), v_fin.linear.y = pt_f.vel(1),
      v_fin.linear.z = pt_f.vel(2);
      v_fin.angular.z = 0;
      a_fin.linear.x = pt_f.acc(0), a_fin.linear.y = pt_f.acc(1),
      a_fin.linear.z = pt_f.acc(2);
      a_fin.angular.z = 0;
      result_->p_stop.push_back(p_fin);
      result_->v_stop.push_back(v_fin);
      result_->a_stop.push_back(a_fin);
      result_->j_stop.push_back(j_fin);
    }
    result_->execution_time =
        goal_->execution_time;  // execution_time (set in replanner)
                                // equals 1.0/local_replan_rate
    result_->epoch = goal_->epoch;
    MPL::Waypoint3D pt = traj.evaluate(traj.getTotalTime());
    result_->traj_end.position.x = pt.pos(0);
    result_->traj_end.position.y = pt.pos(1);
    result_->traj_end.position.z = pt.pos(2);
    result_->traj_end.orientation.w = 1.0;
    result_->traj_end.orientation.z = 0;
  }

  // reset goal
  goal_ = boost::shared_ptr<kr_planning_msgs::PlanTwoPointGoal>();
  // abort if trajectory generation failed
  if (!solved && local_as_->isActive()) {
    ROS_WARN("Current local plan trail: trajectory generation failed!");
    local_as_->setAborted();
  }

  if (local_as_->isActive()) {
    local_as_->setSucceeded(*result_);
  }
}

// record goal position, specify use jrk, acc or vel
void LocalPlanServer::process_goal() {
  boost::mutex::scoped_lock lockt(traj_mtx);
  if (aborted_) {
    if (local_as_->isActive()) {
      ROS_WARN("process_goal: Abort!");
      local_as_->setAborted();
    }
    return;
  }

  // set start and goal, assume goal is not null
  MPL::Waypoint3D start, goal;
  // instead of using current odometry as start, we use the given start position
  // for consistency between old and new trajectories in replan process
  start.pos = kr::pose_to_eigen(goal_->p_init);
  start.vel = kr::twist_to_eigen(goal_->v_init);
  start.acc = kr::twist_to_eigen(goal_->a_init);
  start.jrk = kr::twist_to_eigen(goal_->j_init);

  // Important: define use position, velocity, acceleration or jerk as control
  // inputs, note that the lowest order "false" term will be used as control
  // input, see instructions here:
  // https://github.com/KumarRobotics/kr_autonomous_flight/tree/master/autonomy_core/map_plan/mpl#example-usage
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = true;
  start.use_jrk = false;
  // yaw planning or not (note that yaw alignment with velocity can be turned on
  // in trajectory_tracker)
  start.use_yaw = false;

  goal.pos = kr::pose_to_eigen(goal_->p_final);
  goal.vel = kr::twist_to_eigen(goal_->v_final);
  goal.acc = kr::twist_to_eigen(goal_->a_final);
  goal.jrk = kr::twist_to_eigen(goal_->j_final);
  goal.use_yaw = start.use_yaw;
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;

  kr_planning_msgs::VoxelMap local_map_cleared;
  local_map_cleared = clear_map_position(*local_map_ptr_, start.pos);

  if (pub_cleared_map_) {
    local_map_cleared_pub.publish(local_map_cleared);
    ROS_ERROR("Local map cleared published");
  }

  bool local_planner_succeeded;
  local_planner_succeeded = local_plan_process(start, goal, local_map_cleared);

  if (!local_planner_succeeded) {
    // local plan fails
    aborted_ = true;
    if (local_as_->isActive()) {
      ROS_WARN("Current local plan trail failed!");
      local_as_->setAborted();
    }
  }

  // get the trajectory from local planner, and process result
  MPL::Trajectory3D traj = traj_;
  process_result(traj, local_planner_succeeded);
}

kr_planning_msgs::VoxelMap LocalPlanServer::clear_map_position(
    const kr_planning_msgs::VoxelMap& local_map_original, const Vec3f& start) {
  // make a copy, not the most efficient way, but necessary because we need to
  // maintain both original and cleared maps
  kr_planning_msgs::VoxelMap local_map_cleared;
  local_map_cleared = local_map_original;

  kr_planning_msgs::VoxelMap voxel_map;

  // Replaced with corresponding parameter value from VoxelMsg.msg
  int8_t val_free = voxel_map.val_free;
  ROS_WARN_ONCE("Value free is set as %d", val_free);
  double robot_r = 1.0;
  int robot_r_n = std::ceil(robot_r / local_map_cleared.resolution);

  vec_Vec3i clear_ns;
  for (int nx = -robot_r_n; nx <= robot_r_n; nx++) {
    for (int ny = -robot_r_n; ny <= robot_r_n; ny++) {
      for (int nz = -robot_r_n; nz <= robot_r_n; nz++) {
        clear_ns.push_back(Eigen::Vector3i(nx, ny, nz));
      }
    }
  }

  auto origin_x = local_map_cleared.origin.x;
  auto origin_y = local_map_cleared.origin.y;
  auto origin_z = local_map_cleared.origin.z;
  Eigen::Vector3i dim = Eigen::Vector3i::Zero();
  dim(0) = local_map_cleared.dim.x;
  dim(1) = local_map_cleared.dim.y;
  dim(2) = local_map_cleared.dim.z;
  auto res = local_map_cleared.resolution;
  const Eigen::Vector3i pn =
      Eigen::Vector3i(std::round((start(0) - origin_x) / res),
                      std::round((start(1) - origin_y) / res),
                      std::round((start(2) - origin_z) / res));

  for (const auto& n : clear_ns) {
    Eigen::Vector3i pnn = pn + n;
    int idx_tmp = pnn(0) + pnn(1) * dim(0) + pnn(2) * dim(0) * dim(1);
    if (!is_outside_map(pnn, dim) &&
        local_map_cleared.data[idx_tmp] != val_free) {
      local_map_cleared.data[idx_tmp] = val_free;
      // ROS_ERROR("clearing!!! idx %d", idx_tmp);
    }
  }
  return local_map_cleared;
}

bool LocalPlanServer::is_outside_map(const Eigen::Vector3i& pn,
                                     const Eigen::Vector3i& dim) {
  return pn(0) < 0 || pn(0) >= dim(0) || pn(1) < 0 || pn(1) >= dim(1) ||
         pn(2) < 0 || pn(2) >= dim(2);
}

bool LocalPlanServer::local_plan_process(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  // for visualization: publish a path connecting local start and local goal
  vec_Vec3f sg;
  sg.push_back(start.pos);
  sg.push_back(goal.pos);
  kr_planning_msgs::Path sg_msg = kr::path_to_ros(sg);
  std::string map_frame;  // set frame id
  map_frame = map.header.frame_id;
  sg_msg.header.frame_id = map_frame;
  sg_pub.publish(sg_msg);

  /// Trajectory planning
  setMap(mp_map_util_, map);
  bool valid = false;
  mp_planner_util_->reset();
  // start and new_goal contain full informaiton about
  // position/velocity/acceleration
  valid = mp_planner_util_->plan(start, goal);
  if (valid) {
    traj_ = mp_planner_util_->getTraj();
  }

  // for visualization: publish expanded nodes as a point cloud
  sensor_msgs::PointCloud expanded_ps = kr::vec_to_cloud(
      mp_planner_util_->getExpandedNodes());
  expanded_ps.header.frame_id = map_frame;
  expanded_cloud_pub.publish(expanded_ps);

  return valid;
}

// prevent concurrent planner modes
void LocalPlanServer::goalCB() {
  goal_ = local_as_->acceptNewGoal();
  // check_vel is true if local planner reaches global goal
  if (goal_->check_vel) {
    // Tolerance for goal region, -1 means no limitation
    mp_planner_util_->setTol(tol_pos_, goal_tol_vel_, goal_tol_acc_);
  } else {
    mp_planner_util_->setTol(tol_pos_, -1, -1);
  }
  aborted_ = false;
  process_all();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_planner");

  ros::NodeHandle nh("~");

  LocalPlanServer tpp(nh);

  ros::Rate r(20);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
    // tpp.process_all();
  }

  return 0;
}
