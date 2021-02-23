#include <action_planner/ActionPlannerConfig.h>
#include <action_planner/PlanTwoPointAction.h>
#include <action_planner/PlanWaypointsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <mapper/data_conversions.h>         // setMap, getMap, etc
#include <mpl_basis/trajectory.h>            // mpl related
#include <mpl_collision/map_util.h>          // mpl related
#include <mpl_planner/planner/map_planner.h> // mpl related
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <primitive_to_traj_opt/convert.h>
#include <ros/ros.h>
#include <traj_opt_ros/ros_bridge.h>

#include <boost/range/irange.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

using boost::irange;

// Local planning server for Sikang's motion primitive planner
class LocalPlanServer {
private:
  // local global map sub
  ros::Subscriber local_map_sub_;

  // traj pub
  ros::Publisher traj_pub;

  // visualization messages pub
  ros::Publisher sg_pub;
  ros::Publisher expanded_cloud_pub;

  // mutexes
  boost::mutex map_mtx, traj_mtx; // TODO: do we need this?

  // motion primitive planner util and its map util
  std::shared_ptr<MPL::VoxelMapPlanner> mp_planner_util_;
  std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;

  // motion primitive trajectory
  Trajectory3D traj_;

  // current local map
  planning_ros_msgs::VoxelMap local_map_;

  // actionlib
  boost::shared_ptr<const action_planner::PlanTwoPointGoal> goal_;
  boost::shared_ptr<action_planner::PlanTwoPointResult> result_;
  // action lib
  std::unique_ptr<
      actionlib::SimpleActionServer<action_planner::PlanTwoPointAction>>
      local_as_;

  bool debug_;
  bool verbose_;

  // methods
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
  void process_result(const Trajectory3D &traj, bool solved);

  /**
   * @brief map callback, update local_map_
   */
  void localMapCB(const planning_ros_msgs::VoxelMap::ConstPtr &msg);

  /**
   * @brief Local planner warpper
   */
  bool local_plan_process(const Waypoint3D &start, const Waypoint3D &goal,
                          const planning_ros_msgs::VoxelMap &map);

public:
  /**
   * @brief Constructor, initialization
   */
  LocalPlanServer(ros::NodeHandle &nh);

  /**
   * @brief Call process_goal function, check planner timeout
   */
  void process_all();

  bool aborted_;
};

// map callback, update local_map_
void LocalPlanServer::localMapCB(
    const planning_ros_msgs::VoxelMap::ConstPtr &msg) {
  ROS_WARN_ONCE("Get the local voxel map!");
  local_map_ = *msg;
}

LocalPlanServer::LocalPlanServer(ros::NodeHandle &nh) {
  traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("traj", 1, true);
  local_map_sub_ =
      nh.subscribe("local_voxel_map", 2, &LocalPlanServer::localMapCB, this);
  local_as_ = std::make_unique<
      actionlib::SimpleActionServer<action_planner::PlanTwoPointAction>>(
      nh, "plan_local_trajectory", false);
  // Register goal and preempt callbacks
  local_as_->registerGoalCallback(boost::bind(&LocalPlanServer::goalCB, this));
  local_as_->start();

  // set up visualization publisher for mpl planner
  sg_pub = nh.advertise<planning_ros_msgs::Path>("start_goal", 1, true);
  expanded_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud>("expanded_cloud", 1, true);

  // set up mpl planner
  mp_map_util_ = std::make_shared<MPL::VoxelMapUtil>();
  ros::NodeHandle priv_nh(nh, "trajectory_planner");
  priv_nh.param("debug", debug_, false);
  priv_nh.param("verbose", verbose_, false);
  double v_max, a_max, j_max, u_max;
  priv_nh.param("max_v", v_max, 2.0);
  priv_nh.param("max_a", a_max, 1.0);
  priv_nh.param("max_j", j_max, 1.0);
  priv_nh.param("max_u", u_max, 1.0);

  vec_E<VecDf> U;
  const decimal_t du = u_max;
  for (decimal_t dx = -u_max; dx <= u_max; dx += du)
    for (decimal_t dy = -u_max; dy <= u_max; dy += du)
      U.push_back(Vec3f(dx, dy, 0));

  double dt, tol_pos, tol_vel, tol_acc;
  int ndt, max_num;
  priv_nh.param("tol_pos", tol_pos, 0.5);
  priv_nh.param("tol_vel", tol_vel, 0.5);
  priv_nh.param("tol_acc", tol_acc, 0.5);
  priv_nh.param("dt", dt, 1.0);
  priv_nh.param("ndt", ndt, -1);
  priv_nh.param("max_num", max_num, -1);

  mp_planner_util_.reset(new MPL::VoxelMapPlanner(verbose_)); // verbose
  mp_planner_util_->setMapUtil(mp_map_util_); // Set collision checking function
  mp_planner_util_->setEpsilon(1.0); // Set greedy param (default equal to 1)
  mp_planner_util_->setVmax(v_max);  // Set max velocity
  mp_planner_util_->setAmax(a_max);  // Set max acceleration
  mp_planner_util_->setJmax(j_max);  // Set max jerk
  // mp_planner_util_->setUmax(u_max); // Set max control input
  mp_planner_util_->setDt(dt);         // Set dt for each primitive
  mp_planner_util_->setTmax(ndt * dt); // Set max time horizon of planning
  mp_planner_util_->setMaxNum(
      max_num); // Set maximum allowed expansion, -1 means no limitation
  mp_planner_util_->setU(U); // 2D discretization if false, 3D if true
  mp_planner_util_->setTol(tol_pos, tol_vel,
                           tol_acc);   // Tolerance for goal region
  mp_planner_util_->setLPAstar(false); // Use Astar
}

void LocalPlanServer::process_all() {
  boost::mutex::scoped_lock lockm(map_mtx);

  if (goal_ == NULL)
    return;
  ros::Time t0 = ros::Time::now();
  // record goal position, specify use jrk, acc or vel
  process_goal();
  double dt = (ros::Time::now() - t0).toSec();

  // check timeout
  if (dt > 0.3 && local_as_->isActive()) {
    ROS_WARN("[LocalPlanServer]+++++++++++++++++++++++++");
    ROS_WARN("Time out!!!!!! dt =  %f is too large!!!!!", dt);
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("+++++++++++++++++++++++++");
    local_as_->setAborted();
  }
  printf("Total time for TPP traj gen: %f\n", dt);
}

void LocalPlanServer::process_result(const Trajectory3D &traj, bool solved) {
  result_ = boost::make_shared<action_planner::PlanTwoPointResult>();
  result_->success = solved; // set success status
  result_->policy_status = solved ? 1 : -1;
  if (solved) {
    // covert traj to a ros message
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header.frame_id = local_map_.header.frame_id;
    traj_pub.publish(traj_msg);

    // record trajectory in result
    result_->traj = PrimitiveToTrajOpt::convert(traj_msg);
    result_->traj.header.frame_id = local_map_.header.frame_id;
    TrajRosBridge::publish_msg(result_->traj);

    decimal_t endt =
        goal_->execution_time.toSec(); // execution_time (set in replanner)
                                       // equals 1.0/replan_rate

    // evaluate trajectory for 5 steps, each step duration equals
    // execution_time, get corresponding waypoints and record in result
    // (result_->p_stop etc.) (evaluate the whole traj if execution_time is not
    // set (i.e. not in replan mode))
    int num_goals = 5; // TODO: why 5? should be >= max_horizon in replanner?
    if (endt <= 0) {
      endt = traj.getTotalTime();
      num_goals = 1;
    }

    for (int i = 0; i < num_goals; i++) {
      geometry_msgs::Pose p_fin;
      geometry_msgs::Twist v_fin, a_fin, j_fin;

      Waypoint3D pt_f = traj.evaluate(endt * double(i + 1));
      // check if evaluation is successful, if not, set result->success to be
      // false! (if failure case, a null Waypoint is returned)
      if ((pt_f.pos(0) == 0) && (pt_f.pos(1) == 0) && (pt_f.pos(2) == 0) &&
          (pt_f.vel(0) == 0) && (pt_f.vel(1) == 0) && (pt_f.vel(2) == 0)) {
        result_->success = 0;
        ROS_WARN_STREAM(
            "waypoint evaluation failed, set result->success to be false");
        ROS_WARN_STREAM("trajectory total time:" << traj.total_t_);
        ROS_WARN_STREAM("evaluating at:" << endt * double(i + 1));
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
        goal_->execution_time; // execution_time (set in replanner)
                               // equals 1.0/replan_rate
    result_->epoch = goal_->epoch;
    Waypoint3D pt = traj.evaluate(traj.getTotalTime());
    result_->traj_end.position.x = pt.pos(0);
    result_->traj_end.position.y = pt.pos(1);
    result_->traj_end.position.z = pt.pos(2);
    result_->traj_end.orientation.w = 1.0;
    result_->traj_end.orientation.z = 0;
  }

  // reset goal
  goal_ = boost::shared_ptr<action_planner::PlanTwoPointGoal>();
  // abort if trajectory generation failed
  if (!solved && local_as_->isActive()) {
    ROS_WARN("+++++++++++++++++++++++++");
    ROS_WARN("Local planner: trajectory generation failed!");
    ROS_WARN("Danger!!!!!");
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("+++++++++++++++++++++++++");
    // local_as_->setAborted();
  }

  if (local_as_->isActive())
    local_as_->setSucceeded(*result_);
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
  Waypoint3D start, goal;
  // instead of using current odometry as start, we use the given start position
  // for consistency between old and new trajectories in replan process
  start.pos = pose_to_eigen(goal_->p_init);
  start.vel = twist_to_eigen(goal_->v_init);
  start.acc = twist_to_eigen(goal_->a_init);
  start.jrk = twist_to_eigen(goal_->j_init);
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = true;
  start.use_jrk = false;

  goal.pos = pose_to_eigen(goal_->p_final);
  goal.vel = twist_to_eigen(goal_->v_final);
  goal.acc = twist_to_eigen(goal_->a_final);
  goal.jrk = twist_to_eigen(goal_->j_final);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;

  bool local_planner_succeeded;
  local_planner_succeeded = local_plan_process(start, goal, local_map_);

  if (!local_planner_succeeded) {
    // local plan fails
    aborted_ = true;
    if (local_as_->isActive()) {
      ROS_WARN("+++++++++++++++++++++++++");
      ROS_WARN("Local Plan Fails!!!!!");
      ROS_WARN("Abort!!!!!!");
      ROS_WARN("+++++++++++++++++++++++++");
      local_as_->setAborted();
    }
  }

  // get the trajectory from local planner, and process result
  Trajectory3D traj = traj_;
  process_result(traj, local_planner_succeeded);
}

bool LocalPlanServer::local_plan_process(
    const Waypoint3D &start, const Waypoint3D &goal,
    const planning_ros_msgs::VoxelMap &map) {
  // for visualization: publish a path connecting local start and local goal
  vec_Vec3f sg;
  sg.push_back(start.pos);
  sg.push_back(goal.pos);
  planning_ros_msgs::Path sg_msg = path_to_ros(sg);
  std::string map_frame; // set frame id
  map_frame = map.header.frame_id;
  sg_msg.header.frame_id = map_frame;
  sg_pub.publish(sg_msg);

  /// Trajectory planning
  setMap(mp_map_util_, map);
  bool valid = false;
  mp_planner_util_->reset();
  valid = mp_planner_util_->plan(
      start, goal); // start and new_goal contain full informaiton about
                    // position/velocity/acceleration
  if (valid) {
    traj_ = mp_planner_util_->getTraj();
  }

  // for visualization: publish expanded nodes as a point cloud
  sensor_msgs::PointCloud expanded_ps =
      vec_to_cloud(mp_planner_util_->getExpandedNodes());
  expanded_ps.header.frame_id = map_frame;
  expanded_cloud_pub.publish(expanded_ps);

  return valid;
}

// prevent concurrent planner modes
void LocalPlanServer::goalCB() {
  goal_ = local_as_->acceptNewGoal();
  aborted_ = false;
  process_all();
}

int main(int argc, char **argv) {
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
