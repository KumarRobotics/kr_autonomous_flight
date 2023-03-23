#include <action_planner/ActionPlannerConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <data_conversions.h>  // setMap, getMap, etc
#include <eigen_conversions/eigen_msg.h>
#include <jps/jps_planner.h>  // jps related
#include <jps/map_util.h>     // jps related
#include <kr_planning_msgs/PlanTwoPointAction.h>
#include <kr_planning_msgs/VoxelMap.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <nav_msgs/Odometry.h>  // odometry
#include <primitive_ros_utils.h>
#include <ros/ros.h>
#include <traj_opt_ros/ros_bridge.h>

#include <boost/range/irange.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

using boost::irange;

class GlobalPlanServer {
 public:
  explicit GlobalPlanServer(const ros::NodeHandle& nh);

  /**
   * @brief Call process_goal function, check planner timeout
   */
  void process_all();

  /**
   * @brief Odom callback function, directly subscribing odom to get the path
   * start position
   */
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);

  // global path recorder
  kr_planning_msgs::Path global_path_msg_;

  bool aborted_;

 private:
  // global map sub
  ros::NodeHandle pnh_;
  ros::Subscriber global_map_sub_;
  ros::Subscriber odom_sub_;

  // use 2-d or 3-d for global planner
  bool use_3d_global_;

  // planner verbose
  bool jps_verbose_;

  // global map z cost factor (cost_along_z = z_cost_factor * cost_along_x_or_y)
  int z_cost_factor_;

  // pub
  ros::Publisher path_pub_;

  ros::Publisher global_map_cleared_pub_;

  bool pub_cleared_map_ = false;

  // mutexes
  boost::mutex map_mtx;
  // current global map
  kr_planning_msgs::VoxelMapConstPtr global_map_ptr_;

  bool global_planner_succeeded_{false};
  bool global_plan_exist_{false};

  // actionlib
  boost::shared_ptr<const kr_planning_msgs::PlanTwoPointGoal> goal_;
  boost::shared_ptr<kr_planning_msgs::PlanTwoPointResult> result_;
  // action lib
  std::unique_ptr<
      actionlib::SimpleActionServer<kr_planning_msgs::PlanTwoPointAction>>
      global_as_;

  // planner related
  std::shared_ptr<JPS::JPSPlanner3D> jps_3d_util_;
  std::shared_ptr<JPS::VoxelMapUtil> jps_3d_map_util_;

  std::shared_ptr<JPS::JPSPlanner2D> jps_util_;
  std::shared_ptr<JPS::OccMapUtil> jps_map_util_;

  // odom related
  bool odom_set_{false};
  nav_msgs::Odometry::ConstPtr odom_msg_;

  // methods
  /**
   * @brief Goal callback function, prevent concurrent planner modes, call
   * process_all function
   */
  void goalCB();

  /**
   * @brief Call global planner after setting planner start and goal and specify
   * params (use jrk, acc or vel)
   */
  void process_goal();

  /**
   * @brief Record result (path, status, etc)
   */
  void process_result(bool solved);

  /**
   * @brief map callback, update global_map_ptr_
   */
  void globalMapCB(const kr_planning_msgs::VoxelMap::ConstPtr& msg);

  /**
   * @brief Global planner warpper
   */
  bool global_plan_process(const MPL::Waypoint3D& start,
                           const MPL::Waypoint3D& goal,
                           const kr_planning_msgs::VoxelMap& global_map);

  /**
   * @brief Slice map util, only used if plan with a 2d jps planner
   */
  kr_planning_msgs::VoxelMap SliceMap(double h,
                                      double hh,
                                      const kr_planning_msgs::VoxelMap& map);

  /**
   * @brief Increase the cost of z by dividing global map z resolution by a
   * factor, so that we can still use uniform-cost map for JPS implementation
   * while penlizing z-axis movement (only used if plan with a 3d jps planner)
   */
  kr_planning_msgs::VoxelMap ChangeZCost(const kr_planning_msgs::VoxelMap& map,
                                         int z_cost_factor = 1);

  /**
   * @brief clear global map position
   */
  kr_planning_msgs::VoxelMap clear_map_position(
      const kr_planning_msgs::VoxelMap& global_map, const vec_Vec3f& positions);

  /**
   * @brief global planner check if outside map
   */
  bool is_outside_map(const Eigen::Vector3i& pn, const Eigen::Vector3i& dim);
};

void GlobalPlanServer::globalMapCB(
    const kr_planning_msgs::VoxelMap::ConstPtr& msg) {
  global_map_ptr_ = msg;
}

GlobalPlanServer::GlobalPlanServer(const ros::NodeHandle& nh) : pnh_(nh) {
  path_pub_ = pnh_.advertise<kr_planning_msgs::Path>("path", 1, true);
  global_map_cleared_pub_ = pnh_.advertise<kr_planning_msgs::VoxelMap>(
      "global_voxel_map_cleared", 1, true);

  global_map_sub_ = pnh_.subscribe(
      "global_voxel_map", 2, &GlobalPlanServer::globalMapCB, this);

  ros::NodeHandle traj_planner_nh(pnh_, "trajectory_planner");
  traj_planner_nh.param("use_3d_global", use_3d_global_, false);
  traj_planner_nh.param("z_cost_factor", z_cost_factor_, 1);

  ros::NodeHandle local_global_plan_nh(pnh_, "local_global_server");
  local_global_plan_nh.param("global_planner_verbose", jps_verbose_, false);

  global_as_ = std::make_unique<
      actionlib::SimpleActionServer<kr_planning_msgs::PlanTwoPointAction>>(
      pnh_, "plan_global_path", false);
  // Register goal and preempt callbacks
  global_as_->registerGoalCallback(
      boost::bind(&GlobalPlanServer::goalCB, this));
  global_as_->start();

  // odom callback
  odom_sub_ = pnh_.subscribe("odom",
                             10,
                             &GlobalPlanServer::odom_callback,
                             this,
                             ros::TransportHints().tcpNoDelay());

  // Set map util for jps
  if (use_3d_global_) {
    jps_3d_map_util_ = std::make_shared<JPS::VoxelMapUtil>();
    jps_3d_util_ = std::make_shared<JPS::JPSPlanner3D>(jps_verbose_);
    jps_3d_util_->setMapUtil(jps_3d_map_util_);
  } else {
    jps_map_util_ = std::make_shared<JPS::OccMapUtil>();
    jps_util_ = std::make_shared<JPS::JPSPlanner2D>(jps_verbose_);
    jps_util_->setMapUtil(jps_map_util_);
  }
}

void GlobalPlanServer::process_all() {
  boost::mutex::scoped_lock lockm(map_mtx);

  if (goal_ == NULL) return;
  ros::Time t0 = ros::Time::now();
  process_goal();
  double dt = (ros::Time::now() - t0).toSec();

  // check timeout
  if (dt > 0.3 && global_as_->isActive()) {
    ROS_WARN("[GlobalPlanServer]+++++++++++++++++++++++++");
    ROS_WARN("Time out!!!!!! dt =  %f is too large!!!!!", dt);
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("+++++++++++++++++++++++++");
    global_as_->setAborted();
  }
}

void GlobalPlanServer::process_result(bool solved) {
  result_ = boost::make_shared<kr_planning_msgs::PlanTwoPointResult>();
  result_->success = solved;  // set success status
  result_->policy_status = solved ? 1 : -1;
  result_->path = global_path_msg_;
  if (!solved && global_as_->isActive()) {
    ROS_WARN("+++++++++++++++++++++++++");
    ROS_WARN("Global planner: path planner failed!");
    ROS_WARN("Danger!!!!!");
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("+++++++++++++++++++++++++");
    global_as_->setAborted();
  }

  // reset goal
  goal_ = boost::shared_ptr<kr_planning_msgs::PlanTwoPointGoal>();
  if (global_as_->isActive()) {
    global_as_->setSucceeded(*result_);
  }
}

void GlobalPlanServer::odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
  if (!odom_set_) {
    odom_set_ = true;
  }
  odom_msg_ = odom;
}

void GlobalPlanServer::process_goal() {
  if (aborted_) {
    if (global_as_->isActive()) {
      ROS_WARN("process_goal: Abort!");
      global_as_->setAborted();
    }
    return;
  }

  // set start and goal, assume goal is not null
  MPL::Waypoint3D start, goal;

  // first, check if odom is received
  if (!odom_set_) {
    ROS_WARN("[Replanner:] Odom is not yet received!");
    return;
  }

  // record current odometry as start
  start.pos = kr::pose_to_eigen(odom_msg_->pose.pose);
  goal.pos = kr::pose_to_eigen(goal_->p_final);

  // call the planner

  if (!global_map_ptr_) {
    ROS_ERROR("Voxel Map is not yet received");
    return;
  }

  // clear start and goal
  kr_planning_msgs::VoxelMap global_map_cleared;
  vec_Vec3f positions;
  positions.push_back(start.pos);
  positions.push_back(goal.pos);
  global_map_cleared = clear_map_position(*global_map_ptr_, positions);

  if (pub_cleared_map_) {
    global_map_cleared_pub_.publish(global_map_cleared);
  }
  global_planner_succeeded_ =
      global_plan_process(start, goal, global_map_cleared);

  process_result(global_planner_succeeded_);
}

kr_planning_msgs::VoxelMap GlobalPlanServer::clear_map_position(
    const kr_planning_msgs::VoxelMap& global_map_original,
    const vec_Vec3f& positions) {
  // make a copy, not the most efficient way, but necessary because we need to
  // maintain both original and cleared maps
  kr_planning_msgs::VoxelMap global_map_cleared;
  global_map_cleared = global_map_original;

  kr_planning_msgs::VoxelMap voxel_map;

  // Replaced with corresponding parameter value from VoxelMsg.msg
  int8_t val_free = voxel_map.val_free;
  ROS_WARN_ONCE("Value free is set as %d", val_free);
  double robot_r = 1.5;
  int robot_r_n = std::ceil(robot_r / global_map_cleared.resolution);

  vec_Vec3i clear_ns;
  for (int nx = -robot_r_n; nx <= robot_r_n; nx++) {
    for (int ny = -robot_r_n; ny <= robot_r_n; ny++) {
      for (int nz = -robot_r_n; nz <= robot_r_n; nz++) {
        clear_ns.push_back(Eigen::Vector3i(nx, ny, nz));
      }
    }
  }

  auto origin_x = global_map_cleared.origin.x;
  auto origin_y = global_map_cleared.origin.y;
  auto origin_z = global_map_cleared.origin.z;
  Eigen::Vector3i dim = Eigen::Vector3i::Zero();
  dim(0) = global_map_cleared.dim.x;
  dim(1) = global_map_cleared.dim.y;
  dim(2) = global_map_cleared.dim.z;
  auto res = global_map_cleared.resolution;
  Vec3f position;
  Eigen::Vector3i pn;

  for (unsigned int pos_idx = 0; pos_idx < positions.size(); pos_idx++) {
    position = positions[pos_idx];

    pn = Eigen::Vector3i(std::round((position(0) - origin_x) / res),
                         std::round((position(1) - origin_y) / res),
                         std::round((position(2) - origin_z) / res));

    for (const auto& n : clear_ns) {
      Eigen::Vector3i pnn = pn + n;
      int idx_tmp = pnn(0) + pnn(1) * dim(0) + pnn(2) * dim(0) * dim(1);
      if (!is_outside_map(pnn, dim) &&
          global_map_cleared.data[idx_tmp] != val_free) {
        global_map_cleared.data[idx_tmp] = val_free;
      }
    }
  }
  return global_map_cleared;
}

bool GlobalPlanServer::is_outside_map(const Eigen::Vector3i& pn,
                                      const Eigen::Vector3i& dim) {
  return pn(0) < 0 || pn(0) >= dim(0) || pn(1) < 0 || pn(1) >= dim(1) ||
         pn(2) < 0 || pn(2) >= dim(2);
}

void GlobalPlanServer::goalCB() {
  goal_ = global_as_->acceptNewGoal();
  aborted_ = false;
  process_all();
}

bool GlobalPlanServer::global_plan_process(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& global_map) {
  std::string map_frame;
  map_frame = global_map.header.frame_id;
  if (use_3d_global_) {
    if (z_cost_factor_ > 1) {
      // increase the cost of z in voxel map
      kr_planning_msgs::VoxelMap non_uniform_cost_map =
          ChangeZCost(global_map, z_cost_factor_);
      setMap(jps_3d_map_util_, non_uniform_cost_map);
    } else {
      setMap(jps_3d_map_util_, global_map);
    }
    jps_3d_util_->updateMap();
  } else {
    // slice the 3D map to get 2D map for path planning, at the height of
    // z-value of start point
    kr_planning_msgs::VoxelMap global_occ_map =
        SliceMap(start.pos(2), global_map.resolution, global_map);
    setMap(jps_map_util_, global_occ_map);
    jps_util_->updateMap();
  }

  // Path planning using JPS
  if (use_3d_global_) {
    // scale up the z-axis value of goal and start to accommodate to the scaling
    // of voxel resolution
    auto goal_scaled = goal;
    goal_scaled.pos[2] = goal.pos[2] * z_cost_factor_;
    auto start_scaled = start;
    start_scaled.pos[2] = start.pos[2] * z_cost_factor_;
    if (!jps_3d_util_->plan(start_scaled.pos, goal_scaled.pos, 1.0, true)) {
      // jps_3d_util_->plan params: start, goal, eps, use_jps
      ROS_WARN("Fail to plan a 3d global path!");
    } else {
      vec_Vec3f path_raw = jps_3d_util_->getPath();
      vec_Vec3f global_path;
      for (const auto& it : path_raw) {
        // scale back the z-axis value of path
        global_path.push_back(Vec3f(it(0), it(1), it(2) / z_cost_factor_));
      }

      // publish global_path_msg_
      global_path_msg_ = kr::path_to_ros(global_path);
      global_path_msg_.header.frame_id = map_frame;
      path_pub_.publish(global_path_msg_);
    }
  } else {
    if (!jps_util_->plan(
            start.pos.topRows(2), goal.pos.topRows(2), 1.0, true)) {
      // jps_util_->plan params: start, goal, eps, use_jps
      ROS_WARN("Fail to plan a 2d global path!");
      return false;
    } else {
      vec_Vec3f global_path;
      vec_Vec2f path2d = jps_util_->getPath();
      // assign z-value of the start position to the whole path
      for (const auto& it : path2d) {
        global_path.push_back(Vec3f(it(0), it(1), start.pos(2)));
      }

      // publish
      global_path_msg_ = kr::path_to_ros(global_path);
      global_path_msg_.header.frame_id = map_frame;
      path_pub_.publish(global_path_msg_);
    }
  }
  return true;
}

kr_planning_msgs::VoxelMap GlobalPlanServer::SliceMap(
    double h, double hh, const kr_planning_msgs::VoxelMap& map) {
  // slice a 3D voxel map
  kr_planning_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = map.origin.x;
  voxel_map.origin.y = map.origin.y;
  voxel_map.origin.z = 0;
  voxel_map.dim.x = map.dim.x;
  voxel_map.dim.y = map.dim.y;
  voxel_map.dim.z = 1;
  voxel_map.resolution = map.resolution;
  char val_default = 0;
  voxel_map.data.resize(map.dim.x * map.dim.y, val_default);
  int hi = hh / map.resolution;
  int h_min = (h - map.origin.z) / map.resolution - hi;
  h_min = h_min >= 0 ? h_min : 0;
  h_min = h_min < map.dim.z ? h_min : map.dim.z - 1;
  int h_max = (h - map.origin.z) / map.resolution + hi;
  h_max = h_max > 0 ? h_max : 1;
  h_max = h_max <= map.dim.z ? h_max : map.dim.z;
  Vec3i n;
  for (n(0) = 0; n(0) < map.dim.x; n(0)++) {
    for (n(1) = 0; n(1) < map.dim.y; n(1)++) {
      for (n(2) = h_min; n(2) < h_max; n(2)++) {
        int map_idx = n(0) + map.dim.x * n(1) + map.dim.x * map.dim.y * n(2);
        // if (map.data[map_idx] > val_free) {
        int idx = n(0) + map.dim.x * n(1);
        voxel_map.data[idx] = map.data[map_idx];
        // }
      }
    }
  }
  return voxel_map;
}

kr_planning_msgs::VoxelMap GlobalPlanServer::ChangeZCost(
    const kr_planning_msgs::VoxelMap& map, int z_cost_factor) {
  kr_planning_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = map.origin.x;
  voxel_map.origin.y = map.origin.y;
  voxel_map.origin.z = map.origin.z * z_cost_factor;
  voxel_map.dim.x = map.dim.x;
  voxel_map.dim.y = map.dim.y;
  voxel_map.dim.z = map.dim.z * z_cost_factor;
  voxel_map.resolution = map.resolution;
  char val_default = 0;
  voxel_map.data.resize(voxel_map.dim.x * voxel_map.dim.y * voxel_map.dim.z,
                        val_default);
  Vec3i n;
  for (n(0) = 0; n(0) < map.dim.x; ++n(0)) {
    for (n(1) = 0; n(1) < map.dim.y; ++n(1)) {
      for (n(2) = 0; n(2) < map.dim.z; ++n(2)) {
        int map_idx = n(0) + map.dim.x * n(1) + map.dim.x * map.dim.y * n(2);
        for (int z_add = 0; z_add < z_cost_factor; ++z_add) {
          int idx =
              n(0) + map.dim.x * n(1) + map.dim.x * map.dim.y * (n(2) + z_add);
          voxel_map.data[idx] = map.data[map_idx];
        }
      }
    }
  }
  return voxel_map;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_planner");

  ros::NodeHandle nh("~");

  GlobalPlanServer tpp(nh);

  ros::Rate r(20);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
