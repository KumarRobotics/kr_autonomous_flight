#include <action_planner/ActionPlannerConfig.h>
#include <action_planner/PlanTwoPointAction.h>
#include <action_planner/PlanWaypointsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <jps_collision/map_util.h>               // jps related
#include <jps_planner/jps_planner/jps_planner.h>  // jps related
#include <mapper/data_conversions.h>              // setMap, getMap, etc
#include <nav_msgs/Odometry.h>                    // odometry
#include <planning_ros_msgs/VoxelMap.h>
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

class GlobalPlanServer {
 public:
  explicit GlobalPlanServer(const ros::NodeHandle &nh);

  /**
   * @brief Call process_goal function, check planner timeout
   */
  void process_all();

  /**
   * @brief Odom callback function, directly subscribing odom to get the path
   * start position
   */
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);

  // global path recorder
  planning_ros_msgs::Path global_path_msg_;

  bool aborted_;

 private:
  // global map sub
  ros::NodeHandle pnh_;
  ros::Subscriber global_map_sub_;
  ros::Subscriber odom_sub_;

  // use 2-d or 3-d
  bool use_3d_{false};

  // pub
  ros::Publisher path_pub_;

  // mutexes
  boost::mutex map_mtx;
  // current global map
  planning_ros_msgs::VoxelMap global_map_;

  bool global_planner_succeeded_{false};
  bool global_plan_exist_{false};

  // actionlib
  boost::shared_ptr<const action_planner::PlanTwoPointGoal> goal_;
  boost::shared_ptr<action_planner::PlanTwoPointResult> result_;
  // action lib
  std::unique_ptr<
      actionlib::SimpleActionServer<action_planner::PlanTwoPointAction>>
      global_as_;

  // planner related
  std::shared_ptr<JPSPlanner3D> jps_3d_util_;
  std::shared_ptr<JPS::VoxelMapUtil> jps_3d_map_util_;

  std::shared_ptr<JPSPlanner2D> jps_util_;
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
   * @brief map callback, update global_map_
   */
  void globalMapCB(const planning_ros_msgs::VoxelMap::ConstPtr &msg);

  /**
   * @brief Global planner warpper
   */
  bool global_plan_process(const Waypoint3D &start, const Waypoint3D &goal,
                           const planning_ros_msgs::VoxelMap &global_map);

  /**
   * @brief Slice map util, only used if plan with a 2d jps planner
   */
  planning_ros_msgs::VoxelMap sliceMap(double h, double hh,
                                       const planning_ros_msgs::VoxelMap &map);
};

void GlobalPlanServer::globalMapCB(
    const planning_ros_msgs::VoxelMap::ConstPtr &msg) {
  global_map_ = *msg;
}

GlobalPlanServer::GlobalPlanServer(const ros::NodeHandle &nh) : pnh_(nh) {
  path_pub_ = pnh_.advertise<planning_ros_msgs::Path>("path", 1, true);
  global_map_sub_ = pnh_.subscribe("global_voxel_map", 2,
                                   &GlobalPlanServer::globalMapCB, this);

  global_as_ = std::make_unique<
      actionlib::SimpleActionServer<action_planner::PlanTwoPointAction>>(
      pnh_, "plan_global_path", false);
  // Register goal and preempt callbacks
  global_as_->registerGoalCallback(
      boost::bind(&GlobalPlanServer::goalCB, this));
  global_as_->start();

  // odom callback
  odom_sub_ = pnh_.subscribe("odom", 10, &GlobalPlanServer::odom_callback, this,
                             ros::TransportHints().tcpNoDelay());

  // Set map util for jps
  if (use_3d_) {
    jps_3d_map_util_ = std::make_shared<JPS::VoxelMapUtil>();
    jps_3d_util_ =
        std::make_shared<JPSPlanner3D>(false);  // verbose set as false
    jps_3d_util_->setMapUtil(jps_3d_map_util_);
  } else {
    jps_map_util_ = std::make_shared<JPS::OccMapUtil>();
    jps_util_ = std::make_shared<JPSPlanner2D>(false);  // verbose set as false
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
  printf("Total time for global path planner: %f\n", dt);
}

void GlobalPlanServer::process_result(bool solved) {
  result_ = boost::make_shared<action_planner::PlanTwoPointResult>();
  result_->success = solved;  // set success status
  result_->policy_status = solved ? 1 : -1;
  result_->path = global_path_msg_;
  if (!solved && global_as_->isActive()) {
    ROS_WARN("+++++++++++++++++++++++++");
    ROS_WARN("Global planner: path planner failed!");
    ROS_WARN("Danger!!!!!");
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("+++++++++++++++++++++++++");
    // global_as_->setAborted();
  }

  // reset goal
  goal_ = boost::shared_ptr<action_planner::PlanTwoPointGoal>();
  if (global_as_->isActive()) global_as_->setSucceeded(*result_);
}

void GlobalPlanServer::odom_callback(const nav_msgs::Odometry::ConstPtr &odom) {
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
  Waypoint3D start, goal;

  // first, check if odom is received
  if (!odom_set_) {
    ROS_WARN("[Replanner:] Odom is not yet received!");
    return;
  }

  // record current odometry as start
  start.pos = pose_to_eigen(odom_msg_->pose.pose);
  goal.pos = pose_to_eigen(goal_->p_final);

  // call the planner

  global_planner_succeeded_ = global_plan_process(start, goal, global_map_);

  process_result(global_planner_succeeded_);
}

void GlobalPlanServer::goalCB() {
  goal_ = global_as_->acceptNewGoal();
  aborted_ = false;
  process_all();
}

bool GlobalPlanServer::global_plan_process(
    const Waypoint3D &start, const Waypoint3D &goal,
    const planning_ros_msgs::VoxelMap &global_map) {
  std::string map_frame;
  map_frame = global_map.header.frame_id;
  if (use_3d_) {
    setMap(jps_3d_map_util_, global_map);
    jps_3d_util_->updateMap();
  } else {
    // slice the 3D map to get 2D map for path planning, at the height of
    // z-value of start point
    planning_ros_msgs::VoxelMap global_occ_map =
        sliceMap(start.pos(2), global_map.resolution, global_map);
    setMap(jps_map_util_, global_occ_map);
    jps_util_->updateMap();
  }

  // Path planning using JPS
  if (use_3d_) {
    if (!jps_3d_util_->plan(start.pos, goal.pos, 1.0, true)) {
      // jps_3d_util_->plan params: start, goal, eps, use_jps
      ROS_WARN("Fail to plan a 3d global path!");
    } else {
      vec_Vec3f global_path = jps_3d_util_->getPath();
      // publish
      global_path_msg_ = path_to_ros(global_path);
      global_path_msg_.header.frame_id = map_frame;
      path_pub_.publish(global_path_msg_);
    }
  } else {
    if (!jps_util_->plan(start.pos.topRows(2), goal.pos.topRows(2), 1.0,
                         true)) {
      // jps_util_->plan params: start, goal, eps, use_jps
      ROS_WARN("Fail to plan a 2d global path!");
      return false;
    } else {
      vec_Vec3f global_path;
      vec_Vec2f path2d = jps_util_->getPath();
      // assign z-value of the start position to the whole path
      for (const auto &it : path2d) {
        global_path.push_back(Vec3f(it(0), it(1), start.pos(2)));
      }

      // publish
      global_path_msg_ = path_to_ros(global_path);
      global_path_msg_.header.frame_id = map_frame;
      path_pub_.publish(global_path_msg_);
    }
  }
  return true;
}

planning_ros_msgs::VoxelMap GlobalPlanServer::sliceMap(
    double h, double hh, const planning_ros_msgs::VoxelMap &map) {
  // slice a 3D voxel map
  planning_ros_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = map.origin.x;
  voxel_map.origin.y = map.origin.y;
  voxel_map.origin.z = 0;
  voxel_map.dim.x = map.dim.x;
  voxel_map.dim.y = map.dim.y;
  voxel_map.dim.z = 1;
  voxel_map.resolution = map.resolution;
  char val_free = 0;
  char val_occ = 100;
  voxel_map.data.resize(map.dim.x * map.dim.y, val_free);
  int hi = hh / map.resolution;
  int h_min = (h - map.origin.z) / map.resolution - hi;
  h_min = h_min >= 0 ? h_min : 0;
  h_min = h_min < map.dim.z ? h_min : map.dim.z - 1;
  int h_max = (h - map.origin.z) / map.resolution + hi;
  h_max = h_max > 0 ? h_max : 1;
  h_max = h_max <= map.dim.z ? h_max : map.dim.z;
  // printf("h_min: %d, h_max: %d\n", h_min, h_max);
  Vec3i n;
  for (n(0) = 0; n(0) < map.dim.x; n(0)++) {
    for (n(1) = 0; n(1) < map.dim.y; n(1)++) {
      for (n(2) = h_min; n(2) < h_max; n(2)++) {
        int map_idx = n(0) + map.dim.x * n(1) + map.dim.x * map.dim.y * n(2);
        if (map.data[map_idx] > val_free) {
          int idx = n(0) + map.dim.x * n(1);
          voxel_map.data[idx] = val_occ;
        }
      }
    }
  }
  return voxel_map;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_planner");

  ros::NodeHandle nh("~");

  GlobalPlanServer tpp(nh);

  ros::Rate r(20);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
    // tpp.process_all();
  }

  return 0;
}
