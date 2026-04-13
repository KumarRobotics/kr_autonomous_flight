#include <action_planner/data_conversions.h>  // setMap, getMap, etc
#include <action_planner/primitive_ros_utils.h>
#include <jps/jps_planner.h>  // jps related
#include <jps/map_util.h>     // jps related
#include <kr_planning_msgs/action/plan_two_point.hpp>
#include <kr_planning_msgs/msg/path.hpp>
#include <kr_planning_msgs/msg/voxel_map.hpp>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <traj_opt_ros/ros_bridge.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

class GlobalPlanServer : public rclcpp::Node {
 public:
  using PlanTwoPoint = kr_planning_msgs::action::PlanTwoPoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanTwoPoint>;

  GlobalPlanServer();

  void process_all();

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);

  // global path recorder
  kr_planning_msgs::msg::Path global_path_msg_;

  bool aborted_;

 private:
  // global map sub
  rclcpp::Subscription<kr_planning_msgs::msg::VoxelMap>::SharedPtr
      global_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  bool use_3d_global_;
  bool jps_verbose_;
  int z_cost_factor_;

  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<kr_planning_msgs::msg::VoxelMap>::SharedPtr
      global_map_cleared_pub_;

  bool pub_cleared_map_ = false;

  std::mutex map_mtx;
  kr_planning_msgs::msg::VoxelMap::ConstSharedPtr global_map_ptr_;

  bool global_planner_succeeded_{false};
  bool global_plan_exist_{false};

  // current action goal state
  std::shared_ptr<const PlanTwoPoint::Goal> goal_;
  std::shared_ptr<GoalHandle> current_goal_handle_;

  rclcpp_action::Server<PlanTwoPoint>::SharedPtr global_as_;

  // planner related
  std::shared_ptr<JPS::JPSPlanner3D> jps_3d_util_;
  std::shared_ptr<JPS::VoxelMapUtil> jps_3d_map_util_;

  std::shared_ptr<JPS::JPSPlanner2D> jps_util_;
  std::shared_ptr<JPS::OccMapUtil> jps_map_util_;

  // odom related
  bool odom_set_{false};
  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_;

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const PlanTwoPoint::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  void process_goal();
  void process_result(bool solved);
  void globalMapCB(const kr_planning_msgs::msg::VoxelMap::ConstSharedPtr msg);

  bool global_plan_process(const MPL::Waypoint3D& start,
                           const MPL::Waypoint3D& goal,
                           const kr_planning_msgs::msg::VoxelMap& global_map);

  kr_planning_msgs::msg::VoxelMap SliceMap(
      double h, double hh, const kr_planning_msgs::msg::VoxelMap& map);

  kr_planning_msgs::msg::VoxelMap ChangeZCost(
      const kr_planning_msgs::msg::VoxelMap& map, int z_cost_factor = 1);

  kr_planning_msgs::msg::VoxelMap clear_map_position(
      const kr_planning_msgs::msg::VoxelMap& global_map,
      const vec_Vec3f& positions);

  bool is_outside_map(const Eigen::Vector3i& pn, const Eigen::Vector3i& dim);
};

void GlobalPlanServer::globalMapCB(
    const kr_planning_msgs::msg::VoxelMap::ConstSharedPtr msg) {
  global_map_ptr_ = msg;
}

GlobalPlanServer::GlobalPlanServer() : rclcpp::Node("action_planner") {
  path_pub_ = this->create_publisher<kr_planning_msgs::msg::Path>(
      "path", rclcpp::QoS(1).transient_local());
  global_map_cleared_pub_ =
      this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
          "global_voxel_map_cleared", rclcpp::QoS(1).transient_local());

  global_map_sub_ = this->create_subscription<kr_planning_msgs::msg::VoxelMap>(
      "global_voxel_map",
      2,
      std::bind(
          &GlobalPlanServer::globalMapCB, this, std::placeholders::_1));

  this->declare_parameter<bool>("trajectory_planner.use_3d_global", false);
  this->get_parameter("trajectory_planner.use_3d_global", use_3d_global_);
  this->declare_parameter<int>("trajectory_planner.z_cost_factor", 1);
  this->get_parameter("trajectory_planner.z_cost_factor", z_cost_factor_);

  this->declare_parameter<bool>("local_global_server.global_planner_verbose",
                                false);
  this->get_parameter("local_global_server.global_planner_verbose",
                      jps_verbose_);

  global_as_ = rclcpp_action::create_server<PlanTwoPoint>(
      this,
      "plan_global_path",
      std::bind(&GlobalPlanServer::handleGoal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&GlobalPlanServer::handleCancel,
                this,
                std::placeholders::_1),
      std::bind(&GlobalPlanServer::handleAccepted,
                this,
                std::placeholders::_1));

  rclcpp::SubscriptionOptions sub_opts;
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10,
      std::bind(
          &GlobalPlanServer::odom_callback, this, std::placeholders::_1),
      sub_opts);

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

rclcpp_action::GoalResponse GlobalPlanServer::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const PlanTwoPoint::Goal> /*goal*/) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanServer::handleCancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlanServer::handleAccepted(
    const std::shared_ptr<GoalHandle> goal_handle) {
  std::thread{std::bind(&GlobalPlanServer::execute, this, std::placeholders::_1),
              goal_handle}
      .detach();
}

void GlobalPlanServer::execute(
    const std::shared_ptr<GoalHandle> goal_handle) {
  auto start_timer = std::chrono::high_resolution_clock::now();
  current_goal_handle_ = goal_handle;
  goal_ = goal_handle->get_goal();
  aborted_ = false;
  process_all();
  auto end_timer = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_timer - start_timer);
  std::cout << "Global goalCB took" << duration.count() << "micro sec"
            << std::endl;
}

void GlobalPlanServer::process_all() {
  std::lock_guard<std::mutex> lockm(map_mtx);

  if (goal_ == nullptr) return;
  rclcpp::Time t0 = this->now();
  process_goal();
  double dt = (this->now() - t0).seconds();

  if (dt > 0.3 && current_goal_handle_ && current_goal_handle_->is_active()) {
    RCLCPP_WARN(this->get_logger(), "[GlobalPlanServer]+++++++++++++++++++++++++");
    RCLCPP_WARN(
        this->get_logger(), "Time out!!!!!! dt =  %f is too large!!!!!", dt);
    RCLCPP_WARN(this->get_logger(), "Abort!!!!!!");
    RCLCPP_WARN(this->get_logger(), "+++++++++++++++++++++++++");
    auto result = std::make_shared<PlanTwoPoint::Result>();
    current_goal_handle_->abort(result);
  }
}

void GlobalPlanServer::process_result(bool solved) {
  auto result = std::make_shared<PlanTwoPoint::Result>();
  result->success = solved;
  result->policy_status = solved ? 1 : -1;
  result->path = global_path_msg_;
  if (!solved && current_goal_handle_ && current_goal_handle_->is_active()) {
    RCLCPP_WARN(this->get_logger(), "+++++++++++++++++++++++++");
    RCLCPP_WARN(this->get_logger(), "Global planner: path planner failed!");
    RCLCPP_WARN(this->get_logger(), "Danger!!!!!");
    RCLCPP_WARN(this->get_logger(), "Abort!!!!!!");
    RCLCPP_WARN(this->get_logger(), "+++++++++++++++++++++++++");
    current_goal_handle_->abort(result);
  }

  // reset goal
  goal_ = nullptr;
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    current_goal_handle_->succeed(result);
  }
}

void GlobalPlanServer::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  if (!odom_set_) {
    odom_set_ = true;
  }
  odom_msg_ = odom;
}

void GlobalPlanServer::process_goal() {
  if (aborted_) {
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
      RCLCPP_WARN(this->get_logger(), "process_goal: Abort!");
      auto result = std::make_shared<PlanTwoPoint::Result>();
      current_goal_handle_->abort(result);
    }
    return;
  }

  MPL::Waypoint3D start, goal;

  if (!odom_set_) {
    RCLCPP_WARN(this->get_logger(), "[Replanner:] Odom is not yet received!");
    return;
  }

  start.pos = kr::pose_to_eigen(odom_msg_->pose.pose);
  goal.pos = kr::pose_to_eigen(goal_->p_final);

  std::cout << "start: " << start.pos << std::endl;
  std::cout << "goal: " << goal.pos << std::endl;

  if (!global_map_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Voxel Map is not yet received");
    return;
  }

  kr_planning_msgs::msg::VoxelMap global_map_cleared;
  vec_Vec3f positions;
  positions.push_back(start.pos);
  positions.push_back(goal.pos);
  global_map_cleared = clear_map_position(*global_map_ptr_, positions);

  if (pub_cleared_map_) {
    global_map_cleared_pub_->publish(global_map_cleared);
  }
  global_planner_succeeded_ =
      global_plan_process(start, goal, global_map_cleared);

  process_result(global_planner_succeeded_);
}

kr_planning_msgs::msg::VoxelMap GlobalPlanServer::clear_map_position(
    const kr_planning_msgs::msg::VoxelMap& global_map_original,
    const vec_Vec3f& positions) {
  kr_planning_msgs::msg::VoxelMap global_map_cleared;
  global_map_cleared = global_map_original;

  int8_t val_free = kr_planning_msgs::msg::VoxelMap::VAL_FREE;
  RCLCPP_WARN_ONCE(this->get_logger(), "Value free is set as %d", val_free);
  double robot_r = 1.0;
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

bool GlobalPlanServer::global_plan_process(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::msg::VoxelMap& global_map) {
  std::string map_frame;
  map_frame = global_map.header.frame_id;
  if (use_3d_global_) {
    if (z_cost_factor_ > 1) {
      kr_planning_msgs::msg::VoxelMap non_uniform_cost_map =
          ChangeZCost(global_map, z_cost_factor_);
      setMap(jps_3d_map_util_, non_uniform_cost_map);
    } else {
      setMap(jps_3d_map_util_, global_map);
    }
    jps_3d_util_->updateMap();
  } else {
    kr_planning_msgs::msg::VoxelMap global_occ_map =
        SliceMap(start.pos(2), global_map.resolution, global_map);
    setMap(jps_map_util_, global_occ_map);
    jps_util_->updateMap();
  }

  if (use_3d_global_) {
    auto goal_scaled = goal;
    goal_scaled.pos[2] = goal.pos[2] * z_cost_factor_;
    auto start_scaled = start;
    start_scaled.pos[2] = start.pos[2] * z_cost_factor_;
    if (!jps_3d_util_->plan(start_scaled.pos, goal_scaled.pos, 1.0, true)) {
      RCLCPP_WARN(this->get_logger(), "Fail to plan a 3d global path!");
    } else {
      vec_Vec3f path_raw = jps_3d_util_->getPath();
      vec_Vec3f global_path;
      for (const auto& it : path_raw) {
        global_path.push_back(Vec3f(it(0), it(1), it(2) / z_cost_factor_));
      }

      global_path_msg_ = kr::path_to_ros(global_path);
      global_path_msg_.header.frame_id = map_frame;
      path_pub_->publish(global_path_msg_);
    }
  } else {
    if (!jps_util_->plan(
            start.pos.topRows(2), goal.pos.topRows(2), 1.0, true)) {
      RCLCPP_WARN(this->get_logger(), "Fail to plan a 2d global path!");
      return false;
    } else {
      vec_Vec3f global_path;
      vec_Vec2f path2d = jps_util_->getPath();
      for (const auto& it : path2d) {
        global_path.push_back(Vec3f(it(0), it(1), start.pos(2)));
      }

      global_path_msg_ = kr::path_to_ros(global_path);
      global_path_msg_.header.frame_id = map_frame;
      path_pub_->publish(global_path_msg_);
    }
  }
  return true;
}

kr_planning_msgs::msg::VoxelMap GlobalPlanServer::SliceMap(
    double h, double hh, const kr_planning_msgs::msg::VoxelMap& map) {
  kr_planning_msgs::msg::VoxelMap voxel_map;
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
        int idx = n(0) + map.dim.x * n(1);
        voxel_map.data[idx] = map.data[map_idx];
      }
    }
  }
  return voxel_map;
}

kr_planning_msgs::msg::VoxelMap GlobalPlanServer::ChangeZCost(
    const kr_planning_msgs::msg::VoxelMap& map, int z_cost_factor) {
  kr_planning_msgs::msg::VoxelMap voxel_map;
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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalPlanServer>();
  rclcpp::Rate r(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
