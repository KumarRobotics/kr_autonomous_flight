#ifndef ACTION_PLANNER_PLANNER_DETAILS_H_
#define ACTION_PLANNER_PLANNER_DETAILS_H_

#include <action_planner/data_conversions.h>
#include <action_planner/primitive_ros_utils.h>
#include <jps/jps_planner.h>
#include <kr_planning_msgs/action/plan_two_point.hpp>
#include <kr_planning_msgs/msg/spline_trajectory.hpp>
#include <kr_planning_msgs/msg/trajectory_discretized.hpp>
#include <kr_planning_msgs/msg/voxel_map.hpp>
#include <kr_planning_msgs/msg/path.hpp>
#include <kr_planning_rviz_plugins/data_ros_utils.h>  //vec_to_cloud
#include <motion_primitives/graph_search.h>
#include <motion_primitives/utils.h>
#include <mpl_basis/trajectory.h>
#include <mpl_collision/map_util.h>
#include <mpl_planner/map_planner.h>
#include <plan_manage/planner_manager.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <traj_opt_ros/ros_bridge.h>
#include <traj_utils/planning_visualization.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gcopter/planner.hpp>
// #include "altro/altro.hpp"
#include <kr_ilqr_optimizer/spline_trajectory_sampler.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

class PlannerType {
 public:
  using PlanTwoPoint = kr_planning_msgs::action::PlanTwoPoint;
  using Goal = PlanTwoPoint::Goal;

  explicit PlannerType(rclcpp::Node* node, const std::string& frame_id)
      : node_(node), frame_id_(frame_id) {}
  virtual ~PlannerType() = default;
  virtual void setup() = 0;
  // always called by individual planner
  virtual kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map) {
    (void)start;
    (void)goal;
    (void)map;
    RCLCPP_WARN(
        node_->get_logger(),
        "[Plannner Details]:plan spline not implemented for this planner");
    return kr_planning_msgs::msg::SplineTrajectory();
  }
  virtual kr_planning_msgs::msg::TrajectoryDiscretized plan_discrete(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map) {
    (void)start;
    (void)goal;
    (void)map;
    RCLCPP_WARN(
        node_->get_logger(),
        "[Plannner Details]:plan discrete not implemented for this planner");
    return kr_planning_msgs::msg::TrajectoryDiscretized();
  }  // this does not have to be implemented

  // always called by composite planner
  virtual std::tuple<kr_planning_msgs::msg::SplineTrajectory,
                     kr_planning_msgs::msg::SplineTrajectory,
                     kr_planning_msgs::msg::TrajectoryDiscretized>
  plan_composite(const MPL::Waypoint3D& start,
                 const MPL::Waypoint3D& goal,
                 const kr_planning_msgs::msg::VoxelMap& map,
                 const kr_planning_msgs::msg::VoxelMap& map_no_inflation,
                 float* compute_time_front_end,
                 float* compute_time_back_end,
                 float* compute_time_poly,
                 int& success_status) {
    (void)start;
    (void)goal;
    (void)map;
    (void)map_no_inflation;
    (void)compute_time_front_end;
    (void)compute_time_back_end;
    (void)compute_time_poly;
    (void)success_status;
    RCLCPP_ERROR(node_->get_logger(),
                 "[Plannner Details]:plan composite not implemented");
    return std::make_tuple(kr_planning_msgs::msg::SplineTrajectory(),
                           kr_planning_msgs::msg::SplineTrajectory(),
                           kr_planning_msgs::msg::TrajectoryDiscretized());
  }

  virtual MPL::Waypoint3D evaluate(double t) = 0;
  void setGoal(const Goal& goal) { action_server_goal_ = goal; }
  double getTotalTrajTime() { return traj_total_time_; }
  void setSearchPath(const std::vector<Eigen::Vector3d>& search_path) {
    search_path_ = search_path;
  }
  std::vector<Eigen::Vector3d> SamplePath(double dt);

  bool has_collision(const kr_planning_msgs::msg::VoxelMap& map);

  rclcpp::Node* node_;
  std::string frame_id_;
  double traj_total_time_;
  // TODO(Laura) not sure if this is the best way to pass the search path
  std::vector<Eigen::Vector3d> search_path_;
  kr_planning_msgs::msg::SplineTrajectory search_path_msg_;
  // If replanning, some planners requires the previous trajectory which is
  // contained in the action server goal
  Goal action_server_goal_;
  std::vector<Eigen::MatrixXd> hPolys;  // opt needs to have these
  Eigen::VectorXd allo_ts;
};

class CompositePlanner : public PlannerType {
 public:
  explicit CompositePlanner(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}
  void setup();
  std::tuple<kr_planning_msgs::msg::SplineTrajectory,
             kr_planning_msgs::msg::SplineTrajectory,
             kr_planning_msgs::msg::TrajectoryDiscretized>
  plan_composite(const MPL::Waypoint3D& start,
                 const MPL::Waypoint3D& goal,
                 const kr_planning_msgs::msg::VoxelMap& map,
                 const kr_planning_msgs::msg::VoxelMap& map_no_inflation,
                 float* compute_time_front_end,
                 float* compute_time_back_end,
                 float* compute_time_poly,
                 int& success_status);
  MPL::Waypoint3D evaluate(double t);

 private:
  PlannerType* search_planner_type_{nullptr};
  PlannerType* opt_planner_type_{nullptr};
  double path_sampling_dt_;
  rclcpp::Publisher<kr_planning_msgs::msg::SplineTrajectory>::SharedPtr
      search_traj_pub_;
  opt_planner::PlannerManager::Ptr poly_generator_;
  std::shared_ptr<MPL::VoxelMapUtil> poly_gen_map_util_;
};

namespace OptPlanner {

class iLQR_Planner : public PlannerType {
 public:
  explicit iLQR_Planner(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::TrajectoryDiscretized plan_discrete(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  SplineTrajSampler::Ptr sampler_;
  double ilqr_sampling_dt_ = 0.1;
  std::vector<Eigen::VectorXd> opt_traj_;  // empty if no solution yet
};

class DoubleDescription : public PlannerType {
 public:
  explicit DoubleDescription(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  opt_planner::PlannerManager::Ptr planner_manager_;
  min_jerk::Trajectory opt_traj_;
  std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;
};

class GCOPTER : public PlannerType {
 public:
  explicit GCOPTER(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  gcopter::GcopterPlanner::Ptr planner_manager_;
  Trajectory<5> opt_traj_;
};

}  // namespace OptPlanner

namespace SearchPlanner {
class UniformInputSampling : public PlannerType {
 public:
  explicit UniformInputSampling(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  MPL::Trajectory3D mp_traj_;
  std::shared_ptr<MPL::VoxelMapPlanner> mp_planner_util_;
  std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;
  bool debug_;
  bool verbose_;
  double tol_pos_, goal_tol_vel_, goal_tol_acc_;
  bool use_3d_local_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr
      expanded_cloud_pub;
};

class Dispersion : public PlannerType {
 public:
  explicit Dispersion(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  double tol_pos_;
  std::string heuristic_;
  std::shared_ptr<motion_primitives::MotionPrimitiveGraph> graph_;
  std::vector<std::shared_ptr<motion_primitives::MotionPrimitive>>
      dispersion_traj_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visited_pub_;
  motion_primitives::GraphSearch::Option options_;
};

class Geometric : public PlannerType {
 public:
  explicit Geometric(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  std::shared_ptr<JPS::JPSPlanner3D> jps_3d_util_;
  std::shared_ptr<JPS::VoxelMapUtil> jps_3d_map_util_;
  bool verbose_{true};
  kr_planning_msgs::msg::SplineTrajectory spline_traj_;
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr path_pub_;
};

class PathThrough : public PlannerType {
 public:
  explicit PathThrough(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  void highLevelPlannerCB(const kr_planning_msgs::msg::Path::ConstSharedPtr path);
  bool verbose_{true};
  kr_planning_msgs::msg::SplineTrajectory spline_traj_;
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<kr_planning_msgs::msg::Path>::SharedPtr
      high_level_planner_sub_;
  kr_planning_msgs::msg::Path path_ = kr_planning_msgs::msg::Path();
};

// SST
class DynSampling : public PlannerType {
 public:
  explicit DynSampling(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  gcopter::GcopterPlanner::Ptr sstplanner_;
  bool verbose_{true};
  vec_Vecf<3> path_;
  kr_planning_msgs::msg::SplineTrajectory spline_traj_;
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr path_pub_;
};

// RRT
class Sampling : public PlannerType {
 public:
  explicit Sampling(rclcpp::Node* node, const std::string& frame_id)
      : PlannerType(node, frame_id) {}

  void setup();
  kr_planning_msgs::msg::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::msg::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  gcopter::GcopterPlanner::Ptr rrtplanner_;
  bool verbose_{true};
  vec_Vecf<3> path_;
  kr_planning_msgs::msg::SplineTrajectory spline_traj_;
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace SearchPlanner

#endif  // ACTION_PLANNER_PLANNER_DETAILS_H_