#ifndef ACTION_PLANNER_PLANNER_DETAILS_H_
#define ACTION_PLANNER_PLANNER_DETAILS_H_

#include <action_planner/data_conversions.h>
#include <action_planner/primitive_ros_utils.h>
#include <jps/jps_planner.h>
#include <kr_planning_msgs/PlanTwoPointAction.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>  //vec_to_cloud
#include <motion_primitives/graph_search.h>
#include <motion_primitives/utils.h>
#include <mpl_basis/trajectory.h>
#include <mpl_collision/map_util.h>
#include <mpl_planner/map_planner.h>
#include <plan_manage/planner_manager.h>
#include <traj_opt_ros/ros_bridge.h>
#include <traj_utils/planning_visualization.h>

#include <gcopter/planner.hpp>
// #include "altro/altro.hpp"
#include <kr_ilqr_optimizer/spline_trajectory_sampler.hpp>
// #include <kr_planning_msgs/Traj.h>

#include <memory>
#include <string>
#include <vector>
#include <chrono>

class PlannerType {
 public:
  explicit PlannerType(const ros::NodeHandle& nh, const std::string& frame_id)
      : nh_(nh), frame_id_(frame_id) {}
  virtual void setup() = 0;
    //always called by individual planner
    virtual kr_planning_msgs::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map) {
        ROS_ERROR("[Plannner Details]:plan discrete not implemented");
        // std::logic_error("Function not yet implemented");
        return kr_planning_msgs::SplineTrajectory();
      }
    virtual kr_planning_msgs::TrajectoryDiscretized plan_discrete(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map) {
        ROS_ERROR("[Plannner Details]:plan discrete not implemented");
        // std::logic_error("Function not yet implemented");
        return kr_planning_msgs::TrajectoryDiscretized();
        }//this does not have to be implemented


      //always called by composite planner
  virtual std::pair<kr_planning_msgs::SplineTrajectory,kr_planning_msgs::TrajectoryDiscretized> plan_composite(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map,
      const kr_planning_msgs::VoxelMap& map_no_inflation,
      float* compute_time_front_end,
      float* compute_time_back_end) {
        ROS_ERROR("[Plannner Details]:plan composite not implemented");
        // std::logic_error("Function not yet implemented");
        return std::make_pair(kr_planning_msgs::SplineTrajectory(),kr_planning_msgs::TrajectoryDiscretized());
        }



  virtual MPL::Waypoint3D evaluate(double t) = 0;
  void setGoal(const kr_planning_msgs::PlanTwoPointGoal& goal) {
    action_server_goal_ = goal;
  }
  double getTotalTrajTime() { return traj_total_time_; }
  void setSearchPath(const std::vector<Eigen::Vector3d>& search_path) {
    search_path_ = search_path;
  }
  std::vector<Eigen::Vector3d> SamplePath();

  ros::NodeHandle nh_;
  std::string frame_id_;
  double traj_total_time_;
  // TODO(Laura) pass as param
  double path_sampling_dt_ = 0.15;
  // TODO(Laura) not sure if this is the best way to pass the search path
  std::vector<Eigen::Vector3d> search_path_;
  boost::shared_ptr<kr_planning_msgs::SplineTrajectory const> search_path_msg_;
  // If replanning, some planners requires the previous trajectory which is
  // contained in the action server goal
  kr_planning_msgs::PlanTwoPointGoal action_server_goal_;
};

class CompositePlanner : public PlannerType {
 public:
  explicit CompositePlanner(const ros::NodeHandle& nh,
                            const std::string& frame_id)
      : PlannerType(nh, frame_id) {}
  void setup();
  // kr_planning_msgs::SplineTrajectory plan(
  //     const MPL::Waypoint3D& start,
  //     const MPL::Waypoint3D& goal,
  //     const kr_planning_msgs::VoxelMap& map);
  std::pair<kr_planning_msgs::SplineTrajectory,kr_planning_msgs::TrajectoryDiscretized>  plan_composite(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map,
      const kr_planning_msgs::VoxelMap& map_no_inflation,
      float* compute_time_front_end,
      float* compute_time_back_end);
  MPL::Waypoint3D evaluate(double t);

 private:
  PlannerType* search_planner_type_;
  PlannerType* opt_planner_type_;
  ros::Publisher search_traj_pub_;
};

namespace OptPlanner {


class iLQR_Planner : public PlannerType {
 public:
  explicit iLQR_Planner(const ros::NodeHandle& nh,
                             const std::string& frame_id)
      : PlannerType(nh, frame_id) {}

  void setup();
  kr_planning_msgs::TrajectoryDiscretized plan_discrete(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  SplineTrajSampler::Ptr sampler_;
};

class DoubleDescription : public PlannerType {
 public:
  explicit DoubleDescription(const ros::NodeHandle& nh,
                             const std::string& frame_id)
      : PlannerType(nh, frame_id) {}

  void setup();
  kr_planning_msgs::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  opt_planner::PlannerManager::Ptr planner_manager_;
  min_jerk::Trajectory opt_traj_;
  std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;
};

class GCOPTER : public PlannerType {
 public:
  explicit GCOPTER(const ros::NodeHandle& nh, const std::string& frame_id)
      : PlannerType(nh, frame_id) {}

  void setup();
  kr_planning_msgs::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  gcopter::GcopterPlanner::Ptr planner_manager_;
  Trajectory<5> opt_traj_;
};

}  // namespace OptPlanner

namespace SearchPlanner {
class UniformInputSampling : public PlannerType {
 public:
  explicit UniformInputSampling(const ros::NodeHandle& nh,
                                const std::string& frame_id)
      : PlannerType(nh, frame_id) {}

  void setup();
  kr_planning_msgs::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  MPL::Trajectory3D mp_traj_;
  std::shared_ptr<MPL::VoxelMapPlanner> mp_planner_util_;
  std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;
  bool debug_;
  bool verbose_;
  double tol_pos_, goal_tol_vel_, goal_tol_acc_;
  bool use_3d_local_;
  ros::Publisher expanded_cloud_pub;
};

class Dispersion : public PlannerType {
 public:
  explicit Dispersion(const ros::NodeHandle& nh, const std::string& frame_id)
      : PlannerType(nh, frame_id) {}

  void setup();
  kr_planning_msgs::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  double tol_pos_;
  std::string heuristic_;
  std::shared_ptr<motion_primitives::MotionPrimitiveGraph> graph_;
  std::vector<std::shared_ptr<motion_primitives::MotionPrimitive>>
      dispersion_traj_;
  ros::Publisher visited_pub_;
  motion_primitives::GraphSearch::Option options_;
};

class Geometric : public PlannerType {
 public:
  explicit Geometric(const ros::NodeHandle& nh, const std::string& frame_id)
      : PlannerType(nh, frame_id) {}

  void setup();
  kr_planning_msgs::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  std::shared_ptr<JPS::JPSPlanner3D> jps_3d_util_;
  std::shared_ptr<JPS::VoxelMapUtil> jps_3d_map_util_;
  bool verbose_{true};
  kr_planning_msgs::SplineTrajectory spline_traj_;
  ros::Publisher path_pub_;
};


class PathThrough : public PlannerType {
 public:
  explicit PathThrough(const ros::NodeHandle& nh, const std::string& frame_id)
      : PlannerType(nh, frame_id) {}

  void setup();
  kr_planning_msgs::SplineTrajectory plan(
      const MPL::Waypoint3D& start,
      const MPL::Waypoint3D& goal,
      const kr_planning_msgs::VoxelMap& map);
  MPL::Waypoint3D evaluate(double t);

 private:
  void highLevelPlannerCB(const kr_planning_msgs::Path& path);
  bool verbose_{true};
  kr_planning_msgs::SplineTrajectory spline_traj_;
  ros::Publisher path_pub_;
  ros::Subscriber high_level_planner_sub_;
  kr_planning_msgs::Path path_ = kr_planning_msgs::Path();

};

}  // namespace SearchPlanner

#endif  // ACTION_PLANNER_PLANNER_DETAILS_H_