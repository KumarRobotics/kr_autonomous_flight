#ifndef ACTION_PLANNER_LOCAL_PLAN_SERVER_H_
#define ACTION_PLANNER_LOCAL_PLAN_SERVER_H_

#include <action_planner/ActionPlannerConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <data_conversions.h>  // setMap, getMap, etc
#include <eigen_conversions/eigen_msg.h>
#include <kr_planning_msgs/PlanTwoPointAction.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <motion_primitives/graph_search.h>
#include <motion_primitives/utils.h>
#include <mpl_basis/trajectory.h>
#include <mpl_collision/map_util.h>
#include <mpl_planner/map_planner.h>
#include <plan_manage/planner_manager.h>
#include <primitive_ros_utils.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <traj_opt_ros/ros_bridge.h>
#include <traj_utils/planning_visualization.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <string>

class LocalPlanServer {
 public:
  explicit LocalPlanServer(const ros::NodeHandle& nh);

  bool aborted_;
  class PlannerType {
   public:
    explicit PlannerType(
        const ros::NodeHandle& nh,
        const std::string& frame_id,
        const kr_planning_msgs::PlanTwoPointGoal& action_server_goal =
            kr_planning_msgs::PlanTwoPointGoal())
        : nh_(nh),
          frame_id_(frame_id),
          action_server_goal_(action_server_goal) {}
    virtual void setup() = 0;
    virtual kr_planning_msgs::SplineTrajectory plan(
        const MPL::Waypoint3D& start,
        const MPL::Waypoint3D& goal,
        const kr_planning_msgs::VoxelMap& map) = 0;

    virtual MPL::Waypoint3D evaluate(double t) = 0;
    double getTotalTrajTime() { return traj_total_time_; }

    ros::NodeHandle nh_;
    std::string frame_id_;
    double traj_total_time_;
    // If replanning, some planners requires the previous trajectory which is
    // contained in the action server goal
    const kr_planning_msgs::PlanTwoPointGoal action_server_goal_;
  };

  class MPLPlanner : public PlannerType {
   public:
    explicit MPLPlanner(
        const ros::NodeHandle& nh,
        const std::string& frame_id,
        const kr_planning_msgs::PlanTwoPointGoal& action_server_goal =
            kr_planning_msgs::PlanTwoPointGoal())
        : PlannerType(nh, frame_id, action_server_goal) {}

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
  class OptPlanner : public PlannerType {
   public:
    explicit OptPlanner(
        const ros::NodeHandle& nh,
        const std::string& frame_id,
        const kr_planning_msgs::PlanTwoPointGoal& action_server_goal =
            kr_planning_msgs::PlanTwoPointGoal())
        : PlannerType(nh, frame_id, action_server_goal) {}

    // TODO, be able to pass initial search-based path in
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

  class DispersionPlanner : public PlannerType {
   public:
    explicit DispersionPlanner(
        const ros::NodeHandle& nh,
        const std::string& frame_id,
        const kr_planning_msgs::PlanTwoPointGoal& action_server_goal =
            kr_planning_msgs::PlanTwoPointGoal())
        : PlannerType(nh, frame_id, action_server_goal) {}

    void setup();
    kr_planning_msgs::SplineTrajectory plan(
        const MPL::Waypoint3D& start,
        const MPL::Waypoint3D& goal,
        const kr_planning_msgs::VoxelMap& map);
    MPL::Waypoint3D evaluate(double t);

   private:
    double tol_pos_, tol_vel_;
    std::string heuristic_;
    std::shared_ptr<motion_primitives::MotionPrimitiveGraph> graph_;
    std::vector<std::shared_ptr<motion_primitives::MotionPrimitive>>
        dispersion_traj_;
    ros::Publisher visited_pub_;
  };

 private:
  // TODO(laura) what is the difference btw the node handles
  ros::NodeHandle pnh_;
  ros::NodeHandle traj_planner_nh_;
  ros::Subscriber local_map_sub_;
  ros::Publisher local_map_cleared_pub_;

  // visualization messages pub
  ros::Publisher sg_pub;

  boost::mutex map_mtx, traj_mtx;  // TODO(xu): do we need this?

  double traj_total_time_;

  // current local map
  kr_planning_msgs::VoxelMapConstPtr local_map_ptr_ = nullptr;

  // actionlib
  kr_planning_msgs::PlanTwoPointGoal goal_;
  // action lib
  std::unique_ptr<
      actionlib::SimpleActionServer<kr_planning_msgs::PlanTwoPointAction>>
      local_as_;

  bool pub_cleared_map_ = false;
  bool set_vis_ = false;
  PlannerType* planner_type_;

  std::string frame_id_;

  /**
   * @brief Goal callback function, prevent concurrent planner modes
   */
  void goalCB();

  /**
   * @brief Call planner after setting planner start and goal and specify params
   * (use jrk, acc or vel)
   */
  void process_goal();

  /***@yuwei***/
  /**
   * @brief Record result (trajectory, status, etc)
   */
  void process_result(const kr_planning_msgs::SplineTrajectory& traj_msg);

  /**
   * @brief map callback, update local_map_ptr_
   */
  void localMapCB(const kr_planning_msgs::VoxelMap::ConstPtr& msg);

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
#endif  // ACTION_PLANNER_LOCAL_PLAN_SERVER_H_