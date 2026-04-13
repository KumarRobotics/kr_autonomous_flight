#ifndef ACTION_PLANNER_LOCAL_PLAN_SERVER_H_
#define ACTION_PLANNER_LOCAL_PLAN_SERVER_H_

#include <action_planner/data_conversions.h>  // setMap, getMap, etc
#include <action_planner/planner_details.h>   // PlannerType
#include <kr_planning_msgs/action/plan_two_point.hpp>
#include <kr_planning_msgs/msg/path.hpp>
#include <kr_planning_msgs/msg/spline_trajectory.hpp>
#include <kr_planning_msgs/msg/trajectory_discretized.hpp>
#include <kr_planning_msgs/msg/voxel_map.hpp>
#include <kr_tracker_msgs/action/poly_tracker.hpp>
#include <kr_tracker_msgs/srv/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

class LocalPlanServer : public rclcpp::Node {
 public:
  using PlanTwoPoint = kr_planning_msgs::action::PlanTwoPoint;
  using PlanTwoPointGoalHandle = rclcpp_action::ServerGoalHandle<PlanTwoPoint>;
  using PolyTracker = kr_tracker_msgs::action::PolyTracker;
  using PolyTrackerGoalHandle = rclcpp_action::ClientGoalHandle<PolyTracker>;

  LocalPlanServer();

  bool aborted_;

 private:
  rclcpp::Subscription<kr_planning_msgs::msg::VoxelMap>::SharedPtr
      local_map_sub_;
  rclcpp::Subscription<kr_planning_msgs::msg::VoxelMap>::SharedPtr
      local_no_infla_map_sub_;
  rclcpp::Publisher<kr_planning_msgs::msg::VoxelMap>::SharedPtr
      local_map_cleared_pub_;
  rclcpp::Publisher<kr_planning_msgs::msg::SplineTrajectory>::SharedPtr
      traj_pub_;
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr sg_pub;
  rclcpp::Client<kr_tracker_msgs::srv::Transition>::SharedPtr srv_transition_;

  std::mutex map_mtx, traj_mtx;

  double traj_total_time_;
  float computation_time_;
  float compute_time_front_end_ = 0.0;
  float compute_time_back_end_ = 0.0;
  float compute_time_poly_ = 0.0;
  int success_status_;

  // current local map
  kr_planning_msgs::msg::VoxelMap::ConstSharedPtr local_map_ptr_ = nullptr;
  kr_planning_msgs::msg::VoxelMap::ConstSharedPtr local_nofla_map_ptr_ =
      nullptr;

  // action client and server
  rclcpp_action::Client<PolyTracker>::SharedPtr traj_goal_ac_;
  rclcpp_action::Server<PlanTwoPoint>::SharedPtr local_as_;

  // Current goal handle (set when an action is active)
  std::shared_ptr<PlanTwoPointGoalHandle> current_goal_handle_;

  bool pub_cleared_map_ = false;
  bool set_vis_ = false;
  CompositePlanner* planner_{nullptr};

  std::string frame_id_, poly_srv_name_;
  bool use_discrete_traj_;
  bool use_tracker_client_;

  /**
   * @brief Action goal-accept handler. Prevent concurrent planner modes.
   */
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const PlanTwoPoint::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<PlanTwoPointGoalHandle> goal_handle);

  void handleAccepted(
      const std::shared_ptr<PlanTwoPointGoalHandle> goal_handle);

  /**
   * @brief Execute the goal: calls process_goal and then process_result.
   */
  void execute(const std::shared_ptr<PlanTwoPointGoalHandle> goal_handle);

  /**
   * @brief Call planner after setting planner start and goal and specify params
   * (use jrk, acc or vel)
   */
  void process_goal(const PlanTwoPoint::Goal& goal);

  /**
   * @brief Record result (trajectory, status, etc)
   */
  void process_result(
      const std::tuple<kr_planning_msgs::msg::SplineTrajectory,
                       kr_planning_msgs::msg::SplineTrajectory,
                       kr_planning_msgs::msg::TrajectoryDiscretized>&
          traj_combined,
      const rclcpp::Duration& execution_time,
      int epoch);

  /**
   * @brief map callback, update local_map_ptr_
   */
  void localMapCB(const kr_planning_msgs::msg::VoxelMap::ConstSharedPtr msg);

  /**
   * @brief map no inflation callback, update local_nofla_map_ptr_
   */
  void localMapCBNoInfla(
      const kr_planning_msgs::msg::VoxelMap::ConstSharedPtr msg);

  /**
   * @brief Local planner clear footprint
   */
  kr_planning_msgs::msg::VoxelMap clear_map_position(
      const kr_planning_msgs::msg::VoxelMap& local_map, const Vec3f& start);

  /**
   * @brief Local planner check if outside map
   */
  bool is_outside_map(const Eigen::Vector3i& pn, const Eigen::Vector3i& dim);
};
#endif  // ACTION_PLANNER_LOCAL_PLAN_SERVER_H_
