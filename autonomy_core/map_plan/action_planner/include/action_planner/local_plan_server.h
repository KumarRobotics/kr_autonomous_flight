#ifndef ACTION_PLANNER_LOCAL_PLAN_SERVER_H_
#define ACTION_PLANNER_LOCAL_PLAN_SERVER_H_

#include <action_planner/ActionPlannerConfig.h>
#include <action_planner/data_conversions.h>  // setMap, getMap, etc
#include <action_planner/planner_details.h>   // PlannerType
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <kr_planning_msgs/PlanTwoPointAction.h>
#include <kr_tracker_msgs/Transition.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <string>

#include "kr_tracker_msgs/PolyTrackerAction.h"

class LocalPlanServer {
 public:
  explicit LocalPlanServer(const ros::NodeHandle& nh);

  bool aborted_;

 private:
  // TODO(laura) what is the difference btw the node handles
  ros::NodeHandle pnh_;
  ros::NodeHandle traj_planner_nh_;
  ros::Subscriber local_map_sub_, local_no_infla_map_sub_;
  ros::Publisher local_map_cleared_pub_;
  ros::Publisher traj_pub_;
  ros::Publisher traj_goal_pub_;  // let's eventually get rid of this
  ros::ServiceClient srv_transition_;

  // this is different since you can wait for tracker to finish

  // visualization messages pub
  ros::Publisher sg_pub;

  boost::mutex map_mtx, traj_mtx;  // TODO(xu): do we need this?

  double traj_total_time_;
  float computation_time_;
  float compute_time_front_end_;
  float compute_time_back_end_;
  int success_status_;

  // current local map
  kr_planning_msgs::VoxelMapConstPtr local_map_ptr_ = nullptr;
  kr_planning_msgs::VoxelMapConstPtr local_nofla_map_ptr_ = nullptr;

  // action server
  std::unique_ptr<
      actionlib::SimpleActionClient<kr_tracker_msgs::PolyTrackerAction>>
      traj_goal_ac_;
  std::unique_ptr<
      actionlib::SimpleActionServer<kr_planning_msgs::PlanTwoPointAction>>
      local_as_;

  bool pub_cleared_map_ = false;
  bool set_vis_ = false;
  CompositePlanner* planner_;

  std::string frame_id_, poly_srv_name_;
  bool use_discrete_traj_;
  bool use_tracker_client_;

  /**
   * @brief Goal callback function, prevent concurrent planner modes
   */
  void goalCB();

  /**
   * @brief Call planner after setting planner start and goal and specify params
   * (use jrk, acc or vel)
   */
  void process_goal(const kr_planning_msgs::PlanTwoPointGoal& goal);

  /***@yuwei***/
  /**
   * @brief Record result (trajectory, status, etc)
   */
  void process_result(
      const std::pair<kr_planning_msgs::SplineTrajectory,
                      kr_planning_msgs::TrajectoryDiscretized>& traj_combined,
      ros::Duration execution_time,
      int epoch);

  /**
   * @brief map callback, update local_map_ptr_
   */
  void localMapCB(const kr_planning_msgs::VoxelMap::ConstPtr& msg);

  /**
   * @brief map no inflation callback, update local_nofla_map_ptr_
   */
  void localMapCBNoInfla(const kr_planning_msgs::VoxelMap::ConstPtr& msg);

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