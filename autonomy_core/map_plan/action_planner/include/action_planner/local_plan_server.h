#include <action_planner/ActionPlannerConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <data_conversions.h>  // setMap, getMap, etc
#include <eigen_conversions/eigen_msg.h>
#include <kr_planning_msgs/PlanTwoPointAction.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <mpl_basis/trajectory.h>
#include <mpl_collision/map_util.h>
#include <mpl_planner/map_planner.h>
#include <plan_manage/planner_manager.h>
#include <primitive_ros_utils.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <traj_opt_ros/ros_bridge.h>
#include <traj_utils/planning_visualization.h>

#include <boost/range/irange.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>

using boost::irange;

class LocalPlanServer {
 public:
  explicit LocalPlanServer(const ros::NodeHandle& nh);

  bool aborted_;
  class PlannerType {
   public:
    explicit PlannerType(LocalPlanServer* local_plan_server)
        : local_plan_server_(local_plan_server) {}
    virtual void setup() = 0;
    virtual void plan(const MPL::Waypoint3D& start,
                      const MPL::Waypoint3D& goal,
                      const kr_planning_msgs::VoxelMap& map) = 0;

    virtual kr_planning_msgs::PlanTwoPointResult process_result(
        double endt, int num_goals) = 0;

    LocalPlanServer* local_plan_server_;
  };

  class MPLPlanner : public PlannerType {
   public:
    explicit MPLPlanner(LocalPlanServer* local_plan_server)
        : PlannerType(local_plan_server) {}

    void setup();
    void plan(const MPL::Waypoint3D& start,
              const MPL::Waypoint3D& goal,
              const kr_planning_msgs::VoxelMap& map);
    kr_planning_msgs::PlanTwoPointResult process_result(double endt,
                                                        int num_goals);

   private:
    MPL::Trajectory3D mp_traj_;
    std::shared_ptr<MPL::VoxelMapPlanner> mp_planner_util_;
    std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;
    bool debug_;
    bool verbose_;
    double tol_pos_, goal_tol_vel_, goal_tol_acc_;
    bool use_3d_local_;  // it is for 2D mp planner
    ros::Publisher expanded_cloud_pub;
  };
  class OptPlanner : public PlannerType {
   public:
    explicit OptPlanner(LocalPlanServer* local_plan_server)
        : PlannerType(local_plan_server) {}

    void setup();
    void plan(const MPL::Waypoint3D& start,
              const MPL::Waypoint3D& goal,
              const kr_planning_msgs::VoxelMap& map);
    kr_planning_msgs::PlanTwoPointResult process_result(double endt,
                                                        int num_goals);

   private:
    opt_planner::PlannerManager::Ptr planner_manager_;
    min_jerk::Trajectory opt_traj_;
    std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;
  };

 private:
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
  boost::shared_ptr<const kr_planning_msgs::PlanTwoPointGoal> goal_;
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
  void process_result(const kr_planning_msgs::SplineTrajectory& traj_msg,
                      bool solved);

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
