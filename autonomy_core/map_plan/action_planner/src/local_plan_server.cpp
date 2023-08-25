#include <action_planner/ActionPlannerConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <mpl_basis/trajectory.h>
#include <mpl_collision/map_util.h>
#include <mpl_planner/map_planner.h>
#include <planning_ros_msgs/PlanTwoPointAction.h>
#include <planning_ros_msgs/PlanLocalPathAction.h>
#include <planning_ros_msgs/TrajectoryPoints.h>

#include <sloam_msgs/ShapeWithCovArray.h>

#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>



#include <ros/console.h>
#include <ros/ros.h>
#include <traj_opt_ros/ros_bridge.h>

#include <boost/range/irange.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "data_conversions.h"  // setMap, getMap, etc

#include <local_plan/local_plan_manager.h>
#include <back_end/gp_optimizer.hpp>


#include <chrono>

using boost::irange;
using namespace param_env;

// Local planning server
class LocalPlanServer {
 public:
  explicit LocalPlanServer(const ros::NodeHandle& nh);

  /**
   * @brief Call process_goal function, check planner timeout
   */
  void process_all();

  bool aborted_;

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber local_map_sub_;
  ros::Publisher traj_pub, traj_points_pub;


  // visualization messages pub
  ros::Publisher sg_pub;
  ros::Publisher expanded_cloud_pub;

  ros::Publisher local_map_cleared_pub;

  boost::mutex map_mtx, traj_mtx;  // TODO(xu): do we need this?

  // motion primitive planner util and its map util
  std::shared_ptr<MPL::VoxelMapPlanner> mp_planner_util_;
  std::shared_ptr<MPL::VoxelMapUtil> mp_map_util_;

  double traj_start_time_;

  //@yuwei local planner
  //benchmark
  double total_time_ = 0.0;
  int total_call_ =  0;
  // semantic-aware
  ros::Subscriber semantic_map_sub_;
  sloam_msgs::ShapeWithCovArrayConstPtr semantic_map_ptr_ = nullptr;
  std::shared_ptr<param_env::SemanticMapUtil> sp_map_util_;
  opt_planner::LocalPlanManager::Ptr local_plan_manager_;
  

  typedef Eigen::Matrix<double, 3, 6> CoefficientMat;
  // motion primitive trajectory
  MPL::Trajectory3D mp_traj_;
  MPL::Trajectory3D gp_traj3d_;
  opt_planner::LocalTrajData local_data_;
  double traj_total_time_ = 0.0;
  ros::Time traj_start_rostime_;
  planning_ros_msgs::SplineTrajectory spline_msg_;

  // current local map
  planning_ros_msgs::VoxelMapConstPtr local_map_ptr_ = nullptr;

  // actionlib
  boost::shared_ptr<const planning_ros_msgs::PlanLocalPathGoal> goal_;
  boost::shared_ptr<planning_ros_msgs::PlanLocalPathResult> result_; // yuwei: is this used? 
  // action lib
  std::unique_ptr<
      actionlib::SimpleActionServer<planning_ros_msgs::PlanLocalPathAction>>
      local_as_;

  bool debug_;
  bool verbose_;
  bool pub_cleared_map_ = false;
  bool use_3d_local_; // it is for 2D mp planner
  bool use_opt_planner_ = false;
  bool use_gp_path_;
  bool set_vis_ = false;

  std::string frame_id_;


  // planner tolerance
  double tol_pos_, goal_tol_vel_, goal_tol_acc_;

  // default params
  double w_vis_theta_ = 1.0, w_vis_r_ = 1.0;
  double theta_mean_ = 0.0, theta_range_ = 1.4;
  double r_mean_ = 6.0, r_range_ = 6.0;
  double dis_safe_, w_dis_;



  void update_result(Trajectory<5> &traj, int num_goals, double endt);
  void update_result( MPL::Trajectory3D &traj, int num_goals, double endt);



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

  /***@yuwei***/
  /**
   * @brief Record result (trajectory, status, etc)
   */
  void process_result(const planning_ros_msgs::SplineTrajectory& traj_msg, bool solved);

  /**
   * @brief map callback, update local_map_ptr_
   */
  void localMapCB(const planning_ros_msgs::VoxelMap::ConstPtr& msg);

  void localSementicsCB(const sloam_msgs::ShapeWithCovArray::ConstPtr& msg);


  void setSemanticMap();
  /**
   * @brief Local planner warpper
   */
  bool local_plan_process(const MPL::Waypoint3D& start,
                          const MPL::Waypoint3D& goal,
                          std::vector<Eigen::Vector3d> local_path,
                          const planning_ros_msgs::VoxelMap& map);

  /**
   * @brief Local planner clear footprint
   */
  planning_ros_msgs::VoxelMap clear_map_position(
      const planning_ros_msgs::VoxelMap& local_map, const Vec3f& start);

  /**
   * @brief Local planner check if outside map
   */
  bool is_outside_map(const Eigen::Vector3i& pn, const Eigen::Vector3i& dim);
};

// map callback, update local_map_ptr_
void LocalPlanServer::localMapCB(
    const planning_ros_msgs::VoxelMap::ConstPtr& msg) {
  ROS_WARN_ONCE("[Local planner:] Got the local voxel map!");
  local_map_ptr_ = msg;
  frame_id_ = local_map_ptr_->header.frame_id;

}


void LocalPlanServer::localSementicsCB(const sloam_msgs::ShapeWithCovArray::ConstPtr& msg) {
  ROS_WARN_ONCE("[LocalPlanServer::localSementicsCB] Got the local semantic map!");
  
  //set up semantic map
  //semantic_map_ptr_
  semantic_map_ptr_ = msg;
}

void LocalPlanServer::setSemanticMap() {
  //set up semantic map
  //semantic_map_ptr_

  sp_map_util_->clear();        


  //now only consider the position and position 
  if (semantic_map_ptr_ == nullptr)
  {
    ROS_WARN_ONCE("[LocalPlanServer::setSemanticMap] No semantic map!");
    return;
  }
  // Eigen::VectorXd robot_position(3);
  // robot_position << semantic_map_ptr_->robot_pose_with_cov.pose.position.x,
  //                   semantic_map_ptr_->robot_pose_with_cov.pose.position.y,
  //                   semantic_map_ptr_->robot_pose_with_cov.pose.position.z;
        
  // Eigen::MatrixXd covariance(3, 3);
  // for (int cov_row_idx = 0; cov_row_idx < 3; ++cov_row_idx) {
  //   for (int cov_col_idx = 0; cov_col_idx < 3; ++cov_col_idx) {
  //     int idx = cov_row_idx*6 + cov_col_idx;
  //     covariance(cov_row_idx, cov_col_idx) = semantic_map_ptr_->robot_pose_with_cov.covariance[idx];
  //   }
  // }
  // sp_map_util_->setPoseWithCov(robot_position, covariance);


  //2. setup semantic objects
  Eigen::VectorXd l_g_;
  Eigen::MatrixXd cov_; // covariance associate with points
  for (int i = 0; i < semantic_map_ptr_->shapes_with_cov.size(); ++i)
  {
    switch(semantic_map_ptr_->shapes_with_cov[i].shape_type)
    {
      case 0: //cylinder
      {
        Eigen::VectorXd shape_l_g(7);
        shape_l_g << semantic_map_ptr_->shapes_with_cov[i].cylinder_state[0], 
                     semantic_map_ptr_->shapes_with_cov[i].cylinder_state[1], 
                     semantic_map_ptr_->shapes_with_cov[i].cylinder_state[2], 
                     semantic_map_ptr_->shapes_with_cov[i].cylinder_state[3], 
                     semantic_map_ptr_->shapes_with_cov[i].cylinder_state[4], 
                     semantic_map_ptr_->shapes_with_cov[i].cylinder_state[5], 
                     semantic_map_ptr_->shapes_with_cov[i].cylinder_state[6]; 

        
        Eigen::MatrixXd shape_cov(7,7);
        for (int cov_row_idx = 0; cov_row_idx < 7; ++cov_row_idx) {
          for (int cov_col_idx = 0; cov_col_idx < 7; ++cov_col_idx) {
            int idx = cov_row_idx*7 + cov_col_idx;
            shape_cov(cov_row_idx, cov_col_idx) = semantic_map_ptr_->shapes_with_cov[i].covariance[idx];
          }
        }

        Cylinder cy = Cylinder(shape_l_g, shape_cov);
        cy.setVisParams(w_vis_theta_, w_vis_r_, theta_mean_, theta_range_, r_mean_, r_range_, dis_safe_+5.0, w_dis_);
        std::cout << "============add cylinder" << shape_l_g.transpose() << " " << shape_cov << std::endl;
        sp_map_util_->add(cy);

        break;
      }
      case 1: //cuboid
      {

        Eigen::VectorXd shape_l_g(9);

        double roll, pitch, yaw;
        tf::Quaternion q(semantic_map_ptr_->shapes_with_cov[i].cuboid_pose.orientation.x,
                         semantic_map_ptr_->shapes_with_cov[i].cuboid_pose.orientation.y,
                         semantic_map_ptr_->shapes_with_cov[i].cuboid_pose.orientation.z,
                         semantic_map_ptr_->shapes_with_cov[i].cuboid_pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

      
        shape_l_g << semantic_map_ptr_->shapes_with_cov[i].cuboid_pose.position.x, 
                     semantic_map_ptr_->shapes_with_cov[i].cuboid_pose.position.y,
                     semantic_map_ptr_->shapes_with_cov[i].cuboid_pose.position.z,
                     roll, 
                     pitch, 
                     yaw,
                     0.5 * semantic_map_ptr_->shapes_with_cov[i].cuboid_scale[0], 
                     0.5 * semantic_map_ptr_->shapes_with_cov[i].cuboid_scale[1], 
                     0.5 * semantic_map_ptr_->shapes_with_cov[i].cuboid_scale[2]; 

        
        Eigen::MatrixXd shape_cov(9,9);
        for (int cov_row_idx = 0; cov_row_idx < 9; ++cov_row_idx) {
          for (int cov_col_idx = 0; cov_col_idx < 9; ++cov_col_idx) {
            int idx = cov_row_idx*9 + cov_col_idx;
            shape_cov(cov_row_idx, cov_col_idx) = semantic_map_ptr_->shapes_with_cov[i].covariance[idx];
          }
        }

        Cuboid cu = Cuboid(shape_l_g, shape_cov);
        cu.setVisParams(w_vis_theta_, w_vis_r_, theta_mean_, theta_range_, r_mean_, r_range_, dis_safe_, w_dis_);
        std::cout << "============add cuboid" << shape_l_g.transpose() << " " << shape_cov << std::endl;
        sp_map_util_->add(cu);

        break;
      }
        
    }

  }
  // semantic_map_ptr_.Cylinder

  //   param_env::Cylinder cylinder(l_g, cov);
  //   sp_map_util_.add(cylinder);

  std::vector<Plane> planes_;
  std::vector<Cylinder> cylinders_;
  std::vector<Cuboid> cuboids_;

  
}


LocalPlanServer::LocalPlanServer(const ros::NodeHandle& nh) : pnh_(nh) {
  traj_pub = pnh_.advertise<planning_ros_msgs::Trajectory>("traj", 1, true);
  traj_points_pub = pnh_.advertise<planning_ros_msgs::TrajectoryPoints>("traj_points", 1, true); 

  local_map_sub_ =
      pnh_.subscribe("local_voxel_map", 2, &LocalPlanServer::localMapCB, this);
  local_as_ = std::make_unique<
      actionlib::SimpleActionServer<planning_ros_msgs::PlanLocalPathAction>>(
      pnh_, "plan_local_trajectory", false);

  // set up visualization publisher for mpl planner
  sg_pub = pnh_.advertise<planning_ros_msgs::Path>("start_goal", 1, true);
  expanded_cloud_pub =
      pnh_.advertise<sensor_msgs::PointCloud>("expanded_cloud", 1, true);

  local_map_cleared_pub = pnh_.advertise<planning_ros_msgs::VoxelMap>(
      "local_voxel_map_cleared", 1, true);

  //@yuwei set up semantic-aware conditions
  semantic_map_sub_ = pnh_.subscribe("/sloam/object_shapes_with_covariance", 2, &LocalPlanServer::localSementicsCB, this);


  // set up mpl planner
  ros::NodeHandle traj_planner_nh(pnh_, "trajectory_planner");
  traj_planner_nh.param("debug", debug_, false);
  traj_planner_nh.param("verbose", verbose_, false);

  double dt;
  double W, v_fov;
  int ndt, max_num;
  traj_planner_nh.param("tol_pos", tol_pos_, 0.5);
  traj_planner_nh.param("global_goal_tol_vel", goal_tol_vel_, 0.5);
  traj_planner_nh.param("global_goal_tol_acc", goal_tol_acc_, 0.5);
  /// Execution time for each primitive
  traj_planner_nh.param("dt", dt, 1.0);
  traj_planner_nh.param("ndt", ndt, -1);
  traj_planner_nh.param("max_num", max_num, -1);
  traj_planner_nh.param("heuristic_weight", W, 10.0);
  traj_planner_nh.param("vertical_semi_fov", v_fov, 0.392);
  traj_planner_nh.param("use_opt_planner", use_opt_planner_, false);


  //params for semantic object
  traj_planner_nh.param("gp/w_vis_theta", w_vis_theta_, -1.0);
  traj_planner_nh.param("gp/w_vis_r", w_vis_r_, -1.0);
  traj_planner_nh.param("gp/theta_mean", theta_mean_, -1.0);
  traj_planner_nh.param("gp/theta_range", theta_range_, -1.0);
  traj_planner_nh.param("gp/r_mean", r_mean_, -1.0);
  traj_planner_nh.param("gp/r_range", r_range_, -1.0);
  traj_planner_nh.param("gp/dis_safe", dis_safe_, -1.0);
  traj_planner_nh.param("gp/w_dis", w_dis_, -1.0);
  traj_planner_nh.param("gp/use_gp_path", use_gp_path_, false);
       

  mp_map_util_ = std::make_shared<MPL::VoxelMapUtil>();
  // TODO(xu:) not differentiating between 2D and 3D, causing extra resource
  // usage for 2D case, this needed to be changed in both planner util as well
  // as map util, which requires the slicing map function
  /***@yuwei: optimization based planner ***/
  sp_map_util_.reset(new param_env::SemanticMapUtil);
  if (use_opt_planner_){

    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("[LocalPlanServer:] Optimization Planner Mode");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    /* initialize main modules */
    local_plan_manager_.reset(new opt_planner::LocalPlanManager);
    local_plan_manager_->init(traj_planner_nh);
    local_plan_manager_->initMap(mp_map_util_, sp_map_util_);

  }else{

    traj_planner_nh.param("use_3d_local", use_3d_local_, false);
  
    double vz_max, az_max, jz_max, uz_max;
    traj_planner_nh.param("max_v_z", vz_max, 2.0);
    traj_planner_nh.param("max_a_z", az_max, 1.0);
    traj_planner_nh.param("max_j_z", jz_max, 1.0);
    traj_planner_nh.param("max_u_z", uz_max, 1.0);


    double v_max, a_max, j_max, u_max;
    traj_planner_nh.param("max_v", v_max, 2.0);
    traj_planner_nh.param("max_a", a_max, 1.0);
    traj_planner_nh.param("max_j", j_max, 1.0);
    traj_planner_nh.param("max_u", u_max, 1.0);

    // Important: set motion primitive control inputs
    vec_E<VecDf> U;
    if (!use_3d_local_) {
      const decimal_t du = u_max;
      for (decimal_t dx = -u_max; dx <= u_max; dx += du)
        for (decimal_t dy = -u_max; dy <= u_max; dy += du)
          U.push_back(Vec3f(dx, dy, 0));
    } else {
      const decimal_t du = u_max;
      const decimal_t du_z = uz_max;
      for (decimal_t dx = -u_max; dx <= u_max; dx += du)
        for (decimal_t dy = -u_max; dy <= u_max; dy += du)
          for (decimal_t dz = -uz_max; dz <= uz_max; dz += du_z)
            U.push_back(Vec3f(dx, dy, dz));
    }

    mp_planner_util_.reset(new MPL::VoxelMapPlanner(verbose_));  // verbose
    mp_planner_util_->setMapUtil(
        mp_map_util_);                  // Set collision checking function
    mp_planner_util_->setEpsilon(1.0);  // Set greedy param (default equal to 1)

    mp_planner_util_->setVxy(v_max);  // Set max velocity along x and y
    mp_planner_util_->setVz(vz_max);  // Set max velocity along z

    mp_planner_util_->setAmax(a_max);  // Set max acceleration
    mp_planner_util_->setJmax(j_max);  // Set max jerk
    mp_planner_util_->setVfov(v_fov);  // Set vertical semi-fov
    // mp_planner_util_->setUmax(u_max); // Set max control input
    mp_planner_util_->setDt(dt);          // Set dt for each primitive
    mp_planner_util_->setTmax(ndt * dt);  // Set max time horizon of planning
    mp_planner_util_->setMaxNum(
        max_num);  // Set maximum allowed expansion, -1 means no limitation
    mp_planner_util_->setU(U);  // 2D discretization if false, 3D if true
    mp_planner_util_->setW(W);  // 2D discretization if false, 3D if true
    mp_planner_util_->setLPAstar(false);  // Use Astar
  }



  // Register goal and preempt callbacks
  local_as_->registerGoalCallback(boost::bind(&LocalPlanServer::goalCB, this));
  local_as_->start();


}

void LocalPlanServer::process_all() {
  boost::mutex::scoped_lock lockm(map_mtx);

  if (goal_ == NULL) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("[LocalPlanServer:] Goal is null!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  } else if (local_map_ptr_ == nullptr) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("[LocalPlanServer:] local map is not received!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  } else {
    process_goal();
  }
}

void LocalPlanServer::process_result(const planning_ros_msgs::SplineTrajectory& traj_msg,
                                     bool solved) {
  result_ = boost::make_shared<planning_ros_msgs::PlanLocalPathResult>();
  result_->success = solved;  // set success status
  result_->policy_status = solved ? 1 : -1;
  ROS_WARN_STREAM("result_->success:" << result_->success);
  if (solved) {
    // record trajectory in result
    result_->traj = traj_msg;
    result_->traj.header.frame_id = frame_id_;
    traj_start_rostime_ = ros::Time::now();
    traj_start_time_= 0.0;
    traj_opt::TrajRosBridge::publish_msg(result_->traj);
    ROS_WARN_STREAM("publish a new trajectory");
     
    // execution_time (set in replanner)
    // equals 1.0/local_replan_rate
    double endt = goal_->execution_time.toSec();
    // ROS_WARN_STREAM("endt:" << endt);
    // evaluate trajectory for 5 steps, each step duration equals
    // execution_time, get corresponding waypoints and record in result
    // (result_->p_stop etc.) (evaluate the whole traj if execution_time is not
    // set (i.e. not in replan mode))
    int num_goals = 5;
    if (endt <= 0) {
      endt = traj_total_time_ - traj_start_time_;
      num_goals = 1;
    }
    /***@yuwei: optimization based planner ***/
    if (use_opt_planner_){
      if (use_gp_path_)
      {
        update_result(gp_traj3d_, num_goals, endt);
      }else
      {
        update_result(local_data_.traj_, num_goals, endt);
      }
    }else{
      update_result(mp_traj_, num_goals, endt);
    }
    traj_start_time_ += endt *  num_goals ;
    result_->execution_time =
        goal_->execution_time;  // execution_time (set in replanner)
                                // equals 1.0/local_replan_rate

    result_->epoch = goal_->epoch;
    result_->traj_end.orientation.w = 1.0;
    result_->traj_end.orientation.z = 0;

  }
  
  // reset goal
  goal_ = boost::shared_ptr<planning_ros_msgs::PlanLocalPathGoal>();
  // abort if trajectory generation failed
  if (!solved && local_as_->isActive()) {
    ROS_WARN("Current local plan trail: trajectory generation failed!");
    local_as_->setAborted();
  }

  // if (!traj_msg.data.size() > 0 && local_as_->isActive())
  // {
  //   ROS_INFO("skip this planning");
  //   local_as_->setAborted();
  // }

  if (local_as_->isActive()) {
    local_as_->setSucceeded(*result_);
  }
}


void LocalPlanServer::update_result( MPL::Trajectory3D &traj, int num_goals, double endt)
{

  for (int i = 0; i < num_goals; i++) {
    geometry_msgs::Pose p_fin;
    geometry_msgs::Twist v_fin, a_fin, j_fin;

    MPL::Waypoint3D pt_f = traj.evaluate(traj_start_time_ + endt * double(i + 1));
    // check if evaluation is successful, if not, set result->success to be
    // false! (if failure case, a null Waypoint is returned)
    if ((pt_f.pos(0) == 0) && (pt_f.pos(1) == 0) && (pt_f.pos(2) == 0) &&
        (pt_f.vel(0) == 0) && (pt_f.vel(1) == 0) && (pt_f.vel(2) == 0)) {
      result_->success = 0;
      ROS_WARN_STREAM(
          "waypoint evaluation failed, set result->success to be false");
      ROS_WARN_STREAM("trajectory total time:" << traj_total_time_);
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

  MPL::Waypoint3D pt = traj.evaluate(traj_total_time_);
  result_->traj_end.position.x = pt.pos(0);
  result_->traj_end.position.y = pt.pos(1);
  result_->traj_end.position.z = pt.pos(2);

}


void LocalPlanServer::update_result(Trajectory<5> &traj, int num_goals, double endt)
{
  Eigen::Vector3d pos, vel, acc;
  pos.setZero(), vel.setZero(), acc.setZero();

  for (int i = 0; i < num_goals; i++) {
    geometry_msgs::Pose p_fin;
    geometry_msgs::Twist v_fin, a_fin, j_fin;

    double t_cur = traj_start_time_ + endt * double(i + 1);

    pos = traj.getPos(t_cur);
    vel = traj.getVel(t_cur);
    acc = traj.getAcc(t_cur);

    // std::cout << "pos is " << pos.transpose() << std::endl;
    // std::cout << "t_cur is " << t_cur << std::endl;

    // check if evaluation is successful, if not, set result->success to be
    // false! (if failure case, a null Waypoint is returned)
    if ( pos == Eigen::Vector3d::Zero() && vel == Eigen::Vector3d::Zero() ) {
      result_->success = 0;
      ROS_WARN_STREAM(
          "waypoint evaluation failed, set result->success to be false");
      ROS_WARN_STREAM("trajectory total time:" << traj_total_time_);
      ROS_WARN_STREAM("evaluating at:" << endt * double(i + 1));
    }

    p_fin.position.x = pos(0), p_fin.position.y = pos(1),
    p_fin.position.z = pos(2);
    p_fin.orientation.w = 1, p_fin.orientation.z = 0;
    v_fin.linear.x = vel(0), v_fin.linear.y = vel(1),
    v_fin.linear.z = vel(2);
    v_fin.angular.z = 0;
    a_fin.linear.x = acc(0), a_fin.linear.y = acc(1),
    a_fin.linear.z = acc(2);
    a_fin.angular.z = 0;
    result_->p_stop.push_back(p_fin);
    result_->v_stop.push_back(v_fin);
    result_->a_stop.push_back(a_fin);
    result_->j_stop.push_back(j_fin);
  }

  pos = traj.getPos(traj_total_time_);
  result_->traj_end.position.x = pos(0);
  result_->traj_end.position.y = pos(1);
  result_->traj_end.position.z = pos(2);


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
  MPL::Waypoint3D start, goal;
  // instead of using current odometry as start, we use the given start position
  // for consistency between old and new trajectories in replan process
  start.pos = pose_to_eigen(goal_->p_init);
  start.vel = twist_to_eigen(goal_->v_init);
  start.acc = twist_to_eigen(goal_->a_init);
  start.jrk = twist_to_eigen(goal_->j_init);

  // Important: define use position, velocity, acceleration or jerk as control
  // inputs, note that the lowest order "false" term will be used as control
  // input, see instructions here:
  // https://github.com/KumarRobotics/kr_autonomous_flight/tree/master/autonomy_core/map_plan/mpl#example-usage
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = true;
  start.use_jrk = false;
  // yaw planning or not (note that yaw alignment with velocity can be turned on
  // in trajectory_tracker)
  start.use_yaw = false;

  goal.pos = pose_to_eigen(goal_->p_final);
  goal.vel = twist_to_eigen(goal_->v_final);
  goal.acc = twist_to_eigen(goal_->a_final);
  goal.jrk = twist_to_eigen(goal_->j_final);
  goal.use_yaw = start.use_yaw;
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;

  planning_ros_msgs::VoxelMap local_map_cleared;
  local_map_cleared = clear_map_position(*local_map_ptr_, start.pos);

  if (pub_cleared_map_) {
    local_map_cleared_pub.publish(local_map_cleared);
    ROS_ERROR("Local map cleared published");
  }


  // planning_ros_msgs::SplineTrajectory temp_spline_msg;
  // std::cout << " (ros::Time::now() - traj_start_rostime_).toSec()  " << (ros::Time::now() - traj_start_rostime_).toSec()  << std::endl;
  // if(spline_msg_.data.size() > 0 && goal_->execution_time.toSec() + (ros::Time::now() - traj_start_rostime_).toSec() <= 0.6 * traj_total_time_)
  // {
  //   std::cout << " didn't reach the time.  skip this replanning  " << std::endl;
  //   process_result(spline_msg_, true);
  //   return;
  // }

  
  std::vector<Eigen::Vector3d> jps_local_path;
  
  for (unsigned int i = 0; i < goal_->path.waypoints.size(); i++) 
  {
    jps_local_path.push_back(Eigen::Vector3d(goal_->path.waypoints[i].x,  goal_->path.waypoints[i].y,  goal_->path.waypoints[i].z));
    
  }
  jps_local_path.at(0) = Eigen::Vector3d(start.pos(0), start.pos(1), start.pos(2));

  bool local_planner_succeeded;
  local_planner_succeeded = local_plan_process(start, goal, jps_local_path, local_map_cleared);

  if (!local_planner_succeeded) {
    // local plan fails
    aborted_ = true;
    if (local_as_->isActive()) {
      ROS_WARN("Current local plan trail failed!");
      local_as_->setAborted();
    }
  }

  // get the trajectory from local planner, and process result
  /***@yuwei: optimization based planner ***/

  if(use_opt_planner_){

    if(use_gp_path_)
    {
      auto states = local_data_.gp_traj_.getSupportingStates();
      auto dur = local_data_.gp_traj_.getResTime();
      std::cout << "[debugging] ==================================================" << std::endl;
      std::cout << "[debugging] the dur is " << dur << std::endl; 
      MPL::Waypoint3D cur_wp, pre_wp;
      vec_E<MPL::Primitive3D> gp_primitives;
      std::vector<planning_ros_msgs::TrajectoryCommand> points;
      planning_ros_msgs::TrajectoryCommand tc;
      if (states->size() > 0)
      {
        for (int i = 0; i < states->size(); i++)
        {
          MPL::Waypoint3D cur_wp(MPL::ACC);
          cur_wp.pos << states->at(i)(0), states->at(i)(1), states->at(i)(2);
          cur_wp.vel << states->at(i)(3), states->at(i)(4), states->at(i)(5);
          cur_wp.acc << states->at(i)(6), states->at(i)(7), states->at(i)(8);


          std::cout << "[debugging] cur_wp.pos is " << states->at(i)(0)
                                             << " " << states->at(i)(1)
                                             << " " << states->at(i)(2) <<  std::endl;

          std::cout << "[debugging] cur_wp.acc is " << states->at(i)(6)
                                             << " " << states->at(i)(7)
                                             << " " << states->at(i)(8) <<  std::endl;


          std::cout << "[debugging] ---------------" << std::endl;



          if (i > 0){
            MPL::Primitive3D mp(pre_wp, cur_wp, dur);
            gp_primitives.push_back(mp);
          }
          pre_wp = cur_wp;
         
          tc.position.x = states->at(i)(0);
          tc.position.y = states->at(i)(1);
          tc.position.z = states->at(i)(2);
          tc.velocity.x = states->at(i)(3);
          tc.velocity.y = states->at(i)(4);
          tc.velocity.z = states->at(i)(5);
          tc.acceleration.x = states->at(i)(6), 
          tc.acceleration.y = states->at(i)(7), 
          tc.acceleration.z = states->at(i)(8);
          points.push_back(tc);
        }
      }

      gp_traj3d_ = MPL::Trajectory3D(gp_primitives);

      // covert traj to a ros  message
      planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(gp_traj3d_);
      traj_msg.header.frame_id = frame_id_;
      //traj_pub.publish(traj_msg); // only for visualizations
      planning_ros_msgs::TrajectoryPoints tp;
      tp.points = points;
      tp.header = traj_msg.header;
      traj_points_pub.publish(tp);
      spline_msg_ = traj_opt::SplineTrajectoryFromTrajectory(traj_msg);
    }
    else{


      spline_msg_.data.clear();
      spline_msg_.dimensions = 3;


      int piece_num =  local_data_.traj_.getPieceNum();
      Eigen::VectorXd durs = local_data_.traj_.getDurations();

      for (uint d = 0; d < 3; d++) {
        planning_ros_msgs::Spline spline;
        for (uint s = 0; s < piece_num; s++) {

          planning_ros_msgs::Polynomial poly;

          CoefficientMat coeff = local_data_.traj_[s].normalizePosCoeffMat();

          for (uint c = 0; c < 6; c++) {
            poly.coeffs.push_back(coeff(d,5-c));
          }
          poly.dt = durs[s];
          poly.degree = 5;
          spline.segs.push_back(poly);
        }
        spline.segments = piece_num;
        spline.t_total = traj_total_time_;
        spline_msg_.data.push_back(spline);
      }

    }

    spline_msg_.header.frame_id = frame_id_;


  }else{

    // covert traj to a ros message
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(mp_traj_);
    traj_msg.header.frame_id = frame_id_;
    traj_pub.publish(traj_msg); // only for visualizations

    spline_msg_ = traj_opt::SplineTrajectoryFromTrajectory(traj_msg);
  }
  process_result(spline_msg_, local_planner_succeeded);

}

planning_ros_msgs::VoxelMap LocalPlanServer::clear_map_position(
    const planning_ros_msgs::VoxelMap& local_map_original, const Vec3f& start) {
  // make a copy, not the most efficient way, but necessary because we need to
  // maintain both original and cleared maps
  planning_ros_msgs::VoxelMap local_map_cleared;
  local_map_cleared = local_map_original;

  planning_ros_msgs::VoxelMap voxel_map;
  
  // Replaced with corresponding parameter value from VoxelMsg.msg
  int8_t val_free = voxel_map.val_free;
  ROS_WARN_ONCE("Value free is set as %d", val_free);
  double robot_r = 0.5;
  int robot_r_n = std::ceil(robot_r / local_map_cleared.resolution);

  vec_Vec3i clear_ns;
  for (int nx = -robot_r_n; nx <= robot_r_n; nx++) {
    for (int ny = -robot_r_n; ny <= robot_r_n; ny++) {
      for (int nz = -robot_r_n; nz <= robot_r_n; nz++) {
        clear_ns.push_back(Eigen::Vector3i(nx, ny, nz));
      }
    }
  }

  auto origin_x = local_map_cleared.origin.x;
  auto origin_y = local_map_cleared.origin.y;
  auto origin_z = local_map_cleared.origin.z;
  Eigen::Vector3i dim = Eigen::Vector3i::Zero();
  dim(0) = local_map_cleared.dim.x;
  dim(1) = local_map_cleared.dim.y;
  dim(2) = local_map_cleared.dim.z;
  auto res = local_map_cleared.resolution;
  const Eigen::Vector3i pn =
      Eigen::Vector3i(std::round((start(0) - origin_x) / res),
                      std::round((start(1) - origin_y) / res),
                      std::round((start(2) - origin_z) / res));

  for (const auto& n : clear_ns) {
    Eigen::Vector3i pnn = pn + n;
    int idx_tmp = pnn(0) + pnn(1) * dim(0) + pnn(2) * dim(0) * dim(1);
    if (!is_outside_map(pnn, dim) &&
        local_map_cleared.data[idx_tmp] != val_free) {
      local_map_cleared.data[idx_tmp] = val_free;
      // ROS_ERROR("clearing!!! idx %d", idx_tmp);
    }
  }
  return local_map_cleared;
}

bool LocalPlanServer::is_outside_map(const Eigen::Vector3i& pn,
                                     const Eigen::Vector3i& dim) {
  return pn(0) < 0 || pn(0) >= dim(0) || pn(1) < 0 || pn(1) >= dim(1) ||
         pn(2) < 0 || pn(2) >= dim(2);
}

bool LocalPlanServer::local_plan_process(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    std::vector<Eigen::Vector3d> local_path,
    const planning_ros_msgs::VoxelMap& map) {
  // for visualization: publish a path connecting local start and local goal
  vec_Vec3f sg;
  sg.push_back(start.pos);
  sg.push_back(goal.pos);
  planning_ros_msgs::Path sg_msg = path_to_ros(sg);
  std::string map_frame;  // set frame id
  map_frame = map.header.frame_id;
  sg_msg.header.frame_id = map_frame;
  sg_pub.publish(sg_msg);

  /// Trajectory planning

  //1. update map
  ROS_WARN("[LocalPlanServer] 1. Set up occupancy map and semantic map!");
  setMap(mp_map_util_, map);
  setSemanticMap();

  bool valid = false;

  /***@yuwei: optimization based planner ***/
  ROS_WARN("[LocalPlanServer] 2. Trigger planner ......");
  if(use_opt_planner_){

    Eigen::MatrixXd startState(3, 3), endState(3, 3);
    startState <<   start.pos(0),  start.vel(0), start.acc(0),
                    start.pos(1),  start.vel(1), start.acc(1),
                    start.pos(2),  start.vel(2), start.acc(2);
    endState <<   goal.pos(0),  goal.vel(0), goal.acc(0),
                  goal.pos(1),  goal.vel(1), goal.acc(1),
                  goal.pos(2),  goal.vel(2), goal.acc(2);

    std::cout << "startState is " << startState << std::endl;
    std::cout << "endState is " << endState << std::endl;
    std::cout << "local_path.size() is " << local_path.size() << std::endl;
        
    valid = local_plan_manager_->localPlan(startState, endState, local_path);
    if (valid) {
      local_data_ = local_plan_manager_->local_data_;
      traj_total_time_ = local_data_.duration_;
      ROS_WARN("[LocalPlanServer] planning success ! !!!!!");
    }

  }else{

    std::cout << "start.pos is " << start.pos.transpose() << std::endl;
    std::cout << "goal.pos is "  << goal.pos.transpose()<< std::endl;
    
    mp_planner_util_->reset();
    // start and new_goal contain full informaiton about
    // position/velocity/acceleration
    if(!use_3d_local_) 
    {
      MPL::Waypoint3D new_goal = goal;
      new_goal.pos(2) = start.pos(2); // or else it cannot reach the local goal position.
      valid = mp_planner_util_->plan(start, new_goal);
    }else{
      valid = mp_planner_util_->plan(start, goal);
    }

    if (valid) {
      mp_traj_ = mp_planner_util_->getTraj();
      traj_total_time_ = mp_traj_.getTotalTime();
      ROS_WARN("[LocalPlanServer] planning success ! !!!!!");
    }

    // for visualization: publish expanded nodes as a point cloud
    sensor_msgs::PointCloud expanded_ps =
        vec_to_cloud(mp_planner_util_->getExpandedNodes());
    expanded_ps.header.frame_id = map_frame;
    expanded_cloud_pub.publish(expanded_ps);

  }

  return valid;
}

// prevent concurrent planner modes
void LocalPlanServer::goalCB() {
  auto start_timer = std::chrono::high_resolution_clock::now();
  goal_ = local_as_->acceptNewGoal();
  // check_vel is true if local planner reaches global goal

  if(!use_opt_planner_){
    if (goal_->check_vel) {
      // Tolerance for goal region, -1 means no limitation
      mp_planner_util_->setTol(tol_pos_, goal_tol_vel_, goal_tol_acc_);
    } else {
      mp_planner_util_->setTol(tol_pos_, -1, -1);
    }
  }
  aborted_ = false;
  process_all();
  auto end_timer = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_timer - start_timer);
  std::cout << "[Local goalCB] took "<< (double)duration.count() /1000.0 << " ms"<< std::endl;

  total_time_ += (double)duration.count();
  total_call_ += 1;
  std::cout << "[Local goalCB] everage time is"<< total_time_ / (1000.0 * total_call_) << " ms"<< std::endl;

}

int main(int argc, char** argv) {
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
