#include <ros/ros.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <mapper/data_conversions.h> // setMap, getMap, etc
#include <jps_collision/map_util.h> // jps related
#include <jps_planner/jps_planner/jps_planner.h> // jps related
#include <nav_msgs/Odometry.h> // odometry
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <boost/range/irange.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <traj_opt_ros/ros_bridge.h>
#include <action_planner/PlanTwoPointAction.h>
#include <action_planner/PlanWaypointsAction.h>
#include <action_planner/ActionPlannerConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <primitive_to_traj_opt/convert.h>

using boost::irange;

class GlobalPlanServer {
  public:
    /**
    * @brief Constructor, initialization
    */
    GlobalPlanServer(ros::NodeHandle& nh);

    /**
    * @brief Call process_goal function, check planner timeout
    */ 
    void process_all();

    /**
     * @brief Odom callback function, directly subscribing odom to get the path start position
     */
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);

    // global path recorder
    planning_ros_msgs::Path global_path_msg_;

    bool aborted_;

  private:
    // global map sub
    ros::Subscriber global_map_sub_;
    ros::Subscriber odom_sub_;

    // pub
    ros::Publisher path_pub_;
    
    // mutexes
    boost::mutex map_mtx; 
    // current global map
    planning_ros_msgs::VoxelMap global_map_;

    bool global_planner_succeeded_ {false};
    bool global_plan_exist_ {false};

    // actionlib
    boost::shared_ptr<const action_planner::PlanTwoPointGoal> goal_;
    boost::shared_ptr<action_planner::PlanTwoPointResult> result_;
    // action lib
    std::unique_ptr<actionlib::SimpleActionServer<action_planner::PlanTwoPointAction>> global_as_;

    // planner related
    std::shared_ptr<JPS::VoxelMapUtil> jps_map_util_;
    std::shared_ptr<JPSPlanner3D> jps_util_; 

    // odom related
    bool odom_set_ {false};
    nav_msgs::Odometry::ConstPtr odom_msg_;

    // methods  
    /**
    * @brief Goal callback function, prevent concurrent planner modes, call process_all function
    */
    void goalCB();

    /**
    * @brief Call global planner after setting planner start and goal and specify params (use jrk, acc or vel)
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
    bool global_plan_process(const Waypoint3D& start, const Waypoint3D& goal, const planning_ros_msgs::VoxelMap& global_map);


};


void GlobalPlanServer::globalMapCB(const planning_ros_msgs::VoxelMap::ConstPtr& msg) {
  global_map_ = *msg;
}


GlobalPlanServer::GlobalPlanServer(ros::NodeHandle& nh) {
  path_pub_ = nh.advertise<planning_ros_msgs::Path>("path", 1, true);
  global_map_sub_ = nh.subscribe("global_voxel_map", 2, &GlobalPlanServer::globalMapCB, this);

  global_as_ = std::make_unique<
      actionlib::SimpleActionServer<action_planner::PlanTwoPointAction>>(
      nh, "plan_global_path", false);
  // Register goal and preempt callbacks
  global_as_->registerGoalCallback(boost::bind(&GlobalPlanServer::goalCB, this));
  global_as_->start();

  // odom callback 
  odom_sub_ = nh.subscribe("odom", 10, &GlobalPlanServer::odom_callback,
                              this, ros::TransportHints().tcpNoDelay());
  
  // Set map util for jps
  jps_map_util_ = std::make_shared<JPS::VoxelMapUtil>();
  jps_util_ = std::make_shared<JPSPlanner3D>(false); // verbose set as false
  jps_util_->setMapUtil(jps_map_util_);

}


void GlobalPlanServer::process_all() {
  boost::mutex::scoped_lock lockm(map_mtx);

  if (goal_ == NULL) 
    return;
  ros::Time t0 = ros::Time::now();
  process_goal();
  double dt = (ros::Time::now() - t0).toSec();

  // check timeout
  if(dt > 0.3 && global_as_->isActive())
  {
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
  result_->success = solved; // set success status
  result_->policy_status = solved ? 1 : -1;  
  result_->path = global_path_msg_;
  if (!solved && global_as_->isActive()){
    ROS_WARN("+++++++++++++++++++++++++");
    ROS_WARN("Global planner: path planner failed!");
    ROS_WARN("Danger!!!!!");
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("+++++++++++++++++++++++++");
    //global_as_->setAborted();
  }

  // reset goal
  goal_ = boost::shared_ptr<action_planner::PlanTwoPointGoal>();
  if (global_as_->isActive())
    global_as_->setSucceeded(*result_);
}

void GlobalPlanServer::odom_callback(
    const nav_msgs::Odometry::ConstPtr &odom) {
  if (!odom_set_) {odom_set_ = true;}
  odom_msg_ = odom;
}

void GlobalPlanServer::process_goal() {


  if(aborted_)
  {
	  if(global_as_->isActive()) {
		  ROS_WARN("process_goal: Abort!");
      global_as_->setAborted();
    }
    return;
  }

  // set start and goal, assume goal is not null
  Waypoint3D start, goal;

  // first, check if odom is received
  if (!odom_set_){
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


bool GlobalPlanServer::global_plan_process(const Waypoint3D& start, const Waypoint3D& goal, const planning_ros_msgs::VoxelMap& global_map) {
    std::string map_frame;
  map_frame = global_map.header.frame_id;
  setMap(jps_map_util_, global_map);
  jps_util_->updateMap();

  // Path planning using JPS
  if(!jps_util_->plan(start.pos, goal.pos, 1.0, true)) { 
    // jps_util_->plan params: start, goal, eps, use_jps
    ROS_WARN("Fail to plan a global path!");
  }
  else{
    vec_Vec3f global_path = jps_util_->getPath();
    // publish 
    global_path_msg_ = path_to_ros(global_path);
    global_path_msg_.header.frame_id = map_frame;
    path_pub_.publish(global_path_msg_);
    } 
    return true;
}





int main(int argc, char **argv) {
  ros::init(argc, argv, "action_planner");

  ros::NodeHandle nh("~");

  GlobalPlanServer tpp(nh);

  ros::Rate r(20);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
    //tpp.process_all();
  }

  return 0;
}
