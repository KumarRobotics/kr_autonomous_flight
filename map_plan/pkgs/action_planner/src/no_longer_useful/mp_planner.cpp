#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

#include <boost/range/irange.hpp>
// #include <boost/thread/thread.hpp>
// #include <boost/thread/mutex.hpp>
// #include <boost/shared_ptr.hpp>
#include <action_planner/ActionPlannerConfig.h>
#include <action_planner/PlanTwoPointAction.h>
#include <action_planner/PlanWaypointsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <traj_opt_ros/ros_bridge.h>
#include <trajectory_generator_mp/server.h>
//#include "path_utils.hpp"
#include <primitive_to_traj_opt/convert.h>

using boost::irange;

// Warpper for motion primitive planner
class MPPlanner {
private:
  // local global map sub
  ros::Subscriber local_map_sub_;
  ros::Subscriber global_map_sub_;

  // traj pub
  ros::Publisher traj_pub;
  // mutexes
  boost::mutex map_mtx, goal_mtx, action_mtx, traj_mtx;
  // traj server
  std::unique_ptr<MP::Server> server_;

  // current local and global maps
  planning_ros_msgs::VoxelMap local_map_;
  planning_ros_msgs::VoxelMap global_map_;

  bool local_planner_succeeded_ = false;
  bool global_planner_succeeded_ = false;
  bool global_map_updated_ = false;
  bool global_plan_exist_ = false;

  // actionlib
  boost::shared_ptr<const action_planner::PlanTwoPointGoal> goal_;
  boost::shared_ptr<action_planner::PlanTwoPointResult> result_;
  // action lib
  std::unique_ptr<
      actionlib::SimpleActionServer<action_planner::PlanTwoPointAction>>
      as_;

  // methods
  void goalCB();
  void process_goal();
  void process_result(const Trajectory3D &traj, bool solved);
  void localMapCB(const planning_ros_msgs::VoxelMap::ConstPtr &msg);
  void globalMapCB(const planning_ros_msgs::VoxelMap::ConstPtr &msg);

public:
  void initialize(ros::NodeHandle &nh);
  void process_all();
  bool aborted_;
};

// map callback, update local_map_
void MPPlanner::localMapCB(const planning_ros_msgs::VoxelMap::ConstPtr &msg) {
  ROS_WARN_ONCE("Get the local voxel map!");
  local_map_ = *msg;
}

// map callback, update global_map_
void MPPlanner::globalMapCB(const planning_ros_msgs::VoxelMap::ConstPtr &msg) {
  ROS_WARN_ONCE("Get the global voxel map!");
  ROS_WARN_ONCE("Get the global voxel map!");
  ROS_WARN_ONCE("Get the global voxel map!");
  ROS_WARN_ONCE("Get the global voxel map!");
  ROS_WARN_ONCE("Get the global voxel map!");
  ROS_WARN_ONCE("Get the global voxel map!");

  global_map_ = *msg;
  global_map_updated_ = true;
}

void MPPlanner::initialize(ros::NodeHandle &nh) {
  traj_pub = nh.advertise<planning_ros_msgs::Trajectory>("traj", 1, true);
  local_map_sub_ =
      nh.subscribe("local_voxel_map", 2, &MPPlanner::localMapCB, this);
  global_map_sub_ =
      nh.subscribe("global_voxel_map", 2, &MPPlanner::globalMapCB, this);

  // dynserver_callback_ = boost::bind(&MPPlanner::dyncallback,
  // _1,_2,this);
  //  dynserver_.setCallback(dynserver_callback_);
  as_ = std::make_unique<
      actionlib::SimpleActionServer<action_planner::PlanTwoPointAction>>(
      nh, "plan_trajectory", false);
  // Register goal and preempt callbacks
  as_->registerGoalCallback(boost::bind(&MPPlanner::goalCB, this));
  as_->start();

  server_.reset(new MP::Server());
  server_->init(nh);
}

void MPPlanner::process_all() {
  boost::mutex::scoped_lock lockm(map_mtx);

  if (goal_ == NULL)
    return;
  ros::Time t0 = ros::Time::now();
  // record goal position, specify use jrk, acc or vel
  process_goal();
  double dt = (ros::Time::now() - t0).toSec();

  if (dt > 0.3 && as_->isActive()) {
    ROS_WARN("[MPPlanner]+++++++++++++++++++++++++");
    ROS_WARN("Time out!!!!!! dt =  %f is too large!!!!!", dt);
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("Time out!!!!!! dt =  %f is too large!!!!!", dt);
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("Time out!!!!!! dt =  %f is too large!!!!!", dt);
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("+++++++++++++++++++++++++");
    as_->setAborted();
  }
  printf("Total time for TPP traj gen: %f\n", dt);
}

void MPPlanner::process_result(const Trajectory3D &traj, bool solved) {
  result_ = boost::make_shared<action_planner::PlanTwoPointResult>();
  result_->success = solved; // set success status
  result_->policy_status = solved ? 1 : -1;
  if (solved) {
    // covert traj to a ros message
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header.frame_id = global_map_.header.frame_id;
    traj_pub.publish(traj_msg);

    result_->traj = PrimitiveToTrajOpt::convert(traj_msg);
    result_->traj.header.frame_id = global_map_.header.frame_id;
    TrajRosBridge::publish_msg(result_->traj);

    decimal_t endt =
        goal_->execution_time
            .toSec(); // execution_time if set will equal 1.0/replan_rate

    // look ahead for 5 steps, each step duration equals execution_time, get
    // corresponding waypoints (execute the traj if execution_time is not set
    // (default is -1))
    int num_goals = 5;
    if (endt <= 0) {
      endt = traj.getTotalTime();
      num_goals = 1;
    }
    for (int i = 0; i < num_goals; i++) {
      geometry_msgs::Pose p_fin;
      geometry_msgs::Twist v_fin, a_fin, j_fin;

      Waypoint3D pt_f = traj.evaluate(endt * double(i + 1));
      // check if evaluation is successful, if not, set result->success to be
      // false! (if failure case, a null Waypoint is returned)
      if ((pt_f.pos(0) == 0) && (pt_f.pos(1) == 0) && (pt_f.pos(2) == 0) &&
          (pt_f.vel(0) == 0) && (pt_f.vel(1) == 0) && (pt_f.vel(2) == 0)) {
        result_->success = 0;
        ROS_WARN_STREAM(
            "waypoint evaluation failed, set result->success to be false");
        ROS_WARN_STREAM("trajectory total time:" << traj.total_t_);
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
    result_->execution_time =
        goal_->execution_time; // execution_time if set will
                               // equal 1.0/replan_rate
    result_->epoch = goal_->epoch;
    Waypoint3D pt = traj.evaluate(traj.getTotalTime());
    result_->traj_end.position.x = pt.pos(0);
    result_->traj_end.position.y = pt.pos(1);
    result_->traj_end.position.z = pt.pos(2);
    result_->traj_end.orientation.w = 1.0;
    result_->traj_end.orientation.z = 0;
  }

  goal_ = boost::shared_ptr<action_planner::PlanTwoPointGoal>();
  // abort if trajectory generation failed
  if (!solved && as_->isActive()) {
    ROS_WARN("[M]+++++++++++++++++++++++++");
    ROS_WARN("Danger!!!!!");
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("Danger!!!!!");
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("Danger!!!!!");
    ROS_WARN("Abort!!!!!!");
    ROS_WARN("+++++++++++++++++++++++++");
    // as_->setAborted();
  }

  if (as_->isActive())
    as_->setSucceeded(*result_);
}

// record goal position, specify use jrk, acc or vel
void MPPlanner::process_goal() {
  boost::mutex::scoped_lock lockt(traj_mtx);
  if (aborted_) {
    if (as_->isActive()) {
      ROS_WARN("process_goal: Abort!");
      as_->setAborted();
    }
    return;
  }
  // assmes goal is not null
  Waypoint3D start, goal;
  start.pos = pose_to_eigen(goal_->p_init);
  start.vel = twist_to_eigen(goal_->v_init);
  start.acc = twist_to_eigen(goal_->a_init);
  start.jrk = twist_to_eigen(goal_->j_init);
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = true;
  start.use_jrk = false;

  goal.pos = pose_to_eigen(goal_->p_final);
  goal.vel = twist_to_eigen(goal_->v_final);
  goal.acc = twist_to_eigen(goal_->a_final);
  goal.jrk = twist_to_eigen(goal_->j_final);
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;

  if (goal_->reset)
    ROS_WARN("RESET!");

  // slice the 3D map to get 2D map, plan jps path, crop the path, and plan mp
  // trajectory in a local region around the robot
  // TODO [local global framework]: see comments inside this server_->process()
  // function!
  if (global_map_updated_) {
    global_planner_succeeded_ =
        server_->global_plan_process(start, goal, global_map_);
    global_map_updated_ = false;
    ROS_WARN("++++ planning global path!");
    global_plan_exist_ = true;
  };

  if (global_plan_exist_) {
    if (global_planner_succeeded_) {
      local_planner_succeeded_ =
          server_->local_plan_process(start, goal, local_map_);
      ROS_WARN("++++ planning local traj!");
      if (!local_planner_succeeded_) {
        // local plan fails
        aborted_ = true;
        if (as_->isActive()) {
          ROS_WARN("+++++++++++++++++++++++++");
          ROS_WARN("Local Plan Fails!!!!!");
          ROS_WARN("Abort!!!!!!");
          ROS_WARN("+++++++++++++++++++++++++");
          as_->setAborted();
        }
      }
    } else {
      // global plan fails
      aborted_ = true;
      if (as_->isActive()) {
        ROS_WARN("+++++++++++++++++++++++++");
        ROS_WARN("Global Plan Fails!!!!!");
        ROS_WARN("Abort!!!!!!");
        ROS_WARN("+++++++++++++++++++++++++");
        as_->setAborted();
      }
    }

    bool valid = (global_planner_succeeded_ && local_planner_succeeded_);
    Trajectory3D traj =
        server_->get_traj(); // just return traj_ in server, which is returned
                             // from motion primitive planner
    process_result(traj, valid);
  } else {
    ROS_WARN("+++++++++++++++++++++++++");
    ROS_ERROR(
        "Global Plan Does NOT Exist Yet, Possibly Subscribing to Wrong Map "
        "Msg");
  }
}

// prevent concurrent planner modes
void MPPlanner::goalCB() {
  goal_ = as_->acceptNewGoal();
  aborted_ = false;
  process_all();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_planner");

  ros::NodeHandle nh("~");

  MPPlanner tpp;
  tpp.initialize(nh);

  ros::Rate r(20);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
    // tpp.process_all();
  }

  return 0;
}
