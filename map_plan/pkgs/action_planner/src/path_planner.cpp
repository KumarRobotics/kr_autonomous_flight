#include <action_planner/PlanPathAction.h>
#include <actionlib/server/simple_action_server.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>

#include "path_planner_util.hpp"

ros::Publisher time_pub;
/// traj pub
ros::Publisher path_array_pub_;
/// distance map cloud pub
ros::Publisher distance_cloud_pub_;
/// map util
std::shared_ptr<PathPlannerUtil<3>> planner_util_3d_;
/// Voxel map
planning_ros_msgs::VoxelMap map_;
/// action lib
std::unique_ptr<actionlib::SimpleActionServer<action_planner::PlanPathAction>>
    as_;
/// profiling
bool verbose_;

bool map_initialized_ = false;

vec_Vec3f original_goals_;
std::vector<int> original_goals_reached_;

bool equal(const vec_Vec3f &gs1, const vec_Vec3f &gs2) {
  if (gs1.size() != gs2.size()) return false;
  for (size_t i = 0; i < gs1.size(); ++i)
    if (gs1[i] != gs2[i]) return false;
  return true;
}

void mapCB(const planning_ros_msgs::VoxelMap::ConstPtr &msg) {
  if (verbose_) ROS_WARN_ONCE("[PathPlanner]: Get the voxel map!");

  map_ = *msg;
  map_initialized_ = true;
}

void process(const Vec3f &start, const vec_Vec3f &goals) {
  // start can be seen as the current pose
  bool solved = true;
  planning_ros_msgs::Path path_msg;
  planning_ros_msgs::PathArray path_array_msg;

  if (!map_initialized_) {
    path_msg = path_to_ros(goals);
    ROS_WARN(
        "[PathPlanner]: Map is not initialized! Directly connect start and "
        "goal...");
  } else {
    // Vec3f goal = goals.back();
    Vec3f goal;

    for (size_t i = 0; i < goals.size(); ++i) {
      // Find a goal that is not reached
      if (!original_goals_reached_[i]) {
        // Check if start is close enough to this goal
        const auto dist = (start - goals[i]).norm();

        // If it is close enough, then mark this goal as reached and look for
        // the next one
        if (dist < 1.0) {
          original_goals_reached_[i] = 1;
          ROS_INFO("Start close to Goal %zu, mark as reached", i);
          continue;
        } else {
          ROS_INFO("distance to current goal is %f", dist);
        }

        ROS_INFO("Current goal is %zu", i);
        goal = goals[i];
        break;
      }
    }

    planner_util_3d_->setMap(map_);
    if (planner_util_3d_->plan(start, goal)) {
      // check path_planner_util.hpp for the fix of the start point of the path:
      // use the actual start (i.e. robot pose upon calling the planner) instead
      // of the center of the voxel cloest to the start, much smoother motion.
      // return distance map planner's path, containing x,y,z coordinates of
      // turning points along the path check path_planner_util.hpp (search
      // "path_ = dist_path"), distance_map_planner.cpp and jps_planner.cpp for
      // detailed implementation
      path_msg = path_to_ros(planner_util_3d_->getPath());
      path_array_msg = path_array_to_ros(planner_util_3d_->getPathArray());
    } else {
      solved = false;
    }

    if (distance_cloud_pub_.getNumSubscribers() > 0) {
      vec_Vec3f dist_cloud;
      dist_cloud = planner_util_3d_->getSearchRegion();

      sensor_msgs::PointCloud cloud_msg = vec_to_cloud(dist_cloud);
      cloud_msg.header.frame_id = map_.header.frame_id;
      distance_cloud_pub_.publish(cloud_msg);
    }
  }

  if (original_goals_reached_.back()) solved = false;

  if (as_->isActive()) {
    if (solved) {
      action_planner::PlanPathResult result;
      result.success = solved;
      result.path = path_msg;
      as_->setSucceeded(result);
    } else {
      if (verbose_) {
        ROS_WARN("[PathPlanner]+++++++++++++++++++++++++");
        ROS_WARN("Danger!!!!!");
        ROS_WARN("Abort!!!!!!");
        ROS_WARN("Danger!!!!!");
        ROS_WARN("Abort!!!!!!");
        ROS_WARN("+++++++++++++++++++++++++");
      }
      as_->setAborted();
    }
  }

  if (path_array_pub_.getNumSubscribers() > 0) {
    path_array_msg.header.frame_id = map_.header.frame_id;
    path_array_pub_.publish(path_array_msg);
  }
}

// prevent concurrent planner modes
void goalCB() {
  auto action_goal = as_->acceptNewGoal();
  const Vec3f start = pose_to_eigen(action_goal->p_init);
  vec_Vec3f goals;
  for (const auto &it : action_goal->p_finals)
    goals.push_back(pose_to_eigen(it));

  if (goals.empty()) {
    ROS_WARN("[PathPlanner]: Receive empty goals! use single goal!");
    goals.push_back(pose_to_eigen(action_goal->p_final));
  }

  if (!equal(goals, original_goals_)) {
    original_goals_ = goals;
    original_goals_reached_ = std::vector<int>(goals.size(), 0);
  }

  const auto t0 = ros::Time::now();
  process(start, goals);
  const auto dt = (ros::Time::now() - t0).toSec();
  sensor_msgs::Temperature tmsg;
  tmsg.header.stamp = t0;
  tmsg.temperature = dt * 1000;
  time_pub.publish(tmsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_planner");

  ros::NodeHandle nh("~");

  path_array_pub_ =
      nh.advertise<planning_ros_msgs::PathArray>("path_array", 1, true);
  distance_cloud_pub_ =
      nh.advertise<sensor_msgs::PointCloud>("distance_cloud", 1, true);

  time_pub = nh.advertise<sensor_msgs::Temperature>("/timing/planner", 1);

  ros::Subscriber map_sub = nh.subscribe("voxel_map", 2, mapCB);

  nh.param("verbose", verbose_, false);

  planner_util_3d_ = std::make_shared<PathPlannerUtil<3>>(verbose_);

  as_ = std::make_unique<
      actionlib::SimpleActionServer<action_planner::PlanPathAction>>(
      nh, "plan_path", false);
  // Register goal and preempt callbacks
  as_->registerGoalCallback(goalCB);
  as_->start();

  ros::spin();

  return 0;
}
