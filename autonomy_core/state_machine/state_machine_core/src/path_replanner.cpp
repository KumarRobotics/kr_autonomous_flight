#include <ros/ros.h>
// external actions
#include <action_planner/PlanPathAction.h>
#include <action_trackers/RunPathAction.h>
// internal actions
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <state_machine/ReplanAction.h>

#include <cmath>
using actionlib::SimpleActionClient;
using actionlib::SimpleActionServer;
using kr_mav_msgs::PositionCommand;
class PathRePlanner {
 public:
  PathRePlanner();
  // private:
  std::unique_ptr<SimpleActionServer<state_machine::ReplanAction>>
      replan_server_;
  // std::unique_ptr<actionlib::SimpleActionServer<state_machine::UpdateGoalAction>
  // > update_server_;
  std::unique_ptr<SimpleActionClient<action_trackers::RunPathAction>>
      run_client_;
  std::unique_ptr<SimpleActionClient<action_planner::PlanPathAction>>
      plan_client_;

  ros::NodeHandle nh_;
  bool finished_replanning{true};
  int max_horizon_{6};
  geometry_msgs::Pose last_pose_goal_;
  std::vector<geometry_msgs::Pose> last_pose_goals_;

  boost::optional<PositionCommand> current_cmd_;
  ros::Subscriber cmd_sub_;

  void cmd_cb(const PositionCommand& cmd) { current_cmd_ = cmd; }

  bool is_equal(const geometry_msgs::Pose& p1,
                const geometry_msgs::Pose& p2,
                double thr = 0) {
    return std::abs(p1.position.x - p2.position.x) < thr &&
           std::abs(p1.position.y - p2.position.y) < thr &&
           std::abs(p1.position.z - p2.position.z) < thr;
  }

  void replan_goal() {
    auto goal = replan_server_->acceptNewGoal();
    plan(goal->p_final, goal->p_finals);
  }

  void plan(const geometry_msgs::Pose& pose_goal,
            const std::vector<geometry_msgs::Pose>& pose_goals) {
    if (!is_equal(pose_goal, last_pose_goal_)) finished_replanning = false;

    ROS_INFO("Number of pose_goals: %zu", pose_goals.size());

    if (!finished_replanning && current_cmd_) {
      // this is current pose?
      geometry_msgs::Pose pose_start;
      pose_start.position.x = current_cmd_->position.x;
      pose_start.position.y = current_cmd_->position.y;
      pose_start.position.z = current_cmd_->position.z;

      kr_planning_msgs::Path path;
      // planning from current pose to goal
      if (plan_path(pose_start, pose_goal, pose_goals, &path)) {
        action_trackers::RunPathGoal rungoal;
        rungoal.path = path;
        run_client_->sendGoal(rungoal);
      }
    }
    last_pose_goal_ = pose_goal;
    last_pose_goals_ = pose_goals;
  }

  bool plan_path(const geometry_msgs::Pose& pose_start,
                 const geometry_msgs::Pose& pose_goal,
                 const std::vector<geometry_msgs::Pose>& pose_goals,
                 kr_planning_msgs::Path* path) {
    action_planner::PlanPathGoal tpgoal;
    tpgoal.p_init = pose_start;
    tpgoal.p_final = pose_goal;
    tpgoal.p_finals = pose_goals;
    plan_client_->sendGoal(tpgoal);

    bool finished_before_timeout =
        plan_client_->waitForResult(ros::Duration(1.5));

    if (!finished_before_timeout) {
      ROS_ERROR("[PathReplanner]: Planner timed out");
      state_machine::ReplanResult critical;
      critical.status = state_machine::ReplanResult::CRITICAL_ERROR;
      if (replan_server_->isActive()) replan_server_->setSucceeded(critical);
      return false;
    }

    auto result = plan_client_->getResult();

    if (result->success) {
      path = result->path;
      static int plan_epoch = 0;
      if (path.waypoints.size() == 1) {
        ROS_WARN_STREAM(
            "[PathReplanner]: Got a plan with stop: " << plan_epoch);
        ++plan_epoch;
      }

      if (plan_epoch >= max_horizon_) {
        ROS_WARN(
            "[PathReplanner]: Terminate replanning after time out, epcoh %d >= "
            "max horizon %d",
            plan_epoch,
            max_horizon_);
        finished_replanning = true;
        state_machine::ReplanResult success;
        success.status = state_machine::ReplanResult::SUCCESS;
        if (replan_server_->isActive())
          replan_server_->setSucceeded(success);
        else
          ROS_ERROR("Fail to set success!");
        plan_epoch = 0;
      } else {
        state_machine::ReplanResult in_progress;
        in_progress.status = state_machine::ReplanResult::IN_PROGRESS;

        if (replan_server_->isActive())
          replan_server_->setSucceeded(in_progress);
      }
      return true;
    } else {
      ROS_WARN("[PathReplanner]: Abort full mission manually...");
      state_machine::ReplanResult abort;
      abort.status = state_machine::ReplanResult::ABORT_FULL_MISSION;
      if (replan_server_->isActive()) replan_server_->setSucceeded(abort);
      return false;
    }
  }
};

PathRePlanner::PathRePlanner() : nh_("~") {
  nh_.param("max_horizon", max_horizon_, 6);

  replan_server_.reset(
      new actionlib::SimpleActionServer<state_machine::ReplanAction>(
          nh_, "replan", false));

  run_client_.reset(
      new actionlib::SimpleActionClient<action_trackers::RunPathAction>(
          nh_, "execute_path", true));

  plan_client_.reset(
      new actionlib::SimpleActionClient<action_planner::PlanPathAction>(
          nh_, "plan_path", true));

  cmd_sub_ = nh_.subscribe("position_cmd", 1, &PathRePlanner::cmd_cb, this);

  replan_server_->registerGoalCallback(
      boost::bind(&PathRePlanner::replan_goal, this));

  replan_server_->start();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "replanner");
  ros::NodeHandle nh;

  PathRePlanner replanner;

  ros::spin();

  return 0;
}
