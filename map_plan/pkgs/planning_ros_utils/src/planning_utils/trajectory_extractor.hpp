#pragma once

#include <planning_ros_msgs/TrajectoryCommand.h>
#include <planning_ros_utils/primitive_ros_utils.h>

class TrajectoryExtractor {
 public:
  TrajectoryExtractor(const planning_ros_msgs::Trajectory &msg, double dt) {
    const auto traj = toTrajectory3D(msg);
    int N = std::ceil(traj.getTotalTime() / dt);
    const auto ws = traj.sample(N);

    const auto t0 = ros::Time::now();
    cmd_array_.resize(ws.size());
    for (unsigned int i = 0; i < ws.size(); i++) {
      cmd_array_[i].header.stamp = t0 + ros::Duration(ws[i].t);
      cmd_array_[i].position.x = ws[i].pos(0);
      cmd_array_[i].position.y = ws[i].pos(1);
      cmd_array_[i].position.z = ws[i].pos(2);
      cmd_array_[i].velocity.x = ws[i].vel(0);
      cmd_array_[i].velocity.y = ws[i].vel(1);
      cmd_array_[i].velocity.z = ws[i].vel(2);
      cmd_array_[i].acceleration.x = ws[i].acc(0);
      cmd_array_[i].acceleration.y = ws[i].acc(1);
      cmd_array_[i].acceleration.z = ws[i].acc(2);
      cmd_array_[i].jerk.x = ws[i].jrk(0);
      cmd_array_[i].jerk.y = ws[i].jrk(1);
      cmd_array_[i].jerk.z = ws[i].jrk(2);
      cmd_array_[i].yaw = ws[i].yaw;
      cmd_array_[i].yaw_dot = ws[i].yaw_dot;
    }
  }

  std::vector<planning_ros_msgs::TrajectoryCommand> getCommands() {
    return cmd_array_;
  }

 private:
  std::vector<planning_ros_msgs::TrajectoryCommand> cmd_array_;
};
