#include <X11/Xlib.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <traj_opt_basic/msg_traj.h>
#include <traj_opt_quadrotor/convert.h>
#include <traj_opt_ros/ros_bridge.h>
#include <traj_opt_ros/traj_plot.h>

#include "ros/ros.h"

namespace plt = matplotlibcpp;

using namespace traj_opt;

void trajCallback(const traj_opt_msgs::TrajectoryConstPtr& msg) {
  boost::shared_ptr<Trajectory> traj =
      boost::make_shared<MsgTrajectory>(TrajRosBridge::convert(*msg));
  //  TrajPlot::plot(traj,1);

  std::vector<double> vision_x, vision_y, times;
  int samples = traj->getTotalTime() * 100;
  vision_x.reserve(samples);
  vision_y.reserve(samples);
  times.reserve(samples);

  auto cmd = boost::make_shared<kr_mav_msgs::PositionCommand>();
  geometry_msgs::Point pt;
  for (double t = 0; t <= traj->getTotalTime(); t += 0.01) {
    TrajToQuadCmd::handleSphere(traj, t, cmd, pt);
    vision_x.push_back(pt.x);
    vision_y.push_back(pt.y);
    times.push_back(t);
  }
  plt::figure();
  plt::subplot(2, 1, 1);
  plt::plot(times, vision_x);

  plt::subplot(2, 1, 2);
  plt::plot(times, vision_y);

  plt::show(true);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_debug");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/trajectory", 1, trajCallback);

  ros::spin();

  return 0;
}
