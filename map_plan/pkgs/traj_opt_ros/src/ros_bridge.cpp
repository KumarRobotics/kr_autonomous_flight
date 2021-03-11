// Copyright 2016 Michael Watterson
#include "traj_opt_ros/ros_bridge.h"

namespace traj_opt {

TrajRosBridge::TrajRosBridge() : nh_("~") {
  pub_ =
      nh_.advertise<planning_ros_msgs::SplineTrajectory>("trajectory", 1, true);
}
TrajRosBridge &TrajRosBridge::instance() {
  static TrajRosBridge inst;
  return inst;
}
void TrajRosBridge::publish_msg(const planning_ros_msgs::SplineTrajectory &msg,
                                std::string frame_id) {
  planning_ros_msgs::SplineTrajectory msgc = msg;
  msgc.header.frame_id = frame_id;
  instance().pub_.publish(msgc);
}
void TrajRosBridge::publish_msg(const TrajData &data, std::string frame_id) {
  publish_msg(SplineTrajectoryFromTrajData(data), frame_id);
}

// these convert functions can be written more cleanly with templates
planning_ros_msgs::SplineTrajectory SplineTrajectoryFromTrajData(
    const TrajData &data) {
  planning_ros_msgs::SplineTrajectory traj;
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "map";

  traj.dimension_names = data.dimension_names;
  traj.dimensions = data.dimensions;
  // copy all fields
  for (auto spline : data.data) {
    planning_ros_msgs::Spline s;
    for (auto poly : spline.segs) {
      planning_ros_msgs::Polynomial p;
      p.degree = poly.degree;
      p.dt = poly.dt;
      p.basis = poly.basis;
      p.coeffs = poly.coeffs;
      s.segs.push_back(p);
    }
    s.segments = spline.segments;
    s.t_total = spline.t_total;
    traj.data.push_back(s);
  }
  return traj;
}

TrajData TrajDataFromSplineTrajectory(
    const planning_ros_msgs::SplineTrajectory &msg) {
  TrajData data;
  // copy all fields
  data.dimension_names = msg.dimension_names;
  data.dimensions = msg.dimensions;
  for (auto spline : msg.data) {
    SplineData s;
    for (auto poly : spline.segs) {
      PolynomialData p;
      p.degree = poly.degree;
      p.dt = poly.dt;
      p.basis = (PolyType)poly.basis;
      p.coeffs = poly.coeffs;
      s.segs.push_back(p);
    }
    s.segments = spline.segments;
    s.t_total = spline.t_total;
    data.data.push_back(s);
  }
  return data;
}
// too lazy to include boost math
int factorial(int i) {
  if (i == 0)
    return 1;
  else
    return i * factorial(i - 1);
}

planning_ros_msgs::SplineTrajectory SplineTrajectoryFromTrajectory(
    const planning_ros_msgs::Trajectory &msg) {
  planning_ros_msgs::SplineTrajectory traj;
  traj.header = msg.header;

  double T = 0.0;
  for (uint s = 0; s < msg.primitives.size(); s++) {
    T += msg.primitives.at(s).t;
  }

  for (uint d = 0; d < 3; d++) {
    planning_ros_msgs::Spline spline;
    for (uint s = 0; s < msg.primitives.size(); s++) {
      const std::vector<double> *co;
      // get correct field
      if (d == 0) co = &(msg.primitives.at(s).cx);
      if (d == 1) co = &(msg.primitives.at(s).cy);
      if (d == 2) co = &(msg.primitives.at(s).cz);
      planning_ros_msgs::Polynomial poly;
      for (uint c = 0; c < co->size(); c++) {
        uint cr = co->size() - 1 - c;
        poly.coeffs.push_back(co->at(cr) * std::pow(msg.primitives.at(s).t, c) /
                              static_cast<double>(factorial(c)));
      }
      poly.dt = msg.primitives.at(s).t;
      poly.degree = co->size() - 1;
      spline.segs.push_back(poly);
    }
    spline.segments = msg.primitives.size();
    spline.t_total = T;
    traj.data.push_back(spline);
  }
  traj.dimensions = 3;
  return traj;
}

}  // namespace traj_opt