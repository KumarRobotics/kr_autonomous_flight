// Copyright 2016 Michael Watterson
#include <traj_opt_ros/ros_bridge.h>

#include <string>

TrajRosBridge::TrajRosBridge() : nh_("~") {
  pub_ = nh_.advertise<traj_opt_msgs::Trajectory>("trajectory", 1, true);
}
TrajRosBridge &TrajRosBridge::instance() {
  static TrajRosBridge inst;
  return inst;
}
void TrajRosBridge::publish_msg(const traj_opt_msgs::Trajectory &msg,
                                std::string frame_id) {
  traj_opt_msgs::Trajectory msgc = msg;
  msgc.header.frame_id = frame_id;
  instance().pub_.publish(msgc);
}
void TrajRosBridge::publish_msg(const traj_opt::TrajData &data,
                                std::string frame_id) {
  publish_msg(convert(data), frame_id);
}

// these convert functions can be written more cleanly with templates
traj_opt_msgs::Trajectory TrajRosBridge::convert(
    const traj_opt::TrajData &data) {
  traj_opt_msgs::Trajectory traj;
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "map";

  traj.dimension_names = data.dimension_names;
  traj.dimensions = data.dimensions;
  // copy all fields
  for (auto spline : data.data) {
    traj_opt_msgs::Spline s;
    for (auto poly : spline.segs) {
      traj_opt_msgs::Polynomial p;
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
traj_opt::TrajData TrajRosBridge::convert(
    const traj_opt_msgs::Trajectory &msg) {
  traj_opt::TrajData data;
  // copy all fields
  data.dimension_names = msg.dimension_names;
  data.dimensions = msg.dimensions;
  for (auto spline : msg.data) {
    traj_opt::SplineData s;
    for (auto poly : spline.segs) {
      traj_opt::PolynomialData p;
      p.degree = poly.degree;
      p.dt = poly.dt;
      p.basis = (traj_opt::PolyType)poly.basis;
      p.coeffs = poly.coeffs;
      s.segs.push_back(p);
    }
    s.segments = spline.segments;
    s.t_total = spline.t_total;
    data.data.push_back(s);
  }
  return data;
}
