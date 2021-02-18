// Copyright 2015 Michael Watterson
#ifndef TRAJ_OPT_PRO_CONVERSIONS_H_
#define TRAJ_OPT_PRO_CONVERSIONS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <traj_opt_basic/types.h>

namespace traj_opt {
class Conversions {
 public:
  //  Conversions() {}
  static Vec4 PointToVec(const geometry_msgs::Point &p) {
    Vec4 v;
    v << p.x, p.y, p.z, 0.0;
    return v;
  }
  static geometry_msgs::Point VecToPoint(const Vec4 &v) {
    geometry_msgs::Point p;
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
    return p;
  }
  static Vec4 Vector3ToVec(const geometry_msgs::Vector3 &p) {
    Vec4 v;
    v << p.x, p.y, p.z, 0.0;
    return v;
  }
  static geometry_msgs::Vector3 VecToVector3(const Vec4 &v) {
    geometry_msgs::Vector3 p;
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
    return p;
  }
};

}  // namespace traj_opt

#endif  // TRAJ_OPT_PRO_CONVERSIONS_H_
