#include <angles/angles.h>
#include <traj_opt_quadrotor/convert.h>

using namespace traj_opt;

void TrajToQuadCmd::evaluate(
    const boost::shared_ptr<traj_opt::Trajectory> &traj, traj_opt::decimal_t dt,
    kr_mav_msgs::PositionCommand *out, uint max_derr_eval,
    traj_opt::decimal_t scaling) {
  traj_opt::VecD val;

  traj->evaluate(dt, 0, val);

  if (val.rows() > 0) out->position.x = val(0);
  if (val.rows() > 1) out->position.y = val(1);
  if (val.rows() > 2) out->position.z = val(2);
  if (val.rows() > 3) out->yaw = val(3);
  // if (val.rows() == 5) out->chart = val(4) > 0.5;

  if (max_derr_eval < 1) return;
  traj->evaluate(dt, 1, val);
  if (val.rows() > 0) out->velocity.x = val(0);
  if (val.rows() > 1) out->velocity.y = val(1);
  if (val.rows() > 2) out->velocity.z = val(2);
  if (val.rows() > 3) out->yaw_dot = val(3);
  if (max_derr_eval < 2) return;

  traj->evaluate(dt, 2, val);
  if (val.rows() > 0) out->acceleration.x = val(0);
  if (val.rows() > 1) out->acceleration.y = val(1);
  if (val.rows() > 2) out->acceleration.z = val(2);
  if (max_derr_eval < 3) return;

  traj->evaluate(dt, 3, val);
  if (val.rows() > 0) out->jerk.x = val(0);
  if (val.rows() > 1) out->jerk.y = val(1);
  if (val.rows() > 2) out->jerk.z = val(2);
}

// send all other interfaces to pointer version
void TrajToQuadCmd::evaluate(
    const boost::shared_ptr<traj_opt::Trajectory> &traj, traj_opt::decimal_t dt,
    kr_mav_msgs::PositionCommand::Ptr &out, uint max_derr_eval,
    traj_opt::decimal_t scaling) {
  evaluate(traj, dt, out.get(), max_derr_eval, scaling);
}
kr_mav_msgs::PositionCommand TrajToQuadCmd::evaluate(
    const boost::shared_ptr<traj_opt::Trajectory> &traj, traj_opt::decimal_t dt,
    uint max_derr_eval, traj_opt::decimal_t scaling) {
  kr_mav_msgs::PositionCommand cmd;
  evaluate(traj, dt, &cmd, max_derr_eval, scaling);
  return cmd;
}
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Quaternion<double> Quat;
class S2 {
 public:
  // need to construct from quaternion because of hairy ball problem
  S2(const Quat &R) : R_(R) {
    R_.normalize();
    Mat3 Rm = R_.matrix();
    //      nx_ = Rm.block<3,1>(0,0);
    //      ny_ = Rm.block<3,1>(0,1);
    //      np_ = Rm.block<3,1>(0,2);
    nx_ = Rm.block<3, 1>(0, 1);
    ny_ = Rm.block<3, 1>(0, 2);
    np_ = Rm.block<3, 1>(0, 0);  // switch to paper notation
  }
  Vec2 sphereToPlane(const Vec3 &p) {
    Vec3 err = p - np_;
    double lambda = 1.0 / (1.0 - np_.dot(err));
    Vec3 pr3 = lambda * err + np_;
    return Vec2(pr3.dot(nx_), pr3.dot(ny_));
  }
  Vec3 planeToSphere(const Vec2 &p) {
    Vec3 p3 = p(0) * nx_ + p(1) * ny_;
    Vec3 e = p3 - np_;
    double lambda = -2.0 * e.dot(np_) / e.dot(e);
    return lambda * e + np_;
  }

 private:
  Vec3 np_, nx_, ny_;
  Quat R_;
};
Vec3 distort(const Vec3 &in) {
  //  std::cout << "in " << in.transpose() << std::endl;
  traj_opt::Vec4 d_real;
  d_real << -0.01067, -0.0166, 0.01258, -0.00387;

  Vec3 inn = in;
  inn /= in(2);
  double r = sqrt(inn(0) * inn(0) + inn(1) * inn(1));
  double theta = atan(r);
  double theta2 = theta * theta;
  double theta4 = theta2 * theta2;
  double theta6 = theta4 * theta2;
  double theta8 = theta4 * theta4;
  double thetad = theta * (1 + d_real(0) * theta2 + d_real(1) * theta4 +
                           d_real(2) * theta6 + d_real(3) * theta8);
  double scaling = (r > 1e-8) ? thetad / r : 1.0;
  inn(0) *= scaling;
  inn(1) *= scaling;
  //  std::cout << "inn " << inn.transpose() << std::endl;
  return inn;
}

void TrajToQuadCmd::handleSphere(
    const boost::shared_ptr<traj_opt::Trajectory> &traj, traj_opt::decimal_t dt,
    kr_mav_msgs::PositionCommand::Ptr &out, geometry_msgs::Point &image_point) {
  evaluate(traj, dt, out.get());  // add r3 part, but yaw will be invalid
  out->yaw_dot = 0;
  traj_opt::VecD val;
  traj->evaluate(dt, 0, val);
  traj_opt::Quat q(val(5), val(6), val(7), val(8));
  traj_opt::Mat3 K_real;

  K_real << 608.60271, 0, 491.04226, 0, 608.80908, 410.49738, 0, 0, 1;

  S2 sphere(q);
  Vec2 p = Vec2(val(3), val(4));
  Vec3 pv = sphere.planeToSphere(p);

  image_point.x = pv(0);
  image_point.y = pv(1);
  image_point.z = pv(2);
}

void TrajToQuadCmd::evaluateTagentYaw(
    const boost::shared_ptr<traj_opt::Trajectory> &traj, traj_opt::decimal_t dt,
    kr_mav_msgs::PositionCommand::Ptr &out, double old_yaw, double yaw_speed,
    double ddt, double yaw_thr) {
  evaluate(traj, dt, out, 2);
  double des_yaw = old_yaw;
  if (std::hypot(out->velocity.x, out->velocity.y) > 0.1)
    des_yaw = std::atan2(out->velocity.y, out->velocity.x);
  VecD pos;
  traj->evaluate(100, 0, pos);
  double des_yaw2 = (des_yaw + std::atan2(pos(1) - out->position.y,
                                          pos(0) - out->position.x)) /
                    2;
  if (std::abs(angles::shortest_angular_distance(des_yaw2, des_yaw)) < yaw_thr)
    des_yaw = des_yaw2;

  double dyaw = angles::shortest_angular_distance(old_yaw, des_yaw);
  double des_yaw_dot = 0.0;
  if (dyaw > 0.05)  // deadzone
    des_yaw_dot = yaw_speed;
  else if (dyaw < -0.05)
    des_yaw_dot = -yaw_speed;

  out->yaw = old_yaw + des_yaw_dot * ddt;
  out->yaw_dot = des_yaw_dot;
}
bool TrajToQuadCmd::evaluatePos(
    const boost::shared_ptr<traj_opt::Trajectory> &traj,
    const nav_msgs::Odometry::ConstPtr &odom, double err_max, double t_des,
    double ddt, kr_mav_msgs::PositionCommand::Ptr &out) {
  // return false if need to adjust time
  bool return_v = true;
  VecD pos = VecD(4, 1);
  pos << odom->pose.pose.position.x, odom->pose.pose.position.y,
      odom->pose.pose.position.z,
      2.0 * std::asin(odom->pose.pose.orientation.z);

  VecD val, vel;
  traj->evaluate(t_des, 0, val);  // position of traj
  traj->evaluate(t_des, 1, vel);  // velocity of traj

  Vec2 diff_xy = Vec2(val(0) - pos(0), val(1) - pos(1));
  if (diff_xy.norm() >= err_max) {
    printf("Distance between odom and traj in xy too large! It is: %f \n",
           diff_xy.norm());
    return_v = false;  // return false
  }

  evaluate(traj, t_des, out);

  return return_v;
}
