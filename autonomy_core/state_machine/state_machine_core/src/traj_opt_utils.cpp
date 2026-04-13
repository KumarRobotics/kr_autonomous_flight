#include <state_machine/traj_opt_utils.hpp>

namespace state_machine {

traj_opt::VecD Make4d(const traj_opt::VecD& vec) {
  traj_opt::VecD out = traj_opt::VecD::Zero(4);
  long int rows = std::min(vec.rows(), static_cast<long int>(4));
  out.block(0, 0, rows, 1) = vec.block(0, 0, rows, 1);
  return out;
}

void VecToPose(const traj_opt::VecD& valn, geometry_msgs::msg::Pose* pos) {
  if (pos != nullptr) {
    traj_opt::VecD val = Make4d(valn);
    pos->position.x = val(0), pos->position.y = val(1),
    pos->position.z = val(2);
    pos->orientation.w = std::cos(0.5 * val(3));
    pos->orientation.z = std::sin(0.5 * val(3));
  }
}

void VecToTwist(const traj_opt::VecD& valn, geometry_msgs::msg::Twist* vel) {
  if (vel != nullptr) {
    traj_opt::VecD val = Make4d(valn);
    vel->linear.x = val(0), vel->linear.y = val(1), vel->linear.z = val(2);
    vel->angular.z = val(3);
  }
}

void EvaluateToMsgs(boost::shared_ptr<traj_opt::Trajectory> traj,
                    double dt,
                    geometry_msgs::msg::Pose* pos,
                    geometry_msgs::msg::Twist* vel,
                    geometry_msgs::msg::Twist* acc,
                    geometry_msgs::msg::Twist* jrk) {
  if (traj == nullptr) return;
  traj_opt::VecD val;
  traj->evaluate(dt, 0, val);
  VecToPose(val, pos);

  traj->evaluate(dt, 1, val);
  VecToTwist(val, vel);

  traj->evaluate(dt, 2, val);
  VecToTwist(val, acc);

  traj->evaluate(dt, 3, val);
  VecToTwist(val, jrk);
}

}  // namespace state_machine
