#include "mpl_planner/env_base.h"

#include <iostream>

#include "mpl_basis/math.h"

namespace MPL {

template <int Dim>
bool EnvBase<Dim>::is_goal(const WaypointD& state) const {
  if (state.t >= t_max_) return true;
  bool goaled =
      (state.pos - goal_node_.pos).template lpNorm<Eigen::Infinity>() <=
      tol_pos_;
  if (goaled && tol_vel_ >= 0)
    goaled = (state.vel - goal_node_.vel).template lpNorm<Eigen::Infinity>() <=
             tol_vel_;
  if (goaled && tol_acc_ >= 0)
    goaled = (state.acc - goal_node_.acc).template lpNorm<Eigen::Infinity>() <=
             tol_acc_;
  if (goaled && tol_yaw_ >= 0)
    goaled = std::abs(state.yaw - goal_node_.yaw) <= tol_yaw_;
  return goaled;
}

template <int Dim>
decimal_t EnvBase<Dim>::get_heur(const WaypointD& state) const {
  if (goal_node_ == state) return 0;
  size_t id = state.t / dt_;
  if (!prior_traj_.empty() && id < prior_traj_.size())
    return cal_heur(state, prior_traj_[id].first) + prior_traj_[id].second;
  else
    return cal_heur(state, goal_node_);
}

template <int Dim>
decimal_t EnvBase<Dim>::cal_heur(const WaypointD& state,
                                 const WaypointD& goal,
                                 const double& z_penalty) const {
  // the corresponding cost calculation is in cal_intrinsic_cost
  if (heur_ignore_dynamics_) {
    if (v_max_ > 0 && v_max_z_ > 0 && Dim == 3) {
      // original heuristic:
      // return w_ * (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() /
      //        v_max_;
      const Vecf<Dim> dp = goal.pos - state.pos;
      double t_xy_est = std::max(std::abs(dp[0]), std::abs(dp[1])) / v_max_;
      double t_z_est = std::abs(dp[2]) / v_max_z_;
      // combine the two to account for both xy and z direction cost, and
      // speed up by penalizing z search, but can cause suboptimal trajectory
      // TODO(xu:) alternatively, decrease x y time, but it makes search slow,
      // a better way is to change time cost to treat xy and z differently
      return w_ * t_xy_est + t_z_est * z_penalty;
    } else if (v_max_ > 0) {
      return w_ * (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() /
             v_max_;
    } else {
      printf("[Motion primitive planner:] max_v_xy or max_v_z is negative");
      return w_ * (state.pos - goal.pos).template lpNorm<Eigen::Infinity>();
    }
  }

  if (state.control == MPL::JRK && goal.control == MPL::JRK) {
    const Vecf<Dim> dp = goal.pos - state.pos;
    const Vecf<Dim> v0 = state.vel;
    const Vecf<Dim> v1 = goal.vel;
    const Vecf<Dim> a0 = state.acc;
    const Vecf<Dim> a1 = goal.acc;
    decimal_t a = w_;
    decimal_t b = 0;
    decimal_t c = -9 * a0.dot(a0) + 6 * a0.dot(a1) - 9 * a1.dot(a1);
    decimal_t d = -144 * a0.dot(v0) - 96 * a0.dot(v1) + 96 * a1.dot(v0) +
                  144 * a1.dot(v1);
    decimal_t e = 360 * (a0 - a1).dot(dp) - 576 * v0.dot(v0) -
                  1008 * v0.dot(v1) - 576 * v1.dot(v1);
    decimal_t f = 2880 * dp.dot(v0 + v1);
    decimal_t g = -3600 * dp.dot(dp);

    std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

    decimal_t t_bar =
        (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
    ts.push_back(t_bar);
    decimal_t min_cost = std::numeric_limits<decimal_t>::max();
    for (auto t : ts) {
      if (t < t_bar) continue;
      decimal_t cost = a * t - c / t - d / 2 / t / t - e / 3 / t / t / t -
                       f / 4 / t / t / t / t - g / 5 / t / t / t / t / t;
      if (cost < min_cost) min_cost = cost;
    }
    return min_cost;
  }

  else if (state.control == MPL::JRK && goal.control == MPL::ACC) {
    const Vecf<Dim> dp = goal.pos - state.pos;
    const Vecf<Dim> v0 = state.vel;
    const Vecf<Dim> v1 = goal.vel;
    const Vecf<Dim> a0 = state.acc;

    decimal_t a = w_;
    decimal_t b = 0;
    decimal_t c = -8 * a0.dot(a0);
    decimal_t d = -112 * a0.dot(v0) - 48 * a0.dot(v1);
    decimal_t e = 240 * a0.dot(dp) - 384 * v0.dot(v0) - 432 * v0.dot(v1) -
                  144 * v1.dot(v1);
    decimal_t f = dp.dot(1600 * v0 + 960 * v1);
    decimal_t g = -1600 * dp.dot(dp);

    std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

    decimal_t t_bar =
        (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
    ts.push_back(t_bar);
    decimal_t min_cost = std::numeric_limits<decimal_t>::max();
    for (auto t : ts) {
      if (t < t_bar) continue;
      decimal_t cost = a * t - c / t - d / 2 / t / t - e / 3 / t / t / t -
                       f / 4 / t / t / t / t - g / 5 / t / t / t / t / t;
      if (cost < min_cost) min_cost = cost;
      // printf("t: %f, cost: %f\n",t, cost);
    }
    return min_cost;
  }

  else if (state.control == MPL::JRK && goal.control == MPL::VEL) {
    const Vecf<Dim> dp = goal.pos - state.pos;
    const Vecf<Dim> v0 = state.vel;
    const Vecf<Dim> a0 = state.acc;

    decimal_t a = w_;
    decimal_t b = 0;
    decimal_t c = -5 * a0.dot(a0);
    decimal_t d = -40 * a0.dot(v0);
    decimal_t e = 60 * a0.dot(dp) - 60 * v0.dot(v0);
    decimal_t f = 160 * dp.dot(v0);
    decimal_t g = -100 * dp.dot(dp);

    std::vector<decimal_t> ts = solve(a, b, c, d, e, f, g);

    decimal_t t_bar =
        (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
    ts.push_back(t_bar);

    decimal_t min_cost = std::numeric_limits<decimal_t>::max();
    for (auto t : ts) {
      if (t < t_bar) continue;
      decimal_t cost = a * t - c / t - d / 2 / t / t - e / 3 / t / t / t -
                       f / 4 / t / t / t / t - g / 5 / t / t / t / t / t;
      if (cost < min_cost) min_cost = cost;
    }
    return min_cost;
  }

  else if (state.control == MPL::ACC && goal.control == MPL::ACC) {
    const Vecf<Dim> dp = goal.pos - state.pos;
    const Vecf<Dim> v0 = state.vel;
    const Vecf<Dim> v1 = goal.vel;

    decimal_t c1 = -36 * dp.dot(dp);
    decimal_t c2 = 24 * (v0 + v1).dot(dp);
    decimal_t c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    decimal_t c4 = 0;
    decimal_t c5 = w_;

    std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
    decimal_t t_bar =
        (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
    ts.push_back(t_bar);

    decimal_t cost = std::numeric_limits<decimal_t>::max();
    for (auto t : ts) {
      if (t < t_bar) continue;
      decimal_t c = -c1 / 3 / t / t / t - c2 / 2 / t / t - c3 / t + w_ * t;
      if (c < cost) cost = c;
    }

    return cost;
  }

  else if (state.control == MPL::ACC && goal.control == MPL::VEL) {
    const Vecf<Dim> dp = goal.pos - state.pos;
    const Vecf<Dim> v0 = state.vel;

    decimal_t c1 = -9 * dp.dot(dp);
    decimal_t c2 = 12 * v0.dot(dp);
    decimal_t c3 = -3 * v0.dot(v0);
    decimal_t c4 = 0;
    decimal_t c5 = w_;

    std::vector<decimal_t> ts = quartic(c5, c4, c3, c2, c1);
    decimal_t t_bar =
        (state.pos - goal.pos).template lpNorm<Eigen::Infinity>() / v_max_;
    ts.push_back(t_bar);

    decimal_t cost = std::numeric_limits<decimal_t>::max();
    for (auto t : ts) {
      if (t < t_bar) continue;
      decimal_t c = -c1 / 3 / t / t / t - c2 / 2 / t / t - c3 / t + w_ * t;
      if (c < cost) cost = c;
    }

    return cost;
  } else if (state.control == MPL::VEL && goal.control == MPL::VEL)
    return (w_ + 1) * (state.pos - goal.pos).norm();
  else
    return w_ * (state.pos - goal.pos).norm() / v_max_;
}

template <int Dim>
Veci<Dim> EnvBase<Dim>::round(const Vecf<Dim>& vec, decimal_t res) const {
  Veci<Dim> vecI;
  for (int i = 0; i < Dim; i++) vecI(i) = std::round(vec(i) / res);
  return vecI;
}

// template <int Dim>
// std::string EnvBase<Dim>::to_string(const Veci<Dim> &vec) const {
//  std::string str;
//  for (int i = 0; i < Dim; i++) str += std::to_string(vec(i)) + "-";
//  return str;
//}

template <int Dim>
void EnvBase<Dim>::forward_action(const WaypointD& curr,
                                  int action_id,
                                  PrimitiveD& pr) const {
  pr = Primitive<Dim>(curr, U_[action_id], dt_);
}

template <int Dim>
void EnvBase<Dim>::set_prior_trajectory(const TrajectoryD& traj) {
  prior_traj_.clear();
  decimal_t total_time = traj.getTotalTime();
  for (decimal_t t = 0; t < total_time; t += dt_) {
    prior_traj_.push_back(
        std::make_pair(traj.evaluate(t), w_ * (total_time - t)));
  }
}

template <int Dim>
bool EnvBase<Dim>::set_goal(const WaypointD& state) {
  if (prior_traj_.empty()) goal_node_ = state;
  return prior_traj_.empty();
}

template <int Dim>
void EnvBase<Dim>::info() {
  printf(ANSI_COLOR_YELLOW "\n");
  printf("++++++++++++++++++++ env_base ++++++++++++++++++\n");
  printf("+                  w: %.2f               +\n", w_);
  printf("+               wyaw: %.2f               +\n", wyaw_);
  printf("+                 dt: %.2f               +\n", dt_);
  printf("+              t_max: %.2f               +\n", t_max_);
  printf("+              v_xy_max: %.2f               +\n", v_max_);
  printf("+              v_z_max: %.2f               +\n", v_max_z_);
  printf("+              a_max: %.2f               +\n", a_max_);
  printf("+              j_max: %.2f               +\n", j_max_);
  printf("+            yaw_max: %.2f               +\n", yaw_max_);
  printf("+              U num: %zu                +\n", U_.size());
  printf("+            tol_pos: %.2f               +\n", tol_pos_);
  printf("+            tol_vel: %.2f               +\n", tol_vel_);
  printf("+            tol_acc: %.2f               +\n", tol_acc_);
  printf("+            tol_yaw: %.2f               +\n", tol_yaw_);
  printf("+heur_ignore_dynamics: %d                 +\n",
         heur_ignore_dynamics_);
  printf("++++++++++++++++++++ env_base ++++++++++++++++++\n");
  printf(ANSI_COLOR_RESET "\n");
}

template <int Dim>
decimal_t EnvBase<Dim>::cal_intrinsic_cost(const PrimitiveD& pr) const {
  // the corresponding heuristic calculation is in cal_intrinsic_cost

  // pr.J is integration of square of jerk, dt_ >= dist / v_max (guaranteeing
  // it's admissible and consistent) the larger the heuristic_weight, the more
  // emphasis on min time and the less emphasis on min control effort
  return pr.J(pr.control()) + w_ * dt_;
}

template class EnvBase<2>;
template class EnvBase<3>;

}  // namespace MPL
