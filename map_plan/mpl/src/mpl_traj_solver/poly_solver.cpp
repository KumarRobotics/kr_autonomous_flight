#include <mpl_traj_solver/poly_solver.h>

template <int Dim>
PolySolver<Dim>::PolySolver(unsigned int smooth_derivative_order,
                            unsigned int minimize_derivative, bool debug)
    : N_(2 * (smooth_derivative_order + 1)),
      R_(minimize_derivative),
      debug_(debug) {
  ptraj_ = std::make_shared<PolyTraj<Dim>>();
  if (debug_) {
    std::cout << "smooth_derivative_order: " << smooth_derivative_order
              << std::endl;
    std::cout << "minimize_derivative: " << minimize_derivative << std::endl;
  }
}

template <int Dim>
std::shared_ptr<PolyTraj<Dim>> PolySolver<Dim>::getTrajectory() {
  return ptraj_;
}

template <int Dim>
bool PolySolver<Dim>::solve(const vec_E<Waypoint<Dim>> &waypoints,
                            const std::vector<decimal_t> &dts) {
  ptraj_->clear();
  ptraj_->setWaypoints(waypoints);
  ptraj_->setTime(dts);

  const unsigned int num_waypoints = waypoints.size();
  const unsigned int num_segments = num_waypoints - 1;
  if (num_waypoints < 2) return false;
  if (debug_) {
    for (unsigned int i = 0; i < num_waypoints; i++)
      waypoints[i].print("waypoint" + std::to_string(i) + ":");
  }

  MatDf A = MatDf::Zero(num_segments * N_, num_segments * N_);
  MatDf Q = MatDf::Zero(num_segments * N_, num_segments * N_);
  for (unsigned int i = 0; i < num_segments; i++) {
    decimal_t seg_time = dts[i];
    // n column
    for (unsigned int n = 0; n < N_; n++) {
      // A_0
      if (n < N_ / 2) {
        int val = 1;
        for (unsigned int m = 0; m < n; m++) val *= (n - m);
        A(i * N_ + n, i * N_ + n) = val;
      }
      // A_T
      for (unsigned int r = 0; r < N_ / 2; r++) {
        if (r <= n) {
          int val = 1;
          for (unsigned int m = 0; m < r; m++) val *= (n - m);
          A(i * N_ + N_ / 2 + r, i * N_ + n) = val * power(seg_time, n - r);
        }
      }
      // Q
      for (unsigned int r = 0; r < N_; r++) {
        if (r >= R_ && n >= R_) {
          int val = 1;
          for (unsigned int m = 0; m < R_; m++) val *= (r - m) * (n - m);
          Q(i * N_ + r, i * N_ + n) =
              val * power(seg_time, r + n - 2 * R_ + 1) / (r + n - 2 * R_ + 1);
        }
      }
    }
  }

  A_ = A;
  Q_ = Q;

  const unsigned int smooth_derivative_order = N_ / 2 - 1;
  const unsigned int num_total_derivatives = num_waypoints * N_ / 2;
  unsigned int num_fixed_derivatives = 0;
  for (const auto &it : waypoints) {
    if (it.use_pos && smooth_derivative_order >= 0) num_fixed_derivatives++;
    if (it.use_vel && smooth_derivative_order >= 1) num_fixed_derivatives++;
    if (it.use_acc && smooth_derivative_order >= 2) num_fixed_derivatives++;
    if (it.use_jrk && smooth_derivative_order >= 3) num_fixed_derivatives++;
  }
  const unsigned int num_free_derivatives =
      num_total_derivatives - num_fixed_derivatives;
  if (debug_)
    printf("num_fixed_derivatives: %d, num_free_derivatives: %d\n",
           num_fixed_derivatives, num_free_derivatives);

  std::vector<std::pair<unsigned int, unsigned int>> permutation_table;
  unsigned int raw_cnt = 0;
  unsigned int fix_cnt = 0;
  unsigned int free_cnt = 0;
  unsigned int id = 0;
  for (const auto &it : waypoints) {
    if (it.use_pos && smooth_derivative_order >= 0) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if (id > 0 && id < num_waypoints - 1)
        permutation_table.push_back(std::make_pair(raw_cnt + N_ / 2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    } else if (!it.use_pos && smooth_derivative_order >= 0) {
      permutation_table.push_back(
          std::make_pair(raw_cnt, num_fixed_derivatives + free_cnt));
      if (id > 0 && id < num_waypoints - 1)
        permutation_table.push_back(
            std::make_pair(raw_cnt + N_ / 2, num_fixed_derivatives + free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if (it.use_vel && smooth_derivative_order >= 1) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if (id > 0 && id < num_waypoints - 1)
        permutation_table.push_back(std::make_pair(raw_cnt + N_ / 2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    } else if (!it.use_vel && smooth_derivative_order >= 1) {
      permutation_table.push_back(
          std::make_pair(raw_cnt, num_fixed_derivatives + free_cnt));
      if (id > 0 && id < num_waypoints - 1)
        permutation_table.push_back(
            std::make_pair(raw_cnt + N_ / 2, num_fixed_derivatives + free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if (it.use_acc && smooth_derivative_order >= 2) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if (id > 0 && id < num_waypoints - 1)
        permutation_table.push_back(std::make_pair(raw_cnt + N_ / 2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    } else if (!it.use_acc && smooth_derivative_order >= 2) {
      permutation_table.push_back(
          std::make_pair(raw_cnt, num_fixed_derivatives + free_cnt));
      if (id > 0 && id < num_waypoints - 1)
        permutation_table.push_back(
            std::make_pair(raw_cnt + N_ / 2, num_fixed_derivatives + free_cnt));
      raw_cnt++;
      free_cnt++;
    }
    if (it.use_jrk && smooth_derivative_order >= 3) {
      permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
      if (id > 0 && id < num_waypoints - 1)
        permutation_table.push_back(std::make_pair(raw_cnt + N_ / 2, fix_cnt));
      raw_cnt++;
      fix_cnt++;
    } else if (!it.use_jrk && smooth_derivative_order >= 3) {
      permutation_table.push_back(
          std::make_pair(raw_cnt, num_fixed_derivatives + free_cnt));
      if (id > 0 && id < num_waypoints - 1)
        permutation_table.push_back(
            std::make_pair(raw_cnt + N_ / 2, num_fixed_derivatives + free_cnt));
      raw_cnt++;
      free_cnt++;
    }

    if (id > 0 && id < num_waypoints - 1) raw_cnt += N_ / 2;
    id++;
  }

  if (debug_) {
    std::cout << "permutation_table: \n" << std::endl;
    for (auto it : permutation_table)
      std::cout << "old id: " << it.first << " new_id: " << it.second
                << std::endl;
  }

  // M
  MatDf M = MatDf::Zero(num_segments * N_, num_waypoints * N_ / 2);
  for (const auto &it : permutation_table) M(it.first, it.second) = 1;

  // Eigen::MatrixXf A_inv = A.inverse();
  MatDf A_inv_M = A.partialPivLu().solve(M);
  MatDf R = A_inv_M.transpose() * Q * A_inv_M;
  if (debug_) {
    std::cout << "A:\n" << A << std::endl;
    std::cout << "Q:\n" << Q << std::endl;
    std::cout << "M:\n" << M << std::endl;
    std::cout << "A_inv_M:\n" << A_inv_M << std::endl;
    std::cout << "R:\n" << R << std::endl;
  }
  MatDf Rpp = R.block(num_fixed_derivatives, num_fixed_derivatives,
                      num_free_derivatives, num_free_derivatives);
  MatDf Rpf = R.block(num_fixed_derivatives, 0, num_free_derivatives,
                      num_fixed_derivatives);

  // Fixed derivatives
  MatDNf<Dim> Df = MatDNf<Dim>(num_fixed_derivatives, Dim);
  for (const auto &it : permutation_table) {
    if (it.second < num_fixed_derivatives) {
      int id = std::floor((it.first + N_ / 2) / N_);
      int derivative = it.first % (N_ / 2);
      if (derivative == 0)
        Df.row(it.second) = waypoints[id].pos.transpose();
      else if (derivative == 1)
        Df.row(it.second) = waypoints[id].vel.transpose();
      else if (derivative == 2)
        Df.row(it.second) = waypoints[id].acc.transpose();
      else if (derivative == 3)
        Df.row(it.second) = waypoints[id].jrk.transpose();
    }
  }

  if (debug_) std::cout << "Df:\n" << Df << std::endl;
  MatDNf<Dim> D = MatDNf<Dim>(num_waypoints * N_ / 2, Dim);
  D.topRows(num_fixed_derivatives) = Df;

  if (num_waypoints > 2 && num_free_derivatives > 0) {
    MatDNf<Dim> Dp = -Rpp.partialPivLu().solve(Rpf * Df);
    if (debug_) std::cout << "Dp:\n" << Dp << std::endl;
    D.bottomRows(num_free_derivatives) = Dp;
  }

  MatDNf<Dim> d = M * D;
  if (debug_) std::cout << "d:\n" << d << std::endl;
  for (unsigned int i = 0; i < num_segments; i++) {
    const MatDNf<Dim> p = A.block(i * N_, i * N_, N_, N_)
                              .partialPivLu()
                              .solve(d.block(i * N_, 0, N_, Dim));
    if (debug_) std::cout << "p:\n" << p << std::endl;
    ptraj_->addCoeff(p);
  }
  return true;
}

template class PolySolver<1>;

template class PolySolver<2>;

template class PolySolver<3>;
