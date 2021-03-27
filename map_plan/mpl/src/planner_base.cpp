#include "mpl_planner/planner_base.h"

#include <iostream>

namespace MPL {

template <int Dim>
auto PlannerBase<Dim>::getValidPrimitives() const -> vec_E<PrimitiveD> {
  vec_E<PrimitiveD> prs;
  for (const auto &it : ss_ptr_->hm_) {
    if (it.second && !it.second->pred_coord.empty()) {
      for (unsigned int i = 0; i < it.second->pred_coord.size(); i++) {
        Coord key = it.second->pred_coord[i];
        // if(!ss_ptr_->hm_[key] ||
        // std::isinf(it.second->pred_action_cost[i]))
        if (std::isinf(it.second->pred_action_cost[i])) continue;
        Primitive<Dim> pr;
        env_->forward_action(ss_ptr_->hm_[key]->coord,
                             it.second->pred_action_id[i], pr);
        prs.push_back(pr);
      }
    }
  }

  if (planner_verbose_)
    printf("number of states in hm: %zu, number of valid prs: %zu\n",
           ss_ptr_->hm_.size(), prs.size());

  return prs;
}

template <int Dim>
auto PlannerBase<Dim>::getAllPrimitives() const -> vec_E<PrimitiveD> {
  vec_E<PrimitiveD> prs;
  for (const auto &it : ss_ptr_->hm_) {
    if (it.second && !it.second->pred_coord.empty()) {
      for (unsigned int i = 0; i < it.second->pred_coord.size(); i++) {
        Coord key = it.second->pred_coord[i];
        PrimitiveD pr;
        env_->forward_action(key, it.second->pred_action_id[i], pr);
        prs.push_back(pr);
      }
    }
  }

  if (planner_verbose_)
    printf("getAllPrimitives number of states in hm: %zu, number of prs: %zu\n",
           ss_ptr_->hm_.size(), prs.size());

  return prs;
}

template <int Dim>
vec_Vecf<Dim> PlannerBase<Dim>::getOpenSet() const {
  vec_Vecf<Dim> ps;
  for (const auto &it : ss_ptr_->pq_) {
    ps.push_back(it.second->coord.pos);
  }
  return ps;
}

template <int Dim>
vec_Vecf<Dim> PlannerBase<Dim>::getCloseSet() const {
  vec_Vecf<Dim> ps;
  for (const auto &it : ss_ptr_->hm_) {
    if (it.second && it.second->iterationclosed)
      ps.push_back(it.second->coord.pos);
  }
  return ps;
}

template <int Dim>
vec_Vecf<Dim> PlannerBase<Dim>::getNullSet() const {
  vec_Vecf<Dim> ps;
  for (const auto &it : ss_ptr_->hm_) {
    if (it.second && !it.second->iterationopened)
      ps.push_back(it.second->coord.pos);
  }
  return ps;
}

template <int Dim>
vec_Vecf<Dim> PlannerBase<Dim>::getStates(const Coord &state) const {
  vec_Vecf<Dim> ps;
  vec_Vecf<Dim> vels;
  vec_Vecf<Dim> accs;
  for (const auto &it : ss_ptr_->hm_) {
    if (it.second) {
      auto coord = it.second->coord;
      bool add = true;
      if (state.use_vel && (state.vel - coord.vel).norm() > 1e-3) add = false;
      if (state.use_acc && (state.acc - coord.acc).norm() > 1e-3) add = false;
      if (state.use_jrk && (state.jrk - coord.jrk).norm() > 1e-3) add = false;
      if (add) {
        // std::cout << "add pos: " << coord.pos.transpose() <<
        //  " vel: " << coord.vel.transpose() <<
        //  " acc: " << coord.acc.transpose() << std::endl;
        ps.push_back(coord.pos);
        bool new_vel = true;
        for (const auto &it : vels) {
          if ((it - coord.vel).norm() < 1e-3) {
            new_vel = false;
            break;
          }
        }

        if (new_vel) vels.push_back(coord.vel);
      }
    }
  }

  for (const auto &it : vels)
    std::cout << "vel: " << it.transpose() << std::endl;
  std::cout << "=========================" << std::endl;
  return ps;
}

template <int Dim>
void PlannerBase<Dim>::reset() {
  ss_ptr_ = nullptr;
  traj_ = TrajectoryD();
}

template <int Dim>
void PlannerBase<Dim>::setLPAstar(bool use_lpastar) {
  use_lpastar_ = use_lpastar;
  if (use_lpastar_)
    printf("[PlannerBase] use Lifelong Planning A*\n");
  else
    printf("[PlannerBase] use normal A*\n");
}

template <int Dim>
void PlannerBase<Dim>::setVmax(decimal_t v) {
  env_->set_v_max(v);
  if (planner_verbose_) printf("[PlannerBase] set v_max: %f\n", v);
}

template <int Dim>
void PlannerBase<Dim>::setAmax(decimal_t a) {
  env_->set_a_max(a);
  if (planner_verbose_) printf("[PlannerBase] set a_max: %f\n", a);
}

template <int Dim>
void PlannerBase<Dim>::setJmax(decimal_t j) {
  env_->set_j_max(j);
  if (planner_verbose_) printf("[PlannerBase] set j_max: %f\n", j);
}

template <int Dim>
void PlannerBase<Dim>::setYawmax(decimal_t yaw) {
  env_->set_yaw_max(yaw);
  if (planner_verbose_) printf("[PlannerBase] set yaw_max: %f\n", yaw);
}

template <int Dim>
void PlannerBase<Dim>::setTmax(decimal_t t) {
  env_->set_t_max(t);
  if (planner_verbose_) printf("[PlannerBase] set max time: %f\n", t);
}

template <int Dim>
void PlannerBase<Dim>::setDt(decimal_t dt) {
  env_->set_dt(dt);
  if (planner_verbose_) printf("[PlannerBase] set dt: %f\n", dt);
}

template <int Dim>
void PlannerBase<Dim>::setW(decimal_t w) {
  env_->set_w(w);
  if (planner_verbose_) printf("[PlannerBase] set w: %f\n", w);
}

template <int Dim>
void PlannerBase<Dim>::setWyaw(decimal_t w) {
  env_->set_wyaw(w);
  if (planner_verbose_) printf("[PlannerBase] set wyaw: %f\n", w);
}

template <int Dim>
void PlannerBase<Dim>::setEpsilon(decimal_t eps) {
  epsilon_ = eps;
  if (planner_verbose_) printf("[PlannerBase] set epsilon: %f\n", epsilon_);
}

template <int Dim>
void PlannerBase<Dim>::setHeurIgnoreDynamics(bool ignore) {
  env_->set_heur_ignore_dynamics(ignore);
  if (planner_verbose_)
    printf("[PlannerBase] set heur_ignore_dynamics: %d\n", ignore);
}

template <int Dim>
void PlannerBase<Dim>::setMaxNum(int num) {
  max_num_ = num;
  if (planner_verbose_) printf("[PlannerBase] set max num: %d\n", max_num_);
}

template <int Dim>
void PlannerBase<Dim>::setU(const vec_E<VecDf> &U) {
  env_->set_u(U);
}

template <int Dim>
void PlannerBase<Dim>::setTol(decimal_t tol_pos, decimal_t tol_vel,
                              decimal_t tol_acc) {
  env_->set_tol_pos(tol_pos);
  env_->set_tol_vel(tol_vel);
  env_->set_tol_acc(tol_acc);
  if (planner_verbose_) {
    printf("[PlannerBase] set tol_pos: %f\n", tol_pos);
    printf("[PlannerBase] set tol_vel: %f\n", tol_vel);
    printf("[PlannerBase] set tol_acc: %f\n", tol_acc);
  }
}

template <int Dim>
bool PlannerBase<Dim>::plan(const PlannerBase::Coord &start,
                            const PlannerBase::Coord &goal) {
  if (planner_verbose_) {
    start.print("Start:");
    goal.print("Goal:");
    env_->info();
  }

  if (!env_->is_free(start.pos)) {
    printf(ANSI_COLOR_RED "[PlannerBase] start is not free!" ANSI_COLOR_RESET
                          "\n");
    return false;
  }

  std::unique_ptr<GraphSearch<Dim>> planner_ptr(
      new GraphSearch<Dim>(planner_verbose_));

  // If use A*, reset the state space
  if (!use_lpastar_)
    ss_ptr_.reset(new MPL::StateSpace<Dim>(epsilon_));
  else {
    // If use LPA*, reset the state space only at the initial planning
    if (!initialized()) {
      if (planner_verbose_)
        printf(ANSI_COLOR_CYAN
               "[PlannerBase] reset planner state space!" ANSI_COLOR_RESET
               "\n");
      ss_ptr_.reset(new MPL::StateSpace<Dim>(epsilon_));
    }
  }

  env_->set_goal(goal);

  env_->expanded_nodes_.clear();
  env_->expanded_edges_.clear();

  ss_ptr_->dt_ = env_->get_dt();
  if (use_lpastar_)
    traj_cost_ = planner_ptr->LPAstar(start, env_, ss_ptr_, traj_, max_num_);
  else
    traj_cost_ = planner_ptr->Astar(start, env_, ss_ptr_, traj_, max_num_);

  if (std::isinf(traj_cost_)) {
    if (planner_verbose_)
      printf(ANSI_COLOR_RED "[PlannerBase] Cannot find a traj!" ANSI_COLOR_RESET
                            "\n");
    return false;
  }

  return true;
}

template class PlannerBase<2>;
template class PlannerBase<3>;

}  // namespace MPL
