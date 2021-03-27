#include "mpl_planner/map_planner.h"

namespace MPL {

template <int Dim>
MapPlanner<Dim>::MapPlanner(bool verbose) {
  this->planner_verbose_ = verbose;
  if (this->planner_verbose_)
    printf(ANSI_COLOR_CYAN
           "[MapPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

template <int Dim>
void MapPlanner<Dim>::setMapUtil(
    const std::shared_ptr<MapUtil<Dim>> &map_util) {
  this->env_.reset(new MPL::EnvMap<Dim>(map_util));
  map_util_ = map_util;
}

template <int Dim>
void MapPlanner<Dim>::setPotentialRadius(const Vecf<Dim> &radius) {
  potential_radius_ = radius;
}

template <int Dim>
void MapPlanner<Dim>::setPotentialMapRange(const Vecf<Dim> &range) {
  potential_map_range_ = range;
}

template <int Dim>
void MapPlanner<Dim>::setPotentialWeight(decimal_t w) {
  this->env_->set_potential_weight(w);
}

template <int Dim>
void MapPlanner<Dim>::setGradientWeight(decimal_t w) {
  this->env_->set_gradient_weight(w);
}

template <int Dim>
void MapPlanner<Dim>::setSearchRadius(const Vecf<Dim> &search_radius) {
  search_radius_ = search_radius;
}

template <int Dim>
void MapPlanner<Dim>::setSearchRegion(const vec_Vecf<Dim> &path, bool dense) {
  // create cells along path
  vec_Veci<Dim> ps;
  if (!dense) {
    for (unsigned int i = 1; i < path.size(); i++) {
      auto pns = map_util_->rayTrace(path[i - 1], path[i]);
      ps.insert(ps.end(), pns.begin(), pns.end());
      ps.push_back(map_util_->floatToInt(path[i]));
    }
  } else {
    for (const auto &pt : path) ps.push_back(map_util_->floatToInt(pt));
  }

  // create mask
  Veci<Dim> rn;
  for (int i = 0; i < Dim; i++)
    rn(i) = std::ceil(search_radius_(i) / map_util_->getRes());
  vec_Veci<Dim> ns;
  Veci<Dim> n;
  if (Dim == 2) {
    for (n(0) = -rn(0); n(0) <= rn(0); n(0)++)
      for (n(1) = -rn(1); n(1) <= rn(1); n(1)++) ns.push_back(n);
  } else {
    for (n(0) = -rn(0); n(0) <= rn(0); n(0)++)
      for (n(1) = -rn(1); n(1) <= rn(1); n(1)++)
        for (n(2) = -rn(2); n(2) <= rn(2); n(2)++) ns.push_back(n);
  }

  // create in_region map
  const auto dim = map_util_->getDim();
  std::vector<bool> in_region;
  if (Dim == 2)
    in_region.resize(dim(0) * dim(1), false);
  else
    in_region.resize(dim(0) * dim(1) * dim(2), false);

  for (const auto &it : ps) {
    for (const auto &n : ns) {
      Veci<Dim> pn = it + n;
      if (map_util_->isOutside(pn)) continue;
      int idx = map_util_->getIndex(pn);
      if (!in_region[idx]) {
        in_region[idx] = true;
      }
    }
  }

  this->env_->set_search_region(in_region);
  if (this->planner_verbose_) printf("[MapPlanner] set search region\n");
}

template <int Dim>
vec_Vecf<Dim> MapPlanner<Dim>::getSearchRegion() const {
  const auto in_region = this->env_->get_search_region();
  vec_Vecf<Dim> pts;
  const auto dim = map_util_->getDim();
  Veci<Dim> n;
  if (Dim == 2) {
    for (n(0) = 0; n(0) < dim(0); n(0)++) {
      for (n(1) = 0; n(1) < dim(1); n(1)++) {
        if (in_region[map_util_->getIndex(n)])
          pts.push_back(map_util_->intToFloat(n));
      }
    }
  } else {
    for (n(0) = 0; n(0) < dim(0); n(0)++) {
      for (n(1) = 0; n(1) < dim(1); n(1)++) {
        for (n(2) = 0; n(2) < dim(2); n(2)++) {
          if (in_region[map_util_->getIndex(n)])
            pts.push_back(map_util_->intToFloat(n));
        }
      }
    }
  }

  return pts;
}

template <int Dim>
vec_Vecf<Dim> MapPlanner<Dim>::getLinkedNodes() const {
  lhm_.clear();
  vec_Vecf<Dim> linked_pts;
  for (const auto &it : this->ss_ptr_->hm_) {
    if (!it.second) continue;
    // check pred array
    for (unsigned int i = 0; i < it.second->pred_coord.size(); i++) {
      auto key = it.second->pred_coord[i];
      Primitive<Dim> pr;
      this->env_->forward_action(this->ss_ptr_->hm_[key]->coord,
                                 it.second->pred_action_id[i], pr);
      decimal_t max_v = 0;
      if (Dim == 2)
        max_v = std::max(pr.max_vel(0), pr.max_vel(1));
      else if (Dim == 3)
        max_v = std::max(std::max(pr.max_vel(0), pr.max_vel(1)), pr.max_vel(2));
      int n = 1.0 * std::ceil(max_v * pr.t() / map_util_->getRes());
      int prev_id = -1;
      vec_E<Waypoint<Dim>> ws = pr.sample(n);
      for (const auto &w : ws) {
        int id = map_util_->getIndex(map_util_->floatToInt(w.pos));
        if (id != prev_id) {
          linked_pts.push_back(
              map_util_->intToFloat(map_util_->floatToInt(w.pos)));
          lhm_[id].push_back(std::make_pair(it.second->coord, i));
          prev_id = id;
        }
      }
    }
  }

  return linked_pts;
}

template <int Dim>
void MapPlanner<Dim>::updateBlockedNodes(const vec_Veci<Dim> &blocked_pns) {
  std::vector<std::pair<Waypoint<Dim>, int>> blocked_nodes;
  for (const auto &it : blocked_pns) {
    int id = map_util_->getIndex(it);
    auto search = lhm_.find(id);
    if (search != lhm_.end()) {
      for (const auto &node : lhm_[id]) blocked_nodes.push_back(node);
    }
  }

  this->ss_ptr_->increaseCost(blocked_nodes);
}

template <int Dim>
void MapPlanner<Dim>::updateClearedNodes(const vec_Veci<Dim> &cleared_pns) {
  std::vector<std::pair<Waypoint<Dim>, int>> cleared_nodes;
  for (const auto &it : cleared_pns) {
    int id = map_util_->getIndex(it);
    auto search = lhm_.find(id);
    if (search != lhm_.end()) {
      for (const auto &node : lhm_[id]) cleared_nodes.push_back(node);
    }
  }

  this->ss_ptr_->decreaseCost(cleared_nodes, this->env_);
}

template <int Dim>
vec_Vec3f MapPlanner<Dim>::getPotentialCloud(decimal_t h_max) {
  const auto data = map_util_->getMap();
  const auto dim = map_util_->getDim();
  const decimal_t ratio = h_max / H_MAX;
  vec_Vec3f ps;

  Veci<Dim> n;
  if (Dim == 2) {
    for (n(0) = 0; n(0) < dim(0); n(0)++) {
      for (n(1) = 0; n(1) < dim(1); n(1)++) {
        int idx = map_util_->getIndex(n);
        if (data[idx] > 0) {
          decimal_t h = (decimal_t)data[idx] * ratio;
          Vecf<Dim> pt2d = map_util_->intToFloat(n);
          ps.push_back(Vec3f(pt2d(0), pt2d(1), h));
        }
      }
    }
  } else {
    for (n(0) = 0; n(0) < dim(0); n(0)++) {
      for (n(1) = 0; n(1) < dim(1); n(1)++) {
        for (n(2) = 0; n(2) < dim(2); n(2)++) {
          int idx = map_util_->getIndex(n);
          if (data[idx] > 0) {
            auto pt = map_util_->intToFloat(n);
            Vec3f p;
            p << pt(0), pt(1), pt(2);
            ps.push_back(p);
          }
        }
      }
    }
  }
  return ps;
}

template <int Dim>
vec_Vec3f MapPlanner<Dim>::getGradientCloud(decimal_t h_max, int i) {
  const auto data = map_util_->getMap();
  const auto dim = map_util_->getDim();
  const decimal_t ratio = h_max / H_MAX;
  vec_Vec3f ps;
  if (gradient_map_.empty()) return ps;
  Veci<Dim> n;
  if (Dim == 2) {
    for (n(0) = 0; n(0) < dim(0); n(0)++) {
      for (n(1) = 0; n(1) < dim(1); n(1)++) {
        int idx = map_util_->getIndex(n);
        if (gradient_map_[idx].norm() > 0) {
          Vecf<Dim> pt2d = map_util_->intToFloat(n);
          ps.push_back(Vec3f(pt2d(0), pt2d(1), gradient_map_[idx](i) * ratio));
        }
      }
    }
  }
  return ps;
}

template <int Dim>
vec_E<Vecf<Dim>> MapPlanner<Dim>::calculateGradient(const Veci<Dim> &coord1,
                                                    const Veci<Dim> &coord2) {
  const auto dim = map_util_->getDim();
  const auto cmap = map_util_->getMap();

  int rn = std::ceil(potential_radius_(0) / map_util_->getRes());

  vec_E<Vecf<Dim>> g(dim(0) * dim(1), Vecf<Dim>::Zero());
  Veci<Dim> n = Veci<Dim>::Zero();
  for (n(0) = coord1(0) - rn; n(0) < coord2(0) + rn; n(0)++) {
    for (n(1) = coord1(1) - rn; n(1) < coord2(1) + rn; n(1)++) {
      if (map_util_->isOutside(n)) continue;
      int idx = map_util_->getIndex(n);
      auto nx1 = n;
      nx1(0) -= 1;
      auto nx2 = n;
      nx2(0) += 1;
      int idx_x1 = map_util_->isOutside(nx1) ? -1 : map_util_->getIndex(nx1);
      int idx_x2 = map_util_->isOutside(nx2) ? -1 : map_util_->getIndex(nx2);

      if (idx_x1 >= 0 && idx_x2 >= 0)
        g[idx](0) = 0.5 * (cmap[idx_x2] - cmap[idx_x1]);

      auto ny1 = n;
      ny1(1) -= 1;
      auto ny2 = n;
      ny2(1) += 1;
      int idx_y1 = map_util_->isOutside(ny1) ? -1 : map_util_->getIndex(ny1);
      int idx_y2 = map_util_->isOutside(ny2) ? -1 : map_util_->getIndex(ny2);

      if (idx_y1 >= 0 && idx_y2 >= 0)
        g[idx](1) = 0.5 * (cmap[idx_y2] - cmap[idx_y1]);
    }
  }

  return g;
}

template <int Dim>
void MapPlanner<Dim>::createMask() {
  potential_mask_.clear();
  // create mask
  decimal_t res = map_util_->getRes();
  decimal_t h_max = H_MAX;
  int rn = std::ceil(potential_radius_(0) / res);
  // printf("rn: %d\n", rn);
  // printf("hn: %d\n", hn);
  Veci<Dim> n;
  if (Dim == 2) {
    for (n(0) = -rn; n(0) <= rn; n(0)++) {
      for (n(1) = -rn; n(1) <= rn; n(1)++) {
        if (std::hypot(n(0), n(1)) > rn) continue;
        decimal_t h =
            h_max *
            std::pow((1 - (decimal_t)std::hypot(n(0), n(1)) / rn), pow_);
        if (h > 1e-3) potential_mask_.push_back(std::make_pair(n, (int8_t)h));
      }
    }
  } else {
    int hn = std::ceil(potential_radius_(2) / res);
    for (n(0) = -rn; n(0) <= rn; n(0)++) {
      for (n(1) = -rn; n(1) <= rn; n(1)++) {
        for (n(2) = -hn; n(2) <= hn; n(2)++) {
          if (std::hypot(n(0), n(1)) > rn) continue;
          decimal_t h =
              h_max * std::pow((1 - (decimal_t)std::hypot(n(0), n(1)) / rn) *
                                   (1 - (decimal_t)std::abs(n(2)) / hn),
                               pow_);
          if (h > 1e-3) potential_mask_.push_back(std::make_pair(n, (int8_t)h));
        }
      }
    }
  }
}

template <int Dim>
void MapPlanner<Dim>::updatePotentialMap(const Vecf<Dim> &pos) {
  createMask();
  // compute a 2D local potential map
  const auto dim = map_util_->getDim();
  Veci<Dim> coord1 = Veci<Dim>::Zero();
  Veci<Dim> coord2 = dim;
  if (potential_map_range_.norm() > 0) {
    coord1 = map_util_->floatToInt(pos - potential_map_range_);
    coord2 = map_util_->floatToInt(pos + potential_map_range_);
    for (int i = 0; i < Dim; i++) {
      if (coord1(i) < 0)
        coord1(i) = 0;
      else if (coord1(i) >= dim(i))
        coord1(i) = dim(i) - 1;

      if (coord2(i) < 0)
        coord2(i) = 0;
      else if (coord2(i) >= dim(i))
        coord2(i) = dim(i) - 1;
    }
  }

  std::vector<int8_t> map = map_util_->getMap();
  auto dmap = map;

  Veci<Dim> n;
  if (Dim == 2) {
    for (n(0) = coord1(0); n(0) < coord2(0); n(0)++) {
      for (n(1) = coord1(1); n(1) < coord2(1); n(1)++) {
        int idx = map_util_->getIndex(n);
        if (map[idx] > 0) {
          dmap[idx] = H_MAX;
          for (const auto &it : potential_mask_) {
            const Veci<Dim> new_n = n + it.first;

            if (!map_util_->isOutside(new_n)) {
              const int new_idx = map_util_->getIndex(new_n);
              dmap[new_idx] = std::max(dmap[new_idx], it.second);
            }
          }
        }
      }
    }
  } else {
    for (n(0) = coord1(0); n(0) < coord2(0); n(0)++) {
      for (n(1) = coord1(1); n(1) < coord2(1); n(1)++) {
        for (n(2) = coord1(2); n(2) < coord2(2); n(2)++) {
          int idx = map_util_->getIndex(n);
          if (map[idx] > 0) {
            dmap[idx] = H_MAX;
            for (const auto &it : potential_mask_) {
              const Veci<Dim> new_n = n + it.first;

              if (!map_util_->isOutside(new_n)) {
                const int new_idx = map_util_->getIndex(new_n);
                dmap[new_idx] = std::max(dmap[new_idx], it.second);
              }
            }
          }
        }
      }
    }
  }

  map_util_->setMap(map_util_->getOrigin(), dim, dmap, map_util_->getRes());
  this->env_->set_potential_map(map_util_->getMap());
  // gradient_map_ = calculateGradient(coord1, coord2);
  // this->ENV_->set_gradient_map(gradient_map_);
}

template <int Dim>
bool MapPlanner<Dim>::iterativePlan(const Waypoint<Dim> &start,
                                    const Waypoint<Dim> &goal,
                                    const Trajectory<Dim> &raw_traj,
                                    int max_num) {
  bool verbose = this->planner_verbose_;
  this->planner_verbose_ = false;
  this->traj_ = raw_traj;
  double prev_traj_cost_ = 0;

  int cnt = 0;
  while (cnt < max_num) {
    cnt++;
    // Create a path from planned traj
    const auto ws = this->traj_.getWaypoints();
    vec_Vecf<Dim> path;
    for (const auto &w : ws) path.push_back(w.pos);
    setSearchRegion(path, false);

    if (!this->plan(start, goal)) {
      if (verbose) printf("[MapPlanner] fails the [%d] plan!\n", cnt);
      this->planner_verbose_ = verbose;
      return false;
    }

    if (prev_traj_cost_ == this->traj_cost_) {
      if (verbose)
        printf(
            "[MapPlanner] Converged after %d iterations! Trajectory cost: %f\n",
            cnt, this->traj_cost_);
      break;
    } else
      prev_traj_cost_ = this->traj_cost_;
  }

  this->planner_verbose_ = verbose;
  return true;
}

template class MapPlanner<2>;
template class MapPlanner<3>;

}  // namespace MPL
