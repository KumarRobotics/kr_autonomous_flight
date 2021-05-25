/**
 * @file graph_search.h
 * @brief backend of graph search, implemetation of A* and Lifelong Planning A*
 */

#pragma once

#include "mpl_basis/trajectory.h"
#include "mpl_planner/env_base.h"
#include "mpl_planner/state_space.h"

namespace MPL {

/**
 * @brief GraphSearch class
 *
 * Implement A* and Lifelong Planning A*
 */
template <int Dim>
class GraphSearch {
 public:
  using Coord = Waypoint<Dim>;
  using EnvBaseD = EnvBase<Dim>;
  using TrajectoryD = Trajectory<Dim>;
  using StateSpaceD = StateSpace<Dim>;

  /**
   * @brief Simple empty constructor
   *
   * @param verbose enable print out debug infos, default is set to False
   */
  GraphSearch(bool verbose = false) : verbose_(verbose){};

  /**
   * @brief Astar graph search
   *
   * @param start_coord start state
   * @param ENV object of `env_base' class
   * @param ss_ptr workspace input
   * @param traj output trajectory
   * @param max_expand max number of expanded states, default value is -1 which
   * means there is no limitation
   */
  decimal_t Astar(const Coord &start_coord,
                  const std::shared_ptr<EnvBaseD> &env,
                  std::shared_ptr<StateSpaceD> &ss_ptr, TrajectoryD &traj,
                  int max_expand = -1);

  /**
   * @brief Lifelong Planning Astar graph search
   *
   * @param start_coord start state
   * @param ENV object of `env_base' class
   * @param ss_ptr workspace input
   * @param traj output trajectory
   * @param max_expand max number of expanded states, default value is -1 which
   * means there is no limitation
   */
  decimal_t LPAstar(const Coord &start_coord,
                    const std::shared_ptr<EnvBaseD> &env,
                    std::shared_ptr<StateSpaceD> &ss_ptr, TrajectoryD &traj,
                    int max_expand = -1);

 private:
  /// Recover trajectory
  bool recoverTraj(StatePtr<Coord> currNode_ptr,
                   std::shared_ptr<StateSpaceD> ss_ptr,
                   const std::shared_ptr<EnvBaseD> &env, const Coord &start_key,
                   TrajectoryD &traj);

  /// Verbose flag
  bool verbose_ = false;
};
}  // namespace MPL
