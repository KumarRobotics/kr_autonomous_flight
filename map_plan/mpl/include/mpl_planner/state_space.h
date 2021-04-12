/**
 * @file state_space.h
 * @brief state space class for graph search
 */
#pragma once

#include <boost/heap/d_ary_heap.hpp>  // boost::heap::d_ary_heap
#include <boost/unordered_map.hpp>    // std::unordered_map

#include "mpl_basis/waypoint.h"

namespace MPL {

/// Heap element comparison
template <typename state>
struct ComparePair {
  bool operator()(
      const std::pair<decimal_t, std::shared_ptr<state>> &p1,
      const std::pair<decimal_t, std::shared_ptr<state>> &p2) const {
    if (p1.first == p2.first) {
      // if equal compare gvals
      return std::min(p1.second->g, p1.second->rhs) >
             std::min(p2.second->g, p2.second->rhs);
    }
    return p1.first > p2.first;
  }
};

/// Define priority queue
template <typename state>
using priorityQueue =
    boost::heap::d_ary_heap<std::pair<decimal_t, std::shared_ptr<state>>,
                            boost::heap::mutable_<true>, boost::heap::arity<2>,
                            boost::heap::compare<ComparePair<state>>>;

/// Lattice of the graph in graph search
template <typename Coord>
struct State {
  /// state
  Coord coord;
  /// coordinates of successors
  vec_E<Coord> succ_coord;
  /// action id of successors
  std::vector<int> succ_action_id;
  /// action cost of successors
  std::vector<decimal_t> succ_action_cost;
  /// coordinates of predecessors
  vec_E<Coord> pred_coord;
  /// action id of predecessors
  std::vector<int> pred_action_id;
  /// action cost of predecessors
  std::vector<decimal_t> pred_action_cost;

  /// pointer to heap location
  typename priorityQueue<State<Coord>>::handle_type heapkey;

  // plan data
  /// start-to-state g value
  decimal_t g = std::numeric_limits<decimal_t>::infinity();
  /// rhs value based on g value
  decimal_t rhs = std::numeric_limits<decimal_t>::infinity();
  /// heuristic cost
  decimal_t h = std::numeric_limits<decimal_t>::infinity();
  /// label check if the state has been in the open set
  bool iterationopened = false;
  /// label check if the state has been closed
  bool iterationclosed = false;

  /// Simple constructor
  State(const Coord &coord) : coord(coord) {}
};

/// Declare StatePtr
template <typename Coord>
using StatePtr = std::shared_ptr<State<Coord>>;

/// Define hashmap type
template <typename Coord>
using hashMap =
    boost::unordered_map<Coord, StatePtr<Coord>, boost::hash<Coord>>;

/// State space
template <int Dim>
struct StateSpace {
  using Coord = Waypoint<Dim>;

  /// Priority queue, open set
  priorityQueue<State<Coord>> pq_;
  /// Hashmap, stores all the nodes
  hashMap<Coord> hm_;
  /// Heuristic weight, default as 1
  decimal_t eps_;
  /// Execution time for each primitive
  decimal_t dt_;
  /// The best trajectory from previous plan
  vec_E<StatePtr<Coord>> best_child_;
  /// Number of expansion iteration
  int expand_iteration_ = 0;
  /// Start time
  decimal_t start_t_{0};
  /// Start g value
  decimal_t start_g_{0};
  /// Start rhs value
  decimal_t start_rhs_{0};

  /// Simple constructor
  StateSpace(decimal_t eps = 1) : eps_(eps) {}

  decimal_t getInitTime() const;

  /// Update the node in the graph
  void updateNode(StatePtr<Coord> &currNode_ptr);

  /// Calculate the fval as min(rhs, g) + h
  decimal_t calculateKey(const StatePtr<Coord> &node);

  /// Internal function to check if the graph is valid
  void checkValidation(const hashMap<Coord> hm);
};

}  // namespace MPL
