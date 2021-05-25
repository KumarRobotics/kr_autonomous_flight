#include "mpl_planner/state_space.h"

#include <iostream>

namespace MPL {

template <int Dim>
decimal_t StateSpace<Dim>::getInitTime() const {
  if (best_child_.empty()) {
    return 0;
  } else {
    return best_child_.front()->coord.t;
  }
}

template <int Dim>
void StateSpace<Dim>::updateNode(StatePtr<Coord> &currNode_ptr) {
  // if currNode is not start, update its rhs
  // start rhs is assumed to be 0
  if (currNode_ptr->rhs != start_rhs_) {
    currNode_ptr->rhs = std::numeric_limits<decimal_t>::infinity();
    for (size_t i = 0; i < currNode_ptr->pred_coord.size(); i++) {
      Coord pred_key = currNode_ptr->pred_coord[i];
      if (currNode_ptr->rhs >
          hm_[pred_key]->g + currNode_ptr->pred_action_cost[i])
        currNode_ptr->rhs =
            hm_[pred_key]->g + currNode_ptr->pred_action_cost[i];
    }
  }

  // if currNode is in openset, remove it
  if (currNode_ptr->iterationopened && !currNode_ptr->iterationclosed) {
    pq_.erase(currNode_ptr->heapkey);
    currNode_ptr->iterationclosed = true;
  }

  // if currNode's g value is not equal to its rhs, put it into openset
  if (currNode_ptr->g != currNode_ptr->rhs) {
    decimal_t fval = calculateKey(currNode_ptr);
    currNode_ptr->heapkey = pq_.push(std::make_pair(fval, currNode_ptr));
    currNode_ptr->iterationopened = true;
    currNode_ptr->iterationclosed = false;
  }
}

template <int Dim>
decimal_t StateSpace<Dim>::calculateKey(const StatePtr<Coord> &node) {
  return std::min(node->g, node->rhs) + eps_ * node->h;
}

template <int Dim>
void StateSpace<Dim>::checkValidation(const hashMap<Coord> hm) {
  //****** Check if there is null element in succ graph
  for (const auto &it : hm) {
    if (!it.second) {
      std::cout << "error!!! detect null element!" << std::endl;
      it.first.print("Not exist!");
    } else {
      bool null_succ = false;
      for (size_t i = 0; i < it.second->succ_coord.size(); i++) {
        if (hm.find(it.second->succ_coord[i]) == hm.end()) {
          std::cout << "error!!! detect null succ !" << std::endl;
          // it.second->succ_coord[i].print("Not exist!");
          null_succ = true;
        }
      }
      if (null_succ) {
        it.first.print("From this pred:");
        printf("rhs: %f, g: %f, open: %d, closed: %d\n\n\n", it.second->rhs,
               it.second->g, it.second->iterationopened,
               it.second->iterationclosed);
      }
    }
  }
  return;

  //****** Check rhs and g value of close set
  printf("Check rhs and g value of closeset\n");
  int close_cnt = 0;
  for (const auto &it : hm_) {
    if (it.second->iterationopened && it.second->iterationclosed) {
      printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      close_cnt++;
    }
  }

  // Check rhs and g value of open set
  printf("Check rhs and g value of openset\n");
  int open_cnt = 0;
  for (const auto &it : hm_) {
    if (it.second->iterationopened && !it.second->iterationclosed) {
      printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      open_cnt++;
    }
  }

  // Check rhs and g value of null set
  printf("Check rhs and g value of nullset\n");
  int null_cnt = 0;
  for (const auto &it : hm_) {
    if (!it.second->iterationopened) {
      printf("g: %f, rhs: %f\n", it.second->g, it.second->rhs);
      null_cnt++;
    }
  }

  printf("hm: [%zu], open: [%d], closed: [%d], null: [%d]\n", hm_.size(),
         open_cnt, close_cnt, null_cnt);
}

template struct StateSpace<2>;
template struct StateSpace<3>;

}  // namespace MPL
