/**
 * @file data_utils.h
 * @brief Provide a few widely used function for basic type
 */

#pragma once

#include "jps/data_type.h"

namespace JPS {

/// Template for transforming a vector
template <class T, class TF>
vec_E<T> transform_vec(const vec_E<T> &t, const TF &tf) {
  vec_E<T> new_t;
  for (const auto &it : t) new_t.push_back(tf * it);
  return new_t;
}

/// Template for calculating distance
template <class T>
decimal_t total_distance(const vec_E<T> &vs) {
  decimal_t dist = 0;
  for (unsigned int i = 1; i < vs.size(); i++)
    dist += (vs[i] - vs[i - 1]).norm();

  return dist;
}

}  // namespace JPS
