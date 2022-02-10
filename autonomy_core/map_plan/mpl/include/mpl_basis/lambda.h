/**
 * @file lambda.h
 * @brief Lambda class
 */

#pragma once

#include <cmath>

#include "mpl_basis/data_type.h"
namespace MPL {

/**
 * @brief Used for scaling, ignored for most case
 */
struct VirtualPoint {
  decimal_t p;
  decimal_t v;
  decimal_t t;
};

/**
 * @brief polynomial between two virtual points
 */
class LambdaSeg {
 public:
  LambdaSeg() = default;
  LambdaSeg(const VirtualPoint& v1, const VirtualPoint& v2);

  VirtualPoint evaluate(decimal_t tau) const;
  decimal_t getT(decimal_t t) const;

  Vec4f a;  ///< a3, a2, a1, a0
  decimal_t ti;
  decimal_t tf;
  decimal_t dT;
};

/**
 * @brief piecewise polynomial for scaling trajectory
 *
 */
class Lambda {
 public:
  Lambda() = default;
  Lambda(const std::vector<VirtualPoint>& vs);

  bool exist() const { return !segs.empty(); }
  std::vector<VirtualPoint> sample(int N);
  vec_Vec3f sampleT(int N);
  VirtualPoint evaluate(decimal_t tau) const;
  decimal_t getT(decimal_t tau) const;
  decimal_t getTau(decimal_t t) const;
  decimal_t getTotalTime() const;

  std::vector<LambdaSeg> segs;
};

}  // namespace MPL
