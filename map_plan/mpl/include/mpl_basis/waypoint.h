/**
 * @file waypoint.h
 * @brief Waypoint classes
 */

#pragma once

#include <boost/functional/hash.hpp>

#include "mpl_basis/control.h"
#include "mpl_basis/data_type.h"

namespace MPL {

/**
 * @brief Waypoint base class
 *
 * State includes position, velocity, acceleration and jerk in \f$R^n\f$, where
 * the dimension \f$n\f$ can be either 2 or 3. Yaw is contained by default. The
 * anonymous union is used to set control flag.
 */
template <int Dim>
struct Waypoint {
  /// Empty constructor
  Waypoint() : control(MPL::NONE) {}
  /**
   * @brief Simple constructor
   * @param c control value
   */
  Waypoint(MPL::Control c) : control(c) {}

  Vecf<Dim> pos{Vecf<Dim>::Zero()};  ///< position in \f$R^{n}\f$
  Vecf<Dim> vel{Vecf<Dim>::Zero()};  ///< velocity in \f$R^{n}\f$
  Vecf<Dim> acc{Vecf<Dim>::Zero()};  ///< acceleration in \f$R^{n}\f$
  Vecf<Dim> jrk{Vecf<Dim>::Zero()};  ///< jerk in \f$R^{n}\f$
  decimal_t yaw{0};                  ///< yaw
  decimal_t t{0};  ///< time when reaching this waypoint in graph search

  /**
   * @brief Control flag
   *
   * Anonymous union type that contains 5 bits as xxxxx,
   * each bit from right to left is assigned to `use_pos`, `use_vel`, `use_acc`,
   * `use_jrk` and `use_yaw`.
   */
  union {
    struct {
      bool use_pos : 1;  ///< If true, pos will be used in primitive generation
      bool use_vel : 1;  ///< If true, vel will be used in primitive generation
      bool use_acc : 1;  ///< If true, acc will be used in primitive generation
      bool use_jrk : 1;  ///< If true, jrk will be used in primitive generation
      bool use_yaw : 1;  ///< If true, yaw will be used in primitive generation
    };
    MPL::Control control : 5;  ///< Control value
  };

  bool enable_t{false};  ///< if enabled, use \f$t\f$ when calculating
                         ///< hash_value

  /// Print all attributes
  void print(std::string str = "") const;
};

/// Generate hash value for Waypoint class
template <int Dim>
std::size_t hash_value(const Waypoint<Dim>& key) {
  std::size_t val = 0;
  for (int i = 0; i < Dim; i++) {
    if (key.use_pos) {
      int id = std::round(key.pos(i) / 0.01);
      boost::hash_combine(val, id);
    }
    if (key.use_vel) {
      int id = std::round(key.vel(i) / 0.1);
      boost::hash_combine(val, id);
    }
    if (key.use_acc) {
      int id = std::round(key.acc(i) / 0.1);
      boost::hash_combine(val, id);
    }
    if (key.use_jrk) {
      int id = std::round(key.jrk(i) / 0.1);
      boost::hash_combine(val, id);
    }
  }

  if (key.use_yaw) {
    int id = std::round(key.yaw / 0.1);
    boost::hash_combine(val, id);
  }

  if (key.enable_t) {
    int id = std::round(key.t / 0.1);
    boost::hash_combine(val, id);
  }

  return val;
}

/**
 * @brief  Check if two waypoints are equivalent
 *
 * using the hash value
 */
template <int Dim>
bool operator==(const Waypoint<Dim>& l, const Waypoint<Dim>& r) {
  return hash_value(l) == hash_value(r);
}

/// Check if two waypoints are not equivalent
template <int Dim>
bool operator!=(const Waypoint<Dim>& l, const Waypoint<Dim>& r) {
  return hash_value(l) != hash_value(r);
}

/// Waypoint for 2D
typedef Waypoint<2> Waypoint2D;

/// Waypoint for 3D
typedef Waypoint<3> Waypoint3D;

}  // namespace MPL
