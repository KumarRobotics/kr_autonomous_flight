/**
 * @file waypoint.h
 * @brief Waypoint classes
 */

#ifndef MPL_WAYPOINT_H
#define MPL_WAYPOINT_H
#include <bitset>
#include <boost/functional/hash.hpp>
#include <iostream>

#include "control.h"
#include "data_type.h"

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
  Waypoint() : control(Control::NONE) {}
  /**
   * @brief Simple constructor
   * @param c control value
   */
  Waypoint(Control::Control c) : control(c) {}

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
    Control::Control control : 5;  ///< Control value
  };

  bool enable_t{false};  ///< if enabled, use \f$t\f$ when calculating
                         ///< hash_value

  /// Print all attributes
  void print(std::string str = "") const {
    if (!str.empty()) std::cout << str << std::endl;
    if (use_pos) std::cout << "pos: " << pos.transpose() << std::endl;
    if (use_vel) std::cout << "vel: " << vel.transpose() << std::endl;
    if (use_acc) std::cout << "acc: " << acc.transpose() << std::endl;
    if (use_jrk) std::cout << "jrk: " << jrk.transpose() << std::endl;
    if (use_yaw) std::cout << "yaw: " << yaw << std::endl;
    if (enable_t) std::cout << " t: " << t << std::endl;

    if (control == Control::VEL)
      std::cout << "use vel!" << std::endl;
    else if (control == Control::ACC)
      std::cout << "use acc!" << std::endl;
    else if (control == Control::JRK)
      std::cout << "use jrk!" << std::endl;
    else if (control == Control::SNP)
      std::cout << "use snp!" << std::endl;
    else if (control == Control::VELxYAW)
      std::cout << "use vel & yaw!" << std::endl;
    else if (control == Control::ACCxYAW)
      std::cout << "use acc & yaw!" << std::endl;
    else if (control == Control::JRKxYAW)
      std::cout << "use jrk & yaw!" << std::endl;
    else if (control == Control::SNPxYAW)
      std::cout << "use snp & yaw!" << std::endl;
    else
      std::cout << "use null!" << std::endl;
  }
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

#endif
