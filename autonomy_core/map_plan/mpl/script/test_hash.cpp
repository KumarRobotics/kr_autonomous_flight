#include "../include/mpl_basis/control.h"
#include "../include/mpl_basis/waypoint.h"
#include "boost/unordered_map.hpp"

template <int Dim>
std::size_t hash_value(const Waypoint<Dim> &key) {
  // typedef Waypoint<Dim> argument_type;
  // typedef std::size_t result_type;
  std::size_t val = 0;
  for (int i = 0; i < Dim; i++) {
    if (key.use_pos) {
      int id = std::round(key.pos(i) / 0.1);
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
  }

  return val;
}

boost::unordered_map<Waypoint2D, float, boost::hash<Waypoint2D>> votes;

int main() {
  Waypoint2D w1;
  w1.pos = Vec2f(0, 0);
  w1.vel = Vec2f(0, 0);
  w1.acc = Vec2f(0, 0);
  w1.use_pos = true;
  w1.use_vel = true;
  w1.use_acc = true;

  Waypoint2D w2;
  w2.pos = Vec2f(0, 0);
  w2.vel = Vec2f(0, 0.1);
  w2.acc = Vec2f(0, 0);
  w2.use_pos = true;
  w2.use_vel = true;
  w2.use_acc = true;

  std::cout << "w1: " << hash_value(w1) << std::endl;
  std::cout << "w2: " << hash_value(w2) << std::endl;

  return 0;
}
