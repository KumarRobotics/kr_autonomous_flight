#include "mpl_basis/waypoint.h"

#include <iostream>

namespace MPL {

template <int Dim>
void Waypoint<Dim>::print(std::string str) const {
  if (!str.empty()) std::cout << str << std::endl;
  if (use_pos) std::cout << "pos: " << pos.transpose() << std::endl;
  if (use_vel) std::cout << "vel: " << vel.transpose() << std::endl;
  if (use_acc) std::cout << "acc: " << acc.transpose() << std::endl;
  if (use_jrk) std::cout << "jrk: " << jrk.transpose() << std::endl;
  if (use_yaw) std::cout << "yaw: " << yaw << std::endl;
  if (enable_t) std::cout << " t: " << t << std::endl;

  if (control == MPL::VEL)
    std::cout << "use vel!" << std::endl;
  else if (control == MPL::ACC)
    std::cout << "use acc!" << std::endl;
  else if (control == MPL::JRK)
    std::cout << "use jrk!" << std::endl;
  else if (control == MPL::SNP)
    std::cout << "use snp!" << std::endl;
  else if (control == MPL::VELxYAW)
    std::cout << "use vel & yaw!" << std::endl;
  else if (control == MPL::ACCxYAW)
    std::cout << "use acc & yaw!" << std::endl;
  else if (control == MPL::JRKxYAW)
    std::cout << "use jrk & yaw!" << std::endl;
  else if (control == MPL::SNPxYAW)
    std::cout << "use snp & yaw!" << std::endl;
  else
    std::cout << "use null!" << std::endl;
}

template struct Waypoint<2>;
template struct Waypoint<3>;

}  // namespace MPL
