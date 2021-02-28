#ifndef TRAJ_OPT_BASIC_HOPF_HELPER_H_
#define TRAJ_OPT_BASIC_HOPF_HELPER_H_

#include <traj_opt_basic/types.h>

namespace traj_opt {

class ConvertHopf {
 public:
  static Quat getQuat(const Vec3 &xi, decimal_t yaw = 0.0, bool hover = true);
};

}  // namespace traj_opt

#endif  // TRAJ_OPT_BASIC_HOPF_HELPER_H_
