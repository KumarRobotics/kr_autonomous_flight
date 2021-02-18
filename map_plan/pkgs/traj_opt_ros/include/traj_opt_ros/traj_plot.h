#ifndef TRAJ_PLOT_H
#define TRAJ_PLOT_H

#include <traj_opt_basic/trajectory.h>

namespace traj_opt {

struct TrajPlot {
  static void plot(const boost::shared_ptr<Trajectory> &traj,
                   uint num_derr = 3);
};

}  // namespace traj_opt

#endif  // TRAJ_PLOT_H
