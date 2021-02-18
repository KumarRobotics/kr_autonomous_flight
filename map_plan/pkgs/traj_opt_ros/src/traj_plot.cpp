#include <X11/Xlib.h>
#include <matplotlib-cpp/matplotlibcpp.h>
#include <traj_opt_ros/traj_plot.h>

namespace plt = matplotlibcpp;

namespace traj_opt {
void TrajPlot::plot(const boost::shared_ptr<Trajectory> &traj, uint num_derr) {
  bool screen = XOpenDisplay(NULL);
  if (!screen) plt::backend("agg");
  // input checking
  int res = 1000;
  int dim = traj->getDim();
  double T = traj->getTotalTime();
  double dt = T / double(res - 1);
  std::vector<double> Ts(res, 0.0);
  for (int t = 0; t < res; t++) {
    Ts.at(t) = dt * double(t);
  }
  std::vector<std::string> derr_names = {"Pos", "Vel", "Acc", "Jrk",
                                         "Snp", "Krk", "Pop"};
  while (derr_names.size() <= num_derr)
    derr_names.push_back(std::to_string(derr_names.size()) + "th");

  // get plot object
  for (uint r = 0; r <= num_derr; r++) {
    plt::figure();
    std::vector<std::vector<double> > data(dim, std::vector<double>(res, 0.0));
    for (int t = 0; t < res; t++) {
      double tv = double(t) * dt;
      VecD val;
      traj->evaluate(tv, r, val);
      for (int d = 0; d < dim; d++) data.at(d).at(t) = val(d);
    }
    // do plotting
    for (int d = 0; d < dim; d++) {
      plt::subplot(dim, 1, d + 1);
      plt::plot(Ts, data.at(d));
      plt::title(derr_names.at(r));
    }
    if (!screen) plt::save(derr_names.at(r) + "_traj.png");
  }
  if (screen) plt::show(false);
}
}  // namespace traj_opt
