#include <mpl_traj_solver/traj_solver.h>

#include "opencv_drawing.hpp"

int main(int argc, char **argv) {
  // Build a empty map
  const Vec2f origin(-1, -1);
  const Vec2i dim(800, 400);
  const double res = 0.01;
  std::vector<signed char> map(dim(0) * dim(1), 0);

  std::shared_ptr<MPL::OccMapUtil> map_util;
  map_util.reset(new MPL::OccMapUtil);
  map_util->setMap(origin, dim, map, res);

  // Draw path and trajectories
  vec_Vec2f path;
  path.push_back(Vec2f(0, 0));
  path.push_back(Vec2f(1, 0));
  path.push_back(Vec2f(2, 1));
  path.push_back(Vec2f(5, 1));

  // Plotting
  std::string file_name("test_traj_solver");
  OpenCVDrawing opencv_drawing(map_util);

  // Min Vel Traj
  {
    TrajSolver2D traj_solver(Control::VEL);
    traj_solver.setPath(path);
    traj_solver.setV(1);  // set velocity for time allocation
    opencv_drawing.drawTraj(traj_solver.solve(), red, 2);
    opencv_drawing.drawText("min velocity trajectory", dim - Vec2i(350, 80),
                            0.7, red);
  }
  // Min Acc Traj
  {
    TrajSolver2D traj_solver(Control::ACC);
    traj_solver.setPath(path);
    traj_solver.setV(1);  // set velocity for time allocation
    opencv_drawing.drawTraj(traj_solver.solve(), green, 2);
    opencv_drawing.drawText("min acceleration trajectory", dim - Vec2i(350, 50),
                            0.7, green);
  }
  // Min Jrk Traj
  {
    TrajSolver2D traj_solver(Control::JRK);
    traj_solver.setPath(path);
    traj_solver.setV(1);  // set velocity for time allocation
    opencv_drawing.drawTraj(traj_solver.solve(), blue, 2);
    opencv_drawing.drawText("min jerk trajectory", dim - Vec2i(350, 20), 0.7,
                            blue);
  }

  // Draw keyframes
  opencv_drawing.drawPoints(path, black, 5);

  if (OPENCV_WINDOW) {
    // show the plot
    opencv_drawing.show(file_name);
  } else {
    // save the plot
    opencv_drawing.save(file_name + ".jpg");
    printf("Saved results to %s.jpg. \n", file_name.c_str());
  }

  return 0;
}
