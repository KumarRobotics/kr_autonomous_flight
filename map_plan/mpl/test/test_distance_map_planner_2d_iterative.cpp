#include <mpl_planner/planner/map_planner.h>

#include "opencv_drawing.hpp"
#include "read_map.hpp"
#include "timer.hpp"

int main(int argc, char **argv) {
  if (argc != 2) {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Load the map
  MapReader<Vec2i, Vec2f> reader(argv[1]);
  if (!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot find input file [%s]!\n" ANSI_COLOR_RESET,
           argv[1]);
    return -1;
  }

  // Pass the data into a VoxelMapUtil class for collision checking
  std::shared_ptr<MPL::OccMapUtil> map_util =
      std::make_shared<MPL::OccMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(),
                   reader.resolution());
  map_util->freeUnknown();

  // Initialize start and goal, using acc control
  Waypoint2D start, goal;
  start.pos = Vec2f(reader.start(0), reader.start(1));
  start.vel = Vec2f::Zero();
  start.acc = Vec2f::Zero();
  start.jrk = Vec2f::Zero();
  start.yaw = 0;
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = false;
  start.use_jrk = false;
  start.use_yaw = false;

  goal.pos = Vec2f(reader.goal(0), reader.goal(1));
  goal.vel = Vec2f::Zero();
  goal.acc = Vec2f::Zero();
  goal.jrk = Vec2f::Zero();
  goal.yaw = 0;
  goal.control = start.control;

  // Initialize control input
  decimal_t u = 0.5;
  decimal_t du = u;
  vec_E<VecDf> U;
  for (decimal_t dx = -u; dx <= u; dx += du)
    for (decimal_t dy = -u; dy <= u; dy += du) U.push_back(Vec2f(dx, dy));

  // Initialize planner
  std::unique_ptr<MPL::OccMapPlanner> planner(
      new MPL::OccMapPlanner(false));  // Declare a 2D planner
  planner->setMapUtil(map_util);       // Set collision checking function
  planner->setVmax(1.0);               // Set max velocity
  planner->setAmax(1.0);               // Set max acceleration
  planner->setDt(1.0);                 // Set dt for each primitive
  planner->setU(U);                    // Set control input

  // Planning
  Timer time(true);
  bool valid = planner->plan(start, goal);  // Plan from start to goal
  double dt = time.Elapsed().count();
  printf("MPL Planner takes: %f ms\n", dt);
  const auto traj = planner->getTraj();

  // Initiaize planner as a distance map planner
  planner.reset(new MPL::OccMapPlanner(true));
  planner->setMapUtil(map_util);  // Set collision checking function
  planner->setVmax(1.0);          // Set max velocity
  planner->setAmax(1.0);          // Set max acceleration
  planner->setDt(1.0);            // Set dt for each primitive
  planner->setU(U);               // Set control input
  planner->setEpsilon(1.0);       // Set heursitic to zero

  planner->setSearchRadius(Vec2f(0.5, 0.5));     // Set search region radius
  planner->setPotentialRadius(Vec2f(1.0, 1.0));  // Set potential distance
  planner->setPotentialWeight(0.5);              // Set potential weight
  planner->setGradientWeight(0);                 // Set gradient weight
  planner->updatePotentialMap(start.pos);        // Update potential map

  // Planning
  Timer time_dist(true);
  bool valid_dist =
      planner->iterativePlan(start, goal, traj, 10);  // Plan from start to goal
  double dt_dist = time_dist.Elapsed().count();
  printf("MPL Distance Planner iterative plan takes: %f ms\n", dt_dist);
  const auto traj_dist = planner->getTraj();

  // Plotting
  std::string file_name("test_distance_map_planner_2d_iterative");
  OpenCVDrawing opencv_drawing(map_util);
  // Draw searched region
  opencv_drawing.drawPoints(planner->getSearchRegion(), green, 1);
  // draw obstacles
  opencv_drawing.drawPotential(magenta, cyan);
  opencv_drawing.drawPoints(map_util->getCloud(), black);
  // draw start and goal
  opencv_drawing.drawCircle(start.pos, blue, 5, 2);
  opencv_drawing.drawCircle(goal.pos, cyan, 5, 2);

  // draw trajectory
  if (valid) {
    opencv_drawing.drawTraj(traj, red, 2);
    opencv_drawing.drawTraj(traj_dist, blue, 2);
  }

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
