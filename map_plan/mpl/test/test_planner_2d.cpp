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
  std::shared_ptr<MPL::OccMapUtil> map_util;
  map_util.reset(new MPL::OccMapUtil);
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
      new MPL::OccMapPlanner(true));  // Declare a mp planner using voxel map
  planner->setMapUtil(map_util);      // Set collision checking function
  planner->setVmax(1.0);              // Set max velocity
  planner->setAmax(1.0);              // Set max acceleration
  planner->setDt(1.0);                // Set dt for each primitive
  planner->setU(U);                   // Set control input

  // Planning
  Timer time(true);
  bool valid = planner->plan(start, goal);  // Plan from start to goal
  double dt = time.Elapsed().count();
  printf("MPL Planner takes: %f ms\n", dt);
  printf("MPL Planner expanded states: %zu\n", planner->getCloseSet().size());

  // Plotting
  std::string file_name("test_planner_2d");
  OpenCVDrawing opencv_drawing(map_util);
  // draw obstacles
  opencv_drawing.drawPoints(map_util->getCloud(), black);
  // draw expanded states
  opencv_drawing.drawPoints(planner->getCloseSet(), grey, 3);
  // draw start and goal
  opencv_drawing.drawCircle(start.pos, blue, 5, 2);
  opencv_drawing.drawCircle(goal.pos, cyan, 5, 2);
  // draw trajectory
  if (valid) opencv_drawing.drawTraj(planner->getTraj(), red, 2);

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
