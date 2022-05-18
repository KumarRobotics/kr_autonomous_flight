#include <coverage_utils.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <math.h>

#include <boost/timer/timer.hpp>
#include <fstream>
#include <iostream>
#include <vector>

// Timer stuff
using boost::timer::cpu_timer;
using boost::timer::cpu_times;

CoveragePlanner::CoveragePlanner() : nh_("/"), pnh_("~") {
  initial_polygon_publisher_ =
      nh_.advertise<geometry_msgs::PolygonStamped>("original_polygon", 1);
  path_pub_ = nh_.advertise<kr_planning_msgs::Path>("coverage_path", 1, true);
  all_points_pub_ =
      nh_.advertise<kr_planning_msgs::Path>("all_input_points", 1, true);
  pnh_.param("input_file_path", fname_, std::string("input.txt"));
}

void CoveragePlanner::RunPlanner() {
  // load all points
  auto pt_vec = PreprocessData(fname_);

  std::vector<pt>::iterator it;
  vec_Vec3f all_points;
  for (it = pt_vec.begin(); it != pt_vec.end(); it++) {
    all_points.push_back(Vec3f(it->x, it->y, 5));
  }

  // convex hull of all points
  convex_hull(&pt_vec);
  geometry_msgs::PolygonStamped stamped_poly;
  stamped_poly.header.frame_id = "map";
  stamped_poly.header.stamp = ros::Time::now();
  for (it = pt_vec.begin(); it != pt_vec.end(); it++) {
    geometry_msgs::Point32 point;
    point.x = static_cast<float>(it->x);
    point.y = static_cast<float>(it->y);
    point.z = static_cast<float>(0.0);
    stamped_poly.polygon.points.push_back(point);
  }

  auto all_points_msg = path_to_ros(all_points);
  all_points_msg.header.frame_id = "map";
  all_points_pub_.publish(all_points_msg);

  initial_polygon_publisher_.publish(stamped_poly);
}
