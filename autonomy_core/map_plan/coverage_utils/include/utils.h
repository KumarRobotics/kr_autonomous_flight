#pragma once
#include <kr_planning_msgs/Path.h>

#include <Eigen/StdVector>
#include <vector>
#include <string>

// Functions for convex_hull calculation are from
// https://www.topcoder.com/blog/problem-of-the-week-save-the-trees/

struct pt {
  double x, y;
};
bool cmp(pt a, pt b);
bool cw(pt a, pt b, pt c);
bool ccw(pt a, pt b, pt c);
template <int N>
using Vecf = Eigen::Matrix<double, N, 1>;
typedef Vecf<2> Vec2f;
typedef Vecf<3> Vec3f;
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
typedef vec_E<Vec2f> vec_Vec2f;
typedef vec_E<Vec3f> vec_Vec3f;
kr_planning_msgs::Path path_to_ros(const vec_Vec3f& path, double h = 0);
void convex_hull(std::vector<pt>* pts);
std::vector<pt> PreprocessData(const std::string& fname);
