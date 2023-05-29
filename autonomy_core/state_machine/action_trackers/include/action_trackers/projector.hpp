#pragma once

#include <action_trackers/decompros.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>

#include <boost/circular_buffer.hpp>

class Projector {
 public:
  Projector(int num = 5) { ellipsoid_array_.set_capacity(num); }
  void set_obs(const vec_Vec3f obs) { obs_ = obs; }
  void set_path(const vec_Vec3f& path) { path_ = path; }
  void set_outer_r_max(double r) { outer_r_max_ = r; }
  void set_r_max(double r) { r_max_ = r; }
  void set_r_min(double r) { r_min_ = r; }
  void set_shrink_distance(double r) { shrink_distance_ = r; }

  Ellipsoid3D projected_ellipsoid() { return projected_ellipsoid_; }
  Ellipsoid3D direction_ellipsoid() { return direction_ellipsoid_; }
  Vec3f projected_goal() { return projected_goal_; }
  Vec3f outer_projected_goal() { return outer_projected_goal_; }
  Vec3f direction_goal() { return direction_goal_; }
  double r() { return projected_ellipsoid_.C_(0, 0); }
  double r_max() { return r_max_ - shrink_distance_; }
  double outer_r_max() { return outer_r_max_; }
  vec_Vec3f path() { return path_; }
  bool on_ellipsoid() { return on_ellipsoid_; }

  bool exist() { return !path_.empty(); }

  bool project(const Vec3f& pt);

  vec_E<Ellipsoid3D> project_array(const Vec3f& pt, int num, double res);

  Vec3f search_pt(const Vec3f& pt, double dist);

 private:
  vec_Vec3f path_downsample(const vec_Vec3f& ps, double d);

  bool find_intersection(const vec_Vec3f& path,
                         const Ellipsoid3D& ellipsoid,
                         Vec3f* intersect_pt_ptr);

  vec_Vec3f ps_in_ellipsoid(const Ellipsoid3D& E, const vec_Vec3f& O);

  Ellipsoid3D find_sphere(const Vec3f& pt, const vec_Vec3f& obs, double f);

  bool intersect(const Vec3f& p1,
                 const Vec3f& p2,
                 const Vec3f& c,
                 float r,
                 Vec3f* g_ptr,
                 bool force);

  bool intersect(const Vec3f& p1_w,
                 const Vec3f& p2_w,
                 Vec3f* g_ptr,
                 bool force,
                 const Ellipsoid3D& ellipsoid);

  double outer_r_max_{2.5};
  double r_max_{1.0};
  double r_min_{0.1};
  double shrink_distance_{0.0};
  bool on_ellipsoid_ = false;
  vec_Vec3f obs_;
  vec_Vec3f path_;
  Ellipsoid3D projected_ellipsoid_;
  Ellipsoid3D direction_ellipsoid_;
  boost::circular_buffer<Ellipsoid3D> ellipsoid_array_;

  Vec3f projected_goal_;
  Vec3f outer_projected_goal_;
  Vec3f direction_goal_;
};
