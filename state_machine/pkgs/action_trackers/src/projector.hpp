#pragma once

#include <decomp_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/data_ros_utils.h>

#include <boost/circular_buffer.hpp>

class Projector {
public:
  Projector(int num = 5) { ellipsoid_array_.set_capacity(num); }
  void set_obs(const vec_Vec3f obs) { obs_ = obs; }
  void set_path(const vec_Vec3f &path) { path_ = path; }
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

  bool project(const Vec3f &pt) {
    // obs_ is set by ActionPathTracker::cloudCb in path_tracker
    // it is a list of points from a pointCloud
    // This returns the largest ball centered at the curr_pos (pt) that does not
    // intersect any obs_ (points in point cloud)
    // It starts at r_max_, but can get smaller
    // At the minimum it will be r_min_
    projected_ellipsoid_ = find_sphere(pt, obs_, r_max_);

    // Q: What is direction_ellipsoid?
    // A: Ellipsoid is the largest ball centered at the curr_pos (pt) that does
    // not intersect any obs_ (points in point cloud)
    direction_ellipsoid_ = projected_ellipsoid_;
    for (int i = 0; i < 3; i++) {
      direction_ellipsoid_.C_(i, i) *= 2;
      direction_ellipsoid_.C_(i, i) =
          std::max(direction_ellipsoid_.C_(i, i), 0.3);
    }

    vec_Vec3f path;
    // Add curr_pos (pt) to the front of path
    if ((path_.front() - pt).norm() <
        0.5) // directly replace if the distance is less than a threshold (time
             // delay from when the path is planned to now leads to this
             // distance)
    {
      path_.erase(path_.begin());
      path_.insert(path_.begin(), pt);
      path = path_;
    } else {
      path = path_;
      path.insert(path.begin(), pt); // in this case, only change path variable
                                     // locally to avoid messing up path_.
    }

    // Projects the goal point onto the projected_illpsoid?
    find_intersection(path, projected_ellipsoid_, projected_goal_);
    on_ellipsoid_ = (projected_goal_ - pt).norm() > (r() - 1e-3);

    outer_projected_goal_ = search_pt(projected_goal_, outer_r_max_);

    find_intersection(path, direction_ellipsoid_, direction_goal_);

    if (ellipsoid_array_.empty() ||
        (pt - ellipsoid_array_.back().d_).norm() >= 0.2)
      ellipsoid_array_.push_back(projected_ellipsoid_);

    if (projected_ellipsoid_.C_(0, 0) + 0.1 < (projected_goal_ - pt).norm()) {
      ROS_ERROR("Project error! r: %f, dist: %f", projected_ellipsoid_.C_(0, 0),
                (projected_goal_ - pt).norm());
      return false;
    } else
      return true;
  }

  vec_E<Ellipsoid3D> project_array(const Vec3f &pt, int num, double res) {
    vec_Vec3f path = path_;
    if ((path.front() - pt).norm() > 0.1)
      path.insert(path.begin(), pt);
    const auto ps = path_downsample(path, res);
    vec_Vec3f new_ps;
    for (int i = ps.size() - 1; i >= 0; i--) {
      if ((ps[i] - pt).norm() < res)
        break;
      new_ps.push_back(ps[i]);
    }

    std::reverse(new_ps.begin(), new_ps.end());

    vec_E<Ellipsoid3D> es;
    for (const auto &it : new_ps) {
      es.push_back(find_sphere(it, obs_, r_max_));
      if ((int)es.size() > num)
        break;
    }

    for (const auto &it : ellipsoid_array_)
      es.push_back(it);
    return es;
  }

  Vec3f search_pt(const Vec3f &pt, double dist) {
    // Downsample the path
    const auto ps = path_downsample(path_, 0.1);
    vec_Vec3f new_ps;
    // Add the Vec3f in the paths until very close to the pt (projected goal)
    // Reversed?
    for (int i = ps.size() - 1; i >= 0; i--) {
      new_ps.push_back(ps[i]);
      if ((ps[i] - pt).norm() <= 0.1) // the threshold tolerance is 0.1 (which
                                      // is why we stop when within 0.1?
        break;
    }
    // Reverse back? Which is front and back?
    std::reverse(new_ps.begin(), new_ps.end());

    // Make sure there is at least one point on path
    if (new_ps.empty())
      new_ps.push_back(pt);

    double d = 0;
    for (unsigned int i = 1; i < new_ps.size(); i++) {
      // Accumulate the distances on the path
      d += (new_ps[i] - new_ps[i - 1]).norm();
      // Once we are greater than outer_r_max_ than return that point on path
      // outer_r_max_ is dist_a = ||v||^2 / (2*a_max_)
      if (d >= dist)
        return new_ps[i];
    }
    return new_ps.back();
  }

private:
  vec_Vec3f path_downsample(const vec_Vec3f &ps, decimal_t d) {
    // subdivide according to length
    if (ps.empty())
      return ps;
    vec_Vec3f path;
    for (unsigned int i = 1; i < ps.size(); i++) {
      decimal_t dist = (ps[i] - ps[i - 1]).norm();
      int cnt = std::ceil(dist / d);
      for (int j = 0; j < cnt; j++)
        path.push_back(ps[i - 1] + j * (ps[i] - ps[i - 1]) / cnt);
    }
    path.push_back(ps.back());

    return path;
  }

  bool find_intersection(const vec_Vec3f &path, const Ellipsoid3D &ellipsoid,
                         Vec3f &intersect_pt) {
    int id = -1;
    vec_Vec3f gs;
    for (unsigned int i = 0; i < path.size() - 1; i++) {
      Vec3f gp;
      if (intersect(path[i], path[i + 1], gp, i == path.size() - 2,
                    ellipsoid)) {
        if (i < path.size() - 2)
          id = i;
        else if ((gp - ellipsoid.d_).norm() < ellipsoid.C_(0, 0) + 1e-3)
          id = i;
      }
      gs.push_back(gp);
    }
    if (id >= 0) {
      intersect_pt = gs[id];
      return true;
    } else {
      intersect_pt = path.front();
      return false;
    }
  }

  vec_Vec3f ps_in_ellipsoid(const Ellipsoid3D &E, const vec_Vec3f &O) {
    vec_Vec3f new_O;
    for (const auto &it : O) {
      Vec3f vt = E.C_.inverse() * (it - E.d_);
      if (vt.norm() <= 1)
        new_O.push_back(it);
    }
    return new_O;
  }

  // This returns the largest ball centered at the curr_pos (pt) that does not
  // intersect any obs_ (points in point cloud)
  // It starts at r_max_, but can get smaller
  // At the minimum it will be r_min_
  Ellipsoid3D find_sphere(const Vec3f &pt, const vec_Vec3f &obs, decimal_t f) {
    Ellipsoid3D E(f * Mat3f::Identity(), pt);

    // Find which cloud points (obs) are in the ellipsoid
    vec_Vec3f Os = ps_in_ellipsoid(E, obs);

    // If no points
    if (Os.empty()) {
      E.C_ *= (f - shrink_distance_) / f;
      return E;
    }

    // If there are points, find the closest point
    // Make a ball that is smaller than the distance from curr_pos (pt) and the
    // closest point
    Vec3f closest_o = pt;
    decimal_t min_dist = f;
    for (const auto &it : Os) {
      if ((it - pt).norm() < min_dist) {
        min_dist = (it - pt).norm();
        closest_o = it;
      }
    }

    min_dist -= shrink_distance_;
    // Check to see if it is less than r_min_
    min_dist = min_dist < r_min_ ? r_min_ : min_dist;

    E.C_ = min_dist * Mat3f::Identity();

    return E;
  }

  bool intersect(const Vec3f &p1, const Vec3f &p2, const Vec3f &c, float r,
                 Vec3f &g, bool force) {
    if (p1 == p2)
      return false;

    Vec3f d = (p2 - p1).normalized();
    Vec3f v = (p1 - c);

    decimal_t m = d.dot(v);

    decimal_t dd = m * m - (v.dot(v) - r * r);
    if (dd < 0)
      return false;

    decimal_t dd1 = -m + sqrt(dd);
    decimal_t dd2 = -m - sqrt(dd);

    decimal_t k = std::max(dd1, dd2);

    decimal_t k_max = (p2 - p1).norm();
    if (k > k_max || k < 0) {
      if (force && k > k_max) {
        g = p2;
        return true;
      } else
        return false;
    }

    g = p1 + k * d;
    return true;
  }

  bool intersect(const Vec3f &p1_w, const Vec3f &p2_w, Vec3f &g, bool force,
                 const Ellipsoid3D &ellipsoid) {
    if (p1_w == p2_w) {
      if ((p1_w - ellipsoid.d_).norm() < ellipsoid.C_(0, 0)) {
        g = p1_w;
        return true;
      } else
        return false;
    }
    bool find = intersect(ellipsoid.C_.inverse() * (p1_w - ellipsoid.d_),
                          ellipsoid.C_.inverse() * (p2_w - ellipsoid.d_),
                          Vec3f::Zero(), 1, g, force);
    if (find)
      g = ellipsoid.C_ * g + ellipsoid.d_;
    return find;
  }

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
