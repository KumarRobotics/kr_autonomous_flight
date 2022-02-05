#include <action_trackers/projector.hpp>

bool Projector::project(const Vec3f& pt) {
  // obs_ is set by ActionPathTracker::cloudCb in path_tracker
  // it is a list of points from a pointCloud
  // This returns the largest ball centered at the curr_pos (pt) that does not
  // intersect any obs_ (points in point cloud)
  // It starts at r_max_, but can get smaller
  // At the minimum it will be r_min_
  projected_ellipsoid_ = find_sphere(pt, obs_, r_max_);

  // Ellipsoid is the largest ball centered at the curr_pos (pt) that does
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
      0.5)  // directly replace if the distance is less than a threshold (time
            // delay from when the path is planned to now leads to this
            // distance)
  {
    path_.erase(path_.begin());
    path_.insert(path_.begin(), pt);
    path = path_;
  } else {
    path = path_;
    path.insert(path.begin(), pt);  // in this case, only change path variable
                                    // locally to avoid messing up path_.
  }

  find_intersection(path, projected_ellipsoid_, &projected_goal_);
  on_ellipsoid_ = (projected_goal_ - pt).norm() > (r() - 1e-3);

  outer_projected_goal_ = search_pt(projected_goal_, outer_r_max_);

  find_intersection(path, direction_ellipsoid_, &direction_goal_);

  if (ellipsoid_array_.empty() ||
      (pt - ellipsoid_array_.back().d_).norm() >= 0.2)
    ellipsoid_array_.push_back(projected_ellipsoid_);

  if (projected_ellipsoid_.C_(0, 0) + 0.1 < (projected_goal_ - pt).norm()) {
    ROS_ERROR("Project error! r: %f, dist: %f",
              projected_ellipsoid_.C_(0, 0),
              (projected_goal_ - pt).norm());
    return false;
  } else
    return true;
}

vec_E<Ellipsoid3D> Projector::project_array(const Vec3f& pt,
                                            int num,
                                            double res) {
  vec_Vec3f path = path_;
  if ((path.front() - pt).norm() > 0.1) path.insert(path.begin(), pt);
  const auto ps = path_downsample(path, res);
  vec_Vec3f new_ps;
  for (int i = ps.size() - 1; i >= 0; i--) {
    if ((ps[i] - pt).norm() < res) break;
    new_ps.push_back(ps[i]);
  }

  std::reverse(new_ps.begin(), new_ps.end());

  vec_E<Ellipsoid3D> es;
  for (const auto& it : new_ps) {
    es.push_back(find_sphere(it, obs_, r_max_));
    if (static_cast<int>(es.size()) > num) break;
  }

  for (const auto& it : ellipsoid_array_) es.push_back(it);
  return es;
}

Vec3f Projector::search_pt(const Vec3f& pt, double dist) {
  // Downsample the path
  const auto ps = path_downsample(path_, 0.1);
  vec_Vec3f new_ps;
  // Add the Vec3f in the paths until very close to the pt (projected goal)
  for (int i = ps.size() - 1; i >= 0; i--) {
    new_ps.push_back(ps[i]);
    if ((ps[i] - pt).norm() <= 0.1) break;
  }
  std::reverse(new_ps.begin(), new_ps.end());

  // Make sure there is at least one point on path
  if (new_ps.empty()) new_ps.push_back(pt);

  double d = 0;
  for (unsigned int i = 1; i < new_ps.size(); i++) {
    // Accumulate the distances on the path
    d += (new_ps[i] - new_ps[i - 1]).norm();
    // Once we are greater than outer_r_max_ than return that point on path
    // outer_r_max_ is dist_a = ||v||^2 / (2*a_max_)
    if (d >= dist) return new_ps[i];
  }
  return new_ps.back();
}

vec_Vec3f Projector::path_downsample(const vec_Vec3f& ps, double d) {
  // subdivide according to length
  if (ps.empty()) return ps;
  vec_Vec3f path;
  for (unsigned int i = 1; i < ps.size(); i++) {
    double dist = (ps[i] - ps[i - 1]).norm();
    int cnt = std::ceil(dist / d);
    for (int j = 0; j < cnt; j++)
      path.push_back(ps[i - 1] + j * (ps[i] - ps[i - 1]) / cnt);
  }
  path.push_back(ps.back());

  return path;
}

bool Projector::find_intersection(const vec_Vec3f& path,
                                  const Ellipsoid3D& ellipsoid,
                                  Vec3f* intersect_pt_ptr) {
  int id = -1;
  vec_Vec3f gs;
  for (unsigned int i = 0; i < path.size() - 1; i++) {
    Vec3f gp;
    if (intersect(path[i], path[i + 1], &gp, i == path.size() - 2, ellipsoid)) {
      if (i < path.size() - 2)
        id = i;
      else if ((gp - ellipsoid.d_).norm() < ellipsoid.C_(0, 0) + 1e-3)
        id = i;
    }
    gs.push_back(gp);
  }
  if (id >= 0) {
    *intersect_pt_ptr = gs[id];
    return true;
  } else {
    *intersect_pt_ptr = path.front();
    return false;
  }
}

vec_Vec3f Projector::ps_in_ellipsoid(const Ellipsoid3D& E, const vec_Vec3f& O) {
  vec_Vec3f new_O;
  for (const auto& it : O) {
    Vec3f vt = E.C_.inverse() * (it - E.d_);
    if (vt.norm() <= 1) new_O.push_back(it);
  }
  return new_O;
}

// This returns the largest ball centered at the curr_pos (pt) that does not
// intersect any obs_ (points in point cloud)
// It starts at r_max_, but can get smaller
// At the minimum it will be r_min_
Ellipsoid3D Projector::find_sphere(const Vec3f& pt,
                                   const vec_Vec3f& obs,
                                   double f) {
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
  double min_dist = f;
  for (const auto& it : Os) {
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

bool Projector::intersect(const Vec3f& p1,
                          const Vec3f& p2,
                          const Vec3f& c,
                          float r,
                          Vec3f* g_ptr,
                          bool force) {
  if (p1 == p2) return false;

  Vec3f d = (p2 - p1).normalized();
  Vec3f v = (p1 - c);

  double m = d.dot(v);

  double dd = m * m - (v.dot(v) - r * r);
  if (dd < 0) return false;

  double dd1 = -m + sqrt(dd);
  double dd2 = -m - sqrt(dd);

  double k = std::max(dd1, dd2);

  double k_max = (p2 - p1).norm();
  if (k > k_max || k < 0) {
    if (force && k > k_max) {
      *g_ptr = p2;
      return true;
    } else
      return false;
  }

  *g_ptr = p1 + k * d;
  return true;
}

bool Projector::intersect(const Vec3f& p1_w,
                          const Vec3f& p2_w,
                          Vec3f* g_ptr,
                          bool force,
                          const Ellipsoid3D& ellipsoid) {
  if (p1_w == p2_w) {
    if ((p1_w - ellipsoid.d_).norm() < ellipsoid.C_(0, 0)) {
      *g_ptr = p1_w;
      return true;
    } else
      return false;
  }
  bool find = intersect(ellipsoid.C_.inverse() * (p1_w - ellipsoid.d_),
                        ellipsoid.C_.inverse() * (p2_w - ellipsoid.d_),
                        Vec3f::Zero(),
                        1,
                        g_ptr,
                        force);
  if (find) *g_ptr = ellipsoid.C_ * (*g_ptr) + ellipsoid.d_;
  return find;
}
