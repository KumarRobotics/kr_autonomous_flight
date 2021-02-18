#ifndef STAIR_DETECTOR_H
#define STAIR_DETECTOR_H

#include <angles/angles.h>
#include <motion_primitive_library/common/data_type.h>

class StairDetector {
 public:
  StairDetector() {}

  /// set height threshold
  void setHeightThr(double d) { height_threshold_ = d; }

  /// set distance threshold
  void setDistThr(double d) { dist_thr_ = d; }

  /// set angular threshold
  void setAngThr(double d) { ang_thr_ = d; }

  void setValidRegion(const Vec2f& center, double radius, double yaw) {
    if (radius == 0)
      regions_.clear();
    else {
      Ellipsoid e;
      e.second = Vec3f(center(0), center(1), 0);
      e.first = radius * Mat3f::Identity();
      e.first(2, 2) = 0.5;
      regions_.push_back(std::make_pair(e, yaw));
    }
  }

  /// get history keyframe TF
  vec_E<Aff3f> getTFs() { return Trws_; }

  /// get the valid region as ellipsoids
  vec_Ellipsoid getValidRegion() {
    vec_Ellipsoid es;
    for (const auto& it : regions_) es.push_back(it.first);
    return es;
  }

  bool process(const vec_Vec3f& pts, const Aff3f& Tcw, const Aff3f& Trw) {
    const Vec3f curr_pose(Trw.translation());
    const double curr_yaw = std::atan2(Eigen::Quaterniond(Trw.rotation()).z(),
                                       Eigen::Quaterniond(Trw.rotation()).w()) *
                            2;

    // skip this frame if not in the valid region
    bool in_region = regions_.empty();
    for (const auto& region : regions_) {
      auto e = region.first;
      if ((curr_pose - e.second).topRows<2>().norm() <= e.first(0, 0) &&
          std::abs(angles::shortest_angular_distance(region.second, curr_yaw)) <
              M_PI / 2) {
        in_region = true;
        break;
      }
      // printf("curr_yaw: %f, yaw_ref: %f, diff_yaw: %f\n", curr_yaw,
      // region.second, std::abs(angles::shortest_angular_distance(region.second,
      // curr_yaw)) );
    }

    if (!in_region) return false;

    // skip this frame if too close
    if (!Trws_.empty()) {
      const Aff3f prev_TF = Trws_.back();
      const Vec3f prev_pose(prev_TF.translation());
      const double prev_yaw =
          std::atan2(Eigen::Quaterniond(prev_TF.rotation()).z(),
                     Eigen::Quaterniond(prev_TF.rotation()).w()) *
          2;

      if ((curr_pose - prev_pose).norm() >= dist_thr_ ||
          std::abs(angles::shortest_angular_distance(curr_yaw, prev_yaw)) >=
              ang_thr_) {
      } else
        return false;
    }

    int cnt = 0;
    for (const auto& it : pts) {
      const Vec3f pt = Tcw * it;
      if (pt(2) - curr_pose(2) < height_threshold_) cnt++;

      if (cnt > 20) {
        Trws_.push_back(Trw);
        printf("pt: %f, curr_pose: %f, height_threshold: %f\n", pt(2),
               curr_pose(2), height_threshold_);
        return true;
      }
    }

    return false;
  }

 private:
  double height_threshold_ = -1;
  double dist_thr_ = 0.2;
  double ang_thr_ = M_PI / 12;
  vec_E<Aff3f> Trws_;
  std::vector<std::pair<Ellipsoid, double>> regions_;
};

#endif
