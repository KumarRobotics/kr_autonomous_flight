#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/types.hpp>

namespace fake_lidar {
using PointPCL = pcl::PointXYZI;
using PointCV = cv::Point3d;

template <typename PointT>
void EuclideanSpherical(double radius, double alt, double azimuth, PointT& pt) {
  pt.x = -radius * cos(alt) * sin(azimuth);
  pt.y = -radius * sin(alt);
  pt.z = radius * cos(alt) * cos(azimuth);
}


static constexpr double PointAltitude(double x, double y, double z) {
  // [-pi, pi]
  return std::atan2(z, std::hypot(x, y));
}

static constexpr double PointAzimuth(double x, double y) {
  const auto a = std::atan2(y, x);
  // Convert to [0, 2pi)
  return y >= 0 ? a : a + M_PI * 2;
}
/// Degree from Radian
static constexpr double Deg_Rad(double rad) { return rad * 180 / M_PI; }

/// Radian from Degree
static constexpr double Rad_Deg(double deg) { return deg / 180 * M_PI; }
}  // namespace fake_lidar