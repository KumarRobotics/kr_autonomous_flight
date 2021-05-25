#include "planning_ros_utils/data_ros_utils.h"

vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }

  return pts;
}

vec_Vec3f cloud_to_vec_filter(const sensor_msgs::PointCloud &cloud,
                              const double eps) {
  vec_Vec3f pts;
  pts.reserve(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    if (pow(cloud.points[i].x, 2.0) + pow(cloud.points[i].y, 2.0) +
            pow(cloud.points[i].z, 2.0) >
        eps) {
      Vec3f newPt;
      newPt(0) = cloud.points[i].x;
      newPt(1) = cloud.points[i].y;
      newPt(2) = cloud.points[i].z;
      pts.push_back(newPt);
    }
  }

  return pts;
}

vec_Vec3f ros_to_path(const planning_ros_msgs::Path &msg) {
  vec_Vec3f path;
  for (const auto &it : msg.waypoints) {
    path.push_back(Vec3f(it.x, it.y, it.z));
  }
  return path;
}
