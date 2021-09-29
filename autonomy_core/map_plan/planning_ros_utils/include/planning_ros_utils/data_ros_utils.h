#pragma once

#include <geometry_msgs/Twist.h>
#include <mpl_basis/data_type.h>
#include <planning_ros_msgs/PathArray.h>
#include <sensor_msgs/PointCloud.h>
#include <tf_conversions/tf_eigen.h>

inline Vec3f pose_to_eigen(const geometry_msgs::Pose &pose) {
  return Vec3f(pose.position.x, pose.position.y, pose.position.z);
}

inline Vec3f twist_to_eigen(const geometry_msgs::Twist &twist) {
  return Vec3f(twist.linear.x, twist.linear.y, twist.linear.z);
}

inline Vec3f vec_to_eigen(const geometry_msgs::Vector3 &v) {
  return Vec3f(v.x, v.y, v.z);
}

inline vec_Vec3f vec2_to_vec3(const vec_Vec2f &pts2d, double z = 0) {
  vec_Vec3f pts(pts2d.size());

  for (size_t i = 0; i < pts.size(); i++)
    pts[i] = Vec3f(pts2d[i](0), pts2d[i](1), z);

  return pts;
}

inline Aff3f toTF(const geometry_msgs::Pose &p) {
  tf::Pose Ttf;
  tf::poseMsgToTF(p, Ttf);
  Eigen::Affine3d Td;
  tf::poseTFToEigen(Ttf, Td);
  return Td.cast<double>();
}

template <int Dim>
sensor_msgs::PointCloud vec_to_cloud(const vec_Vecf<Dim> &pts, double h = 0) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(pts.size());

  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = Dim == 2 ? h : pts[i](2);
  }
  return cloud;
}

vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud &cloud);

vec_Vec3f cloud_to_vec_filter(const sensor_msgs::PointCloud &cloud,
                              const double eps);

vec_Vec3f ros_to_path(const planning_ros_msgs::Path &msg);

template <int Dim>
planning_ros_msgs::Path path_to_ros(const vec_Vecf<Dim> &path, double h = 0) {
  planning_ros_msgs::Path msg;
  for (const auto &itt : path) {
    geometry_msgs::Point pt;
    pt.x = itt(0);
    pt.y = itt(1);
    pt.z = Dim == 2 ? h : itt(2);
    msg.waypoints.push_back(pt);
  }
  return msg;
}

template <int Dim>
planning_ros_msgs::PathArray path_array_to_ros(
    const vec_E<vec_Vecf<Dim>> &paths, double h = 0) {
  planning_ros_msgs::PathArray msg;
  for (const auto &it : paths) msg.paths.push_back(path_to_ros(it, h));
  return msg;
}

template <int Dim>
planning_ros_msgs::PathArray path_array_to_ros(
    const std::vector<std::pair<std::string, vec_Vecf<Dim>>> &paths,
    double h = 0) {
  planning_ros_msgs::PathArray msg;
  for (const auto &it : paths) {
    planning_ros_msgs::Path path_msg = path_to_ros(it.second, h);
    path_msg.name = it.first;
    msg.paths.push_back(path_msg);
  }
  return msg;
}
