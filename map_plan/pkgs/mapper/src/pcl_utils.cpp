#include <mapper/pcl_utils.h>

namespace PCLUtils {

PCLPointCloud toPCL(const sensor_msgs::PointCloud &cloud_ros) {
  PCLPointCloud cloud;
  cloud.points.resize(cloud_ros.points.size());
  for (unsigned int i = 0; i < cloud_ros.points.size(); i++) {
    cloud.points[i].x = cloud_ros.points[i].x;
    cloud.points[i].y = cloud_ros.points[i].y;
    cloud.points[i].z = cloud_ros.points[i].z;
  }
  return cloud;
}

vec_Vec3f fromPCL(const PCLPointCloud &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }
  return pts;
}

PCLPointCloud eigenToPCL(const vec_Vec3f &pts) {
  PCLPointCloud cloud;
  cloud.points.resize(pts.size());
  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = pts[i](2);
  }

  return cloud;
}

sensor_msgs::PointCloud toROS(const PCLPointCloud &cloud_pcl) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(cloud_pcl.points.size());
  cloud.channels.resize(1);
  cloud.channels[0].name = "rgb";
  cloud.channels[0].values.resize(cloud.points.size());

  for (unsigned int i = 0; i < cloud_pcl.points.size(); i++) {
    cloud.points[i].x = cloud_pcl.points[i].x;
    cloud.points[i].y = cloud_pcl.points[i].y;
    cloud.points[i].z = cloud_pcl.points[i].z;
    cloud.channels[0].values[i] = cloud_pcl.points[i].rgb;
  }
  return cloud;
}

PCLPointCloud outlier_removal(const PCLPointCloud &cloud, float radius, int N) {
  if (cloud.points.empty()) {
    return cloud;
  }
  auto cloud_tmp = cloud.makeShared();
  pcl::RadiusOutlierRemoval<PCLPoint> sor;
  sor.setInputCloud(cloud_tmp);
  sor.setRadiusSearch(radius);
  sor.setMinNeighborsInRadius(N);
  sor.filter(*cloud_tmp);
  return *cloud_tmp
}

PCLPointCloud voxel_filter(const PCLPointCloud &cloud, float res) {
  auto cloud_tmp = cloud.makeShared();
  pcl::VoxelGrid<PCLPointCloud2> sor;
  sor.setInputCloud(cloud_tmp);
  sor.setLeafSize(res, res, res);
  sor.filter(*cloud_tmp);
  return *cloud_tmp;
}

} // namespace PCLUtils
