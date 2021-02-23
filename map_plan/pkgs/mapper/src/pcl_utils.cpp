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

void outlier_removal(PCLPointCloud &cloud, float radius, int N) {
  if (cloud.points.empty())
    return;
  pcl::RadiusOutlierRemoval<PCLPoint> sor;
  sor.setInputCloud(boost::make_shared<PCLPointCloud>(cloud));
  sor.setRadiusSearch(radius);
  sor.setMinNeighborsInRadius(N);
  sor.filter(cloud);
}

void bound_filter(PCLPointCloud &cloud, const Vec3f &lower_bound,
                  const Vec3f &upper_bound) {
  // build the condition
  pcl::ConditionAnd<PCLPoint>::Ptr range_cond(
      new pcl::ConditionAnd<PCLPoint>());
  range_cond->addComparison(pcl::FieldComparison<PCLPoint>::ConstPtr(
      new pcl::FieldComparison<PCLPoint>("x", pcl::ComparisonOps::GT,
                                         lower_bound(0))));
  range_cond->addComparison(pcl::FieldComparison<PCLPoint>::ConstPtr(
      new pcl::FieldComparison<PCLPoint>("x", pcl::ComparisonOps::LT,
                                         upper_bound(0))));
  range_cond->addComparison(pcl::FieldComparison<PCLPoint>::ConstPtr(
      new pcl::FieldComparison<PCLPoint>("y", pcl::ComparisonOps::GT,
                                         lower_bound(1))));
  range_cond->addComparison(pcl::FieldComparison<PCLPoint>::ConstPtr(
      new pcl::FieldComparison<PCLPoint>("y", pcl::ComparisonOps::LT,
                                         upper_bound(1))));
  range_cond->addComparison(pcl::FieldComparison<PCLPoint>::ConstPtr(
      new pcl::FieldComparison<PCLPoint>("z", pcl::ComparisonOps::GT,
                                         lower_bound(2))));
  range_cond->addComparison(pcl::FieldComparison<PCLPoint>::ConstPtr(
      new pcl::FieldComparison<PCLPoint>("z", pcl::ComparisonOps::LT,
                                         upper_bound(2))));

  // build the filter
  pcl::ConditionalRemoval<PCLPoint> condrem(range_cond);
  condrem.setInputCloud(boost::make_shared<PCLPointCloud>(cloud));
  condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter(cloud);
}

void voxel_filter(PCLPointCloud &cloud, decimal_t res) {
  PCLPointCloud2::Ptr pcl_cloud2(new PCLPointCloud2);
  PCLPointCloud2 cloud_filtered;

  pcl::toPCLPointCloud2(cloud, *pcl_cloud2);
  pcl::VoxelGrid<PCLPointCloud2> sor;
  sor.setInputCloud(pcl_cloud2);
  sor.setLeafSize(res, res, res);
  sor.filter(cloud_filtered);

  pcl::fromPCLPointCloud2(cloud_filtered, cloud);
}

} // namespace PCLUtils
