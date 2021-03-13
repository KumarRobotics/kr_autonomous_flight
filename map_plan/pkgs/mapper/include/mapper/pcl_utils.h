#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace PCLUtils {

typedef pcl::PointXYZRGB PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::PCLPointCloud2 PCLPointCloud2;

PCLPointCloud toPCL(const sensor_msgs::PointCloud &cloud_ros);

sensor_msgs::PointCloud toROS(const PCLPointCloud &cloud_pcl);

PCLPointCloud outlier_removal(const PCLPointCloud &cloud, float radius = 0.4,
                              int N = 4);

PCLPointCloud voxel_filter(const PCLPointCloud &cloud, float res);

}  // namespace PCLUtils
