#pragma once
#include <mpl_basis/data_type.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef pcl::PointXYZRGB PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::PCLPointCloud2 PCLPointCloud2;

namespace PCLUtils {
PCLPointCloud toPCL(const sensor_msgs::PointCloud &cloud_ros);

sensor_msgs::PointCloud toROS(const PCLPointCloud &cloud_pcl);

PCLPointCloud outlier_removal(const PCLPointCloud &cloud, float radius = 0.4,
                              int N = 4);
PCLPointCloud voxel_filter(const PCLPointCloud &cloud, float res);
}  // namespace PCLUtils
