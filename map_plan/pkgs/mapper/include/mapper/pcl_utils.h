#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <mpl_basis/data_type.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZRGB PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::PCLPointCloud2 PCLPointCloud2;

namespace PCLUtils {
PCLPointCloud toPCL(const sensor_msgs::PointCloud& cloud_ros);

vec_Vec3f fromPCL(const PCLPointCloud& cloud);

PCLPointCloud eigenToPCL(const vec_Vec3f& pts);
sensor_msgs::PointCloud toROS(const PCLPointCloud& cloud_pcl);

void outlier_removal(PCLPointCloud& cloud, float radius = 0.4, int N = 4);
void bound_filter(PCLPointCloud& cloud, const Vec3f& lower_bound,
                  const Vec3f& upper_bound);
void voxel_filter(PCLPointCloud& cloud, decimal_t res);
}  // namespace PCLUtils
#endif
