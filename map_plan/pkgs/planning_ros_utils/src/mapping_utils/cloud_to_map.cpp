#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <topic_tools/shape_shifter.h>

std::unique_ptr<VoxelGrid> voxel_grid_;
ros::Publisher map_pub;

void processCloud(const sensor_msgs::PointCloud &cloud) {
  voxel_grid_->addCloud(cloud_to_vec(cloud));
  planning_ros_msgs::VoxelMap map = voxel_grid_->getMap();
  map.header = cloud.header;
  map_pub.publish(map);
  ROS_INFO("Publish the voxel map! [%zu]", cloud.points.size());
}

void cloudCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
  if (msg->getDataType() == "sensor_msgs/PointCloud") {
    auto cloud_ptr = msg->instantiate<sensor_msgs::PointCloud>();
    processCloud(*cloud_ptr);
  } else if (msg->getDataType() == "sensor_msgs/PointCloud2") {
    auto cloud2_ptr = msg->instantiate<sensor_msgs::PointCloud2>();
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud2_ptr, cloud);
    processCloud(cloud);
  } else
    return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_to_map");
  ros::NodeHandle nh("~");

  ros::Subscriber cloud_sub = nh.subscribe("cloud", 1, cloudCallback);

  map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);

  float res, range_x, range_y, range_z;
  float origin_x, origin_y, origin_z;
  nh.param("resolution", res, 0.20f);
  nh.param("origin_x", origin_x, -5.0f);
  nh.param("origin_y", origin_y, -10.0f);
  nh.param("origin_z", origin_z, 0.0f);
  nh.param("range_x", range_x, 50.0f);
  nh.param("range_y", range_y, 20.0f);
  nh.param("range_z", range_z, 4.0f);

  Vec3f origin, dim;
  origin(0) = origin_x;
  origin(1) = origin_y;
  origin(2) = origin_z;
  dim(0) = range_x;
  dim(1) = range_y;
  dim(2) = range_z;

  voxel_grid_.reset(new VoxelGrid(origin, dim, res));

  ros::spin();
  return 0;
}
