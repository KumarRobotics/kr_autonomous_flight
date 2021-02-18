#include <eigen_conversions/eigen_msg.h>
#include <mapper/tf_listener.h>
#include <mpl_basis/data_type.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher laser_pub;
std::string horizon_frame;
std::string laser_frame;

float resolution_, min_dist_, max_dist_;
bool to_horizon_frame, use_cloud2;

sensor_msgs::PointCloud scan_to_cloud(const sensor_msgs::LaserScan &scan) {
  sensor_msgs::PointCloud::Ptr cloud_ptr(new sensor_msgs::PointCloud);
  cloud_ptr->header = scan.header;
  float angle_min = scan.angle_min;
  float angle_increment = scan.angle_increment;
  for (size_t i = 0; i < scan.ranges.size(); i++) {
    if (min_dist_ <= scan.ranges[i] && scan.ranges[i] <= max_dist_) {
      float curr_angle = angle_min + i * angle_increment;
      float x = scan.ranges[i] * cos(curr_angle);
      float y = scan.ranges[i] * sin(curr_angle);

      geometry_msgs::Point32 point;
      point.x = x;
      point.y = y;
      point.z = 0;
      cloud_ptr->points.push_back(point);
    }
  }
  return *cloud_ptr;
}

sensor_msgs::PointCloud down_sample_cloud(const sensor_msgs::PointCloud &cloud,
                                          float res) {
  // Only works for point cloud directly from laser scan
  sensor_msgs::PointCloud new_cloud;
  new_cloud.header = cloud.header;
  Vec3f prev(100, 100, 100);
  for (size_t k = 0; k < cloud.points.size(); k++) {
    Vec3f vec(cloud.points[k].x, cloud.points[k].y, cloud.points[k].z);
    if ((vec - prev).norm() >= res) {
      geometry_msgs::Point32 p;
      p.x = vec[0];
      p.y = vec[1];
      p.z = vec[2];
      new_cloud.points.push_back(p);
      prev = vec;
    }
  }
  return new_cloud;
}

sensor_msgs::PointCloud transform_cloud(const sensor_msgs::PointCloud &cloud,
                                        const Aff3f &TF) {
  sensor_msgs::PointCloud new_cloud = cloud;
  int i = 0;
  for (auto it : cloud.points) {
    Vec3f raw(it.x, it.y, it.z);
    raw = TF * raw;
    new_cloud.points[i].x = raw(0);
    new_cloud.points[i].y = raw(1);
    new_cloud.points[i].z = raw(2);
    i++;
  }
  return new_cloud;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  sensor_msgs::PointCloud cloud = scan_to_cloud(*msg);
  cloud = down_sample_cloud(cloud, resolution_);

  if (!to_horizon_frame) {
    if (!use_cloud2) {
      cloud.header.stamp = msg->header.stamp;
      cloud.header.frame_id = laser_frame;
      laser_pub.publish(cloud);
    } else {
      sensor_msgs::PointCloud2 cloud2;
      sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
      cloud2.header.stamp = msg->header.stamp;
      cloud2.header.frame_id = laser_frame;
      laser_pub.publish(cloud2);
    }
  } else {
    static TFListener tf_listener;
    geometry_msgs::Pose laser_to_horizon_pose;
    if (!tf_listener.getPose(msg->header.stamp, laser_frame, horizon_frame,
                             laser_to_horizon_pose))
      return;

    Eigen::Affine3d laser_to_horizon_eigen;
    tf::poseMsgToEigen(laser_to_horizon_pose, laser_to_horizon_eigen);
    cloud = transform_cloud(cloud, laser_to_horizon_eigen.cast<decimal_t>());

    cloud.header.stamp = msg->header.stamp;
    cloud.header.frame_id = horizon_frame;
    laser_pub.publish(cloud);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_to_cloud");
  ros::NodeHandle n("~");

  n.param("resolution", resolution_, 0.05f);
  n.param("min_dist", min_dist_, 0.6f);
  n.param("max_dist", max_dist_, 15.0f);

  n.param("to_horizon_frame", to_horizon_frame, false);
  n.param("use_cloud2", use_cloud2, false);
  n.param("horizon_frame", horizon_frame, std::string("horizon_frame"));
  n.param("laser_frame", laser_frame, std::string("laser_frame"));

  if (!use_cloud2)
    laser_pub = n.advertise<sensor_msgs::PointCloud>("laser_cloud", 5, true);
  else
    laser_pub = n.advertise<sensor_msgs::PointCloud2>("laser_cloud", 5, true);
  ros::Subscriber laser_sub = n.subscribe("laser_in", 5, scanCallback);
  if (to_horizon_frame)
    ROS_WARN("laser_to_cloud: publish cloud in horizon frame");
  ros::spin();

  return 0;
}
