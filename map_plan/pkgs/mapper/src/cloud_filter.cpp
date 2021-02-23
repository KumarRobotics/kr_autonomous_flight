#include <pcl_utils/pcl_utils.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

double outlier_res_;
double min_range_, max_range_, res_;
ros::Publisher cloud_pub;

void filter(const sensor_msgs::PointCloud &cloud) {
  ros::Time t0 = ros::Time::now();
  vec_Vec3f pts;
  Vec3f prev_pt = Vec3f::Zero();
  for (const auto &it : cloud.points) {
    Vec3f pt(it.x, it.y, it.z);
    if (pt.norm() < max_range_ && pt.norm() > min_range_) {
      if ((pt - prev_pt).norm() > res_) {
        prev_pt = pt;
        pts.push_back(pt);
      }
    }
  }

  sensor_msgs::PointCloud cloud_out;
  if (outlier_res_ > 0) {
    PCLPointCloud pcl_pts = PCLUtils::eigenToPCL(pts);
    PCLUtils::outlier_removal(pcl_pts, outlier_res_, 2);

    // printf("size of points: [%d, %d, %d]\n", s1, s2, s3);
    cloud_out = PCLUtils::toROS(pcl_pts);
  } else
    cloud_out = vec_to_cloud(pts);
  cloud_out.header = cloud.header;
  cloud_pub.publish(cloud_out);

  double dt = (ros::Time::now() - t0).toSec();
  if (dt > 0.05)
    ROS_WARN_THROTTLE(1.0, "Time to create map %f", dt);
}

void cloudCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
  // ROS_INFO_STREAM("Type: " << msg->getDataType());

  if (msg->getDataType() == "sensor_msgs/PointCloud") {
    auto cloud_msg = msg->instantiate<sensor_msgs::PointCloud>();
    filter(*cloud_msg);
  } else if (msg->getDataType() == "sensor_msgs/PointCloud2") {
    auto cloud_msg = msg->instantiate<sensor_msgs::PointCloud2>();
    sensor_msgs::PointCloud cloud1;
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, cloud1);
    filter(cloud1);
  } else {
    ROS_ERROR_STREAM(ros::this_node::getName()
                     << " got unsupported message type " << msg->getDataType());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_filter");
  ros::NodeHandle nh("~");

  nh.param("res", res_, 0.1);
  nh.param("outlier_res", outlier_res_, -1.0);
  nh.param("min_range", min_range_, 0.5);
  nh.param("max_range", max_range_, 30.0);
  ros::Subscriber cloud_sub = nh.subscribe("cloud_in", 5, cloudCallback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud_out", 1, true);

  ros::spin();

  return 0;
}
