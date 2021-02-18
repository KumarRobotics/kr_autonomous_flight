#include "vision_msgs/Detection2DArray.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace publish_car_det {

using geometry_msgs::Pose;
using nav_msgs::Odometry;
using vision_msgs::Detection2DArray;

class PubCarDetNode {
 public:
 
  explicit PubCarDetNode(const ros::NodeHandle& pnh);
  void OdomCallBack(const Odometry& odom_msg);
  void DetectionCallBack(const Detection2DArray& detection_msg);
                

 private:
  ros::NodeHandle pnh_;
  ros::Publisher odom_pub_;
  Odometry last_odom_;
  ros::Subscriber odom_sub_, detection_sub_;
  bool odom_received_;
};

}  // namespace publish_car_det
