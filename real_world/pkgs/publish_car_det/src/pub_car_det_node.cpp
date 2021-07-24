#include "pub_car_det_node.hpp"

namespace publish_car_det {

PubCarDetNode::PubCarDetNode(const ros::NodeHandle& pnh)
    : pnh_(pnh),
      last_odom_(),
      odom_received_(false) {
  odom_pub_ = pnh_.advertise<Odometry>("odom_on_detection", 1),
  odom_sub_ = pnh_.subscribe("odom_throttled", 10, &PubCarDetNode::OdomCallBack, this);
  detection_sub_ = pnh_.subscribe("detection", 10, &PubCarDetNode::DetectionCallBack, this);
}

void PubCarDetNode::OdomCallBack(const Odometry& odom_msg) {
  last_odom_ = odom_msg;
  odom_received_ = true;    
}

void PubCarDetNode::DetectionCallBack(const Detection2DArray& detection_msg) {
  // If not empty, publish the robot odom
  if (!(detection_msg.detections.empty())) {
    if (odom_received_) {
       odom_pub_.publish(last_odom_);}
    else{std::cout <<"PubCarDetNode: odom is not yet received!"<<std::endl;}
  }
}
}  // namespace publish_car_det

int main(int argc, char** argv) {
  ros::init(argc, argv, "pub_car_det_node");
  ros::NodeHandle pnh("~");
  publish_car_det::PubCarDetNode node(pnh);
  ros::spin();
}
