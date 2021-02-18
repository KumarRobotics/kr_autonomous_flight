#include "flea3/flea3_ros.h"

namespace flea3 {

Flea3Ros::Flea3Ros(const ros::NodeHandle& pnh, const std::string& prefix)
    : CameraRosBase(pnh, prefix), flea3_(identifier()), pnh_(pnh) {
  SetHardwareId(flea3_.serial());
}

Flea3Camera& Flea3Ros::camera() { return flea3_; }

bool Flea3Ros::Grab(const sensor_msgs::ImagePtr& image_msg,
                    const sensor_msgs::CameraInfoPtr& cinfo_msg) {
  return flea3_.GrabImage(*image_msg);
}

// void Flea3Ros::PublishImageMetadata(const ros::Time& time) {
//  auto image_metadata_msg = boost::make_shared<ImageMetadata>();
//  flea3_.GrabImageMetadata(*image_metadata_msg);
//  image_metadata_msg->header.stamp = time;
//  image_metadata_msg->header.frame_id = frame_id();
//  image_metadata_pub_.publish(image_metadata_msg);
//}

void Flea3Ros::Stop() { flea3_.StopCapture(); }

void Flea3Ros::Start() { flea3_.StartCapture(); }

bool Flea3Ros::RequestSingle() { return flea3_.RequestSingle(); }

}  // namespace flea3
