#pragma once

#include <camera_base/camera_ros_base.h>

#include "flea3/flea3_camera.h"

namespace flea3 {

class Flea3Ros : public camera_base::CameraRosBase {
 public:
  explicit Flea3Ros(const ros::NodeHandle& pnh,
                    const std::string& prefix = std::string());

  Flea3Camera& camera();

  bool RequestSingle();

  bool Grab(const sensor_msgs::ImagePtr& image_msg,
            const sensor_msgs::CameraInfoPtr& cinfo_msg = nullptr) override;
  bool GrabNonBlocking(const sensor_msgs::ImagePtr& image_msg,
                       const sensor_msgs::CameraInfoPtr& cinfo_msg = nullptr) {
    return flea3_.GrabImageNonBlocking(*image_msg);
  }
  bool GrabWithTimestamp(const sensor_msgs::ImagePtr& image_msg,
                         double* timestamp) {
    return flea3_.GrabImageWithTimestamp(*image_msg, timestamp);
  }
  bool GrabNonBlockingWithTimestamp(const sensor_msgs::ImagePtr& image_msg,
                                    double* timestamp) {
    return flea3_.GrabImageNonBlockingWithTimestamp(*image_msg, timestamp);
  }
  void PublishImageMetadata(const ros::Time& time);

  void Stop();
  void Start();

 private:
  Flea3Camera flea3_;
  ros::NodeHandle pnh_;
  //  ros::Publisher image_metadata_pub_;
};

}  // namespace flea3
