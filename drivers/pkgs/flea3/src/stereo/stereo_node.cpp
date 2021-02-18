#include "flea3/stereo_node.h"

namespace flea3 {

StereoNode::StereoNode(const ros::NodeHandle& pnh)
    : CameraNodeBase(pnh), left_ros_(pnh, "left"), right_ros_(pnh, "right") {}

void StereoNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    if (left_ros_.RequestSingle() && right_ros_.RequestSingle()) {
      const auto expose_duration =
          ros::Duration(left_ros_.camera().GetShutterTimeSec() / 2);
      const auto time = ros::Time::now() + expose_duration;
      left_ros_.PublishCamera(time);
      right_ros_.PublishCamera(time);
      Sleep();
    }
  }
}

void StereoNode::Setup(Flea3DynConfig& config) {
  left_ros_.Stop();
  right_ros_.Stop();
  left_ros_.set_fps(config.fps);
  right_ros_.set_fps(config.fps);
  Flea3DynConfig config_cpy = config;
  left_ros_.camera().Configure(config_cpy);
  right_ros_.camera().Configure(config);
  left_ros_.Start();
  right_ros_.Start();
}

}  // namespace flea3
