#pragma once

#include "camera_base/camera_node_base.h"
#include "flea3/Flea3DynConfig.h"
#include "flea3/flea3_ros.h"

namespace flea3 {

class StereoNode : public camera_base::CameraNodeBase<Flea3DynConfig> {
 public:
  explicit StereoNode(const ros::NodeHandle &pnh);

  virtual void Acquire() override;
  virtual void Setup(Flea3DynConfig &config) override;

 private:
  Flea3Ros left_ros_;
  Flea3Ros right_ros_;
};

}  // namespace flea3
