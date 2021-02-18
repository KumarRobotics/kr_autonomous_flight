#pragma once

#include <camera_base/camera_node_base.h>

#include "flea3/Flea3DynConfig.h"
#include "flea3/flea3_ros.h"

namespace flea3 {

using Config = ::flea3::Flea3DynConfig;

class SingleNode : public camera_base::CameraNodeBase<Config> {
 public:
  explicit SingleNode(const ros::NodeHandle &pnh);

  void Acquire() override;
  void Setup(Config &config) override;

 private:
  Flea3Ros flea3_ros_;
};

}  // namespace flea3
