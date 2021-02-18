/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "exposure_control/ExposureControl.h"
#include <ros/console.h>
#include <algorithm>
#include <std_msgs/Float64MultiArray.h>

namespace exposure_control {
  ExposureControl::ExposureControl(const ros::NodeHandle& pnh) :
    nh_(pnh), configServer_(pnh), controller_(this) {

    configServer_.setCallback(
      boost::bind(&ExposureControl::configure, this, _1, _2));

    expPub_ = nh_.advertise<std_msgs::Float64MultiArray>("exposure", 1);

    imSub_ = nh_.subscribe("image", 1,
      &ExposureController::imageCallback,
                           static_cast<ExposureController *>(&controller_));
  }

  void ExposureControl::configure(ExposureControlDynConfig& config, int l) {
    // configure all controller-specific params
    controller_.configure(config, l);
    // Get the frame rate and based on those
    // update the shutter limits.
    double fps;
    if (!nh_.getParam("fps", fps)) {
      throw std::runtime_error("fps parameter not specified!");
    }
    controller_.setFrameRate(fps);
    controller_.updateShutterLimit();
  }

  void ExposureControl::publish(double shutter, double gain) {
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(shutter);
    msg.data.push_back(gain);
    expPub_.publish(msg);
  }

}  // namespace exposure_control
