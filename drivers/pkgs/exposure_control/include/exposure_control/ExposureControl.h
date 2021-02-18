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

#ifndef EXPOSURECONTROL_EXPOSURECONTROL_H
#define EXPOSURECONTROL_EXPOSURECONTROL_H

#include "exposure_control/DefaultController.h"
#include "exposure_control/ExposureControlDynConfig.h"

#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>

namespace exposure_control {
  class ExposureControl {
  public:
    ExposureControl(const ros::NodeHandle &pnh);
    ExposureControl(const ExposureControl&) = delete;
    ExposureControl& operator=(const ExposureControl&) = delete;

    void configure(ExposureControlDynConfig& config, int level);
    void publish(double shutter, double gain);

  protected:
    ros::NodeHandle     nh_;
    dynamic_reconfigure::Server<exposure_control::ExposureControlDynConfig>
    configServer_;
    DefaultController   controller_;
    ros::Subscriber     imSub_;
    ros::Publisher      expPub_;
  };

}  // namespace exposure_control

#endif
