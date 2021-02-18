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

#ifndef EXPOSURECONTROL_DEFAULTEXPOSURECONTROLLER_H
#define EXPOSURECONTROL_DEFAULTEXPOSURECONTROLLER_H

#include "exposure_control/ExposureController.h"

namespace exposure_control {
  class DefaultController : public ExposureController {
  public:
    DefaultController(ExposureControl *ec);
    DefaultController(const DefaultController&) = delete;
    DefaultController& operator=(const DefaultController&) = delete;

    // --- inherited from the ExposureController class
    void imageCallback(const sensor_msgs::ImageConstPtr& im) override;
    void doConfigure(ExposureControlDynConfig& config, int level) override;

  private:
    double calculateGain(double brightRatio) const;
    double alpha_{1.0};
  };

}  // namespace exposure_control

#endif
