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

#ifndef EXPOSURECONTROL_MEASURESENSOR_H
#define EXPOSURECONTROL_MEASURESENSOR_H

#include "exposure_control/ExposureController.h"
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <map>


namespace exposure_control {
  typedef std::vector<int64_t> Histogram;
  struct ImageStats {
    ImageStats(): count(0), brightness(0) {}
    int         count;
    double      brightness;
    Histogram   hist;
  };
  class MeasureSensor : public ExposureController {
  public:
    MeasureSensor(ExposureControl *ec);
    MeasureSensor(const MeasureSensor&) = delete;
    MeasureSensor& operator=(const MeasureSensor&) = delete;

    // --- inherited from the ExposureController class
    void imageCallback(const sensor_msgs::ImageConstPtr& im) override;
    void doConfigure(ExposureControlDynConfig& config, int level) override;

  private:
    void dumpStatistics() const;
    std::map<double, std::map<double, ImageStats> > stats_;
    double  shutterIncrement_{0.5};
    double  gainIncrement_{1};
    double  optimalShutter_{0};  // in msec
    double  optimalGain_{0};     // in db
  };
  std::ostream &operator<<(std::ostream &os, const ImageStats &is);
}


#endif
