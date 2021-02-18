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

#ifndef EXPOSURECONTROL_EXPOSURECONTROLLER_H
#define EXPOSURECONTROL_EXPOSURECONTROLLER_H

#include "exposure_control/ExposureControlDynConfig.h"
#include <sensor_msgs/Image.h>
#include <algorithm>

namespace exposure_control {
  class ExposureControl;
  double  getAverageBrightness(const unsigned char *data,
                               int rows, int cols, int stride);
  class ExposureController {
  public:
    ExposureController(ExposureControl *ec);
    virtual ~ExposureController() {};
    ExposureController(const ExposureController&) = delete;
    ExposureController& operator=(const ExposureController&) = delete;
    // ----- to be implemented by the derived classes
    virtual void imageCallback(const sensor_msgs::ImageConstPtr& im) = 0;
    virtual void doConfigure(ExposureControlDynConfig& config, int level) = 0;
    // ------ own methods
    void    setFrameRate(double fps);
    void    setMaxFPSFree(double fps) { maxFPSFree_ = fps; }
    void    configure(ExposureControlDynConfig& config, int level);
    void    setCurrentShutter(double x) { currentShutter_ = x; }
    void    setCurrentGain(double x)    { currentGain_ = x; }
    double  getMaxShutter() const { return std::min(maxShutter_, maxShutterLim_);}
    double  getMaxGain() const { return (maxGain_); }
    double  getMaxFPSFree() const { return (maxFPSFree_); }
    void    updateShutterLimit();
  protected:
    double  getAverageBrightness(const unsigned char *data,
                                 int rows, int cols, int stride);
    void    publish(double shutter, double gain);
    // ------ variables
    ExposureControl *exposureControl_;
    //
    // configurable constants
    //
    bool    enabled_{false};
    double  minShutter_{1e-6};  // in msec
    double  maxShutter_{100};   // in msec, due to dyn config
    double  maxShutterLim_{100};// in msec, due to frame rate
    double  fps_{100};          // frame rate
    double  maxFPSFree_{200};   // in 1/sec
    double  minGain_{0};        // in db
    double  maxGain_{10};       // in db
    double  brightness_{35};    // 0-255
    double  brightnessTol_{0};  // 0-255
    int     downSamplingRate_{1}; // horiz and vertical pix skipping
    int     topMargin_{0};      // in percent
    int     bottomMargin_{0};   // in percent
    int     waitFrames_{0};     // # of frames to wait
    //
    // dynamically changing
    //
    int     numFramesSkip_{0};   // num frames to skip
    double  currentShutter_{-1}; // in msec
    double  currentGain_{-1};    // in db
    double  normFac_{0};
    int     firstRow_{-1};
    int     lastRow_{-1};
    int     republishInterval_{20};
    int     publishCnt_{0};
  };

}  // namespace exposure_control

#endif
