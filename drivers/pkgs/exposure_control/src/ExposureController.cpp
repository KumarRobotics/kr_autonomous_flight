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

#include "exposure_control/ExposureController.h"
#include "exposure_control/ExposureControl.h"
#include <ros/console.h>
#include <algorithm>
#include <fstream>

namespace exposure_control {

  ExposureController::ExposureController(ExposureControl *ec) :
    exposureControl_(ec) {
  }

  void ExposureController::setFrameRate(double fps)    {
    fps_ = fps;
    republishInterval_ = fps;
  }
  void ExposureController::configure(ExposureControlDynConfig& config,
                                     int level) {
    if (enabled_ != config.enabled) {
      enabled_          = config.enabled;
      ROS_INFO("exposure control %s", enabled_ ? "ON" : "OFF");
    }
    #define STRFY(X) #X 
    #define SET_AND_REPORT(X, Y, Z) if (X != Y.Z) \
        { X = Y.Z; ROS_INFO_STREAM("changed " STRFY(Z) " to " << X); }

    SET_AND_REPORT(minShutter_, config, min_shutter);
    SET_AND_REPORT(maxShutter_, config, max_shutter);
    SET_AND_REPORT(minGain_, config, min_gain);
    SET_AND_REPORT(maxGain_, config, max_gain);
    SET_AND_REPORT(brightness_, config, brightness);
    SET_AND_REPORT(brightnessTol_, config, brightness_tol);
    SET_AND_REPORT(topMargin_, config, top_margin);
    SET_AND_REPORT(bottomMargin_, config, bottom_margin);
    SET_AND_REPORT(downSamplingRate_, config, down_sampling_rate);
    SET_AND_REPORT(waitFrames_, config, wait_frames);
    firstRow_           = -1;  // indicator that margins must be recomputed
    doConfigure(config, level);
  }

  double
  ExposureController::getAverageBrightness(const unsigned char *data,
                                           int rows, int cols, int stride) {
    if (firstRow_ < 0) {
      // should only run once after reconfiguration
      firstRow_  = (rows * topMargin_) / 100;
      lastRow_   = (rows * (100 - bottomMargin_)) / 100;
      int nlines = (lastRow_ - firstRow_ - 1) / downSamplingRate_ + 1;
      int ncols  = (cols - 1)/ downSamplingRate_ + 1;
      normFac_   =  1.0/(double)(nlines * ncols);
    }
    int64_t totalBrightness(0);
    int     rowByteStride(stride * downSamplingRate_);
    size_t  ioff(firstRow_ * stride);
    for (int i = firstRow_; i < lastRow_; i += downSamplingRate_) {
      for (int j = 0; j < cols; j += downSamplingRate_) {
        totalBrightness += data[ioff + j];
      }
      ioff += rowByteStride;
    }
    return (totalBrightness * normFac_);
  }

  void ExposureController::publish(double shutter, double gain) {
    exposureControl_->publish(shutter, gain);
    ROS_INFO("publishing shutter: %f  gain: %f", shutter, gain);
    numFramesSkip_ = waitFrames_;
  }
  
  void ExposureController::updateShutterLimit() {
    
    // The frame rate at which the camera is triggered
    // determines the maximum exposure time. From PtGreys documents
    // (search for "Maximum frame rate possible in asynchronous
    // (external trigger) mode"):
    //
    // Max_Frame_Rate_Trigger = 1/(Shutter + (1/Max_Frame_Rate_Free_Running))
    //
    // or conversely: max_shutter = 1/target_fps - 1/max_fps_free
    //
    // Experiments showed that this is actually precisely the hard limit,
    // i.e. the frame rate will drop to 1/2 the moment this limit is exceeded
    // by just 1msec. For this reason the max shutter time is lowered by
    // a bit more to make sure the frame rate stays up
    
    double fps = fmax(fps_, 0.1);  // lower bound of 0.1 Hz
    const double headRoom = 2.0;   // additional margin of safety (msec) to avoid frame drops
    const double minShutter = 1.0; // at least one msec max shutter
    double maxShutter = fmax(1000.0/fps - 1000.0/maxFPSFree_ - headRoom,
                             minShutter);
    maxShutter = std::max(maxShutter, 1.0);
    if (maxShutter != maxShutterLim_) {
      ROS_INFO("updating FPS shutter limit to %.2fms", maxShutter);
    }
    maxShutterLim_ = maxShutter;
  }

}  // namespace exposure_control
