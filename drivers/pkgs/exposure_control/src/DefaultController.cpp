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

#include "exposure_control/DefaultController.h"
#include <ros/console.h>
#include <algorithm>
#include <fstream>

//#define DEBUG

#ifdef DEBUG
#include <time.h>
#include <sys/time.h>
#endif

namespace exposure_control {

#ifdef DEBUG
  static std::ofstream debugFile("exposure_ctrl.txt");
  static double get_wall_time(){
    struct timeval time;
    if (gettimeofday(&time,NULL)){
      return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
  }
  static double t0 = get_wall_time();
#endif

  DefaultController::DefaultController(ExposureControl *ec) :
    ExposureController(ec)
  {
  }
  #define STRFY(X) #X 
  #define SET_AND_REPORT(X, Y, Z) if (X != Y.Z) \
    { X = Y.Z; ROS_INFO_STREAM("changed " STRFY(Z) " to " << X); }

  void DefaultController::doConfigure(ExposureControlDynConfig& config, int l){
    SET_AND_REPORT(alpha_, config, auto_shutter_alpha);
  }

  double DefaultController::calculateGain(double brightRatio) const {
    const double kp = 5.0;  // empirical
    double desiredGain = currentGain_ + kp * log(brightRatio);
    return (std::max(std::min(desiredGain, maxGain_), minGain_));
  }

  void DefaultController::imageCallback(const sensor_msgs::ImageConstPtr &img) {
    if (!enabled_) {
      return;
    }
    if (currentShutter_ < 0) {
      // First image arrives, what to do? Need to publish exposure
      // settings so we know what the camera is set to.
      currentShutter_ = getMaxShutter();
      currentGain_ = maxGain_;
      publish(currentShutter_, currentGain_);
    }
    if (numFramesSkip_ > 0) {
      // Changes in gain or shutter take a few
      // frames to arrive at the camera, so we just skip
      // those.
      numFramesSkip_--;
      return;
    }
    double b = std::max(getAverageBrightness(&img->data[0], img->height,
                                             img->width, img->step), 1.0);
    double err_b        = (brightness_ - b);

#ifdef DEBUG
    ROS_INFO("b: %6.3f err_b: %7.4f curr shut: %7.5f curr gain: %7.5f",
             b, err_b, currentShutter_, currentGain_);
    debugFile << std::fixed << std::setprecision(3) << get_wall_time()
              << " " << currentShutter_ << " " << b << " " << err_b
              << " " <<  std::endl;
#endif

    if (fabs(err_b) > brightnessTol_) {
      double brightRatio  = std::max(std::min(brightness_/b, 10.0), 0.1);
      if (currentGain_ > minGain_) {
        //
        // We are in gain control mode. Note that calculateGain()
        // can return minGain_, which switches off gain mode.
        //
        double optGain = calculateGain(brightRatio);
        if (optGain != currentGain_) {
          currentGain_ = optGain;
          publish(-1.0, currentGain_);
#ifdef DEBUG
          ROS_INFO_STREAM("changing gain to: " << currentGain_);
#endif
        }
      } else {
        //
        // We are in shutter control mode
        //
        double maxShutter = getMaxShutter();
        if (currentShutter_ >= maxShutter && err_b > 0 && maxGain_>minGain_) {
          // Already have shutter at maximum, but still not bright enough,
          // must  switch to gain control mode.
          currentGain_ = calculateGain(brightRatio);
          publish(-1.0, currentGain_);
#ifdef DEBUG
          ROS_INFO_STREAM("entering gain mode, gain: " << currentGain_);
#endif
        } else {
          // regular shutter control
          double brpow = (alpha_ < 1.0) ?
            std::pow(brightRatio, alpha_) : brightRatio;
          double desiredShutter = currentShutter_ * brpow;
          double optShutter = std::max(minShutter_,
                                       std::min(currentShutter_ * brpow,
                                                maxShutter));
          if (optShutter != currentShutter_) {
            // we assume the driver sees the update flags
            // and will update the camera shutter to the desired value
            currentShutter_ = optShutter;
            publish(currentShutter_, -1.0);
#ifdef DEBUG
            ROS_INFO_STREAM("updating shutter to: " << currentShutter_);
#endif
          }
        }
      }
      // need to publish gains periodically to make sure the camera
      // actually gets the exposure settings. Without this, the
      // camera might not get updated if it is started *after* the
      // exposure control node is already running
      if (publishCnt_ ++ > republishInterval_) {
        publishCnt_ = 0;
        publish(currentShutter_, currentGain_);
      }
    }
  }
  
}  // namespace exposure_control
