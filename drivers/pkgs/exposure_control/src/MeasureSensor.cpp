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

#include "exposure_control/MeasureSensor.h"
#include <ros/console.h>
#include <algorithm>
#include <iostream>
#include <fstream>

namespace exposure_control {
  MeasureSensor::MeasureSensor(ExposureControl *ec) :
    ExposureController(ec) {
  }

  void MeasureSensor::doConfigure(ExposureControlDynConfig& config, int l){
    optimalShutter_ = minShutter_;
    optimalGain_    = minGain_;
    maxGain_        = 0;
    currentShutter_ = optimalShutter_;
    currentGain_    = optimalGain_;
  }

  static void addHistogram(Histogram *h1,
                           const Histogram &h2) {
    if (h1->empty()) {
      *h1 = Histogram(256, 0);
    }
    for (int i = 0; i < h2.size(); i++) {
      (*h1)[i] += h2[i];
    }
  }

  static Histogram getHistogram(const unsigned char *data, int rows,
                                int cols, int stride) {
    Histogram h(256, 0);
    size_t ioff(0);
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        h[data[ioff + j]]++;
      }
      ioff += stride;
    }
    return (h);
  }

  void MeasureSensor::imageCallback(const sensor_msgs::ImageConstPtr &img) {
    if (!enabled_) {
      return;
    }
 
    if (numFramesSkip_ > 0) {
      // Changes in gain or shutter take a few
      // frames to arrive at the camera, so we just skip
      // those.
      numFramesSkip_--;
      return;
    }

    double b = getAverageBrightness(&img->data[0], img->height,
                                    img->width, img->step);
    Histogram hist = getHistogram(&img->data[0], img->height,
                                  img->width, img->step);
    if (stats_.find(currentShutter_) == stats_.end()) {
      stats_[currentShutter_] = std::map<double, ImageStats>();
    }
    if (stats_[currentShutter_].find(currentGain_) ==
        stats_[currentShutter_].end()) {
      stats_[currentShutter_][currentGain_] = ImageStats();
    }

    ImageStats &stats = stats_[currentShutter_][currentGain_];
    stats.count++;
    stats.brightness += b;
    addHistogram(&stats.hist, hist);
    ROS_INFO("shut: %10.2f gain: %10.2f bright: %10.2f",
             currentShutter_, currentGain_, b);
    const int NUM_SAMPLES(20);
    if (stats.count >= NUM_SAMPLES) {
      //ROS_INFO("stats avg: %10.2f %d", stats.brightness, stats.count);
      stats.brightness = stats.brightness / (double)stats.count;
      double maxShutter = fmin(maxShutter_, maxShutterLim_);
      if (optimalShutter_ < maxShutter) {
        optimalShutter_ += shutterIncrement_;   // bump shutter speed
        optimalShutter_ = std::min(optimalShutter_, maxShutter);
        currentShutter_ = optimalShutter_;
        publish(currentShutter_, -1.0);
        
        ROS_INFO("set shutter to: %.2f", optimalShutter_);
      } else {
        if (optimalGain_ < maxGain_) {
          optimalShutter_ = minShutter_;    // reset shutter
          optimalGain_ += gainIncrement_;
          optimalGain_ = std::min(optimalGain_, maxGain_);
          currentGain_ = optimalGain_;
          publish(currentShutter_, currentGain_);
          ROS_INFO("set gain to: %.2f", optimalGain_);
        } else {
          // done scanning through all gains
          dumpStatistics();
          enabled_ = false;
        }
      }
    }
  }

  void MeasureSensor::dumpStatistics() const {
    if (stats_.empty()) return;
    const std::string fname = "brightness.txt";
    std::ofstream file(fname);
    ROS_INFO("dumping response to file: %s", fname.c_str());
    // write dimensions of array
    file << stats_.size() << " " << stats_.begin()->second.size() << std::endl;
    // write the shutter speeds
    for (const auto &m : stats_) {
      file << " " << m.first;
    }
    file << std::endl;
    // write gains
    if (!stats_.empty()) {
      for (const auto &s : stats_.begin()->second) {
        file << " " << s.first;
      }
    }
    file << std::endl;
    // write brightness
    for (const auto &m : stats_) {
      for (const auto &s : m.second) {
        file << " " << s.second.brightness;
        std::cout << " " << s.second;
      }
      file << std::endl;
      std::cout << std::endl;
    }
    // write histograms
    for (const auto &m : stats_) {
      for (const auto &s : m.second) {
        const Histogram &st = s.second.hist;
        for (int i = 0; i < st.size(); i++) {
          file << " " << st[i];
        }
        file << std::endl;
      }
    }
  }

  std::ostream &operator<<(std::ostream &os, const ImageStats &is) {
    os << "cnt:"<< is.count << ",b:" << is.brightness;
    return (os);
  }
}
