/*
 * 2016 Bernd Pfrommer
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
#ifndef EXPOSURE_CONTROL_NODELET_H
#define EXPOSURE_CONTROL_NODELET_H

#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <exposure_control/ExposureControl.h>

namespace exposure_control {
class ExposureControlNodelet : public nodelet::Nodelet {
public:
  ExposureControlNodelet() {}
  ~ExposureControlNodelet() {}

private:
  virtual void onInit();
  std::shared_ptr<ExposureControl> exposureControl_;
};
} // end namespace

#endif

