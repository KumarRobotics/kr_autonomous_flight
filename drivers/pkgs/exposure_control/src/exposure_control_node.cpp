/*
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

#include <ros/ros.h>
#include <exposure_control/ExposureControl.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "exposure_control");
  ros::NodeHandle pnh("~");

  try {
    exposure_control::ExposureControl expcntrl(pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
