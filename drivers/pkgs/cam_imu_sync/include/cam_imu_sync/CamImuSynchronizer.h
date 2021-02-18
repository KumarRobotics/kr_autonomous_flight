/*
 * Copyright [2015] [Ke Sun  sunke.polyu@gmail.com]
 *                  [Chao Qu quchao@seas.upenn.edu]
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

#pragma once
#include <dynamic_reconfigure/server.h>
#include <flea3/Flea3DynConfig.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <mutex>

namespace flea3 {
class Flea3Ros;
}  // namespace flea3

namespace imu_vn_100 {
class ImuVn100;
}  // namespace imu_vn_100

namespace cam_imu_sync {

class CamImuSynchronizer {
 public:
  using Imu = imu_vn_100::ImuVn100;
  using ImuPtr = boost::shared_ptr<Imu>;
  using Cam = flea3::Flea3Ros;
  using CamPtr = boost::shared_ptr<Cam>;
  using Config = flea3::Flea3DynConfig;

  CamImuSynchronizer(const ros::NodeHandle& pnh);
  ~CamImuSynchronizer();
  CamImuSynchronizer(const CamImuSynchronizer&) = delete;
  CamImuSynchronizer& operator=(const CamImuSynchronizer&) = delete;

  /**
   * @brief configure
   */
  void configure(Config& config, int level);
  void expCallback(const std_msgs::Float64MultiArray::ConstPtr& expMsg);

 private:
  void setShutter(double s);
  void setGain(double g);

  void updateExposure() {
    std::lock_guard<std::mutex> lock(exp_mutex_);
    if (update_shutter_) {
      setShutter(optimal_shutter_);
      update_shutter_ = false;
    }
    if (update_gain_) {
      setGain(optimal_gain_);
      update_gain_ = false;
    }
  }

  bool keepPolling_{false};
  ros::NodeHandle pnh_;
  ros::NodeHandle imu_nh_;
  ImuPtr imu_;
  std::vector<CamPtr> cameras_;
  std::vector<boost::shared_ptr<boost::thread> > pollThreads_;
  dynamic_reconfigure::Server<flea3::Flea3DynConfig> cam_cfg_server_;

  ros::Subscriber exp_sub_;
  ros::Publisher exp_pub_;
  ros::Duration dt_warn_frame_frame_{1.0};
  ros::Duration dt_warn_imu_frame_{1.0};
  int64_t max_wait_for_imu_{0};
  double dt_warn_cam_timestamp_{1.0};
  double max_shutter_ms_{1};
  double optimal_shutter_{10};
  double optimal_gain_{0};
  bool update_shutter_{false};
  bool update_gain_{false};
  std::mutex exp_mutex_;
  std::mutex pollMutex_;

  void pollThread(int camid);
  void startPoll();
  void stopPoll();
  void configureCameras(Config& config);
};

}  // namespace cam_imu_sync
