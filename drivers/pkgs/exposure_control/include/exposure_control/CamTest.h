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

#ifndef EXPOSURECONTROL_CAMTEST_H
#define EXPOSURECONTROL_CAMTEST_H

#include <mutex>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <flea3/Flea3DynConfig.h>
#include <std_msgs/Float64MultiArray.h>

namespace flea3 {
class Flea3Ros;
}

namespace exposure_control {

class CamTest {
  
public:
  using Cam = flea3::Flea3Ros;
  using CamPtr = boost::shared_ptr<Cam>;
  using Config = flea3::Flea3DynConfig;

  CamTest(const ros::NodeHandle& parentNode);
  ~CamTest();
  CamTest(const CamTest&) = delete;
  CamTest& operator=(const CamTest&) = delete;

  void configure(Config& config, int level);
  void expCallback(const std_msgs::Float64MultiArray::ConstPtr &expMsg);

private:
  void setShutter(double s);
  void setGain(double g);
  void publishCurrentExposure();
  void updateExposure();
  void pollImages();
  void triggerThread();
  void startPoll();
  void stopPoll();
  void configureCameras(Config& config);
  void startSoftTrigger();
  void stopSoftTrigger();


  // ---------------- variables
  ros::NodeHandle   parentNode_;
  ros::Subscriber   expSub_;
  ros::Publisher    expPub_;
  std::mutex        expMutex_;
  CamPtr            camera_;
  bool              isPolling_{false};
  ros::Time         lastPublishTime_;
  ros::Duration     publishExposureInterval_{1.0};  // in seconds
  double            optimalShutter_{10};
  double            optimalGain_{0};
  bool              updateShutter_{false};
  bool              updateGain_{false};
  int               triggerSleepTime_{100000}; // in usec
  bool              isTriggering_{false};
  boost::shared_ptr<boost::thread>      imgPollThread_;
  boost::shared_ptr<boost::thread>      triggerThread_;
  dynamic_reconfigure::Server<Config>   configServer_;
};

}

#endif
