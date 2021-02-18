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

#include <exposure_control/CamTest.h>
#include <flea3/Flea3DynConfig.h>
#include <flea3/flea3_ros.h>

namespace exposure_control {

  CamTest::CamTest(const ros::NodeHandle& parentNode)
    : parentNode_(parentNode), configServer_(parentNode) {
    camera_ = boost::make_shared<Cam>(parentNode_, "cam0");
    // for publishing the currently set exposure values
    expPub_ = parentNode_.advertise<std_msgs::Float64MultiArray>("current_exposure", 1);
    expSub_ = parentNode_.subscribe("exposure", 1,
                                    &CamTest::expCallback, this);
    configServer_.setCallback(boost::bind(&CamTest::configure, this, _1, _2));
  }

  CamTest::~CamTest()
  {
    stopPoll();
  }
  
  void CamTest::expCallback(const std_msgs::Float64MultiArray::ConstPtr &expMsg) {
    std::lock_guard<std::mutex> lock(expMutex_);
    double shutter(expMsg->data[0]), gain(expMsg->data[1]);
    if (shutter >= 0) {
      optimalShutter_ = shutter;
      updateShutter_  = true;
    }
    if (gain >= 0) {
      optimalGain_ = gain;
      updateGain_ = true;
    }
  }

  void CamTest::setShutter(double s) {
    bool auto_shutter(false);
    double rs(s);
    camera_->camera().SetShutter(auto_shutter, rs);
    ROS_INFO("set shutter to: %8.4fms, driver returned %8.4fms", s, rs);}

  void CamTest::setGain(double g) {
    bool auto_gain(false);
    double rg(g);
    camera_->camera().SetGain(auto_gain, rg);
    ROS_INFO("set gain to:    %8.4fms, driver returned %8.4fms", g, rg);
  }

  void CamTest::configure(Config& config, int level) {
    if (level < 0) {
      ROS_INFO("%s: %s", parentNode_.getNamespace().c_str(),
               "Initializing reconfigure server");
    }
    stopPoll();
    stopSoftTrigger();
    triggerSleepTime_ = 1000000 / config.fps;
    configureCameras(config);
    startPoll();
    startSoftTrigger();
  }

  void CamTest::pollImages() {

    // first get the current camera settings and
    // publish them for the exposure control module
    optimalShutter_ = camera_->camera().GetShutterTimeSec() * 1000;
    optimalGain_    = camera_->camera().GetGain();
    publishCurrentExposure();
    lastPublishTime_ = ros::Time::now();
    ROS_INFO("node up and running!");
    while (isPolling_ && ros::ok()) {
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      bool ret = camera_->Grab(image_msg);
      ros::Time time = ros::Time::now();
      updateExposure();
      image_msg->header.stamp = time;
      if (time - lastPublishTime_ > publishExposureInterval_) {
        publishCurrentExposure();
        lastPublishTime_ = time;
      }
      // Publish takes less then 0.1ms to finish, so it is safe to put it here
      // in the loop
      if (ret) camera_->Publish(image_msg);
    }
  }


  void CamTest::updateExposure() {
    std::lock_guard<std::mutex> lock(expMutex_);
    if (updateShutter_) {
      setShutter(optimalShutter_);
      updateShutter_ = false;
    }
    if (updateGain_) {
      setGain(optimalGain_);
      updateGain_ = false;
    }
  }

  void CamTest::publishCurrentExposure() {
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(optimalShutter_);
    msg.data.push_back(optimalGain_);
    expPub_.publish(msg);
  }

  void CamTest::configureCameras(Config& config) {
    camera_->Stop();
    camera_->camera().Configure(config);
    camera_->set_fps(config.fps);
    camera_->Start();
  }

  void CamTest::startPoll() {
    isPolling_ = true;
    imgPollThread_ =
      boost::make_shared<boost::thread>(&CamTest::pollImages, this);
  }

  void CamTest::stopPoll() {
    if (!isPolling_) return;
    isPolling_ = false;
    imgPollThread_->join();
  }

  void CamTest::startSoftTrigger() {
    isTriggering_ = true;
    triggerThread_ =
      boost::make_shared<boost::thread>(&CamTest::triggerThread, this);
  }

  void CamTest::stopSoftTrigger() {
    if (!isTriggering_) return;
    isTriggering_ = false;
    triggerThread_->join();
  }

  void CamTest::triggerThread() {
    while (isTriggering_ && ros::ok()) {
      usleep(triggerSleepTime_);
      camera_->camera().FireSoftwareTrigger();
    }
  }

}  // namespace exposure_control
