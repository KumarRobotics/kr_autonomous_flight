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

#include <cam_imu_sync/CamImuSynchronizer.h>
#include <flea3/Flea3DynConfig.h>
#include <flea3/flea3_ros.h>
#include <imu_vn_100/imu_vn_100.h>

namespace cam_imu_sync {

CamImuSynchronizer::CamImuSynchronizer(const ros::NodeHandle &pnh)
    : pnh_(pnh), imu_nh_(pnh, "imu"), cam_cfg_server_(pnh_) {
  // Initialize imu
  imu_ = boost::make_shared<Imu>(imu_nh_);
  imu_->Stream(true);

  // Initialize cameras
  int num_cameras = 0;
  if (!pnh_.getParam("num_cameras", num_cameras)) {
    throw std::runtime_error("Number of cameras is not specified");
  }

  ROS_INFO("Initializing %d cameras.", num_cameras);
  for (int i = 0; i < num_cameras; ++i) {
    const auto prefix = "cam" + std::to_string(i);
    cameras_.push_back(boost::make_shared<Cam>(pnh_, prefix));
  }

  // Initialize reconfigure server
  cam_cfg_server_.setCallback(
      boost::bind(&CamImuSynchronizer::configure, this, _1, _2));
  // listen to changes in exposure value
  exp_sub_ =
      pnh_.subscribe("exposure", 1, &CamImuSynchronizer::expCallback, this);
}

CamImuSynchronizer::~CamImuSynchronizer() { stopPoll(); }

void CamImuSynchronizer::expCallback(
    const std_msgs::Float64MultiArray::ConstPtr &expMsg) {
  std::lock_guard<std::mutex> lock(exp_mutex_);
  double shutter(expMsg->data[0]), gain(expMsg->data[1]);
  if (shutter >= 0) {
    optimal_shutter_ = shutter;
    update_shutter_ = true;
  }
  if (gain >= 0) {
    optimal_gain_ = gain;
    update_gain_ = true;
  }
}

void CamImuSynchronizer::setShutter(double s) {
  bool auto_shutter(false);
  double shutter = std::min(s, max_shutter_ms_);
  double rs(shutter);
  for (auto &cam : cameras_) {
    cam->camera().SetShutter(auto_shutter, rs);
  }
  ROS_INFO("set shutter to: %10.4fms, driver returned %8.4fms", shutter, rs);
}

void CamImuSynchronizer::setGain(double g) {
  bool auto_gain(false);
  double rg(g);
  for (auto &cam : cameras_) {
    cam->camera().SetGain(auto_gain, rg);
  }
  ROS_INFO("set gain to:    %10.4fdb, driver returned %8.4fdb", g, rg);
}

static double clamp_shutter(const flea3::Flea3DynConfig &config) {
  double maxShutter =
      fmax(1000.0 / config.fps - 1000.0 / config.max_free_fps - 2.0, 0.0001);
  maxShutter = std::max(maxShutter, 0.0);
  return (maxShutter);
}

void CamImuSynchronizer::configure(Config &config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }

  stopPoll();
  // Configure cameras
  max_shutter_ms_ = clamp_shutter(config);
  if (config.shutter_ms > max_shutter_ms_) {
    ROS_WARN("configured shutter_ms %5.2f dropped to %5.2f due to frame rate!",
             config.shutter_ms, max_shutter_ms_);
    config.shutter_ms = max_shutter_ms_;
  }
  configureCameras(config);
  startPoll();
}

void CamImuSynchronizer::pollThread(int camid) {
  double lastTimeStamp, timeStamp;
  ros::Time time = ros::Time::now();
  while (ros::ok()) {
    // check and change shutter/gain if needed
    updateExposure();
    auto imageMsg = boost::make_shared<sensor_msgs::Image>();
    ros::Time t0 = ros::Time::now();
    // Grab frame from the camera. Since this is a blocking call,
    // we expect both camera threads to hang here until the strobe
    // from the IMU has arrived and triggered a frame grab on the camera.
    bool ret = cameras_[camid]->GrabWithTimestamp(imageMsg, &timeStamp);
    ros::Time t1 = ros::Time::now();
    bool publishFrame(true);
    if (!ret) {
      // This should pretty much never happen
      ROS_WARN("camera frame grab failed for camera %d!", camid);
      publishFrame = false;
    }
    // Now examine the timestamps we received from the camera. The timestamps
    // are in seconds, and wrap around after 128s. If a frame is missed (due to
    // e.g CPU load), it should show up here.
    if (lastTimeStamp > timeStamp) lastTimeStamp -= 128.0;  // wrap around
    double dt = timeStamp - lastTimeStamp;
    if (dt > dt_warn_cam_timestamp_) {
      ROS_WARN("cam %d camera timestamps: gap of %.5f frames", camid,
               dt * (double)imu_->sync_info().rate - 1.0);
      publishFrame = false;
    }
    lastTimeStamp = timeStamp;

    // Wait here until the IMU has had a chance to update the global timestamp.
    // Since the IMU has already triggered the strobe, and the camera threads
    // had to wait for the image exposure and frame delivery, by the time the
    // camera threads get here they should not have to wait at all. If however a
    // cam thread has somehow fallen behind by 1 frame, a vicious circle sets
    // in: It will be hanging here, waiting for the IMU to update the timestamp,
    // and when it does so, the thread will quickly pick up the frame *from the
    // previous strobe*, then hang again waiting for the IMU to strobe and
    // update the timestamp etc. For this reason there is a generous timeout
    // provided. If it is exceeded, we know the thread has fallen behind by one
    // frame.
    ros::Time imu_sync_time =
        imu_->sync_info().waitForNewTime(time, max_wait_for_imu_);
    ros::Time t2 = ros::Time::now();
    if (imu_sync_time <= time) {
      // So probably the situation mentioned above is occuring: we have fallen
      // behind, and there is an extra frame in the buffer. Let's try and drain
      // it.
      ret = cameras_[camid]->GrabNonBlockingWithTimestamp(imageMsg, &timeStamp);
      if (!ret) publishFrame = false;
      ROS_WARN("cam %d was blocked on IMU, recovered %s draining frame", camid,
               ret ? "by" : "without");
    }
    // ROS_INFO_STREAM("cam " << camid << " grab time: " << (t1-t0) << " sync
    // time: " << (t2-t1));
    if (imu_sync_time - time > dt_warn_frame_frame_) {
      // One more check: any gaps in the IMU time stamps?
      ROS_WARN("cam %d frame arrived late by %.5f frames", camid,
               (imu_sync_time - time).toSec() * (double)imu_->sync_info().rate);
      publishFrame = false;
    }
    time = imu_sync_time;

    // if all frames have been grabbed successfully, publish them!
    if (publishFrame) {
      imageMsg->header.stamp = time;
      cameras_[camid]->Publish(imageMsg);
    }
    {
      std::unique_lock<std::mutex> lock(pollMutex_);
      if (!keepPolling_) break;
    }
  }
}

void CamImuSynchronizer::configureCameras(Config &config) {
  config.fps = imu_->sync_info().rate;
  // print warning if successive time stamps
  // from the IMU are more than two frames apart
  double T = 1.0 / (double)imu_->sync_info().rate;
  dt_warn_frame_frame_ = ros::Duration(2.0 * T);
  dt_warn_imu_frame_ = ros::Duration(1.0 * T);
  dt_warn_cam_timestamp_ = 1.5 * T;
  max_wait_for_imu_ = (int64_t)(0.25 * (1e9 * T));

  for (auto &cam : cameras_) {
    cam->Stop();
    cam->camera().Configure(config);
    cam->set_fps(config.fps);
    cam->Start();
  }
}

void CamImuSynchronizer::startPoll() {
  std::unique_lock<std::mutex> lock(pollMutex_);
  if (pollThreads_.empty()) {
    keepPolling_ = true;
    for (int i = 0; i < cameras_.size(); ++i) {
      pollThreads_.push_back(boost::make_shared<boost::thread>(
          &CamImuSynchronizer::pollThread, this, i));
    }
  }
}

void CamImuSynchronizer::stopPoll() {
  // signal we are done
  {
    std::unique_lock<std::mutex> lock(pollMutex_);
    keepPolling_ = false;
  }
  // collect the threads
  if (!pollThreads_.empty()) {
    for (auto &th : pollThreads_) {
      th->join();
    }
    pollThreads_.clear();
  }
}
}  // namespace cam_imu_sync
