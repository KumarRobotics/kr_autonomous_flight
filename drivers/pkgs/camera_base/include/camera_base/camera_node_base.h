#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <memory>
#include <thread>

#include "camera_base/camera_ros_base.h"

namespace camera_base {

/**
 * @brief The CameraNodeBase class
 * A base class that implements a ros node for a camera
 */
template <typename ConfigType>
class CameraNodeBase {
 public:
  explicit CameraNodeBase(const ros::NodeHandle& pnh)
      : is_acquire_(false), pnh_(pnh), cfg_server_(pnh) {}

  CameraNodeBase() = delete;
  CameraNodeBase(const CameraNodeBase&) = delete;
  CameraNodeBase& operator=(const CameraNodeBase&) = delete;
  virtual ~CameraNodeBase() = default;

  const ros::NodeHandle& pnh() const { return pnh_; }
  bool is_acquire() const { return is_acquire_; }

  /**
   * @brief Run Run the node
   * This will setup the dynamic reconfigure server, this will start the
   * acquisition automatically when the server is initialized
   */
  void Run() {
    cfg_server_.setCallback(
        boost::bind(&CameraNodeBase::ConfigCb, this, _1, _2));
  }

  /**
   * @brief End
   */
  void End() { Stop(); }

  void Sleep() const { rate_->sleep(); }

  /**
   * @brief ConfigCb Dynamic reconfigure callback
   * @param config Config type
   * @param level Reconfigure level, not really used
   * Entering this callback will stop the acquisition thread, do the
   * reconfiguration and restart acquisition thread
   */
  void ConfigCb(ConfigType& config, int level) {
    if (level < 0) {
      ROS_INFO("%s: %s", pnh().getNamespace().c_str(),
               "Initializing reconfigure server");
    }
    if (is_acquire()) {
      Stop();
    }
    Setup(config);
    SetRate(config.fps);
    Start();
  }

  /**
   * @brief Acquire Do acquisition here
   */
  virtual void Acquire() = 0;

  /**
   * @brief Setup Setup your camera here
   * @param config Config type
   */
  virtual void Setup(ConfigType& config) = 0;

 private:
  void SetRate(double fps) { rate_.reset(new ros::Rate(fps)); }

  void Start() {
    is_acquire_ = true;
    acquire_thread_.reset(new std::thread(&CameraNodeBase::Acquire, this));
  }

  void Stop() {
    if (!is_acquire_) return;
    is_acquire_ = false;
    acquire_thread_->join();
  }

  bool is_acquire_;
  ros::NodeHandle pnh_;
  std::unique_ptr<ros::Rate> rate_;
  std::unique_ptr<std::thread> acquire_thread_;
  dynamic_reconfigure::Server<ConfigType> cfg_server_;
};

}  // namespace camera_base
