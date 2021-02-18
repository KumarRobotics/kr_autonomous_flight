#pragma once

#include <flycapture/FlyCapture2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "flea3/Flea3DynConfig.h"
//#include "flea3/ImageMetadata.h"

namespace flea3 {
using namespace FlyCapture2;

class Flea3Camera {
 public:
  using Config = ::flea3::Flea3DynConfig;

  explicit Flea3Camera(const std::string& serial);
  ~Flea3Camera();

  const std::string& serial() const { return serial_; }
  unsigned serial_id() const { return std::atoi(serial_.c_str()); }

  bool GrabImage(sensor_msgs::Image& image_msg);
  bool GrabImageNonBlocking(sensor_msgs::Image& image_msg);
  bool GrabImageWithTimestamp(sensor_msgs::Image& image_msg, double* ts);
  bool GrabImageNonBlockingWithTimestamp(sensor_msgs::Image& image_msg,
                                         double* ts);
  //  void GrabImageMetadata(flea3::ImageMetadata& image_metadata_msg);

  bool Connect();
  void Configure(Config& config);
  void StartCapture();
  void StopCapture();
  bool RequestSingle();
  double GetShutterTimeSec();
  double GetGain();
  void SetShutter(bool& auto_shutter, double& shutter_ms);
  void SetGain(bool& auto_gain, double& gain_db);
  bool FireSoftwareTrigger();

 private:
  std::string AvailableDevice();
  void enableTimestamps();

  // Start up
  void EnableAutoWhiteBalance();
  void SetConfiguration();

  // Video Mode
  void SetVideoMode(int& video_mode, int& format7_mode, int& pixel_format,
                    int& width, int& height);
  void SetFormat7VideoMode(int format7_mode, int pixel_format, int width,
                           int height);
  void SetStandardVideoMode(int video_mode);
  void SetRoi(const Format7Info& format7_info,
              Format7ImageSettings& format7_settings, int width, int height);

  // Frame Rate
  void SetFrameRate(double& frame_rate);

  // White Balance
  void SetWhiteBalanceRedBlue(bool& white_balance, bool& auto_white_balance,
                              int& red, int& blue);

  // Raw Bayer
  void SetRawBayerOutput(bool& raw_bayer_output);

  void SetExposure(bool& exposure, bool& auto_exposure, double& exposure_value);
  void SetBrightness(double& brightness);
  void SetGamma(double& gamma);

  // Trigger
  void SetTrigger(int& trigger_source, int& trigger_polarity,
                  int& trigger_mode);
  bool PollForTriggerReady();

  // Strobe
  void SetStrobe(int& strobe_control, int& polarity);
  void TurnOffStrobe(const std::vector<int>& strobes);

  // Blocking/Non-blocking
  void setNonBlocking();
  void setBlocking();

  bool capturing_{false};
  BusManager bus_manager_;
  Camera camera_;
  CameraInfo camera_info_;
  std::string serial_;
  Config config_;
};

}  // namespace flea3
