#include "flea3/flea3_camera.h"

#include <sensor_msgs/fill_image.h>

#include <utility>

#include "flea3/flea3_setting.h"

namespace flea3 {

using namespace FlyCapture2;

unsigned hsb(unsigned i) {
  unsigned c = 0;
  while (i >>= 1) {
    c++;
  }
  return c;
}

union AbsValueConversion {
  unsigned int uint_val;
  float float_val;
};

Flea3Camera::Flea3Camera(const std::string& serial) : serial_(serial) {
  // Wait for camera to power up
  int num_tries{3};
  while (num_tries > 0) {
    if (Connect()) break;
    usleep(100000);  // Sleep for 100ms
    --num_tries;
  }
  if (num_tries == 0) {
    throw std::runtime_error("Failed after multiple tries, abort.");
  }
}

Flea3Camera::~Flea3Camera() {
  if (camera_.IsConnected()) {
    TurnOffStrobe({1, 2, 3});
    PgrError(camera_.Disconnect(), "Failed to disconnect camera");
  }
}

bool Flea3Camera::Connect() {
  PGRGuid guid;
  PgrError(bus_manager_.GetCameraFromSerialNumber(serial_id(), &guid),
           serial_ + " not found. " + AvailableDevice());

  const auto error = camera_.Connect(&guid);
  if (error == PGRERROR_OK) {
    // This is a total hack, it exists because one of my camera doesn't enable
    // auto white balance by default. You have to write to its presence register
    // to bring it online.
    //    EnableAutoWhiteBalance();
    // For now this only set the grab timeout
    SetConfiguration();
    return true;
  } else {
    ROS_INFO("Failed to connect to camera: %s. Try again. | %s",
             serial().c_str(), error.GetDescription());
    return false;
  }
}

void Flea3Camera::SetConfiguration() {
  FC2Config config;
  PgrError(camera_.GetConfiguration(&config), "Failed to get configuration");
  // Set the grab timeout to 1 seconds
  config.grabTimeout = 1000;
  // Try 2 times before declaring failure
  config.registerTimeoutRetries = 2;
  // NOTE: Cannot do this here, will block all the following settings on format7
  // Maybe put this in configure
  //  config.highPerformanceRetrieveBuffer = true;

  enableTimestamps();  // is this the right place to enable timestamps?

  // Set the camera configuration
  PgrError(camera_.SetConfiguration(&config), "Failed to set configuration");
}

std::string Flea3Camera::AvailableDevice() {
  unsigned num_devices = 0;
  PgrError(bus_manager_.GetNumOfCameras(&num_devices),
           "Failed to get number for cameras");

  std::string devices = std::to_string(num_devices) + " available device(s): ";
  for (unsigned i = 0; i < num_devices; ++i) {
    unsigned serial_id;
    PgrError(bus_manager_.GetCameraSerialNumberFromIndex(i, &serial_id),
             "Failed to get camera serial number from index");
    devices += std::to_string(serial_id) + " ";
  }
  return devices;
}

void Flea3Camera::StartCapture() {
  if (camera_.IsConnected() && !capturing_) {
    PgrError(camera_.StartCapture(), "Failed to start capture");
    capturing_ = true;
  }
}

void Flea3Camera::StopCapture() {
  if (camera_.IsConnected() && capturing_) {
    PgrError(camera_.StopCapture(), "Failed to stop capture");
    capturing_ = false;
  }
}

void Flea3Camera::Configure(Config& config) {
  // Video Mode
  SetVideoMode(config.video_mode, config.format7_mode, config.pixel_format,
               config.width, config.height);

  // Update CameraInfo here after video mode is changed
  camera_info_ = GetCameraInfo(camera_);

  // Frame Rate
  SetFrameRate(config.fps);

  // Raw Bayer
  SetRawBayerOutput(config.raw_bayer_output);

  // White Balance
  SetWhiteBalanceRedBlue(config.white_balance, config.auto_white_balance,
                         config.wb_red, config.wb_blue);

  // Exposure
  SetExposure(config.exposure, config.auto_exposure, config.exposure_value);
  SetShutter(config.auto_shutter, config.shutter_ms);
  SetGain(config.auto_gain, config.gain_db);

  SetBrightness(config.brightness);
  SetGamma(config.gamma);

  // Strobe
  SetStrobe(config.strobe_control, config.strobe_polarity);
  // Trigger
  SetTrigger(config.trigger_source, config.trigger_polarity,
             config.trigger_mode);

  // Save this config
  config_ = config;
}

// TODO: simplify logic here
void Flea3Camera::SetVideoMode(int& video_mode, int& format7_mode,
                               int& pixel_format, int& width, int& height) {
  // Try setting video mode based on video_mode, fail silently
  if (video_mode == Flea3Dyn_format7) {
    SetFormat7VideoMode(format7_mode, pixel_format, width, height);
  } else {
    SetStandardVideoMode(video_mode);
  }

  // Update params
  const auto video_mode_and_frame_rate_pg = GetVideoModeAndFrameRate(camera_);
  if (video_mode_and_frame_rate_pg.first == VIDEOMODE_FORMAT7) {
    const auto fmt7_settings = GetFormat7ImageSettings(camera_);
    pixel_format = hsb(fmt7_settings.pixelFormat);
    width = fmt7_settings.width;
    height = fmt7_settings.height;
    format7_mode = fmt7_settings.mode;
    video_mode = Flea3Dyn_format7;
  } else {
    video_mode = static_cast<int>(video_mode_and_frame_rate_pg.first);
    format7_mode = 0;
    pixel_format = 0;
    width = 0;
    height = 0;
  }
}

void Flea3Camera::SetFormat7VideoMode(int format7_mode, int pixel_format,
                                      int width, int height) {
  const auto fmt7_mode_pg = static_cast<Mode>(format7_mode);
  const auto fmt7_info = GetFormat7Info(camera_, fmt7_mode_pg);
  if (!fmt7_info.second) return;

  Format7ImageSettings fmt7_settings;
  fmt7_settings.mode = fmt7_mode_pg;
  // Set format7 pixel format
  // The 22 here corresponds to PIXEL_FORMAT_RAW8
  pixel_format = (pixel_format == 0) ? 22 : pixel_format;
  fmt7_settings.pixelFormat = static_cast<PixelFormat>(1 << pixel_format);
  // Set format7 ROI
  // NOTE: Center ROI for now
  SetRoi(fmt7_info.first, fmt7_settings, width, height);

  // Validate the settings
  const auto fmt7_packet_info = IsFormat7SettingsValid(camera_, fmt7_settings);
  if (!fmt7_packet_info.second) ROS_WARN("Format 7 Setting is not valid");
  PgrWarn(camera_.SetFormat7Configuration(
              &fmt7_settings, fmt7_packet_info.first.recommendedBytesPerPacket),
          "Failed to set format7 mode");
}

void Flea3Camera::SetStandardVideoMode(int video_mode) {
  const auto video_mode_pg = static_cast<VideoMode>(video_mode);
  if (!IsVideoModeSupported(camera_, video_mode_pg)) return;
  const auto max_frame_rate_pg = GetMaxFrameRate(camera_, video_mode_pg);
  PgrWarn(camera_.SetVideoModeAndFrameRate(video_mode_pg, max_frame_rate_pg));
}

void Flea3Camera::SetFrameRate(double& frame_rate) {
  SetProperty(camera_, FRAME_RATE, true, false, frame_rate);
  const auto prop = GetProperty(camera_, FRAME_RATE);
  if (prop.onOff) {
    frame_rate = prop.absValue;
  }
}

void Flea3Camera::setNonBlocking() {
  FC2Config config;
  PgrError(camera_.GetConfiguration(&config), "Failed to get configuration");
  config.grabTimeout = TIMEOUT_NONE;  // return immediately
  PgrError(camera_.SetConfiguration(&config), "Failed to set configuration");
}

void Flea3Camera::setBlocking() {
  FC2Config config;
  PgrError(camera_.GetConfiguration(&config), "Failed to get configuration");
  config.grabTimeout = 1000;  // set to one second;
  PgrError(camera_.SetConfiguration(&config), "Failed to set configuration");
}

void Flea3Camera::enableTimestamps() {
  unsigned int reg_val = 0;
  Error error = camera_.ReadRegister(0x12F8, &reg_val);
  if (error == PGRERROR_OK) {
    WriteRegister(camera_, 0x12F8, (1 << 0) | (reg_val));
  }
}

static inline void get_encoding(const Image& image, bool isColorCam,
                                std::string* encoding) {
  // Set image encodings
  const auto bayer_format = image.GetBayerTileFormat();
  const auto bits_per_pixel = image.GetBitsPerPixel();
  if (isColorCam) {
    if (bayer_format != NONE) {
      *encoding = BayerFormatToEncoding(bayer_format, bits_per_pixel);
    } else if (bits_per_pixel == 24) {
      *encoding = sensor_msgs::image_encodings::RGB8;
    } else {
      *encoding = PixelFormatToEncoding(bits_per_pixel);
    }
  } else {
    *encoding = PixelFormatToEncoding(bits_per_pixel);
  }
}

bool Flea3Camera::GrabImageNonBlocking(sensor_msgs::Image& image_msg) {
  setNonBlocking();
  bool ret = GrabImage(image_msg);
  setBlocking();
  return (ret);
}

bool Flea3Camera::GrabImageNonBlockingWithTimestamp(
    sensor_msgs::Image& image_msg, double* ts) {
  setNonBlocking();
  bool ret = GrabImageWithTimestamp(image_msg, ts);
  setBlocking();
  return (ret);
}

bool Flea3Camera::GrabImage(sensor_msgs::Image& image_msg) {
  if (!(camera_.IsConnected() && capturing_)) return false;

  Image image;
  const auto error = camera_.RetrieveBuffer(&image);
  if (error != PGRERROR_OK) return false;

  std::string encoding;
  get_encoding(image, camera_info_.isColorCamera, &encoding);
  return sensor_msgs::fillImage(image_msg, encoding, image.GetRows(),
                                image.GetCols(), image.GetStride(),
                                image.GetData());
}

bool Flea3Camera::GrabImageWithTimestamp(sensor_msgs::Image& image_msg,
                                         double* tstamp) {
  if (!(camera_.IsConnected() && capturing_)) return false;

  Image image;
  const auto error = camera_.RetrieveBuffer(&image);
  if (error != PGRERROR_OK) return false;
  // 2) Then read it from the image structure:
  //
  //    TimeStamp ts = image.GetTimeStamp();
  //    now look at ptgrey manuals for meaning of fields:
  //    ts.cycleSeconds, ts.cycleCount, ts.cycleOffset
  TimeStamp ts = image.GetTimeStamp();
  const double CYCLE_COUNT_TO_SEC = 0.000125;                    // 8kHz
  const double CYCLE_OFFSET_TO_SEC = CYCLE_COUNT_TO_SEC / 4096;  // 12 bit

  *tstamp = ts.cycleSeconds + ts.cycleCount * CYCLE_COUNT_TO_SEC +
            ts.cycleOffset * CYCLE_OFFSET_TO_SEC;

  std::string encoding;
  get_encoding(image, camera_info_.isColorCamera, &encoding);
  return sensor_msgs::fillImage(image_msg, encoding, image.GetRows(),
                                image.GetCols(), image.GetStride(),
                                image.GetData());
}

// void Flea3Camera::GrabImageMetadata(ImageMetadata& image_metadata_msg) {
//  AbsValueConversion abs_val;
//  // These registers can be found in register reference manual
//  camera_.ReadRegister(0x908, &abs_val.uint_val);
//  image_metadata_msg.exposure_value = abs_val.float_val;

//  // The value read from abs register is in seconds
//  camera_.ReadRegister(0x918, &abs_val.uint_val);
//  image_metadata_msg.shutter_ms = abs_val.float_val * 1000;

//  camera_.ReadRegister(0x928, &abs_val.uint_val);
//  image_metadata_msg.gain_db = abs_val.float_val;

//  camera_.ReadRegister(0x938, &abs_val.uint_val);
//  image_metadata_msg.brightness = abs_val.float_val;
//}

void Flea3Camera::SetWhiteBalanceRedBlue(bool& white_balance,
                                         bool& auto_white_balance, int& red,
                                         int& blue) {
  if (!camera_info_.isColorCamera) {
    // Not even a color camera
    ROS_WARN("Camera %s is not a color camera, white balance not supported",
             serial().c_str());
    auto_white_balance = false;
    white_balance = false;
    red = 0;
    blue = 0;
    return;
  }

  // Check if white balance is supported
  const auto prop_info = GetPropertyInfo(camera_, WHITE_BALANCE);
  if (!prop_info.present) {
    white_balance = false;
    auto_white_balance = false;
    blue = 0;
    red = 0;
    return;
  }

  // Set white balance
  Error error;
  Property prop;
  prop.type = WHITE_BALANCE;
  prop.onOff = white_balance;
  prop.autoManualMode = auto_white_balance;
  prop.absControl = false;
  prop.valueA = red;
  prop.valueB = blue;
  error = camera_.SetProperty(&prop);

  // Get white balance
  camera_.GetProperty(&prop);
  white_balance = prop.onOff;
  auto_white_balance = prop.autoManualMode;
  red = prop.valueA;
  blue = prop.valueB;
}

void Flea3Camera::EnableAutoWhiteBalance() {
  WriteRegister(camera_, 0x80C, 1 << 31);
}

void Flea3Camera::SetExposure(bool& exposure, bool& auto_exposure,
                              double& exposure_value) {
  const auto prop_type = AUTO_EXPOSURE;
  SetProperty(camera_, prop_type, exposure, auto_exposure, exposure_value);
  const auto prop = GetProperty(camera_, prop_type);
  exposure = prop.onOff;
  auto_exposure = prop.autoManualMode;
  exposure_value = prop.absValue;
}

void Flea3Camera::SetShutter(bool& auto_shutter, double& shutter_ms) {
  const auto prop_type = SHUTTER;
  SetProperty(camera_, prop_type, true, auto_shutter, shutter_ms);
  const auto prop = GetProperty(camera_, prop_type);
  auto_shutter = prop.autoManualMode;
  shutter_ms = prop.absValue;
}

void Flea3Camera::SetGain(bool& auto_gain, double& gain_db) {
  const auto prop_type = GAIN;
  SetProperty(camera_, prop_type, true, auto_gain, gain_db);
  const auto prop = GetProperty(camera_, prop_type);
  auto_gain = prop.autoManualMode;
  gain_db = prop.absValue;
}

void Flea3Camera::SetBrightness(double& brightness) {
  const auto prop_type = BRIGHTNESS;
  SetProperty(camera_, prop_type, true, false, brightness);
  const auto prop = GetProperty(camera_, prop_type);
  brightness = prop.absValue;
}

void Flea3Camera::SetGamma(double& gamma) {
  const auto prop_type = GAMMA;
  SetProperty(camera_, prop_type, true, false, gamma);
  const auto prop = GetProperty(camera_, prop_type);
  gamma = prop.absValue;
}

void Flea3Camera::SetRawBayerOutput(bool& raw_bayer_output) {
  // Because this only works in standard video mode, we only enable this if
  // video mode is not format 7
  const auto video_mode_frame_rate_pg = GetVideoModeAndFrameRate(camera_);
  if (video_mode_frame_rate_pg.first == VIDEOMODE_FORMAT7) {
    raw_bayer_output = false;
    return;
  }
  // See Point Grey Register Reference document section 5.8
  WriteRegister(camera_, 0x1050, static_cast<unsigned>(raw_bayer_output));
}

void Flea3Camera::SetRoi(const Format7Info& format7_info,
                         Format7ImageSettings& format7_settings, int width,
                         int height) {
  const auto width_setting =
      CenterRoi(width, format7_info.maxWidth, format7_info.imageHStepSize);
  const auto height_setting =
      CenterRoi(height, format7_info.maxHeight, format7_info.imageVStepSize);
  format7_settings.width = width_setting.first;
  format7_settings.offsetX = width_setting.second;
  format7_settings.height = height_setting.first;
  format7_settings.offsetY = height_setting.second;
}

void Flea3Camera::SetTrigger(int& trigger_source, int& trigger_polarity,
                             int& trigger_mode) {
  TriggerModeInfo trigger_mode_info;
  PgrWarn(camera_.GetTriggerModeInfo(&trigger_mode_info),
          "Failed to get trigger mode info");
  // Check if trigger is supported
  if (!trigger_mode_info.present) {
    if (trigger_source == Flea3Dyn_ts_sw &&
        !trigger_mode_info.softwareTriggerSupported) {
      ROS_WARN("Camera does not support software trigger");
      trigger_source = Flea3Dyn_ts_off;
    }
    ROS_WARN("Camera does not support external trigger");
    trigger_source = Flea3Dyn_ts_off;
    return;
  }

  // Turn off external trigger
  TriggerMode trigger_mode_struct;
  if (trigger_source == Flea3Dyn_ts_off) {
    trigger_mode_struct.onOff = false;
    PgrWarn(camera_.SetTriggerMode(&trigger_mode_struct),
            "Failed to set trigger mode");
    return;
  }

  trigger_mode_struct.onOff = true;
  trigger_mode_struct.mode = trigger_mode;
  trigger_mode_struct.parameter = 0;

  // Source 7 means software trigger
  trigger_mode_struct.source = trigger_source;
  trigger_mode_struct.polarity = trigger_polarity;
  PgrWarn(camera_.SetTriggerMode(&trigger_mode_struct),
          "Failed to set trigger mode");
  PgrWarn(camera_.GetTriggerMode(&trigger_mode_struct),
          "Failed to get trigger mode");
  trigger_source = trigger_mode_struct.source;
  trigger_polarity = trigger_mode_struct.polarity;
  trigger_mode = trigger_mode_struct.mode;
}

void Flea3Camera::SetStrobe(int& strobe_control, int& polarity) {
  // Turn off all strobe when we switch it off
  if (strobe_control == Flea3Dyn_sc_off) {
    TurnOffStrobe({1, 2, 3});
    return;
  }

  // Check if required strobe is supported
  StrobeInfo strobe_info;
  strobe_info.source = strobe_control;
  PgrWarn(camera_.GetStrobeInfo(&strobe_info), "Failed to get strobe info");
  if (!strobe_info.present) {
    ROS_WARN("Camera does not support strobe");
    strobe_control = Flea3Dyn_sc_off;
    return;
  }

  StrobeControl strobe;
  strobe.source = strobe_control;
  strobe.onOff = true;
  strobe.polarity = polarity;
  strobe.duration = 0;
  PgrWarn(camera_.SetStrobe(&strobe), "Failed to set strobe");
}

void Flea3Camera::TurnOffStrobe(const std::vector<int>& strobes) {
  for (const auto& s : strobes) {
    StrobeControl strobe;
    strobe.source = s;
    strobe.onOff = false;
    PgrWarn(camera_.SetStrobe(&strobe), "Failed to set strobe");
  }
}

bool Flea3Camera::PollForTriggerReady() {
  const unsigned int software_trigger_addr = 0x62C;
  unsigned int reg_val = 0;

  Error error;
  do {
    error = camera_.ReadRegister(software_trigger_addr, &reg_val);
    if (error != PGRERROR_OK) {
      return false;
    }
  } while ((reg_val >> 31) != 0);

  return true;
}

bool Flea3Camera::FireSoftwareTrigger() {
  const unsigned software_trigger_addr = 0x62C;
  const unsigned fire = 0x80000000;
  return camera_.WriteRegister(software_trigger_addr, fire) == PGRERROR_OK;
}

bool Flea3Camera::RequestSingle() {
  if (config_.trigger_source == Flea3Dyn_ts_sw) {
    if (PollForTriggerReady()) {
      return FireSoftwareTrigger();
    }
    return false;
  }
  return true;
}

double Flea3Camera::GetShutterTimeSec() {
  if (config_.auto_shutter) {
    AbsValueConversion abs_val;
    // Register for abs_shutter_val, which is already in second
    camera_.ReadRegister(0x918, &abs_val.uint_val);
    return abs_val.float_val;
  }
  return config_.shutter_ms / 1000.0;
}

double Flea3Camera::GetGain() {
  if (config_.auto_gain) {
    AbsValueConversion abs_val;
    // Register for abs_val_gain, which is in db
    camera_.ReadRegister(0x928, &abs_val.uint_val);
    return abs_val.float_val;
  }
  return config_.gain_db;
}

}  // namespace flea3
