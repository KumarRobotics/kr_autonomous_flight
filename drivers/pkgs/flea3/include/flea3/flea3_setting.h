#pragma once

#include <flycapture/FlyCapture2.h>

#include <string>

namespace flea3 {

using namespace FlyCapture2;

void PgrError(const Error& error, const std::string& message = "");
bool PgrWarn(const Error& error, const std::string& message = "");

void PrintPropertyInfo(const PropertyInfo& prop_info,
                       const std::string& prop_name);
void PrintProperty(const Property& prop, const std::string& prop_name);

std::string BayerFormatToEncoding(const BayerTileFormat& bayer_format,
                                  unsigned bits_per_pixel);
std::string PixelFormatToEncoding(unsigned bits_per_pixel);

PropertyInfo GetPropertyInfo(Camera& camera, const PropertyType& prop_type);
Property GetProperty(Camera& camera, const PropertyType& prop_type);
std::pair<Format7Info, bool> GetFormat7Info(Camera& camera, const Mode& mode);
CameraInfo GetCameraInfo(Camera& camera);
float GetCameraFrameRate(Camera& camera);
float GetCameraTemperature(Camera& camera);
FrameRate GetMaxFrameRate(Camera& camera, const VideoMode& video_mode);
std::pair<VideoMode, FrameRate> GetVideoModeAndFrameRate(Camera& camera);
Format7ImageSettings GetFormat7ImageSettings(Camera& camera);

void SetProperty(Camera& camera, const PropertyType& prop_type, bool on,
                 bool auto_on, double value);

unsigned ReadRegister(Camera& camera, unsigned address);
void WriteRegister(Camera& camera, unsigned address, unsigned value);

bool IsAutoWhiteBalanceSupported(Camera& camera);
bool IsFormat7Supported(Camera& camera);
std::pair<Format7PacketInfo, bool> IsFormat7SettingsValid(
    Camera& camera, Format7ImageSettings& fmt7_settings);
bool IsVideoModeSupported(Camera& camera, const VideoMode& video_mode);
bool IsVideoModeAndFrameRateSupported(Camera& camera,
                                      const VideoMode& video_mode,
                                      const FrameRate& frame_rate);

std::pair<int, int> CenterRoi(int size, int max_size, int step);

}  // namespace flea3
