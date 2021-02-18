#include <iostream>
#include <sstream>
#include <stdexcept>

#include "flea3/flea3_setting.h"

using namespace FlyCapture2;
using namespace flea3;

std::string InterfaceTypeToString(const InterfaceType& interface_type) {
  switch (interface_type) {
    case INTERFACE_IEEE1394:
      return "IEEE1394";
    case INTERFACE_USB2:
      return "USB2.0";
    case INTERFACE_USB3:
      return "USB3.0";
    case INTERFACE_GIGE:
      return "GigE";
    case INTERFACE_UNKNOWN:
    default:
      return "unknown";
  }
}

void printCameraInfo(const CameraInfo& cinfo) {
  const auto interface_type = InterfaceTypeToString(cinfo.interfaceType);
  std::cout << "Serial: " << cinfo.serialNumber
            << ", Model: " << cinfo.modelName
            << ", Vendor: " << cinfo.vendorName
            << ", Sensor: " << cinfo.sensorInfo
            << ", Resolution: " << cinfo.sensorResolution
            << ", Color: " << std::boolalpha << cinfo.isColorCamera
            << ", Firmware Version: " << cinfo.firmwareVersion
            << ", Interface Type: " << interface_type << std::endl;
}

int main(int argc, char** argv) {
  BusManager bus_manager;

  try {
    unsigned num_devices = 0;
    PgrError(bus_manager.GetNumOfCameras(&num_devices),
             "Failed get number of cameras");
    if (num_devices) {
      std::cout << "Number of cameras found: " << num_devices << std::endl;
      for (unsigned i = 0; i < num_devices; ++i) {
        PGRGuid guid;
        PgrError(bus_manager.GetCameraFromIndex(i, &guid),
                 "Failed to get camera from index" + std::to_string(i));

        Camera camera;
        PgrError(camera.Connect(&guid), "Failed to connect to camera");

        CameraInfo cinfo;
        PgrError(camera.GetCameraInfo(&cinfo), "Failed to get camera info");

        std::cout << "[" << i << "]";
        printCameraInfo(cinfo);
      }
    } else {
      // No cameras found
      std::cout << "No PointGrey cameras detected on this computer."
                << std::endl
                << std::endl;

      std::cout << "Note that you may need to restart udev and "
                   "replug your camera, eg:"
                << std::endl
                << "  sudo service udev restart" << std::endl;
    }
  } catch (const std::runtime_error& e) {
    std::cout << "There was an error checking the active cameras: " << e.what()
              << std::endl;
  }
}
