//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: ExtendedShutterEx.cpp,v 1.9 2009-12-08 18:58:36 soowei Exp $
//=============================================================================

#include <bitset>

#include "flycapture/FlyCapture2.h"

using namespace FlyCapture2;

enum ExtendedShutterType {
  NO_EXTENDED_SHUTTER,
  DRAGONFLY_EXTENDED_SHUTTER,
  GENERAL_EXTENDED_SHUTTER
};

void PrintBuildInfo() {
  FC2Version fc2Version;
  Utilities::GetLibraryVersion(&fc2Version);
  char version[128];
  sprintf(version, "FlyCapture2 library version: %d.%d.%d.%d\n",
          fc2Version.major, fc2Version.minor, fc2Version.type,
          fc2Version.build);

  printf("%s", version);

  char timeStamp[512];
  sprintf(timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__);

  printf("%s", timeStamp);
}

void PrintCameraInfo(CameraInfo* pCamInfo) {
  printf(
      "\n*** CAMERA INFORMATION ***\n"
      "Serial number - %u\n"
      "Camera model - %s\n"
      "Camera vendor - %s\n"
      "Sensor - %s\n"
      "Resolution - %s\n"
      "Firmware version - %s\n"
      "Firmware build time - %s\n\n",
      pCamInfo->serialNumber, pCamInfo->modelName, pCamInfo->vendorName,
      pCamInfo->sensorInfo, pCamInfo->sensorResolution,
      pCamInfo->firmwareVersion, pCamInfo->firmwareBuildTime);
}

void PrintError(Error error) { error.PrintErrorTrace(); }

int main(int /*argc*/, char** /*argv*/) {
  PrintBuildInfo();

  const int k_numImages = 5;

  Error error;

  BusManager busMgr;
  unsigned int numCameras;
  error = busMgr.GetNumOfCameras(&numCameras);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  printf("Number of cameras detected: %u\n", numCameras);

  if (numCameras < 1) {
    printf("Insufficient number of cameras... exiting\n");
    return -1;
  }

  PGRGuid guid;
  error = busMgr.GetCameraFromIndex(0, &guid);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  Camera cam;

  // Connect to a camera
  error = cam.Connect(&guid);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  // Get the camera information
  CameraInfo camInfo;
  error = cam.GetCameraInfo(&camInfo);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  PrintCameraInfo(&camInfo);

  // Check if the camera supports the FRAME_RATE property
  PropertyInfo propInfo;
  propInfo.type = FRAME_RATE;
  error = cam.GetPropertyInfo(&propInfo);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  ExtendedShutterType shutterType = NO_EXTENDED_SHUTTER;

  if (propInfo.present == true) {
    // Turn off frame rate

    Property prop;
    prop.type = FRAME_RATE;
    error = cam.GetProperty(&prop);
    if (error != PGRERROR_OK) {
      PrintError(error);
      return -1;
    }

    prop.autoManualMode = false;
    prop.onOff = false;

    error = cam.SetProperty(&prop);
    if (error != PGRERROR_OK) {
      PrintError(error);
      return -1;
    }

    shutterType = GENERAL_EXTENDED_SHUTTER;
  } else {
    // Frame rate property does not appear to be supported.
    // Disable the extended shutter register instead.
    // This is only applicable for Dragonfly.

    const unsigned int k_extendedShutter = 0x1028;
    unsigned int extendedShutterRegVal = 0;

    error = cam.ReadRegister(k_extendedShutter, &extendedShutterRegVal);
    if (error != PGRERROR_OK) {
      PrintError(error);
      return -1;
    }

    std::bitset<32> extendedShutterBS((int)extendedShutterRegVal);
    if (extendedShutterBS[31] == true) {
      // Set the camera into extended shutter mode
      error = cam.WriteRegister(k_extendedShutter, 0x80020000);
      if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
      }
    } else {
      printf("Frame rate and extended shutter are not supported... exiting\n");
      return -1;
    }

    shutterType = DRAGONFLY_EXTENDED_SHUTTER;
  }

  // Set the shutter property of the camera
  Property prop;
  prop.type = SHUTTER;
  error = cam.GetProperty(&prop);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  prop.autoManualMode = false;
  prop.absControl = true;

  const float k_shutterVal = 3000.0;
  prop.absValue = k_shutterVal;

  error = cam.SetProperty(&prop);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  printf("Shutter time set to %.2fms\n", k_shutterVal);

  // Enable timestamping
  EmbeddedImageInfo embeddedInfo;

  error = cam.GetEmbeddedImageInfo(&embeddedInfo);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  if (embeddedInfo.timestamp.available != 0) {
    embeddedInfo.timestamp.onOff = true;
  }

  error = cam.SetEmbeddedImageInfo(&embeddedInfo);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  // Start the camera
  error = cam.StartCapture();
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  for (int i = 0; i < k_numImages; i++) {
    Image image;
    error = cam.RetrieveBuffer(&image);
    if (error != PGRERROR_OK) {
      PrintError(error);
      return -1;
    }

    TimeStamp timestamp = image.GetTimeStamp();
    printf("TimeStamp [%d %d]\n", timestamp.cycleSeconds, timestamp.cycleCount);
  }

  // Stop capturing images
  error = cam.StopCapture();
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  // Set the camera back to its original state

  prop.type = SHUTTER;
  error = cam.GetProperty(&prop);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  prop.autoManualMode = true;

  error = cam.SetProperty(&prop);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  if (shutterType == GENERAL_EXTENDED_SHUTTER) {
    Property prop;
    prop.type = FRAME_RATE;
    error = cam.GetProperty(&prop);
    if (error != PGRERROR_OK) {
      PrintError(error);
      return -1;
    }

    prop.autoManualMode = true;
    prop.onOff = true;

    error = cam.SetProperty(&prop);
    if (error != PGRERROR_OK) {
      PrintError(error);
      return -1;
    }

  } else if (shutterType == DRAGONFLY_EXTENDED_SHUTTER) {
    const unsigned int k_extendedShutter = 0x1028;
    unsigned int extendedShutterRegVal = 0;

    error = cam.ReadRegister(k_extendedShutter, &extendedShutterRegVal);
    if (error != PGRERROR_OK) {
      PrintError(error);
      return -1;
    }

    std::bitset<32> extendedShutterBS((int)extendedShutterRegVal);
    if (extendedShutterBS[31] == true) {
      // Set the camera into extended shutter mode
      error = cam.WriteRegister(k_extendedShutter, 0x80000000);
      if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
      }
    }
  }

  // Disconnect the camera
  error = cam.Disconnect();
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  printf("Done! Press Enter to exit...\n");
  getchar();

  return 0;
}
