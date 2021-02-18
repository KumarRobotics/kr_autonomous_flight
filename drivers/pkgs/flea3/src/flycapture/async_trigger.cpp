
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
// $Id: AsyncTriggerEx.cpp,v 1.21 2010-07-22 22:51:51 soowei Exp $
//=============================================================================

#include <unistd.h>
#include <iostream>

#include <flycapture/FlyCapture2.h>

//
// Software trigger the camera instead of using an external hardware trigger
//
#define SOFTWARE_TRIGGER_CAMERA

using namespace FlyCapture2;

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

void PrintProperty(const Property& prop) {
  std::cout << "present: " << prop.present
            << "\nabsControl: " << prop.absControl
            << "\nonePush:" << prop.onePush << "\nonOff:" << prop.onOff
            << "\nautoManualMode: " << prop.autoManualMode
            << "\nabsValue: " << prop.absValue << std::endl;
}

void PrintPropertyInfo(const PropertyInfo& prop_info) {}

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

void PrintError(const Error error) { error.PrintErrorTrace(); }

bool CheckSoftwareTriggerPresence(Camera* pCam) {
  const unsigned int k_triggerInq = 0x530;

  Error error;
  unsigned int regVal = 0;

  error = pCam->ReadRegister(k_triggerInq, &regVal);

  if (error != PGRERROR_OK) {
    PrintError(error);
    return false;
  }

  if ((regVal & 0x10000) != 0x10000) {
    return false;
  }

  return true;
}

bool PollForTriggerReady(Camera* pCam) {
  const unsigned int k_softwareTrigger = 0x62C;
  Error error;
  unsigned int regVal = 0;

  int num_polls = 0;
  do {
    error = pCam->ReadRegister(k_softwareTrigger, &regVal);
    if (error != PGRERROR_OK) {
      PrintError(error);
      return false;
    }
    std::cout << "polling: " << ++num_polls << std::endl;

  } while ((regVal >> 31) != 0);

  return true;
}

bool FireSoftwareTrigger(Camera* pCam) {
  const unsigned int k_softwareTrigger = 0x62C;
  const unsigned int k_fireVal = 0x80000000;
  Error error;

  error = pCam->WriteRegister(k_softwareTrigger, k_fireVal);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return false;
  }

  return true;
}

int main(int /*argc*/, char** /*argv*/) {
  PrintBuildInfo();

  const int k_numImages = 2;

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

  // Power on the camera
  const unsigned int k_cameraPower = 0x610;
  const unsigned int k_powerVal = 0x80000000;
  error = cam.WriteRegister(k_cameraPower, k_powerVal);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  const unsigned int millisecondsToSleep = 100;
  unsigned int regVal = 0;
  unsigned int retries = 10;

  // Wait for camera to complete power-up
  do {
#if defined(WIN32) || defined(WIN64)
    Sleep(millisecondsToSleep);
#else
    usleep(millisecondsToSleep * 1000);
#endif
    error = cam.ReadRegister(k_cameraPower, &regVal);
    if (error == PGRERROR_TIMEOUT) {
      // ignore timeout errors, camera may not be responding to
      // register reads during power-up
    } else if (error != PGRERROR_OK) {
      PrintError(error);
      return -1;
    }

    retries--;
  } while ((regVal & k_powerVal) == 0 && retries > 0);

  // Check for timeout errors after retrying
  if (error == PGRERROR_TIMEOUT) {
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

#ifndef SOFTWARE_TRIGGER_CAMERA
  // Check for external trigger support
  TriggerModeInfo triggerModeInfo;
  error = cam.GetTriggerModeInfo(&triggerModeInfo);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  if (triggerModeInfo.present != true) {
    printf("Camera does not support external trigger! Exiting...\n");
    return -1;
  }
#endif

  // Get current trigger settings
  TriggerMode triggerMode;
  error = cam.GetTriggerMode(&triggerMode);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  // Set camera to trigger mode 0
  triggerMode.onOff = true;
  triggerMode.mode = 0;
  triggerMode.parameter = 0;

#ifdef SOFTWARE_TRIGGER_CAMERA
  // A source of 7 means software trigger
  triggerMode.source = 7;
#else
  // Triggering the camera externally using source 0.
  triggerMode.source = 0;
#endif

  error = cam.SetTriggerMode(&triggerMode);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  // Poll to ensure camera is ready
  bool retVal = PollForTriggerReady(&cam);
  if (!retVal) {
    printf("\nError polling for trigger ready!\n");
    return -1;
  }

  // Get the camera configuration
  FC2Config config;
  error = cam.GetConfiguration(&config);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  // Set the grab timeout to 5 seconds
  config.grabTimeout = 1000;

  // Set the camera configuration
  error = cam.SetConfiguration(&config);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  Property prop;
  prop.type = FRAME_RATE;
  error = cam.GetProperty(&prop);
  if (error != PGRERROR_OK) {
    error.PrintErrorTrace();
    return -1;
  }
  std::cout << "Current frame rate: " << std::endl;
  PrintProperty(prop);

  prop.onOff = false;
  prop.absControl = true;
  prop.absValue = 100;
  prop.autoManualMode = false;
  error = cam.SetProperty(&prop);
  if (error != PGRERROR_OK) {
    error.PrintErrorTrace();
    return -1;
  }
  std::cout << "Set frame rate to: " << prop.absValue << std::endl;

  error = cam.GetProperty(&prop);
  if (error != PGRERROR_OK) {
    error.PrintErrorTrace();
    return -1;
  }
  std::cout << "After setting frame rate: " << prop.absValue << std::endl;

  // Camera is ready, start capturing images
  error = cam.StartCapture();
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

#ifdef SOFTWARE_TRIGGER_CAMERA
  if (!CheckSoftwareTriggerPresence(&cam)) {
    printf(
        "SOFT_ASYNC_TRIGGER not implemented on this camera!  Stopping "
        "application\n");
    return -1;
  }
#else
  printf("Trigger the camera by sending a trigger pulse to GPIO%d.\n",
         triggerMode.source);
#endif

  Image image;
  for (int imageCount = 0; imageCount < k_numImages; imageCount++) {
#ifdef SOFTWARE_TRIGGER_CAMERA
    // Check that the trigger is ready
    PollForTriggerReady(&cam);

    printf("Press the Enter key to initiate a software trigger.\n");
    getchar();

    printf("waiting for buffer 1\n");
    error = cam.WaitForBufferEvent(&image, 0);
    if (error != PGRERROR_OK) {
      PrintError(error);
    }
    printf("after waiting buffer 1\n");

    // Fire software trigger
    bool retVal = FireSoftwareTrigger(&cam);
    if (!retVal) {
      printf("\nError firing software trigger!\n");
      return -1;
    }
    usleep(2e6);
#endif

    printf("waiting for buffer 2\n");
    cam.WaitForBufferEvent(&image, 0);
    printf("after waiting buffer 2\n");

    // Grab image
    printf("calling retrieve buffer\n");
    error = cam.RetrieveBuffer(&image);
    if (error != PGRERROR_OK) {
      PrintError(error);
    }
    printf("after retrieve buffer\n");

    printf(".\n");
  }

  // Turn trigger mode off.
  triggerMode.onOff = false;
  error = cam.SetTriggerMode(&triggerMode);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }
  printf("\nFinished grabbing images\n");

  // Stop capturing images
  error = cam.StopCapture();
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
  }

  // Turn off trigger mode
  triggerMode.onOff = false;
  error = cam.SetTriggerMode(&triggerMode);
  if (error != PGRERROR_OK) {
    PrintError(error);
    return -1;
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
