//=============================================================================
// Copyright © 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

//=============================================================================
// $Id: FlyCapture2.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_FLYCAPTURE2_H
#define PGR_FC2_FLYCAPTURE2_H

//=============================================================================
// Global header file for FlyCapture2.
//
// By including this file, all required header files for full FlyCapture2
// operation will be included automatically. It is recommended that this file
// be used instead of manually including individual header files.
//=============================================================================

//=============================================================================
// Platform-specific definitions
//=============================================================================
#include "FlyCapture2Platform.h"

//=============================================================================
// Global definitions
//=============================================================================
#include "FlyCapture2Defs.h"

//=============================================================================
// PGR Error class
//=============================================================================
#include "Error.h"

//=============================================================================
// FlyCapture2 classes
//=============================================================================
#include "BusManager.h"
#include "Camera.h"
#include "GigECamera.h"
#include "Image.h"

//=============================================================================
// Utility classes
//=============================================================================
#include "Utilities.h"
#include "AVIRecorder.h"
#include "TopologyNode.h"
#include "ImageStatistics.h"

#endif // PGR_FC2_FLYCAPTURE2_H

