/**
 * \cond INCLUDE_PRIVATE
 * \file
 *
 * \section LICENSE
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 VectorNav Technologies, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * \section DESCRIPTION
 * This file implements the functions for interfacing with a VN-200 device.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vn200.h"
#include "vn_errorCodes.h"

/* Defines and constants. ****************************************************/

/* Private type definitions. *************************************************/

/* Private function definitions. *********************************************/

/* Function definitions. *****************************************************/

VN_ERROR_CODE vn200_connect(
	Vn200* newVn200,
	const char* portName,
	int baudrate)
{
	VN_ERROR_CODE errorCode;

	newVn200->portName = (char*) malloc(strlen(portName) + 1);
	strcpy(newVn200->portName, portName);
	newVn200->baudRate = baudrate;
	newVn200->isConnected = false;

	errorCode = vndevice_initializeVnDevice(&newVn200->vndevice, portName, baudrate, newVn200);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vndevice_startHandlingCommunication(&newVn200->vndevice);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	newVn200->isConnected = true;

	errorCode = vndevice_waitForThreadToStartHandlingCommunicationPort(&newVn200->vndevice);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_disconnect(
	Vn200* vn200)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	vndevice_deinitializeVnDevice(&vn200->vndevice);

	/* Free the memory associated with the Vn100 structure. */
	free(vn200->portName);

	vn200->isConnected = false;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_registerAsyncDataReceivedListener(
	Vn200* vn200,
	VnDeviceNewAsyncDataReceivedListener listener)
{
	return vndevice_registerAsyncDataReceivedListener(&vn200->vndevice, listener);
}

VN_ERROR_CODE vn200_unregisterAsyncDataReceivedListener(
	Vn200* vn200,
	VnDeviceNewAsyncDataReceivedListener listener)
{
	return vndevice_unregisterAsyncDataReceivedListener(&vn200->vndevice, listener);
}

VN_ERROR_CODE vn200_registerErrorCodeReceivedListener(
	Vn200* vn200,
	VnDeviceErrorCodeReceivedListener listener)
{
	return vndevice_registerErrorCodeReceivedListener(&vn200->vndevice, listener);
}

VN_ERROR_CODE vn200_unregisterErrorCodeReceivedListener(
	Vn200* vn200,
	VnDeviceErrorCodeReceivedListener listener)
{
	return vndevice_unregisterErrorCodeReceivedListener(&vn200->vndevice, listener);
}

VN_ERROR_CODE vn200_setFilterBias(
	Vn200* vn200,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNSFB";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, "VNSFB");
	else
		errorCode = vndevice_writeOutCommand(&vn200->vndevice, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn200_getBinaryOutputConfiguration(
	Vn200* vn200,
	uint8_t binaryOutputRegisterId,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup2Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup4Selections,
	uint16_t* outputGroup5Selections,
	uint16_t* outputGroup6Selections)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getBinaryOutputConfiguration(
		&vn200->vndevice,
		binaryOutputRegisterId,
		asyncMode,
		rateDivisor,
		selectedOutputGroups,
		outputGroup1Selections,
		outputGroup2Selections,
		outputGroup3Selections,
		outputGroup4Selections,
		outputGroup5Selections,
		outputGroup6Selections);
}

VN_ERROR_CODE vn200_getBinaryOutput1Configuration(
	Vn200* vn200,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup2Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup4Selections,
	uint16_t* outputGroup5Selections,
	uint16_t* outputGroup6Selections)
{
	return vn200_getBinaryOutputConfiguration(
		vn200,
		1,
		asyncMode,
		rateDivisor,
		selectedOutputGroups,
		outputGroup1Selections,
		outputGroup2Selections,
		outputGroup3Selections,
		outputGroup4Selections,
		outputGroup5Selections,
		outputGroup6Selections);
}

VN_ERROR_CODE vn200_getBinaryOutput2Configuration(
	Vn200* vn200,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup2Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup4Selections,
	uint16_t* outputGroup5Selections,
	uint16_t* outputGroup6Selections)
{
	return vn200_getBinaryOutputConfiguration(
		vn200,
		2,
		asyncMode,
		rateDivisor,
		selectedOutputGroups,
		outputGroup1Selections,
		outputGroup2Selections,
		outputGroup3Selections,
		outputGroup4Selections,
		outputGroup5Selections,
		outputGroup6Selections);
}

VN_ERROR_CODE vn200_getBinaryOutput3Configuration(
	Vn200* vn200,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup2Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup4Selections,
	uint16_t* outputGroup5Selections,
	uint16_t* outputGroup6Selections)
{
	return vn200_getBinaryOutputConfiguration(
		vn200,
		3,
		asyncMode,
		rateDivisor,
		selectedOutputGroups,
		outputGroup1Selections,
		outputGroup2Selections,
		outputGroup3Selections,
		outputGroup4Selections,
		outputGroup5Selections,
		outputGroup6Selections);
}

VN_ERROR_CODE vn200_setBinaryOutputConfiguration(
	Vn200* vn200,
	uint8_t binaryOutputRegisterId,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup2Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup4Selections,
	uint16_t outputGroup5Selections,
	uint16_t outputGroup6Selections,
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setBinaryOutputConfiguration(
		&vn200->vndevice,
		binaryOutputRegisterId,
		asyncMode,
		rateDivisor,
		outputGroup1Selections,
		outputGroup2Selections,
		outputGroup3Selections,
		outputGroup4Selections,
		outputGroup5Selections,
		outputGroup6Selections,
		waitForResponse);

}

VN_ERROR_CODE vn200_setBinaryOutput1Configuration(
	Vn200* vn200,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup2Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup4Selections,
	uint16_t outputGroup5Selections,
	uint16_t outputGroup6Selections,
	bool waitForResponse)
{
	return vn200_setBinaryOutputConfiguration(
		vn200,
		1,
		asyncMode,
		rateDivisor,
		outputGroup1Selections,
		outputGroup2Selections,
		outputGroup3Selections,
		outputGroup4Selections,
		outputGroup5Selections,
		outputGroup6Selections,
		waitForResponse);
}

VN_ERROR_CODE vn200_setBinaryOutput2Configuration(
	Vn200* vn200,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup2Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup4Selections,
	uint16_t outputGroup5Selections,
	uint16_t outputGroup6Selections,
	bool waitForResponse)
{
	return vn200_setBinaryOutputConfiguration(
		vn200,
		2,
		asyncMode,
		rateDivisor,
		outputGroup1Selections,
		outputGroup2Selections,
		outputGroup3Selections,
		outputGroup4Selections,
		outputGroup5Selections,
		outputGroup6Selections,
		waitForResponse);
}

VN_ERROR_CODE vn200_setBinaryOutput3Configuration(
	Vn200* vn200,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup2Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup4Selections,
	uint16_t outputGroup5Selections,
	uint16_t outputGroup6Selections,
	bool waitForResponse)
{
	return vn200_setBinaryOutputConfiguration(
		vn200,
		3,
		asyncMode,
		rateDivisor,
		outputGroup1Selections,
		outputGroup2Selections,
		outputGroup3Selections,
		outputGroup4Selections,
		outputGroup5Selections,
		outputGroup6Selections,
		waitForResponse);
}

bool vn200_verifyConnectivity(
	Vn200* vn200)
{
	const char* cmdToSend = "$VNRRG,1";
	const char* responseMatch = "VNRRG,";
	const char* responseMatch1 = "VNRRG,01,VN-200";
	const char* responseMatch2 = "VNRRG,1,VN-200";
	char modelBuffer[25];
	int errorCode;

	if (!vn200->isConnected)
		return false;

	memset(modelBuffer, 0, 25);
	memset(vndevice_getResponseBuffer(&vn200->vndevice), 0, VN_MAX_RESPONSE_SIZE + 1);

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return false;

	if (strncmp(vndevice_getResponseBuffer(&vn200->vndevice), responseMatch1, strlen(responseMatch1)) == 0)
		return true;

	if (strncmp(vndevice_getResponseBuffer(&vn200->vndevice), responseMatch2, strlen(responseMatch2)) == 0)
		return true;

	return false;
}

VN_ERROR_CODE vn200_getGpsConfiguration_preFirmwareVersion1d0(
	Vn200* vn200,
	unsigned char* mode,
	unsigned char* nmeaSerial1,
	unsigned char* nmeaSerial2,
	unsigned char* nmeaRate)
{
	const char* cmdToSend = "$VNRRG,55";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*mode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*nmeaSerial1 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*nmeaSerial2 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*nmeaRate = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setGpsConfiguration_preFirmwareVersion1d0(
	Vn200* vn200,
	unsigned char mode,
	unsigned char nmeaSerial1,
	unsigned char nmeaSerial2,
	unsigned char nmeaRate,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,55,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", mode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", nmeaSerial1);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", nmeaSerial2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", nmeaRate);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn200->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn200->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getGpsConfiguration(
	Vn200* vn200,
	unsigned char* mode,
	unsigned char* ppsSource)
{
	const char* cmdToSend = "$VNRRG,55";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);												/* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*mode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*ppsSource = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setGpsConfiguration(
	Vn200* vn200,
	unsigned char mode,
	unsigned char ppsSource,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,55,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", mode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", ppsSource);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", 5);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", 0);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", 0);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn200->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn200->vndevice, cmdToSendBuilder);

	return errorCode;
}


VN_ERROR_CODE vn200_getGpsAntennaOffset(
	Vn200* vn200,
	VnVector3* position)
{
	const char* cmdToSend = "$VNRRG,57";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setGpsAntennaOffset(
	Vn200* vn200,
	VnVector3 position,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,57,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", position.c0, position.c1, position.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn200->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn200->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getGpsSolutionLla(
	Vn200* vn200,
	double* gpsTime,
	unsigned short* gpsWeek,
	unsigned char* gpsFix,
	unsigned char* numberOfSatellites,
	VnVector3* lattitudeLongitudeAltitude,
	VnVector3* nedVelocity,
	VnVector3* positionAccuracy,
	float* speedAccuracy,
	float* timeAccuracy)
{
	const char* cmdToSend = "$VNRRG,58";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsTime = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsWeek = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsFix = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*numberOfSatellites = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*speedAccuracy = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*timeAccuracy = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getGpsSolutionEcef(
	Vn200* vn200,
	double* gpsTime,
	unsigned short* gpsWeek,
	unsigned char* gpsFix,
	unsigned char* numberOfSatellites,
	VnVector3* position,
	VnVector3* velocity,
	VnVector3* positionAccuracy,
	float* speedAccuracy,
	float* timeAccuracy)
{
	const char* cmdToSend = "$VNRRG,59";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsTime = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsWeek = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsFix = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*numberOfSatellites = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	positionAccuracy->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*speedAccuracy = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*timeAccuracy = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getInsSolutionLla(
	Vn200* vn200,
	double* gpsTime,
	unsigned short* gpsWeek,
	unsigned short* status,
	VnVector3* ypr,
	VnVector3* lattitudeLongitudeAltitude,
	VnVector3* nedVelocity,
	float* attitudeUncertainty,
	float* positionUncertainty,
	float* velocityUncertainty)
{
	const char* cmdToSend = "$VNRRG,63";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsTime = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsWeek = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*status = (unsigned short) strtol(result, NULL, 16);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lattitudeLongitudeAltitude->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	nedVelocity->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*attitudeUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*positionUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*velocityUncertainty = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getInsSolutionEcef(
	Vn200* vn200,
	double* gpsTime,
	uint16_t* gpsWeek,
	uint16_t* status,
	VnVector3* ypr,
	VnVector3* position,
	VnVector3* velocity,
	float* attitudeUncertainty,
	float* positionUncertainty,
	float* velocityUncertainty)
{
	const char* cmdToSend = "$VNRRG,64";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsTime = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gpsWeek = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*status = (unsigned short) strtol(result, NULL, 16);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*attitudeUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*positionUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*velocityUncertainty = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getInsStateLla(
	Vn200* vn200,
	VnVector3* ypr,
	VnVector3* lla,
	VnVector3* velocity,
	VnVector3* accel,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,72";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lla->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lla->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lla->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accel->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accel->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accel->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getInsStateEcef(
	Vn200* vn200,
	VnVector3* ypr,
	VnVector3* position,
	VnVector3* velocity,
	VnVector3* accel,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,73";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	ypr->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	position->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accel->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accel->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accel->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRate->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_getInsBasicConfiguration(
	Vn200* vn200,
	uint8_t* scenario,
	uint8_t* ahrsAiding)
{
	const char* cmdToSend = "$VNRRG,67";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*scenario = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*ahrsAiding = (uint8_t) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setInsBasicConfiguration(
	Vn200* vn200,
	uint8_t scenario,
	uint8_t ahrsAiding,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,67,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d,%d,0,0", scenario, ahrsAiding);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn200->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn200->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn200_getStartupFilterBiasEstimate(
	Vn200* vn200,
	VnVector3* gyroBias,
	VnVector3* accelBias,
	float* pressureBias)
{
	const char* cmdToSend = "$VNRRG,74";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn200->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn200->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroBias->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroBias->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroBias->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelBias->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelBias->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelBias->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*pressureBias = atof(result);
	
	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn200_setStartupFilterBiasEstimate(
	Vn200* vn200,
	VnVector3 gyroBias,
	VnVector3 accelBias,
	float pressureBias,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,74,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc,
		"%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f",
		gyroBias.c0,
		gyroBias.c1,
		gyroBias.c2,
		accelBias.c0,
		accelBias.c1,
		accelBias.c2,
		pressureBias);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn200->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn200->vndevice, cmdToSendBuilder);

	return errorCode;
}

int vn200_get_timeout(
	Vn200* vn200)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_get_timeout(
		&vn200->vndevice);
}

VN_ERROR_CODE vn200_set_timeout(
	Vn200* vn200, 
	int timeout)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_set_timeout(
		&vn200->vndevice,
		timeout);
}

VN_ERROR_CODE vn200_getCurrentAsyncData(
	Vn200* vn200, 
	VnDeviceCompositeData* curData)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getCurrentAsyncData(
		&vn200->vndevice,
		curData);
}

VN_ERROR_CODE vn200_writeSettings(
	Vn200* vn200, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_writeSettings(
		&vn200->vndevice,
		waitForResponse);
}

VN_ERROR_CODE vn200_restoreFactorySettings(
	Vn200* vn200, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_restoreFactorySettings(
		&vn200->vndevice,
		waitForResponse);
}

VN_ERROR_CODE vn200_reset(
	Vn200* vn200)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_reset(
		&vn200->vndevice);
}

VN_ERROR_CODE vn200_getUserTag(
	Vn200* vn200, 
	char* userTagBuffer, 
	uint32_t userTagBufferLength)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getUserTag(
		&vn200->vndevice,
		userTagBuffer,
		userTagBufferLength);
}

VN_ERROR_CODE vn200_setUserTag(
	Vn200* vn200, 
	char* userTagData, 
	uint32_t userTagDataLength, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setUserTag(
		&vn200->vndevice,
		userTagData,
		userTagDataLength,
		waitForResponse);
}

VN_ERROR_CODE vn200_getModelNumber(
	Vn200* vn200, 
	char* modelBuffer, 
	uint32_t modelBufferLength)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getModelNumber(
		&vn200->vndevice,
		modelBuffer,
		modelBufferLength);
}

VN_ERROR_CODE vn200_getHardwareRevision(
	Vn200* vn200, 
	int32_t* hardwareRevision)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getHardwareRevision(
		&vn200->vndevice,
		hardwareRevision);
}

VN_ERROR_CODE vn200_getSerialNumber(
	Vn200* vn200, 
	char* serialNumberBuffer, 
	uint32_t serialNumberBufferLength)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getSerialNumber(
		&vn200->vndevice,
		serialNumberBuffer,
		serialNumberBufferLength);
}

VN_ERROR_CODE vn200_getFirmwareVersion(
	Vn200* vn200, 
	char* firmwareVersionBuffer, 
	uint32_t firmwareVersionBufferLength)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getFirmwareVersion(
		&vn200->vndevice,
		firmwareVersionBuffer,
		firmwareVersionBufferLength);
}

VN_ERROR_CODE vn200_getSerialBaudRate(
	Vn200* vn200, 
	uint32_t* serialBaudrate)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getSerialBaudRate(
		&vn200->vndevice,
		serialBaudrate);
}

VN_ERROR_CODE vn200_setSerialBaudRate(
	Vn200* vn200, 
	uint32_t serialBaudrate, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setSerialBaudRate(
		&vn200->vndevice,
		serialBaudrate,
		waitForResponse);
}

VN_ERROR_CODE vn200_getAsynchronousDataOutputType(
	Vn200* vn200, 
	uint32_t* asyncDataOutputType)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAsynchronousDataOutputType(
		&vn200->vndevice,
		asyncDataOutputType);
}

VN_ERROR_CODE vn200_setAsynchronousDataOutputType(
	Vn200* vn200, 
	uint32_t asyncDataOutputType, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setAsynchronousDataOutputType(
		&vn200->vndevice,
		asyncDataOutputType,
		waitForResponse);
}

VN_ERROR_CODE vn200_getAsynchronousDataOutputFrequency(
	Vn200* vn200, 
	uint32_t* asyncDataOutputFrequency)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAsynchronousDataOutputFrequency(
		&vn200->vndevice,
		asyncDataOutputFrequency);
}

VN_ERROR_CODE vn200_setAsynchronousDataOutputFrequency(
	Vn200* vn200, 
	uint32_t asyncDataOutputFrequency, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setAsynchronousDataOutputFrequency(
		&vn200->vndevice,
		asyncDataOutputFrequency,
		waitForResponse);
}

VN_ERROR_CODE vn200_getYawPitchRoll(
	Vn200* vn200, 
	VnYpr* attitude)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getYawPitchRoll(
		&vn200->vndevice,
		attitude);
}

VN_ERROR_CODE vn200_getQuaternion(
	Vn200* vn200, 
	VnQuaternion* attitude)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getQuaternion(
		&vn200->vndevice,
		attitude);
}

VN_ERROR_CODE vn200_getYawPitchRollMagneticAccelerationAngularRate(
	Vn200* vn200, 
	VnYpr* attitude, 
	VnVector3* magnetic, 
	VnVector3* acceleration, 
	VnVector3* angularRate)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getYawPitchRollMagneticAccelerationAngularRate(
		&vn200->vndevice,
		attitude,
		magnetic,
		acceleration,
		angularRate);
}

VN_ERROR_CODE vn200_getQuaternionMagneticAccelerationAngularRate(
	Vn200* vn200, 
	VnQuaternion* attitude, 
	VnVector3* magnetic, 
	VnVector3* acceleration, 
	VnVector3* angularRate)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getQuaternionMagneticAccelerationAngularRate(
		&vn200->vndevice,
		attitude,
		magnetic,
		acceleration,
		angularRate);
}

VN_ERROR_CODE vn200_getMagnetic(
	Vn200* vn200, 
	VnVector3* magnetic)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagnetic(
		&vn200->vndevice,
		magnetic);
}

VN_ERROR_CODE vn200_getAcceleration(
	Vn200* vn200, 
	VnVector3* acceleration)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAcceleration(
		&vn200->vndevice,
		acceleration);
}

VN_ERROR_CODE vn200_getAngularRate(
	Vn200* vn200, 
	VnVector3* angularRate)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAngularRate(
		&vn200->vndevice,
		angularRate);
}

VN_ERROR_CODE vn200_getMagneticAccelerationAngularRate(
	Vn200* vn200, 
	VnVector3* magnetic, 
	VnVector3* acceleration, 
	VnVector3* angularRate)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagneticAccelerationAngularRate(
		&vn200->vndevice,
		magnetic,
		acceleration,
		angularRate);
}

VN_ERROR_CODE vn200_getYawPitchRollTrueBodyAccelerationAngularRate(
	Vn200* vn200, 
	VnYpr* attitude, 
	VnVector3* bodyAcceleration, 
	VnVector3* angularRate)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getYawPitchRollTrueBodyAccelerationAngularRate(
		&vn200->vndevice,
		attitude,
		bodyAcceleration,
		angularRate);
}

VN_ERROR_CODE vn200_getYawPitchRollTrueInertialAccelerationAngularRate(
	Vn200* vn200, 
	VnYpr* attitude, 
	VnVector3* inertialAcceleration, 
	VnVector3* angularRate)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getYawPitchRollTrueInertialAccelerationAngularRate(
		&vn200->vndevice,
		attitude,
		inertialAcceleration,
		angularRate);
}

VN_ERROR_CODE vn200_getVpeControl(
	Vn200* vn200, 
	uint8_t* enable, 
	uint8_t* headingMode, 
	uint8_t* filteringMode, 
	uint8_t* tuningMode)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getVpeControl(
		&vn200->vndevice,
		enable,
		headingMode,
		filteringMode,
		tuningMode);
}

VN_ERROR_CODE vn200_setVpeControl(
	Vn200* vn200, 
	uint8_t enable, 
	uint8_t headingMode, 
	uint8_t filteringMode, 
	uint8_t tuningMode, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setVpeControl(
		&vn200->vndevice,
		enable,
		headingMode,
		filteringMode,
		tuningMode,
		waitForResponse);
}

VN_ERROR_CODE vn200_getVpeMagnetometerBasicTuning(
	Vn200* vn200, 
	VnVector3* baseTuning, 
	VnVector3* adaptiveTuning, 
	VnVector3* adaptiveFiltering)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getVpeMagnetometerBasicTuning(
		&vn200->vndevice,
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering);
}

VN_ERROR_CODE vn200_setVpeMagnetometerBasicTuning(
	Vn200* vn200, 
	VnVector3 baseTuning, 
	VnVector3 adaptiveTuning, 
	VnVector3 adaptiveFiltering, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setVpeMagnetometerBasicTuning(
		&vn200->vndevice,
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering,
		waitForResponse);
}

VN_ERROR_CODE vn200_getVpeAccelerometerBasicTuning(
	Vn200* vn200, 
	VnVector3* baseTuning, 
	VnVector3* adaptiveTuning, 
	VnVector3* adaptiveFiltering)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getVpeAccelerometerBasicTuning(
		&vn200->vndevice,
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering);
}

VN_ERROR_CODE vn200_setVpeAccelerometerBasicTuning(
	Vn200* vn200, 
	VnVector3 baseTuning, 
	VnVector3 adaptiveTuning, 
	VnVector3 adaptiveFiltering, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setVpeAccelerometerBasicTuning(
		&vn200->vndevice,
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering,
		waitForResponse);
}

VN_ERROR_CODE vn200_getImuMeasurements(
	Vn200* vn200, 
	VnVector3* magnetic, 
	VnVector3* acceleration, 
	VnVector3* angularRate, 
	float* temperature, 
	float* pressure)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getImuMeasurements(
		&vn200->vndevice,
		magnetic,
		acceleration,
		angularRate,
		temperature,
		pressure);
}

VN_ERROR_CODE vn200_getReferenceFrameRotation(
	Vn200* vn200, 
	VnMatrix3x3* c)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getReferenceFrameRotation(
		&vn200->vndevice,
		c);
}

VN_ERROR_CODE vn200_setReferenceFrameRotation(
	Vn200* vn200, 
	VnMatrix3x3 c, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setReferenceFrameRotation(
		&vn200->vndevice,
		c,
		waitForResponse);
}

VN_ERROR_CODE vn200_getSynchronizationControl(
	Vn200* vn200, 
	uint8_t* syncInMode, 
	uint8_t* syncInEdge, 
	uint16_t* syncInSkipFactor, 
	uint8_t* syncOutMode, 
	uint8_t* syncOutPolarity, 
	uint16_t* syncOutSkipFactor, 
	uint32_t* syncOutPulseWidth)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getSynchronizationControl(
		&vn200->vndevice,
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth);
}

VN_ERROR_CODE vn200_setSynchronizationControl(
	Vn200* vn200, 
	uint8_t syncInMode, 
	uint8_t syncInEdge, 
	uint16_t syncInSkipFactor, 
	uint8_t syncOutMode, 
	uint8_t syncOutPolarity, 
	uint16_t syncOutSkipFactor, 
	uint32_t syncOutPulseWidth, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setSynchronizationControl(
		&vn200->vndevice,
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth,
		waitForResponse);
}

VN_ERROR_CODE vn200_getSynchronizationStatus(
	Vn200* vn200, 
	uint32_t* syncInCount, 
	uint32_t* syncInTime, 
	uint32_t* syncOutCount)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getSynchronizationStatus(
		&vn200->vndevice,
		syncInCount,
		syncInTime,
		syncOutCount);
}

VN_ERROR_CODE vn200_setSynchronizationStatus(
	Vn200* vn200, 
	uint32_t syncInCount, 
	uint32_t syncInTime, 
	uint32_t syncOutCount, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setSynchronizationStatus(
		&vn200->vndevice,
		syncInCount,
		syncInTime,
		syncOutCount,
		waitForResponse);
}

VN_ERROR_CODE vn200_getDeltaThetaAndDeltaVelocity(
	Vn200* vn200, 
	float* deltaTime, 
	VnVector3* deltaTheta, 
	VnVector3* deltaVelocity)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getDeltaThetaAndDeltaVelocity(
		&vn200->vndevice,
		deltaTime,
		deltaTheta,
		deltaVelocity);
}

VN_ERROR_CODE vn200_getAccelerationCompensation(
	Vn200* vn200, 
	VnMatrix3x3* c, 
	VnVector3* b)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAccelerationCompensation(
		&vn200->vndevice,
		c,
		b);
}

VN_ERROR_CODE vn200_setAccelerationCompensation(
	Vn200* vn200, 
	VnMatrix3x3 c, 
	VnVector3 b, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setAccelerationCompensation(
		&vn200->vndevice,
		c,
		b,
		waitForResponse);
}

VN_ERROR_CODE vn200_getMagneticCompensation(
	Vn200* vn200, 
	VnMatrix3x3* c, 
	VnVector3* b)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagneticCompensation(
		&vn200->vndevice,
		c,
		b);
}

VN_ERROR_CODE vn200_setMagneticCompensation(
	Vn200* vn200, 
	VnMatrix3x3 c, 
	VnVector3 b, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setMagneticCompensation(
		&vn200->vndevice,
		c,
		b,
		waitForResponse);
}

VN_ERROR_CODE vn200_getGyroCompensation(
	Vn200* vn200, 
	VnMatrix3x3* c, 
	VnVector3* b)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getGyroCompensation(
		&vn200->vndevice,
		c,
		b);
}

VN_ERROR_CODE vn200_setGyroCompensation(
	Vn200* vn200, 
	VnMatrix3x3 c, 
	VnVector3 b, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setGyroCompensation(
		&vn200->vndevice,
		c,
		b,
		waitForResponse);
}

VN_ERROR_CODE vn200_getImuFilteringConfiguration(
	Vn200* vn200, 
	uint16_t* magWindowSize, 
	uint16_t* accelWindowSize, 
	uint16_t* gyroWindowSize, 
	uint16_t* tempWindowSize, 
	uint16_t* presWindowSize, 
	uint8_t* magFilterMode, 
	uint8_t* accelFilterMode, 
	uint8_t* gyroFilterMode, 
	uint8_t* tempFilterMode, 
	uint8_t* presFilterMode)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getImuFilteringConfiguration(
		&vn200->vndevice,
		magWindowSize,
		accelWindowSize,
		gyroWindowSize,
		tempWindowSize,
		presWindowSize,
		magFilterMode,
		accelFilterMode,
		gyroFilterMode,
		tempFilterMode,
		presFilterMode);
}

VN_ERROR_CODE vn200_setImuFilteringConfiguration(
	Vn200* vn200, 
	uint16_t magWindowSize, 
	uint16_t accelWindowSize, 
	uint16_t gyroWindowSize, 
	uint16_t tempWindowSize, 
	uint16_t presWindowSize, 
	uint8_t magFilterMode, 
	uint8_t accelFilterMode, 
	uint8_t gyroFilterMode, 
	uint8_t tempFilterMode, 
	uint8_t presFilterMode, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setImuFilteringConfiguration(
		&vn200->vndevice,
		magWindowSize,
		accelWindowSize,
		gyroWindowSize,
		tempWindowSize,
		presWindowSize,
		magFilterMode,
		accelFilterMode,
		gyroFilterMode,
		tempFilterMode,
		presFilterMode,
		waitForResponse);
}

VN_ERROR_CODE vn200_getDeltaThetaAndDeltaVelocityConfiguration(
	Vn200* vn200, 
	uint8_t* integrationFrame, 
	uint8_t* gyroCompensation, 
	uint8_t* accelCompensation)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getDeltaThetaAndDeltaVelocityConfiguration(
		&vn200->vndevice,
		integrationFrame,
		gyroCompensation,
		accelCompensation);
}

VN_ERROR_CODE vn200_setDeltaThetaAndDeltaVelocityConfiguration(
	Vn200* vn200, 
	uint8_t integrationFrame, 
	uint8_t gyroCompensation, 
	uint8_t accelCompensation, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setDeltaThetaAndDeltaVelocityConfiguration(
		&vn200->vndevice,
		integrationFrame,
		gyroCompensation,
		accelCompensation,
		waitForResponse);
}

VN_ERROR_CODE vn200_getMagnetometerCalibrationControl(
	Vn200* vn200, 
	uint8_t* hsiMode, 
	uint8_t* hsiOutput, 
	uint8_t* convergeRate)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagnetometerCalibrationControl(
		&vn200->vndevice,
		hsiMode,
		hsiOutput,
		convergeRate);
}

VN_ERROR_CODE vn200_setMagnetometerCalibrationControl(
	Vn200* vn200, 
	uint8_t hsiMode, 
	uint8_t hsiOutput, 
	uint8_t convergeRate, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setMagnetometerCalibrationControl(
		&vn200->vndevice,
		hsiMode,
		hsiOutput,
		convergeRate,
		waitForResponse);
}

VN_ERROR_CODE vn200_getCalculatedMagnetometerCalibration(
	Vn200* vn200, 
	VnMatrix3x3* c, 
	VnVector3* b)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getCalculatedMagnetometerCalibration(
		&vn200->vndevice,
		c,
		b);
}

VN_ERROR_CODE vn200_getMagneticGravityReferenceVectors(
	Vn200* vn200, 
	VnVector3* magneticReference, 
	VnVector3* gravityReference)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagneticGravityReferenceVectors(
		&vn200->vndevice,
		magneticReference,
		gravityReference);
}

VN_ERROR_CODE vn200_setMagneticGravityReferenceVectors(
	Vn200* vn200, 
	VnVector3 magneticReference, 
	VnVector3 gravityReference, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setMagneticGravityReferenceVectors(
		&vn200->vndevice,
		magneticReference,
		gravityReference,
		waitForResponse);
}

VN_ERROR_CODE vn200_getCommunicationProtocolControl(
	Vn200* vn200, 
	uint8_t* serialCount, 
	uint8_t* serialStatus, 
	uint8_t* spiCount, 
	uint8_t* spiStatus, 
	uint8_t* serialChecksum, 
	uint8_t* spiChecksum, 
	uint8_t* errorMode)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getCommunicationProtocolControl(
		&vn200->vndevice,
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode);
}

VN_ERROR_CODE vn200_setCommunicationProtocolControl(
	Vn200* vn200, 
	uint8_t serialCount, 
	uint8_t serialStatus, 
	uint8_t spiCount, 
	uint8_t spiStatus, 
	uint8_t serialChecksum, 
	uint8_t spiChecksum, 
	uint8_t errorMode, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setCommunicationProtocolControl(
		&vn200->vndevice,
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode,
		waitForResponse);
}

VN_ERROR_CODE vn200_getReferenceVectorConfiguration(
	Vn200* vn200, 
	uint8_t* useMagModel, 
	uint8_t* useGravityModel, 
	uint32_t* recalcThreshold, 
	float* year, 
	VnVector3* lla)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getReferenceVectorConfiguration(
		&vn200->vndevice,
		useMagModel,
		useGravityModel,
		recalcThreshold,
		year,
		lla);
}

VN_ERROR_CODE vn200_setReferenceVectorConfiguration(
	Vn200* vn200, 
	uint8_t useMagModel, 
	uint8_t useGravityModel, 
	uint32_t recalcThreshold, 
	float year, 
	VnVector3 lla, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setReferenceVectorConfiguration(
		&vn200->vndevice,
		useMagModel,
		useGravityModel,
		recalcThreshold,
		year,
		lla,
		waitForResponse);
}

VN_ERROR_CODE vn200_pauseAsyncOutputs(
	Vn200* vn200, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_pauseAsyncOutputs(
		&vn200->vndevice,
		waitForResponse);
}

VN_ERROR_CODE vn200_resumeAsyncOutputs(
	Vn200* vn200, 
	bool waitForResponse)
{
	if (!vn200->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_resumeAsyncOutputs(
		&vn200->vndevice,
		waitForResponse);
}

/** \endcond */
