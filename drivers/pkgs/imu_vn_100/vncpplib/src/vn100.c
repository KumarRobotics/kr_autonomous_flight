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
 * This file implements the functions for interfacing with a VN-100 device.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vn100.h"
#include "vn_errorCodes.h"

/* Defines and constants. ****************************************************/

/* Private type definitions. *************************************************/

/* Private function definitions. *********************************************/

/* Function definitions. *****************************************************/

VN_ERROR_CODE vn100_connect(
	Vn100* newVn100,
	const char* portName,
	int baudrate)
{
	VN_ERROR_CODE errorCode;

	newVn100->portName = (char*) malloc(strlen(portName) + 1);
	strcpy(newVn100->portName, portName);
	newVn100->baudRate = baudrate;
	newVn100->isConnected = false;

	errorCode = vndevice_initializeVnDevice(&newVn100->vndevice, portName, baudrate, newVn100);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vndevice_startHandlingCommunication(&newVn100->vndevice);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	newVn100->isConnected = true;

	errorCode = vndevice_waitForThreadToStartHandlingCommunicationPort(&newVn100->vndevice);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_disconnect(
	Vn100* vn100)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	vndevice_deinitializeVnDevice(&vn100->vndevice);

	/* Free the memory associated with the Vn100 structure. */
	free(vn100->portName);

	vn100->isConnected = false;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getBinaryOutputConfiguration(
	Vn100* vn100,
	uint8_t binaryOutputRegisterId,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup5Selections)
{
	uint16_t outputGroup2SelectionsPlaceholder, outputGroup4SelectionsPlaceholder, outputGroup6SelectionsPlaceholder;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getBinaryOutputConfiguration(
		&vn100->vndevice,
		binaryOutputRegisterId,
		asyncMode,
		rateDivisor,
		selectedOutputGroups,
		outputGroup1Selections,
		&outputGroup2SelectionsPlaceholder,
		outputGroup3Selections,
		&outputGroup4SelectionsPlaceholder,
		outputGroup5Selections,
		&outputGroup6SelectionsPlaceholder);
}

VN_ERROR_CODE vn100_getBinaryOutput1Configuration(
	Vn100* vn100,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup5Selections)
{
	return vn100_getBinaryOutputConfiguration(
		vn100,
		1,
		asyncMode,
		rateDivisor,
		selectedOutputGroups,
		outputGroup1Selections,
		outputGroup3Selections,
		outputGroup5Selections);
}

VN_ERROR_CODE vn100_getBinaryOutput2Configuration(
	Vn100* vn100,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup5Selections)
{
	return vn100_getBinaryOutputConfiguration(
		vn100,
		2,
		asyncMode,
		rateDivisor,
		selectedOutputGroups,
		outputGroup1Selections,
		outputGroup3Selections,
		outputGroup5Selections);
}

VN_ERROR_CODE vn100_getBinaryOutput5Configuration(
	Vn100* vn100,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup5Selections)
{
	return vn100_getBinaryOutputConfiguration(
		vn100,
		5,
		asyncMode,
		rateDivisor,
		selectedOutputGroups,
		outputGroup1Selections,
		outputGroup3Selections,
		outputGroup5Selections);
}

VN_ERROR_CODE vn100_setBinaryOutputConfiguration(
	Vn100* vn100,
	uint8_t binaryOutputRegisterId,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup5Selections,
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setBinaryOutputConfiguration(
		&vn100->vndevice,
		binaryOutputRegisterId,
		asyncMode,
		rateDivisor,
		outputGroup1Selections,
		0,
		outputGroup3Selections,
		0,
		outputGroup5Selections,
		0,
		waitForResponse);

}

VN_ERROR_CODE vn100_setBinaryOutput1Configuration(
	Vn100* vn100,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup5Selections,
	bool waitForResponse)
{
	return vn100_setBinaryOutputConfiguration(
		vn100,
		1,
		asyncMode,
		rateDivisor,
		outputGroup1Selections,
		outputGroup3Selections,
		outputGroup5Selections,
		waitForResponse);
}

VN_ERROR_CODE vn100_setBinaryOutput2Configuration(
	Vn100* vn100,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup5Selections,
	bool waitForResponse)
{
	return vn100_setBinaryOutputConfiguration(
		vn100,
		2,
		asyncMode,
		rateDivisor,
		outputGroup1Selections,
		outputGroup3Selections,
		outputGroup5Selections,
		waitForResponse);
}

VN_ERROR_CODE vn100_setBinaryOutput3Configuration(
	Vn100* vn100,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup5Selections,
	bool waitForResponse)
{
	return vn100_setBinaryOutputConfiguration(
		vn100,
		3,
		asyncMode,
		rateDivisor,
		outputGroup1Selections,
		outputGroup3Selections,
		outputGroup5Selections,
		waitForResponse);
}

VN_ERROR_CODE vn100_tare(
	Vn100* vn100,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNTAR";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, "VNTAR");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_knownMagneticDisturbance(
	Vn100* vn100,
	bool isDisturbancePresent,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;
	
	cmdToSend = isDisturbancePresent ? "$VNKMD,1" : "$VNKMD,0";

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, "VNKMD,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_knownAccelerationDisturbance(
	Vn100* vn100,
	bool isDisturbancePresent,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend;

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;
	
	cmdToSend = isDisturbancePresent ? "$VNKAD,1" : "$VNKAD,0";

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, "VNKAD,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_setGyroBias(
	Vn100* vn100,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNSGB";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, "VNSGB");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vn100_registerAsyncDataReceivedListener(
	Vn100* vn100,
	VnDeviceNewAsyncDataReceivedListener listener)
{
	return vndevice_registerAsyncDataReceivedListener(&vn100->vndevice, listener);
}

VN_ERROR_CODE vn100_unregisterAsyncDataReceivedListener(
	Vn100* vn100,
	VnDeviceNewAsyncDataReceivedListener listener)
{
	return vndevice_unregisterAsyncDataReceivedListener(&vn100->vndevice, listener);
}

VN_ERROR_CODE vn100_registerErrorCodeReceivedListener(
	Vn100* vn100,
	VnDeviceErrorCodeReceivedListener listener)
{
	return vndevice_registerErrorCodeReceivedListener(&vn100->vndevice, listener);
}

VN_ERROR_CODE vn100_unregisterErrorCodeReceivedListener(
	Vn100* vn100,
	VnDeviceErrorCodeReceivedListener listener)
{
	return vndevice_unregisterErrorCodeReceivedListener(&vn100->vndevice, listener);
}

bool vn100_verifyConnectivity(
	Vn100* vn100)
{
	const char* cmdToSend = "$VNRRG,1";
	const char* responseMatch = "VNRRG,";
	const char* responseMatch1 = "VNRRG,01,VN-100";
	const char* responseMatch2 = "VNRRG,1,VN-100";
	char modelBuffer[25];
	int errorCode;

	if (!vn100->isConnected)
		return false;

	memset(modelBuffer, 0, 25);
	memset(vndevice_getResponseBuffer(&vn100->vndevice), 0, VN_MAX_RESPONSE_SIZE + 1);

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return false;

	if (strncmp(vndevice_getResponseBuffer(&vn100->vndevice), responseMatch1, strlen(responseMatch1)) == 0)
		return true;

	if (strncmp(vndevice_getResponseBuffer(&vn100->vndevice), responseMatch2, strlen(responseMatch2)) == 0)
		return true;

	return false;
}


VN_ERROR_CODE vn100_getQuaternionMagnetic(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* magnetic)
{
	const char* cmdToSend = "$VNRRG,10";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionAcceleration(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* acceleration)
{
	const char* cmdToSend = "$VNRRG,11";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionAngularRate(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,12";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
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

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionMagneticAcceleration(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration)
{
	const char* cmdToSend = "$VNRRG,13";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	acceleration->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getQuaternionAccelerationAngularRate(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* magnetic,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,14";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetic->c2 = atof(result);
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

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getDirectionCosineMatrix(
	Vn100* vn100,
	VnMatrix3x3* attitude)
{
	const char* cmdToSend = "$VNRRG,16";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->c22 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getFilterMeasurementVarianceParameters(
	Vn100* vn100,
	double* angularWalkVariance,
	VnVector3* angularRateVariance,
	VnVector3* magneticVariance,
	VnVector3* accelerationVariance)
{
	const char* cmdToSend = "$VNRRG,22";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*angularWalkVariance = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateVariance->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateVariance->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateVariance->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticVariance->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticVariance->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticVariance->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerationVariance->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerationVariance->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerationVariance->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setFilterMeasurementVarianceParameters(
	Vn100* vn100,
	double angularWalkVariance,
	VnVector3 angularRateVariance,
	VnVector3 magneticVariance,
	VnVector3 accelerationVariance,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,22,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", angularWalkVariance);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", angularRateVariance.c0, angularRateVariance.c1, angularRateVariance.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", magneticVariance.c0, magneticVariance.c1, magneticVariance.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", accelerationVariance.c0, accelerationVariance.c1, accelerationVariance.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getFilterActiveTuningParameters(
	Vn100* vn100,
	double* magneticGain,
	double* accelerationGain,
	double* magneticMemory,
	double* accelerationMemory)
{
	const char* cmdToSend = "$VNRRG,24";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magneticGain = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelerationGain = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magneticMemory = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelerationMemory = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setFilterActiveTuningParameters(
	Vn100* vn100,
	double magneticGain,
	double accelerationGain,
	double magneticMemory,
	double accelerationMemory,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,24,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", magneticGain);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", accelerationGain);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", magneticMemory);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", accelerationMemory);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getAccelerometerGain(
	Vn100* vn100,
	unsigned int* accelerometerGain)
{
	const char* cmdToSend = "$VNRRG,28";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelerometerGain = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setAccelerometerGain(
	Vn100* vn100,
	unsigned int accelerometerGain,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,28,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", accelerometerGain);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_setCommunicationProtocolStatus(
	Vn100* vn100,
	unsigned int numOfParsedSerialMessages,
	unsigned int numOfParsedSpiMessages,
	unsigned char maxUsageSerialRxBuffer,
	unsigned char maxUsageSerialTxBuffer,
	unsigned char maxUsageSpiRxBuffer,
	unsigned char maxUsageSpiTxBuffer,
	unsigned short systemError0,
	unsigned short systemError1,
	unsigned short systemError2,
	unsigned short systemError3,
	unsigned short systemError4,
	unsigned short systemError5,
	unsigned short systemError6,
	unsigned short systemError7,
	unsigned short systemError8,
	unsigned short systemError9,
	unsigned short systemError10,
	unsigned short systemError11,
	unsigned short systemError12,
	unsigned short systemError13,
	unsigned short systemError14,
	unsigned short systemError15,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,31,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", numOfParsedSerialMessages);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", numOfParsedSpiMessages);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", maxUsageSerialRxBuffer);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", maxUsageSerialTxBuffer);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", maxUsageSpiRxBuffer);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", maxUsageSpiTxBuffer);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError0);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError1);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError3);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError4);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError5);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError6);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError7);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError8);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError9);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError10);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError11);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError12);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError13);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError14);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", systemError15);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getFilterBasicControl(
	Vn100* vn100,
	unsigned char* magneticMode,
	unsigned char* externalMagnetometerMode,
	unsigned char* externalAccelerometerMode,
	unsigned char* externalGyroscopeMode,
	VnVector3* angularRateLimit)
{
	const char* cmdToSend = "$VNRRG,34";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magneticMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*externalMagnetometerMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*externalAccelerometerMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*externalGyroscopeMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateLimit->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateLimit->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	angularRateLimit->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setFilterBasicControl(
	Vn100* vn100,
	unsigned char magneticMode,
	unsigned char externalMagnetometerMode,
	unsigned char externalAccelerometerMode,
	unsigned char externalGyroscopeMode,
	VnVector3 angularRateLimit,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,34,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", magneticMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalMagnetometerMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalAccelerometerMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", externalGyroscopeMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", angularRateLimit.c0, angularRateLimit.c1, angularRateLimit.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeMagnetometerAdvancedTuning(
	Vn100* vn100,
	VnVector3* minimumFiltering,
	VnVector3* maximumFiltering,
	float* maximumAdaptRate,
	float* disturbanceWindow,
	float* maximumTuning)
{
	const char* cmdToSend = "$VNRRG,37";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maximumAdaptRate = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*disturbanceWindow = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maximumTuning = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeMagnetometerAdvancedTuning(
	Vn100* vn100,
	VnVector3 minimumFiltering,
	VnVector3 maximumFiltering,
	float maximumAdaptRate,
	float disturbanceWindow,
	float maximumTuning,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,37,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", minimumFiltering.c0, minimumFiltering.c1, minimumFiltering.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", maximumFiltering.c0, maximumFiltering.c1, maximumFiltering.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumAdaptRate);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", disturbanceWindow);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumTuning);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeAccelerometerAdvancedTuning(
	Vn100* vn100,
	VnVector3* minimumFiltering,
	VnVector3* maximumFiltering,
	float* maximumAdaptRate,
	float* disturbanceWindow,
	float* maximumTuning)
{
	const char* cmdToSend = "$VNRRG,39";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	minimumFiltering->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	maximumFiltering->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maximumAdaptRate = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*disturbanceWindow = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maximumTuning = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeAccelerometerAdvancedTuning(
	Vn100* vn100,
	VnVector3 minimumFiltering,
	VnVector3 maximumFiltering,
	float maximumAdaptRate,
	float disturbanceWindow,
	float maximumTuning,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,39,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", minimumFiltering.c0, minimumFiltering.c1, minimumFiltering.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", maximumFiltering.c0, maximumFiltering.c1, maximumFiltering.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumAdaptRate);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", disturbanceWindow);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maximumTuning);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVpeGyroBasicTuning(
	Vn100* vn100,
	VnVector3* varianceAngularWalk,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning)
{
	const char* cmdToSend = "$VNRRG,40";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	varianceAngularWalk->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	varianceAngularWalk->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	varianceAngularWalk->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	baseTuning->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveTuning->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVpeGyroBasicTuning(
	Vn100* vn100,
	VnVector3 varianceAngularWalk,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,40,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", varianceAngularWalk.c0, varianceAngularWalk.c1, varianceAngularWalk.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getFilterStatus(
	Vn100* vn100,
	unsigned short* solutionStatus,
	float* yawUncertainty,
	float* pitchUncertainty,
	float* rollUncertainty,
	float* gyroBiasUncertainty,
	float* magUncertainty,
	float* accelUncertainty)
{
	const char* cmdToSend = "$VNRRG,42";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*solutionStatus = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*yawUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*pitchUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*rollUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gyroBiasUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magUncertainty = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelUncertainty = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getFilterStartupGyroBias(
	Vn100* vn100,
	VnVector3* gyroBias)
{
	const char* cmdToSend = "$VNRRG,43";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
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

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setFilterStartupGyroBias(
	Vn100* vn100,
	VnVector3 gyroBias,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,43,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", gyroBias.c0, gyroBias.c1, gyroBias.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getMagnetometerCalibrationStatus(
	Vn100* vn100,
	unsigned char* lastBin,
	unsigned short* numOfMeasurements,
	float* avgResidual,
	VnVector3* lastMeasurement,
	unsigned char* bin0,
	unsigned char* bin1,
	unsigned char* bin2,
	unsigned char* bin3,
	unsigned char* bin4,
	unsigned char* bin5,
	unsigned char* bin6,
	unsigned char* bin7)
{
	const char* cmdToSend = "$VNRRG,46";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*lastBin = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*numOfMeasurements = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*avgResidual = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lastMeasurement->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lastMeasurement->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	lastMeasurement->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin0 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin1 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin2 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin3 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin4 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin5 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin6 = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*bin7 = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getIndoorHeadingModeControl(
	Vn100* vn100,
	float* maxRateError,
	float* reserved)
{
	const char* cmdToSend = "$VNRRG,48";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*maxRateError = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*reserved = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setIndoorHeadingModeControl(
	Vn100* vn100,
	float maxRateError,
	float reserved,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,48,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", maxRateError);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", reserved);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVelocityCompenstationControl(
	Vn100* vn100,
	uint8_t* mode,
	float* velocityTuning,
	float* rateTuning)
{
	const char* cmdToSend = "$VNRRG,51";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*mode = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*velocityTuning = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*rateTuning = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVelocityCompenstationControl(
	Vn100* vn100,
	uint8_t mode,
	float velocityTuning,
	float rateTuning,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,51,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", mode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", velocityTuning);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", rateTuning);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getVelocityCompenstationMeasurement(
	Vn100* vn100,
	VnVector3* velocity)
{
	const char* cmdToSend = "$VNRRG,50";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c0 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c1 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	velocity->c2 = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_setVelocityCompenstationMeasurement(
	Vn100* vn100,
	VnVector3 velocity,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,50,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", velocity.c0);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", velocity.c1);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f", velocity.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(&vn100->vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(&vn100->vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vn100_getYawPitchRollInertialCalibratedMeasurements(
	Vn100* vn100,
	VnYpr* attitude,
	VnVector3* inertialMagnetic,
	VnVector3* inertialAcceleration,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,241";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->yaw = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->pitch = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->roll = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialMagnetic->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialMagnetic->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialMagnetic->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	inertialAcceleration->c2 = atof(result);
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

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getRawVoltageMeasurements(
	Vn100* vn100,
	VnVector3* magnetometer,
	VnVector3* accelerometer,
	VnVector3* gyroscope,
	float* temperature)
{
	const char* cmdToSend = "$VNRRG,251";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetometer->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetometer->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magnetometer->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerometer->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerometer->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	accelerometer->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscope->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscope->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscope->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*temperature = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getKalmanFilterStateVector(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* gyroscopeBias)
{
	const char* cmdToSend = "$VNRRG,253";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->x = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->y = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->z = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	attitude->w = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscopeBias->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscopeBias->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gyroscopeBias->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vn100_getKalmanFilterCovarianceMatrixDiagonal(
	Vn100* vn100,
	float* p00,
	float* p11,
	float* p22,
	float* p33,
	float* p44,
	float* p55)
{
	const char* cmdToSend = "$VNRRG,254";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	errorCode = vndevice_transaction(&vn100->vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice_getResponseBuffer(&vn100->vndevice), delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p00 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p11 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p22 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p33 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p44 = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*p55 = (float) atof(result);

	return VNERR_NO_ERROR;
}

int vn100_get_timeout(
	Vn100* vn100)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_get_timeout(
		&vn100->vndevice);
}

VN_ERROR_CODE vn100_set_timeout(
	Vn100* vn100, 
	int timeout)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_set_timeout(
		&vn100->vndevice,
		timeout);
}

VN_ERROR_CODE vn100_getCurrentAsyncData(
	Vn100* vn100, 
	VnDeviceCompositeData* curData)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getCurrentAsyncData(
		&vn100->vndevice,
		curData);
}

VN_ERROR_CODE vn100_writeSettings(
	Vn100* vn100, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_writeSettings(
		&vn100->vndevice,
		waitForResponse);
}

VN_ERROR_CODE vn100_restoreFactorySettings(
	Vn100* vn100, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_restoreFactorySettings(
		&vn100->vndevice,
		waitForResponse);
}

VN_ERROR_CODE vn100_reset(
	Vn100* vn100)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_reset(
		&vn100->vndevice);
}

VN_ERROR_CODE vn100_getUserTag(
	Vn100* vn100, 
	char* userTagBuffer, 
	uint32_t userTagBufferLength)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getUserTag(
		&vn100->vndevice,
		userTagBuffer,
		userTagBufferLength);
}

VN_ERROR_CODE vn100_setUserTag(
	Vn100* vn100, 
	char* userTagData, 
	uint32_t userTagDataLength, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setUserTag(
		&vn100->vndevice,
		userTagData,
		userTagDataLength,
		waitForResponse);
}

VN_ERROR_CODE vn100_getModelNumber(
	Vn100* vn100, 
	char* modelBuffer, 
	uint32_t modelBufferLength)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getModelNumber(
		&vn100->vndevice,
		modelBuffer,
		modelBufferLength);
}

VN_ERROR_CODE vn100_getHardwareRevision(
	Vn100* vn100, 
	int32_t* hardwareRevision)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getHardwareRevision(
		&vn100->vndevice,
		hardwareRevision);
}

VN_ERROR_CODE vn100_getSerialNumber(
	Vn100* vn100, 
	char* serialNumberBuffer, 
	uint32_t serialNumberBufferLength)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getSerialNumber(
		&vn100->vndevice,
		serialNumberBuffer,
		serialNumberBufferLength);
}

VN_ERROR_CODE vn100_getFirmwareVersion(
	Vn100* vn100, 
	char* firmwareVersionBuffer, 
	uint32_t firmwareVersionBufferLength)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getFirmwareVersion(
		&vn100->vndevice,
		firmwareVersionBuffer,
		firmwareVersionBufferLength);
}

VN_ERROR_CODE vn100_getSerialBaudRate(
	Vn100* vn100, 
	uint32_t* serialBaudrate)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getSerialBaudRate(
		&vn100->vndevice,
		serialBaudrate);
}

VN_ERROR_CODE vn100_setSerialBaudRate(
	Vn100* vn100, 
	uint32_t serialBaudrate, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setSerialBaudRate(
		&vn100->vndevice,
		serialBaudrate,
		waitForResponse);
}

VN_ERROR_CODE vn100_getAsynchronousDataOutputType(
	Vn100* vn100, 
	uint32_t* asyncDataOutputType)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAsynchronousDataOutputType(
		&vn100->vndevice,
		asyncDataOutputType);
}

VN_ERROR_CODE vn100_setAsynchronousDataOutputType(
	Vn100* vn100, 
	uint32_t asyncDataOutputType, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setAsynchronousDataOutputType(
		&vn100->vndevice,
		asyncDataOutputType,
		waitForResponse);
}

VN_ERROR_CODE vn100_getAsynchronousDataOutputFrequency(
	Vn100* vn100, 
	uint32_t* asyncDataOutputFrequency)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAsynchronousDataOutputFrequency(
		&vn100->vndevice,
		asyncDataOutputFrequency);
}

VN_ERROR_CODE vn100_setAsynchronousDataOutputFrequency(
	Vn100* vn100, 
	uint32_t asyncDataOutputFrequency, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setAsynchronousDataOutputFrequency(
		&vn100->vndevice,
		asyncDataOutputFrequency,
		waitForResponse);
}

VN_ERROR_CODE vn100_getYawPitchRoll(
	Vn100* vn100, 
	VnYpr* attitude)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getYawPitchRoll(
		&vn100->vndevice,
		attitude);
}

VN_ERROR_CODE vn100_getQuaternion(
	Vn100* vn100, 
	VnQuaternion* attitude)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getQuaternion(
		&vn100->vndevice,
		attitude);
}

VN_ERROR_CODE vn100_getYawPitchRollMagneticAccelerationAngularRate(
	Vn100* vn100, 
	VnYpr* attitude, 
	VnVector3* magnetic, 
	VnVector3* acceleration, 
	VnVector3* angularRate)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getYawPitchRollMagneticAccelerationAngularRate(
		&vn100->vndevice,
		attitude,
		magnetic,
		acceleration,
		angularRate);
}

VN_ERROR_CODE vn100_getQuaternionMagneticAccelerationAngularRate(
	Vn100* vn100, 
	VnQuaternion* attitude, 
	VnVector3* magnetic, 
	VnVector3* acceleration, 
	VnVector3* angularRate)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getQuaternionMagneticAccelerationAngularRate(
		&vn100->vndevice,
		attitude,
		magnetic,
		acceleration,
		angularRate);
}

VN_ERROR_CODE vn100_getMagnetic(
	Vn100* vn100, 
	VnVector3* magnetic)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagnetic(
		&vn100->vndevice,
		magnetic);
}

VN_ERROR_CODE vn100_getAcceleration(
	Vn100* vn100, 
	VnVector3* acceleration)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAcceleration(
		&vn100->vndevice,
		acceleration);
}

VN_ERROR_CODE vn100_getAngularRate(
	Vn100* vn100, 
	VnVector3* angularRate)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAngularRate(
		&vn100->vndevice,
		angularRate);
}

VN_ERROR_CODE vn100_getMagneticAccelerationAngularRate(
	Vn100* vn100, 
	VnVector3* magnetic, 
	VnVector3* acceleration, 
	VnVector3* angularRate)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagneticAccelerationAngularRate(
		&vn100->vndevice,
		magnetic,
		acceleration,
		angularRate);
}

VN_ERROR_CODE vn100_getYawPitchRollTrueBodyAccelerationAngularRate(
	Vn100* vn100, 
	VnYpr* attitude, 
	VnVector3* bodyAcceleration, 
	VnVector3* angularRate)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getYawPitchRollTrueBodyAccelerationAngularRate(
		&vn100->vndevice,
		attitude,
		bodyAcceleration,
		angularRate);
}

VN_ERROR_CODE vn100_getYawPitchRollTrueInertialAccelerationAngularRate(
	Vn100* vn100, 
	VnYpr* attitude, 
	VnVector3* inertialAcceleration, 
	VnVector3* angularRate)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getYawPitchRollTrueInertialAccelerationAngularRate(
		&vn100->vndevice,
		attitude,
		inertialAcceleration,
		angularRate);
}

VN_ERROR_CODE vn100_getVpeControl(
	Vn100* vn100, 
	uint8_t* enable, 
	uint8_t* headingMode, 
	uint8_t* filteringMode, 
	uint8_t* tuningMode)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getVpeControl(
		&vn100->vndevice,
		enable,
		headingMode,
		filteringMode,
		tuningMode);
}

VN_ERROR_CODE vn100_setVpeControl(
	Vn100* vn100, 
	uint8_t enable, 
	uint8_t headingMode, 
	uint8_t filteringMode, 
	uint8_t tuningMode, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setVpeControl(
		&vn100->vndevice,
		enable,
		headingMode,
		filteringMode,
		tuningMode,
		waitForResponse);
}

VN_ERROR_CODE vn100_getVpeMagnetometerBasicTuning(
	Vn100* vn100, 
	VnVector3* baseTuning, 
	VnVector3* adaptiveTuning, 
	VnVector3* adaptiveFiltering)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getVpeMagnetometerBasicTuning(
		&vn100->vndevice,
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering);
}

VN_ERROR_CODE vn100_setVpeMagnetometerBasicTuning(
	Vn100* vn100, 
	VnVector3 baseTuning, 
	VnVector3 adaptiveTuning, 
	VnVector3 adaptiveFiltering, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setVpeMagnetometerBasicTuning(
		&vn100->vndevice,
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering,
		waitForResponse);
}

VN_ERROR_CODE vn100_getVpeAccelerometerBasicTuning(
	Vn100* vn100, 
	VnVector3* baseTuning, 
	VnVector3* adaptiveTuning, 
	VnVector3* adaptiveFiltering)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getVpeAccelerometerBasicTuning(
		&vn100->vndevice,
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering);
}

VN_ERROR_CODE vn100_setVpeAccelerometerBasicTuning(
	Vn100* vn100, 
	VnVector3 baseTuning, 
	VnVector3 adaptiveTuning, 
	VnVector3 adaptiveFiltering, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setVpeAccelerometerBasicTuning(
		&vn100->vndevice,
		baseTuning,
		adaptiveTuning,
		adaptiveFiltering,
		waitForResponse);
}

VN_ERROR_CODE vn100_getImuMeasurements(
	Vn100* vn100, 
	VnVector3* magnetic, 
	VnVector3* acceleration, 
	VnVector3* angularRate, 
	float* temperature, 
	float* pressure)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getImuMeasurements(
		&vn100->vndevice,
		magnetic,
		acceleration,
		angularRate,
		temperature,
		pressure);
}

VN_ERROR_CODE vn100_getReferenceFrameRotation(
	Vn100* vn100, 
	VnMatrix3x3* c)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getReferenceFrameRotation(
		&vn100->vndevice,
		c);
}

VN_ERROR_CODE vn100_setReferenceFrameRotation(
	Vn100* vn100, 
	VnMatrix3x3 c, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setReferenceFrameRotation(
		&vn100->vndevice,
		c,
		waitForResponse);
}

VN_ERROR_CODE vn100_getSynchronizationControl(
	Vn100* vn100, 
	uint8_t* syncInMode, 
	uint8_t* syncInEdge, 
	uint16_t* syncInSkipFactor, 
	uint8_t* syncOutMode, 
	uint8_t* syncOutPolarity, 
	uint16_t* syncOutSkipFactor, 
	uint32_t* syncOutPulseWidth)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getSynchronizationControl(
		&vn100->vndevice,
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth);
}

VN_ERROR_CODE vn100_setSynchronizationControl(
	Vn100* vn100, 
	uint8_t syncInMode, 
	uint8_t syncInEdge, 
	uint16_t syncInSkipFactor, 
	uint8_t syncOutMode, 
	uint8_t syncOutPolarity, 
	uint16_t syncOutSkipFactor, 
	uint32_t syncOutPulseWidth, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setSynchronizationControl(
		&vn100->vndevice,
		syncInMode,
		syncInEdge,
		syncInSkipFactor,
		syncOutMode,
		syncOutPolarity,
		syncOutSkipFactor,
		syncOutPulseWidth,
		waitForResponse);
}

VN_ERROR_CODE vn100_getSynchronizationStatus(
	Vn100* vn100, 
	uint32_t* syncInCount, 
	uint32_t* syncInTime, 
	uint32_t* syncOutCount)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getSynchronizationStatus(
		&vn100->vndevice,
		syncInCount,
		syncInTime,
		syncOutCount);
}

VN_ERROR_CODE vn100_setSynchronizationStatus(
	Vn100* vn100, 
	uint32_t syncInCount, 
	uint32_t syncInTime, 
	uint32_t syncOutCount, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setSynchronizationStatus(
		&vn100->vndevice,
		syncInCount,
		syncInTime,
		syncOutCount,
		waitForResponse);
}

VN_ERROR_CODE vn100_getDeltaThetaAndDeltaVelocity(
	Vn100* vn100, 
	float* deltaTime, 
	VnVector3* deltaTheta, 
	VnVector3* deltaVelocity)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getDeltaThetaAndDeltaVelocity(
		&vn100->vndevice,
		deltaTime,
		deltaTheta,
		deltaVelocity);
}

VN_ERROR_CODE vn100_getAccelerationCompensation(
	Vn100* vn100, 
	VnMatrix3x3* c, 
	VnVector3* b)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getAccelerationCompensation(
		&vn100->vndevice,
		c,
		b);
}

VN_ERROR_CODE vn100_setAccelerationCompensation(
	Vn100* vn100, 
	VnMatrix3x3 c, 
	VnVector3 b, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setAccelerationCompensation(
		&vn100->vndevice,
		c,
		b,
		waitForResponse);
}

VN_ERROR_CODE vn100_getMagneticCompensation(
	Vn100* vn100, 
	VnMatrix3x3* c, 
	VnVector3* b)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagneticCompensation(
		&vn100->vndevice,
		c,
		b);
}

VN_ERROR_CODE vn100_setMagneticCompensation(
	Vn100* vn100, 
	VnMatrix3x3 c, 
	VnVector3 b, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setMagneticCompensation(
		&vn100->vndevice,
		c,
		b,
		waitForResponse);
}

VN_ERROR_CODE vn100_getGyroCompensation(
	Vn100* vn100, 
	VnMatrix3x3* c, 
	VnVector3* b)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getGyroCompensation(
		&vn100->vndevice,
		c,
		b);
}

VN_ERROR_CODE vn100_setGyroCompensation(
	Vn100* vn100, 
	VnMatrix3x3 c, 
	VnVector3 b, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setGyroCompensation(
		&vn100->vndevice,
		c,
		b,
		waitForResponse);
}

VN_ERROR_CODE vn100_getImuFilteringConfiguration(
	Vn100* vn100, 
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
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getImuFilteringConfiguration(
		&vn100->vndevice,
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

VN_ERROR_CODE vn100_setImuFilteringConfiguration(
	Vn100* vn100, 
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
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setImuFilteringConfiguration(
		&vn100->vndevice,
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

VN_ERROR_CODE vn100_getDeltaThetaAndDeltaVelocityConfiguration(
	Vn100* vn100, 
	uint8_t* integrationFrame, 
	uint8_t* gyroCompensation, 
	uint8_t* accelCompensation)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getDeltaThetaAndDeltaVelocityConfiguration(
		&vn100->vndevice,
		integrationFrame,
		gyroCompensation,
		accelCompensation);
}

VN_ERROR_CODE vn100_setDeltaThetaAndDeltaVelocityConfiguration(
	Vn100* vn100, 
	uint8_t integrationFrame, 
	uint8_t gyroCompensation, 
	uint8_t accelCompensation, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setDeltaThetaAndDeltaVelocityConfiguration(
		&vn100->vndevice,
		integrationFrame,
		gyroCompensation,
		accelCompensation,
		waitForResponse);
}

VN_ERROR_CODE vn100_getMagnetometerCalibrationControl(
	Vn100* vn100, 
	uint8_t* hsiMode, 
	uint8_t* hsiOutput, 
	uint8_t* convergeRate)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagnetometerCalibrationControl(
		&vn100->vndevice,
		hsiMode,
		hsiOutput,
		convergeRate);
}

VN_ERROR_CODE vn100_setMagnetometerCalibrationControl(
	Vn100* vn100, 
	uint8_t hsiMode, 
	uint8_t hsiOutput, 
	uint8_t convergeRate, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setMagnetometerCalibrationControl(
		&vn100->vndevice,
		hsiMode,
		hsiOutput,
		convergeRate,
		waitForResponse);
}

VN_ERROR_CODE vn100_getCalculatedMagnetometerCalibration(
	Vn100* vn100, 
	VnMatrix3x3* c, 
	VnVector3* b)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getCalculatedMagnetometerCalibration(
		&vn100->vndevice,
		c,
		b);
}

VN_ERROR_CODE vn100_getMagneticGravityReferenceVectors(
	Vn100* vn100, 
	VnVector3* magneticReference, 
	VnVector3* gravityReference)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getMagneticGravityReferenceVectors(
		&vn100->vndevice,
		magneticReference,
		gravityReference);
}

VN_ERROR_CODE vn100_setMagneticGravityReferenceVectors(
	Vn100* vn100, 
	VnVector3 magneticReference, 
	VnVector3 gravityReference, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setMagneticGravityReferenceVectors(
		&vn100->vndevice,
		magneticReference,
		gravityReference,
		waitForResponse);
}

VN_ERROR_CODE vn100_getCommunicationProtocolControl(
	Vn100* vn100, 
	uint8_t* serialCount, 
	uint8_t* serialStatus, 
	uint8_t* spiCount, 
	uint8_t* spiStatus, 
	uint8_t* serialChecksum, 
	uint8_t* spiChecksum, 
	uint8_t* errorMode)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getCommunicationProtocolControl(
		&vn100->vndevice,
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode);
}

VN_ERROR_CODE vn100_setCommunicationProtocolControl(
	Vn100* vn100, 
	uint8_t serialCount, 
	uint8_t serialStatus, 
	uint8_t spiCount, 
	uint8_t spiStatus, 
	uint8_t serialChecksum, 
	uint8_t spiChecksum, 
	uint8_t errorMode, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setCommunicationProtocolControl(
		&vn100->vndevice,
		serialCount,
		serialStatus,
		spiCount,
		spiStatus,
		serialChecksum,
		spiChecksum,
		errorMode,
		waitForResponse);
}

VN_ERROR_CODE vn100_getReferenceVectorConfiguration(
	Vn100* vn100, 
	uint8_t* useMagModel, 
	uint8_t* useGravityModel, 
	uint32_t* recalcThreshold, 
	float* year, 
	VnVector3* lla)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_getReferenceVectorConfiguration(
		&vn100->vndevice,
		useMagModel,
		useGravityModel,
		recalcThreshold,
		year,
		lla);
}

VN_ERROR_CODE vn100_setReferenceVectorConfiguration(
	Vn100* vn100, 
	uint8_t useMagModel, 
	uint8_t useGravityModel, 
	uint32_t recalcThreshold, 
	float year, 
	VnVector3 lla, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_setReferenceVectorConfiguration(
		&vn100->vndevice,
		useMagModel,
		useGravityModel,
		recalcThreshold,
		year,
		lla,
		waitForResponse);
}

VN_ERROR_CODE vn100_pauseAsyncOutputs(
	Vn100* vn100, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_pauseAsyncOutputs(
		&vn100->vndevice,
		waitForResponse);
}

VN_ERROR_CODE vn100_resumeAsyncOutputs(
	Vn100* vn100, 
	bool waitForResponse)
{
	if (!vn100->isConnected)
		return VNERR_NOT_CONNECTED;

	return vndevice_resumeAsyncOutputs(
		&vn100->vndevice,
		waitForResponse);
}

/** \endcond */
