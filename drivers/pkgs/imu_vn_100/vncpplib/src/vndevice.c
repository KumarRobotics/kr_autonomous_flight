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
 * This file implements functionality common between VectorNav devices.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "vndevice.h"

/* Defines and constants. ****************************************************/

/*#define COMMAND_HEADER_SIZE				5*/
#define ASCII_RECEIVE_BUFFER_SIZE		256
#define BINARY_RECEIVE_BUFFER_SIZE		256
#define READ_BUFFER_SIZE				256
#define NUMBER_OF_MILLISECONDS_TO_SLEEP_AFTER_RECEIVING_NO_BYTES_ON_COM_PORT_READ	1
#define DEFAULT_TIMEOUT_IN_MS			1000
#define ASCII_START_CHAR				'$'
#define ASCII_FIRST_END_CHAR			'\r'
#define ASCII_SECOND_END_CHAR			'\n'
#define BINARY_START_CHAR				((char) 0xFA)

/* Collection of group lengths for binary packets. This array is copied from the
   VN-200 user manual. */
const unsigned char BinaryPacketGroupLengths[6][16] = {
	{8, 8, 8, 12, 16, 12, 24, 12, 12, 24, 20, 28, 2, 4, 8, 0},		/* Group 1 */
	{8, 8, 8, 2, 8, 8, 8, 4, 0, 0, 0, 0, 0, 0, 0, 0},				/* Group 2 */
	{2, 12, 12, 12, 4, 4, 16, 12, 12, 12, 12, 2, 40, 0, 0, 0},		/* Group 3 */
	{8, 8, 2, 1, 1, 24, 24, 12, 12, 12, 4, 4, 32, 0, 0, 0},			/* Group 4 */
	{2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24, 0, 0, 0, 0},	/* Group 5 */
	{2, 24, 24, 12, 12, 12, 12, 12, 12, 4, 4, 68, 64, 0, 0, 0},		/* Group 6 */
};

/* Private function definitions. *********************************************/

/**
 * \brief Indicates whether the comPortServiceThread should be checking
 * incoming data packets for matches to a command sent out.
 *
 * \param[in]	vndevice	Pointer to the VnDevice control object.
 *
 * \param[out]	responseMatchBuffer
 * Buffer where the match string will be placed if response checking is current
 * enabled. The size of this buffer should be VN_RESPONSE_MATCH_SIZE + 1. The
 * returned string will be null-terminated.
 *
 * \return true if response checking should be performed; false if no
 * checking should be performed.
 */
bool vndevice_shouldCheckForResponse_threadSafe(
	VnDevice* vndevice,
	char* responseMatchBuffer);

/**
 * \brief Disable response checking by the comPortServiceThread.
 *
 * \param[in]	vndevice	Pointer to the VnDevice control object.
 */
void vndevice_disableResponseChecking_threadSafe(
	VnDevice* vndevice);

/**
 * \brief Reads data from the connected COM port in a thread-safe manner.
 *
 * Reads data from the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions vndevice_writeData_threadSafe and vndevice_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int vndevice_readData_threadSafe(
	VnDevice* vndevice,
	char* dataBuffer,
	unsigned int numOfBytesToRead,
	unsigned int* numOfBytesActuallyRead);

/**
 * \brief Sends out data over the connected COM port in a thread-safe manner.
 *
 * Sends out data over the connected COM port in a thread-safe manner to avoid
 * conflicts between the comPortServiceThread and the user thread. Use only the
 * functions vn100_writeData_threadSafe and vn100_readData_threadSafe to ensure
 * communcation over the COM port is thread-safe.
 */
int vndevice_writeData_threadSafe(
	VnDevice* vndevice,
	const char* dataToSend,
	unsigned int dataLength);

/**
 * \brief Enabled response checking by the comPortServiceThread.
 *
 * \param[in]	vndevice	Pointer to the VnDevice control object.
 *
 * \param[in]	responseMatch
 * Null-terminated string with a maximum length of VN_RESPONSE_MATCH_SIZE which
 * will be used by the comPortServiceThread to detect a received data packet
 * is an appropriate match to a command sent to the VectorNav device.
 */
void vndevice_enableResponseChecking_threadSafe(
	VnDevice* vndevice,
	const char* responseMatch);

void vndevice_processAsyncData(
	VnDevice* vndevice,
	char* buffer);

void* vndevice_communicationHandler(
	void* vndevice);

void vndevice_processReceivedPacket(
	VnDevice* vndevice,
	char* buffer);

unsigned char vndevice_numberOfSetBits(
	unsigned char toCount);

int vndevice_computeLengthOfExpectedBinaryPayload(
	char* ptrToPacketStart);

void vndevice_processReceivedBinaryPacket(
	VnDevice* vndevice,
	char* buffer);

uint16_t vndevice_processGroup1Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data);

uint16_t vndevice_processGroup2Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data);

uint16_t vndevice_processGroup3Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data);

uint16_t vndevice_processGroup4Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data);

uint16_t vndevice_processGroup5Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data);

uint16_t vndevice_processGroup6Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data);

/**
 * \brief Converts a sensor error code into the library's system error code.
 *
 * \param[in] sensorError The sensor error code.
 *
 * \return The sensor error code converted to the library's system error code.
 */
VN_ERROR_CODE vndevice_convertSensorErrorToSystemError(
	uint8_t sensorError);

/* Function implementations. *************************************************/

VN_ERROR_CODE vndevice_convertSensorErrorToSystemError(
	uint8_t sensorError)
{
	return sensorError + VNERR_PERMISSION_DENIED;
}

int vndevice_computeLengthOfBinaryGroupPayload(
	unsigned char groupIndex,
	uint16_t groupField)
{
	unsigned char i;
	int runningLength = 0;

	for (i = 0; i < sizeof (uint16_t) * 8; i++) {

		if ((groupField >> i) & 1) {
			runningLength += BinaryPacketGroupLengths[groupIndex][i];
		}
	}

	return runningLength;
}

VN_ERROR_CODE vndevice_initializeVnDevice(
	VnDevice* vndevice,
	const char* portName,
	int baudrate,
	void* deviceMask)
{
	VN_ERROR_CODE errorCode;

	vndevice->asyncDataListener = NULL;
	vndevice->errorCodeListener = NULL;
	vndevice->sensorError = 0;
	vndevice->continueServicingComPort = true;
	vndevice->deviceMask = deviceMask;
	vndevice->checkForResponse = false;
	vndevice->timeout = DEFAULT_TIMEOUT_IN_MS;

	memset(&vndevice->lastestAsyncData, 0, sizeof(VnDeviceCompositeData));

	errorCode = vncp_comPort_open(&vndevice->comPortHandle, portName, baudrate);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_criticalSection_initialize(&vndevice->critSecForComPort);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_criticalSection_initialize(&vndevice->critSecForResponseMatchAccess);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_event_create(&vndevice->waitForThreadToStopServicingComPortEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_event_create(&vndevice->waitForCommandResponseEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_event_create(&vndevice->waitForThreadToStartServicingComPortEvent);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	errorCode = vncp_criticalSection_initialize(&vndevice->critSecForLatestAsyncDataAccess);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_deinitializeVnDevice(
	VnDevice* vndevice)
{
	vndevice->continueServicingComPort = false;

	vncp_event_waitFor(vndevice->waitForThreadToStopServicingComPortEvent, -1);

	vncp_comPort_close(vndevice->comPortHandle);

	vncp_criticalSection_dispose(&vndevice->critSecForComPort);

	vncp_criticalSection_dispose(&vndevice->critSecForResponseMatchAccess);

	vncp_criticalSection_dispose(&vndevice->critSecForLatestAsyncDataAccess);

	return VNERR_NO_ERROR;
}

char* vndevice_getResponseBuffer(
	VnDevice* vndevice)
{
	return vndevice->cmdResponseBuffer;
}

VN_ERROR_CODE vndevice_set_timeout(
	VnDevice* vndevice,
	int timeout)
{
	if (timeout < -1)
		return VNERR_INVALID_VALUE;

	vndevice->timeout = timeout;

	return VNERR_NO_ERROR;
}

int vndevice_get_timeout(
	VnDevice* vndevice)
{
	return vndevice->timeout;
}

VN_ERROR_CODE vndevice_getCurrentAsyncData(
	VnDevice* vndevice,
	VnDeviceCompositeData* curData)
{
	vncp_criticalSection_enter(&vndevice->critSecForLatestAsyncDataAccess);

	memcpy(curData, &vndevice->lastestAsyncData, sizeof(VnDeviceCompositeData));

	vncp_criticalSection_leave(&vndevice->critSecForLatestAsyncDataAccess);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_registerAsyncDataReceivedListener(
	VnDevice* vndevice,
	VnDeviceNewAsyncDataReceivedListener listener)
{
	/* We can only handle one listener for now. */
	if (vndevice->asyncDataListener != NULL)
		return VNERR_UNKNOWN_ERROR;

	vndevice->asyncDataListener = listener;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_unregisterAsyncDataReceivedListener(
	VnDevice* vndevice,
	VnDeviceNewAsyncDataReceivedListener listener)
{
	if (vndevice->asyncDataListener == NULL)
		return VNERR_UNKNOWN_ERROR;

	if (vndevice->asyncDataListener != listener)
		return VNERR_UNKNOWN_ERROR;

	vndevice->asyncDataListener = NULL;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_registerErrorCodeReceivedListener(
	VnDevice* vndevice,
	VnDeviceErrorCodeReceivedListener listener)
{
	/* We can only handle one listener for now. */
	if (vndevice->errorCodeListener != NULL)
		return VNERR_UNKNOWN_ERROR;

	vndevice->errorCodeListener = listener;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_unregisterErrorCodeReceivedListener(
	VnDevice* vndevice,
	VnDeviceErrorCodeReceivedListener listener)
{
	if (vndevice->errorCodeListener == NULL)
		return VNERR_UNKNOWN_ERROR;

	if (vndevice->errorCodeListener != listener)
		return VNERR_UNKNOWN_ERROR;

	vndevice->errorCodeListener = NULL;

	return VNERR_NO_ERROR;
}

void vndevice_processReceivedBinaryPacket(
	VnDevice* vndevice,
	char* buffer)
{
	VnDeviceCompositeData data;
	uint8_t groups;
	uint8_t numberOfTotalGroupFieldBytes;
	char* curDataPointer;
	char* curGroupFieldPointer;
	uint16_t numberOfDataBytesProcessed;

	memset(&data, 0, sizeof(VnDeviceCompositeData));

	groups = *((uint8_t*) (buffer + 1));

	numberOfTotalGroupFieldBytes = vndevice_numberOfSetBits(groups) * 2;

	curGroupFieldPointer = buffer + 1 + 1;

	curDataPointer = buffer + 1 + 1 + numberOfTotalGroupFieldBytes;

	if (groups & 0x01) {

		numberOfDataBytesProcessed = vndevice_processGroup1Data( *((uint16_t*) curGroupFieldPointer), curDataPointer, &data);

		curDataPointer += numberOfDataBytesProcessed;

		curGroupFieldPointer += sizeof (uint16_t);
	}

	if (groups & 0x02) {

		numberOfDataBytesProcessed = vndevice_processGroup2Data( *((uint16_t*) curGroupFieldPointer), curDataPointer, &data);

		curDataPointer += numberOfDataBytesProcessed;

		curGroupFieldPointer += sizeof (uint16_t);
	}

	if (groups & 0x04) {

		numberOfDataBytesProcessed = vndevice_processGroup3Data( *((uint16_t*) curGroupFieldPointer), curDataPointer, &data);

		curDataPointer += numberOfDataBytesProcessed;

		curGroupFieldPointer += sizeof (uint16_t);
	}

	if (groups & 0x08) {

		numberOfDataBytesProcessed = vndevice_processGroup4Data( *((uint16_t*) curGroupFieldPointer), curDataPointer, &data);

		curDataPointer += numberOfDataBytesProcessed;

		curGroupFieldPointer += sizeof (uint16_t);
	}

	if (groups & 0x10) {

		numberOfDataBytesProcessed = vndevice_processGroup5Data( *((uint16_t*) curGroupFieldPointer), curDataPointer, &data);

		curDataPointer += numberOfDataBytesProcessed;

		curGroupFieldPointer += sizeof (uint16_t);
	}

	if (groups & 0x20) {

		numberOfDataBytesProcessed = vndevice_processGroup6Data( *((uint16_t*) curGroupFieldPointer), curDataPointer, &data);

		curDataPointer += numberOfDataBytesProcessed;

		curGroupFieldPointer += sizeof (uint16_t);
	}

	/* We had an async data packet and need to move it to vndevice->lastestAsyncData. */
	vncp_criticalSection_enter(&vndevice->critSecForLatestAsyncDataAccess);
	memcpy(&vndevice->lastestAsyncData, &data, sizeof(VnDeviceCompositeData));
	vncp_criticalSection_leave(&vndevice->critSecForLatestAsyncDataAccess);

	if (vndevice->asyncDataListener != NULL)
		vndevice->asyncDataListener(vndevice, &vndevice->lastestAsyncData);
}

uint16_t vndevice_processGroup1Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data)
{
	char* originalGroupDataPtr = groupDataPtr;

	if (groupField & 0x0001) {

		data->timeStartup = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0002) {

		data->timeGps = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0004) {

		data->timeSyncIn = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0008) {

		data->ypr.yaw = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->ypr.pitch = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->ypr.roll = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0010) {

		data->quaternion.x = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->quaternion.y = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->quaternion.z = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->quaternion.w = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0020) {

		data->angularRate.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->angularRate.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->angularRate.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0040) {

		data->latitudeLongitudeAltitude.c0 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->latitudeLongitudeAltitude.c1 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->latitudeLongitudeAltitude.c2 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);
	}

	if (groupField & 0x0080) {

		data->velocity.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->velocity.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->velocity.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0100) {

		data->acceleration.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->acceleration.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->acceleration.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0200) {

		data->angularRateUncompensated.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->angularRateUncompensated.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->angularRateUncompensated.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelerationUncompensated.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelerationUncompensated.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelerationUncompensated.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0400) {

		data->magnetic.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magnetic.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magnetic.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->temperature = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->pressure = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0800) {

		data->deltaTime = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaTheta.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaTheta.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaTheta.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaVelocity.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaVelocity.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaVelocity.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x1000) {

		data->insStatus = *((uint16_t*) groupDataPtr);

		groupDataPtr += sizeof (uint16_t);
	}

	if (groupField & 0x2000) {

		data->syncInCnt = *((uint32_t*) groupDataPtr);

		groupDataPtr += sizeof (uint32_t);
	}

	if (groupField & 0x4000) {

		data->timeGpsPps = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	return (uint16_t) (groupDataPtr - originalGroupDataPtr);
}

uint16_t vndevice_processGroup2Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data)
{
	char* originalGroupDataPtr = groupDataPtr;

	if (groupField & 0x0001) {

		data->timeStartup = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0002) {

		data->timeGps = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0004) {

		data->gpsTowNs = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0008) {

		data->gpsWeek = *((uint16_t*) groupDataPtr);

		groupDataPtr += sizeof (uint16_t);
	}

	if (groupField & 0x0010) {

		data->timeSyncIn = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0020) {

		data->timeGpsPps = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0040) {

		data->timeUtc.year = *((int8_t*) groupDataPtr);

		groupDataPtr += sizeof (int8_t);

		data->timeUtc.month = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.day = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.hour = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.min = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.sec = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.ms = *((uint16_t*) groupDataPtr);

		groupDataPtr += sizeof (uint16_t);
	}

	if (groupField & 0x0080) {

		data->syncInCnt = *((uint32_t*) groupDataPtr);

		groupDataPtr += sizeof (uint32_t);
	}

	return (uint16_t) (groupDataPtr - originalGroupDataPtr);
}

uint16_t vndevice_processGroup3Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data)
{
	char* originalGroupDataPtr = groupDataPtr;

	if (groupField & 0x0001) {

		/* ImuStatus is currently not used. */

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0002) {

		data->magneticUncompensated.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magneticUncompensated.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magneticUncompensated.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0004) {

		data->accelerationUncompensated.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelerationUncompensated.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelerationUncompensated.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0008) {

		data->angularRateUncompensated.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->angularRateUncompensated.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->angularRateUncompensated.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0010) {

		data->temperature = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0020) {

		data->pressure = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0040) {

		data->deltaTime = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaTheta.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaTheta.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaTheta.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0080) {

		data->deltaVelocity.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaVelocity.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->deltaVelocity.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0100) {

		data->magnetic.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magnetic.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magnetic.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0200) {

		data->acceleration.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->acceleration.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->acceleration.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0400) {

		data->angularRate.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->angularRate.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->angularRate.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0800) {

		data->sensSat = *((uint16_t*) groupDataPtr);

		groupDataPtr += sizeof (uint16_t);
	}

	return (uint16_t) (groupDataPtr - originalGroupDataPtr);
}

uint16_t vndevice_processGroup4Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data)
{
	char* originalGroupDataPtr = groupDataPtr;

	if (groupField & 0x0001) {

		data->timeUtc.year = *((int8_t*) groupDataPtr);

		groupDataPtr += sizeof (int8_t);

		data->timeUtc.month = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.day = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.hour = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.min = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.sec = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);

		data->timeUtc.ms = *((uint16_t*) groupDataPtr);

		groupDataPtr += sizeof (uint16_t);
	}

	if (groupField & 0x0002) {

		data->gpsTowNs = *((uint64_t*) groupDataPtr);

		groupDataPtr += sizeof (uint64_t);
	}

	if (groupField & 0x0004) {

		data->gpsWeek = *((uint16_t*) groupDataPtr);

		groupDataPtr += sizeof (uint16_t);
	}

	if (groupField & 0x0008) {

		data->numSats = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);
	}

	if (groupField & 0x0010) {

		data->gpsFix = *((uint8_t*) groupDataPtr);

		groupDataPtr += sizeof (uint8_t);
	}

	if (groupField & 0x0020) {

		data->gpsPosLla.c0 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->gpsPosLla.c1 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->gpsPosLla.c2 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);
	}

	if (groupField & 0x0040) {

		data->gpsPosEcef.c0 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->gpsPosEcef.c1 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->gpsPosEcef.c2 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);
	}

	if (groupField & 0x0080) {

		data->gpsVelocity.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->gpsVelocity.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->gpsVelocity.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0100) {

		data->gpsVelEcef.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->gpsVelEcef.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->gpsVelEcef.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0200) {

		data->gpsPosU.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->gpsPosU.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->gpsPosU.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0400) {

		data->gpsVelU = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0800) {

		data->timeU = *((uint32_t*) groupDataPtr);

		groupDataPtr += sizeof (uint32_t);
	}

	return (uint16_t) (groupDataPtr - originalGroupDataPtr);
}

uint16_t vndevice_processGroup5Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data)
{
	char* originalGroupDataPtr = groupDataPtr;

	if (groupField & 0x0001) {

		data->vpeStatus = *((uint16_t*) groupDataPtr);

		groupDataPtr += sizeof (uint16_t);
	}

	if (groupField & 0x0002) {

		data->ypr.yaw = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->ypr.pitch = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->ypr.roll = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0004) {

		data->quaternion.x = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->quaternion.y = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->quaternion.z = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->quaternion.w = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0008) {

		data->dcm.c00 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->dcm.c10 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->dcm.c20 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->dcm.c01 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->dcm.c11 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->dcm.c21 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->dcm.c02 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->dcm.c12 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->dcm.c22 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0010) {

		data->magNed.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magNed.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magNed.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0020) {

		data->accelNed.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelNed.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelNed.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0040) {

		data->linearAccelBody.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->linearAccelBody.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->linearAccelBody.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0080) {

		data->linearAccelNed.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->linearAccelNed.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->linearAccelNed.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0100) {

		data->yprU.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->yprU.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->yprU.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	return (uint16_t) (groupDataPtr - originalGroupDataPtr);
}

uint16_t vndevice_processGroup6Data(
	uint16_t groupField,
	char* groupDataPtr,
	VnDeviceCompositeData* data)
{
	char* originalGroupDataPtr = groupDataPtr;

	if (groupField & 0x0001) {

		data->insStatus = *((uint16_t*) groupDataPtr);

		groupDataPtr += sizeof (uint16_t);
	}

	if (groupField & 0x0002) {

		data->latitudeLongitudeAltitude.c0 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->latitudeLongitudeAltitude.c1 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->latitudeLongitudeAltitude.c2 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);
	}

	if (groupField & 0x0004) {

		data->posEcef.c0 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->posEcef.c1 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);

		data->posEcef.c2 = *((double*) groupDataPtr);

		groupDataPtr += sizeof (double);
	}

	if (groupField & 0x0008) {

		data->velBody.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->velBody.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->velBody.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0010) {

		data->velNed.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->velNed.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->velNed.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0020) {

		data->velEcef.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->velEcef.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->velEcef.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0040) {

		data->magEcef.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magEcef.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->magEcef.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0080) {

		data->accelEcef.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelEcef.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->accelEcef.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0100) {

		data->linearAccelEcef.c0 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->linearAccelEcef.c1 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);

		data->linearAccelEcef.c2 = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0200) {

		data->posU = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}

	if (groupField & 0x0400) {

		data->velU = *((float*) groupDataPtr);

		groupDataPtr += sizeof (float);
	}


	return (uint16_t) (groupDataPtr - originalGroupDataPtr);
}


unsigned char vndevice_numberOfSetBits(
	unsigned char toCount)
{
	unsigned char i;
	unsigned char numberOfSetBits = 0;

	for (i = 0; i < sizeof(unsigned char) * 8; i++)
	{
		if (toCount & 1)
			numberOfSetBits++;

		toCount = toCount >> 1;
	}

	return numberOfSetBits;
}

VN_ERROR_CODE vndevice_startHandlingCommunication(
	VnDevice* vndevice)
{
	VN_ERROR_CODE errorCode;

	errorCode = vncp_thread_startNew(&vndevice->comPortServiceThreadHandle, &vndevice_communicationHandler, vndevice);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_waitForThreadToStartHandlingCommunicationPort(
	VnDevice* vndevice)
{
	VN_ERROR_CODE errorCode;

	errorCode = vncp_event_waitFor(vndevice->waitForThreadToStartServicingComPortEvent, -1);
	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return VNERR_NO_ERROR;
}

int vndevice_writeData_threadSafe(
	VnDevice* vndevice,
	const char* dataToSend,
	unsigned int dataLength)
{
	int errorCode;

	vncp_criticalSection_enter(&vndevice->critSecForComPort);
	errorCode = vncp_comPort_writeData(vndevice->comPortHandle, dataToSend, dataLength);
	vncp_criticalSection_leave(&vndevice->critSecForComPort);

	return errorCode;
}

int vndevice_readData_threadSafe(
	VnDevice* vndevice,
	char* dataBuffer,
	unsigned int numOfBytesToRead,
	unsigned int* numOfBytesActuallyRead)
{
	int errorCode;

	vncp_criticalSection_enter(&vndevice->critSecForComPort);
	errorCode = vncp_comPort_readData(vndevice->comPortHandle, dataBuffer, numOfBytesToRead, numOfBytesActuallyRead);
	vncp_criticalSection_leave(&vndevice->critSecForComPort);

	return errorCode;
}

void vndevice_enableResponseChecking_threadSafe(
	VnDevice* vndevice,
	const char* responseMatch)
{
	vncp_criticalSection_enter(&vndevice->critSecForResponseMatchAccess);

	vndevice->checkForResponse = true;
	strcpy(vndevice->cmdResponseMatchBuffer, responseMatch);

	vncp_criticalSection_leave(&vndevice->critSecForResponseMatchAccess);
}

void vndevice_checksum_computeAndReturnAsHex(
	const char* cmdToCheck,
	char* checksum)
{
	unsigned char cs;
	char tempChecksumHolder[3];

	cs = vndevice_checksum_computeCrc8FromCommand(cmdToCheck);

	/* We cannot sprintf into the parameter checksum because sprintf always
	   appends a null at the end. */
	sprintf(tempChecksumHolder, "%02X", cs);

	checksum[0] = tempChecksumHolder[0];
	checksum[1] = tempChecksumHolder[1];
}

uint8_t vndevice_checksum_computeCrc8FromCommand(
	const char* cmdToCheck)
{
	int i;
	unsigned char xorVal = 0;
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	for (i = 0; i < cmdLength; i++)
		xorVal ^= (uint8_t) cmdToCheck[i];

	return xorVal;
}

uint16_t vndevice_checksum_computeCrc16(
	const char data[],
	uint32_t length)
{
	uint32_t i;
	uint16_t crc = 0;

	for (i = 0; i < length; i++) {

		crc = (uint16_t) ((crc >> 8) | (crc << 8));

		crc ^= (uint8_t) data[i];
		crc ^= (uint16_t) ((uint8_t)(crc & 0xFF) >> 4);
		crc ^= (uint16_t) ((crc << 8) << 4);
		crc ^= (uint16_t) (((crc & 0xFF) << 4) << 1);
	}

	return crc;
}

uint16_t vndevice_checksum_computeCrc16FromCommand(
	const char* cmdToCheck)
{
	int cmdLength;

	cmdLength = strlen(cmdToCheck);

	return vndevice_checksum_computeCrc16(
		cmdToCheck,
		cmdLength);
}

VN_ERROR_CODE vndevice_writeOutCommand(
	VnDevice* vndevice,
	const char* cmdToSend)
{
	char packetTail[] = "*FF\r\n";

	/* We add one to the cmdToSend pointer to skip over the '$' at the beginning. */
	/* We add one to the packetTail pointer so the "FF" string is overwritten with the checksum. */
	vndevice_checksum_computeAndReturnAsHex(cmdToSend + 1, packetTail + 1);

	vndevice_writeData_threadSafe(vndevice, cmdToSend, strlen(cmdToSend));
	vndevice_writeData_threadSafe(vndevice, packetTail, strlen(packetTail));

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_transaction(
	VnDevice* vndevice,
	const char* cmdToSend,
	const char* responseMatch)
{
	char packetTail[] = "*FF\r\n";
	VN_ERROR_CODE errorCode;

	vndevice->sensorError = 0;

	/* We add one to the cmdToSend pointer to skip over the '$' at the beginning. */
	/* We add one to the packetTail pointer so the "FF" string is overwritten with the checksum. */
	vndevice_checksum_computeAndReturnAsHex(cmdToSend + 1, packetTail + 1);

	vndevice_enableResponseChecking_threadSafe(vndevice, responseMatch);

	vndevice_writeData_threadSafe(vndevice, cmdToSend, strlen(cmdToSend));
	vndevice_writeData_threadSafe(vndevice, packetTail, strlen(packetTail));

	errorCode = vncp_event_waitFor(vndevice->waitForCommandResponseEvent, vndevice->timeout);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return vndevice->sensorError;
}

void* vndevice_communicationHandler(
	void* vndeviceObj)
{
	VnDevice* vndevice;
	unsigned int asciiBufferIndex = 0;
	char asciiReceiveBuffer[ASCII_RECEIVE_BUFFER_SIZE];
	bool possibleStartOfAsciiPacketFound = false;
	bool possibleFirstEndDelimiterOfAsciiPacketFound = false;
	unsigned int binaryBufferIndex = 0;
	char binaryReceiveBuffer[BINARY_RECEIVE_BUFFER_SIZE];
	char readBuffer[READ_BUFFER_SIZE];
	bool possibleStartOfBinaryPacketFound = false;
	bool binaryGroupsFieldFound = false;
	char expectedNumberOfBinaryGroupFieldBytes = 0;
	char numberOfBinaryGroupFieldBytesProcessed = 0;
	unsigned int expectedBinaryPacketLength = 0;

	unsigned char binaryGroups = 0;
	unsigned int numOfBytesRead = 0;

	vndevice = (VnDevice*) vndeviceObj;

	vncp_event_signal(vndevice->waitForThreadToStartServicingComPortEvent);

	while (vndevice->continueServicingComPort) {

		unsigned int curResponsePos = 0;

		vndevice_readData_threadSafe(vndevice, readBuffer, READ_BUFFER_SIZE, &numOfBytesRead);

		if (numOfBytesRead == 0) {
			/* There was no data. Sleep for a short amount of time before continuing. */
			vncp_sleepInMs(NUMBER_OF_MILLISECONDS_TO_SLEEP_AFTER_RECEIVING_NO_BYTES_ON_COM_PORT_READ);
			continue;
		}

		for ( ; curResponsePos < numOfBytesRead; curResponsePos++) {

			char curChar = readBuffer[curResponsePos];

			/* Make sure we are not overrunning our ASCII buffer. */
			if (asciiBufferIndex == ASCII_RECEIVE_BUFFER_SIZE) {

				/* We are about to overflow our buffer. Reset the index and
				   start looking for another ASCII packet. */
				asciiBufferIndex = 0;
				possibleStartOfAsciiPacketFound = false;
			}

			/* See if we have possible found the start of a new ASCII packet.
			   Note that we only check if we have found the character '$' and
			   do not also check if possibleStartOfAsciiPacketFound == false.
			   This is because if we are also in the condition
			   possibleStartOfAsciiPacketFound == true and we find another
			   '$' character, which should not be in any ASCII packet except at
			   the beginning of the packet, we will start over looking for an
			   ASCII packet in. */
			if (readBuffer[curResponsePos] == ASCII_START_CHAR) {

				/* Start of command has been found. */
				possibleStartOfAsciiPacketFound = true;
				asciiBufferIndex = 0;
				asciiReceiveBuffer[asciiBufferIndex] = curChar;
				asciiBufferIndex++;
			}
			else if (possibleStartOfAsciiPacketFound) {

				/* We are in the process of handling receipt of an ASCII packet. */

				if (!possibleFirstEndDelimiterOfAsciiPacketFound && readBuffer[curResponsePos] == ASCII_FIRST_END_CHAR) {

					/* We have found the first end delimiter character. */
					possibleFirstEndDelimiterOfAsciiPacketFound = true;
					asciiReceiveBuffer[asciiBufferIndex] = curChar;
					asciiBufferIndex++;

				}

				else if (possibleFirstEndDelimiterOfAsciiPacketFound && readBuffer[curResponsePos] == ASCII_SECOND_END_CHAR) {

					/* We have found a complete ASCII packet now. */
					asciiReceiveBuffer[asciiBufferIndex] = curChar;
					asciiBufferIndex++;
					possibleFirstEndDelimiterOfAsciiPacketFound = false;
					possibleStartOfAsciiPacketFound = false;

					/* Make sure we have enough room to append null termination. */
					if (asciiBufferIndex == ASCII_RECEIVE_BUFFER_SIZE) {

						/* We are about to overflow our buffer so we cannot continue processing. */

					}
					else {

						asciiReceiveBuffer[asciiBufferIndex] = '\0';
						asciiBufferIndex++;

						/* Found a packet, now dispatch it. */
						vndevice_processReceivedPacket(vndevice, asciiReceiveBuffer);
					}

					asciiBufferIndex = 0;
				}

				else {

					/* The previous character we handled might have been an '/r'.
					   Let's clear out the flag that we might have found the end
					   sequence of a packet. */
					possibleFirstEndDelimiterOfAsciiPacketFound = false;

					/* Copy over the found char. */
					asciiReceiveBuffer[asciiBufferIndex] = curChar;
					asciiBufferIndex++;
				}
			}

			/* Make sure we are not overrunning our BINARY buffer. */
			if (binaryBufferIndex == BINARY_RECEIVE_BUFFER_SIZE) {

				/* We are about to overflow our buffer. Reset the index and
				   start looking for another BINARY packet. */
				binaryBufferIndex = 0;
				possibleStartOfBinaryPacketFound = false;
				binaryGroupsFieldFound = false;
				binaryGroups = 0;
			}

			if (!possibleStartOfBinaryPacketFound && curChar == BINARY_START_CHAR) {

				/* We have possibly found the start of a binary packet. */
				possibleStartOfBinaryPacketFound = true;
				binaryReceiveBuffer[binaryBufferIndex] = curChar;
				binaryBufferIndex++;
			}

			else if (possibleStartOfBinaryPacketFound) {

				if (!binaryGroupsFieldFound) {

					/* We now have a possible binary packet groups field. */
					binaryGroups = curChar;
					binaryGroupsFieldFound = true;

					binaryReceiveBuffer[binaryBufferIndex] = curChar;
					binaryBufferIndex++;

					expectedNumberOfBinaryGroupFieldBytes = vndevice_numberOfSetBits(binaryGroups) * 2;
					numberOfBinaryGroupFieldBytesProcessed = 0;
				}

				else {

					if (numberOfBinaryGroupFieldBytesProcessed < expectedNumberOfBinaryGroupFieldBytes) {

						/* We have not received all of the expected group fields. */

						binaryReceiveBuffer[binaryBufferIndex] = curChar;
						binaryBufferIndex++;

						numberOfBinaryGroupFieldBytesProcessed++;

						if (numberOfBinaryGroupFieldBytesProcessed == expectedNumberOfBinaryGroupFieldBytes) {

							/* We have all of our group fields. Now we can calculate the expected
							   length of this packet. */

							int payloadLength;

							payloadLength = vndevice_computeLengthOfExpectedBinaryPayload(binaryReceiveBuffer);

							expectedBinaryPacketLength = 2 + expectedNumberOfBinaryGroupFieldBytes + payloadLength + 2;

							if (expectedBinaryPacketLength > BINARY_RECEIVE_BUFFER_SIZE) {

								/* This packet will be too large for our buffer
								   and is likely not a valid packet. */
								binaryBufferIndex = 0;
								possibleStartOfBinaryPacketFound = false;
								binaryGroupsFieldFound = false;
							}
						}
					}

					else {

						/* Add this byte to the binary buffer. */
						binaryReceiveBuffer[binaryBufferIndex] = curChar;
						binaryBufferIndex++;

						/* See if we have reached the end of a binary packet. */
						if (binaryBufferIndex == expectedBinaryPacketLength) {

							/* We might have a packet if we reach here. */
							uint16_t calculatedCrc;

							/* Verify the packet is valid. */
							calculatedCrc = vndevice_checksum_computeCrc16(binaryReceiveBuffer + 1, expectedBinaryPacketLength - 1);
							if (calculatedCrc == 0) {

								/* We have a valid binary packet. */

								vndevice_processReceivedBinaryPacket(vndevice, binaryReceiveBuffer);

							}
							else {

								/* We did not have a valid data packet. Let's
								   process the data we have to see if there are
								   any packets contained in it. */

								/* TODO: Reprocess the binary data buffer to see if there is any data in it. */
							}

							/* Reset the search for a new binary packet. */
							binaryBufferIndex = 0;
							possibleStartOfBinaryPacketFound = false;
							binaryGroupsFieldFound = false;
						}
					}
				}
			}
		}
	}

	vncp_event_signal(vndevice->waitForThreadToStopServicingComPortEvent);

	return VN_NULL;
}

int vndevice_computeLengthOfExpectedBinaryPayload(
	char* ptrToPacketStart)
{
	unsigned char groups = ptrToPacketStart[1];
	int runningPayloadLength = 0;
	char* ptrToCurrentGroupField = ptrToPacketStart + 2;

	if (groups & 0x01) {

		/* We have group 1 present. */
		runningPayloadLength += vndevice_computeLengthOfBinaryGroupPayload(0, *((uint16_t*) ptrToCurrentGroupField));

		ptrToCurrentGroupField += 2;
	}

	if (groups & 0x02) {

		/* We have group 2 present. */
		runningPayloadLength += vndevice_computeLengthOfBinaryGroupPayload(1, *((uint16_t*) ptrToCurrentGroupField));

		ptrToCurrentGroupField += 2;
	}

	if (groups & 0x04) {

		/* We have group 3 present. */
		runningPayloadLength += vndevice_computeLengthOfBinaryGroupPayload(2, *((uint16_t*) ptrToCurrentGroupField));

		ptrToCurrentGroupField += 2;
	}

	if (groups & 0x08) {

		/* We have group 4 present. */
		runningPayloadLength += vndevice_computeLengthOfBinaryGroupPayload(3, *((uint16_t*) ptrToCurrentGroupField));

		ptrToCurrentGroupField += 2;
	}

	if (groups & 0x10) {

		/* We have group 5 present. */
		runningPayloadLength += vndevice_computeLengthOfBinaryGroupPayload(4, *((uint16_t*) ptrToCurrentGroupField));

		ptrToCurrentGroupField += 2;
	}

	if (groups & 0x20) {

		/* We have group 6 present. */
		runningPayloadLength += vndevice_computeLengthOfBinaryGroupPayload(5, *((uint16_t*) ptrToCurrentGroupField));

		ptrToCurrentGroupField += 2;
	}

	return runningPayloadLength;
}


void vndevice_processReceivedPacket(
	VnDevice* vndevice,
	char* buffer)
{
	char responseMatch[VN_RESPONSE_MATCH_SIZE + 1];

	/* See if we have an error from the sensor. */
	if (strncmp("VNERR", buffer + 1, strlen("VNERR")) == 0) {

		char* result;

		/* Error encountered when trying to get response from the sensor. */

		result = strtok(buffer, ",*");	/* Returns VNERR */
		result = strtok(0, ",*");		/* Returns error code */

		vndevice->sensorError = vndevice_convertSensorErrorToSystemError((uint8_t) atoi(result));

		/* Signal to the user thread we have received a response. */
		if (vndevice_shouldCheckForResponse_threadSafe(vndevice, responseMatch)) {
			vndevice_disableResponseChecking_threadSafe(vndevice);
			vncp_event_signal(vndevice->waitForCommandResponseEvent);
		}

		if (vndevice->errorCodeListener != NULL)
			vndevice->errorCodeListener(vndevice->deviceMask, vndevice->sensorError);
	}

	/* See if we should be checking for a command response. */
	else if (vndevice_shouldCheckForResponse_threadSafe(vndevice, responseMatch)) {

		/* Does the data packet match the command response we expect? */
		if (strncmp(responseMatch, buffer + 1, strlen(responseMatch)) == 0) {

			/* We found a command response match! */

			/* If everything checks out on this command packet, let's disable
			 * further response checking. */
			vndevice_disableResponseChecking_threadSafe(vndevice);

			/* The line below should be thread-safe since the user thread should be
			 * blocked until we signal that we have received the response. */
			strcpy(vndevice->cmdResponseBuffer, buffer);

			/* Signal to the user thread we have received a response. */
			vncp_event_signal(vndevice->waitForCommandResponseEvent);
		}
	}
	else {
		vndevice_processAsyncData(vndevice, buffer);
	}
}

void vndevice_processAsyncData(
	VnDevice* vndevice,
	char* buffer)
{
	VnDeviceCompositeData data;
	char delims[] = ",";
	char* result;
  char sync_count[11] = {0};

	memset(&data, 0, sizeof(VnDeviceCompositeData));

	if (strncmp(buffer, "$VNYPR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNQTN", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNQTM", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNQTA", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNQTR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNQMA", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNQAR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNQMR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNDCM", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c00 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c01 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c02 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c10 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c11 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c12 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c20 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c21 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.dcm.c22 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNMAG", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNACC", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNGYR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNMAR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNYMR", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNYCM", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.temperature = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNYBA", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNYIA", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNICM", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNRAW", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magneticVoltage.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magneticVoltage.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magneticVoltage.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.accelerationVoltage.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.accelerationVoltage.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.accelerationVoltage.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateVoltage.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateVoltage.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateVoltage.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.temperatureVoltage = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNCMV", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.temperature = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNSTV", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.x = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.y = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.z = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.quaternion.w = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBias.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBias.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBias.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNCOV", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.attitudeVariance.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.attitudeVariance.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.attitudeVariance.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBiasVariance.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBiasVariance.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRateBiasVariance.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNIMU", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.magnetic.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.acceleration.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.angularRate.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.temperature = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.pressure = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNGPS", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsTowSec = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsWeek = (unsigned short) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsFix = (unsigned char) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.numSats = (unsigned char) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsPosU.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsPosU.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsPosU.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsVelU = (float) atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.timeAccSec = (float) atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNGPE", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsTowSec = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsWeek = (unsigned short) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsFix = (unsigned char) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.numSats = (unsigned char) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.posEcef.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.posEcef.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.posEcef.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velEcef.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velEcef.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velEcef.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsPosU.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsPosU.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsPosU.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsVelU = (float) atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.timeAccSec = (float) atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNINS", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsTowSec = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.gpsWeek = (unsigned short) atoi(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.insStatus = (unsigned short) strtol(result, NULL, 16);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.yaw = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.pitch = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.ypr.roll = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.latitudeLongitudeAltitude.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velocity.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.attitudeUncertainty = (float) atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.posU = (float) atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.velU = (float) atof(result);
		result = strtok(0, delims);
		if (result == NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}
	else if (strncmp(buffer, "$VNDTV", 5) == 0) {

		result = strtok(buffer, delims);	/* Returns async header. */
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.deltaTime = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.deltaTheta.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.deltaTheta.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.deltaTheta.c2 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.deltaVelocity.c0 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.deltaVelocity.c1 = atof(result);
		result = strtok(0, delims);
		if (result == NULL)
			return;
		data.deltaVelocity.c2 = atof(result);
		result = strtok(0, delims);
		if (result != NULL) {
      memcpy(sync_count, result+1, 10);
      data.syncInCnt = atoi(sync_count);
    }
	}

	else {
		/* We must not have had an async data packet. */
		return;
	}

	/* We had an async data packet and need to move it to vndevice->lastestAsyncData. */
	vncp_criticalSection_enter(&vndevice->critSecForLatestAsyncDataAccess);
	memcpy(&vndevice->lastestAsyncData, &data, sizeof(VnDeviceCompositeData));
	vncp_criticalSection_leave(&vndevice->critSecForLatestAsyncDataAccess);

	if (vndevice->asyncDataListener != NULL)
		vndevice->asyncDataListener(vndevice->deviceMask, &vndevice->lastestAsyncData);
}

void vndevice_disableResponseChecking_threadSafe(
	VnDevice* vndevice)
{
	vncp_criticalSection_enter(&vndevice->critSecForResponseMatchAccess);

	vndevice->checkForResponse = false;
	vndevice->cmdResponseMatchBuffer[0] = 0;

	vncp_criticalSection_leave(&vndevice->critSecForResponseMatchAccess);
}

bool vndevice_shouldCheckForResponse_threadSafe(
	VnDevice* vndevice,
	char* responseMatchBuffer)
{
	bool shouldCheckResponse;

	vncp_criticalSection_enter(&vndevice->critSecForResponseMatchAccess);

	shouldCheckResponse = vndevice->checkForResponse;

	if (shouldCheckResponse)
		strcpy(responseMatchBuffer, vndevice->cmdResponseMatchBuffer);

	vncp_criticalSection_leave(&vndevice->critSecForResponseMatchAccess);

	return shouldCheckResponse;
}

VN_ERROR_CODE vndevice_writeSettings(
	VnDevice* vndevice,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNWNV";

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSend, "VNWNV");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vndevice_restoreFactorySettings(
	VnDevice* vndevice,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNRFS";

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSend, "VNRFS");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vndevice_reset(
	VnDevice* vndevice)
{
	return vndevice_writeOutCommand(vndevice, "$VNRST");
}

VN_ERROR_CODE vndevice_getBinaryOutputConfiguration(
	VnDevice* vndevice,
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
	char* result;
	int errorCode;
	char delims[] = ",*";
	int curBufLoc = 0;
	const char* responseMatch = "VNRRG,";
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNRRG,%u",
		binaryOutputRegisterId + 74);
	cmdToSendBuilder[curBufLoc] = '\0';

	errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);												/* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*asyncMode = (uint16_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*rateDivisor = (uint16_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*selectedOutputGroups = (uint16_t) atoi(result);

	*outputGroup1Selections = 0;
	*outputGroup2Selections = 0;
	*outputGroup3Selections = 0;
	*outputGroup4Selections = 0;
	*outputGroup5Selections = 0;
	*outputGroup6Selections = 0;

	if (*selectedOutputGroups & 0x01) {
		result = strtok(0, delims);
		if (result == NULL)
			return VNERR_INVALID_VALUE;
		*outputGroup1Selections = (uint16_t) atoi(result);
	}

	if (*selectedOutputGroups & 0x02) {
		result = strtok(0, delims);
		if (result == NULL)
			return VNERR_INVALID_VALUE;
		*outputGroup2Selections = (uint16_t) atoi(result);
	}

	if (*selectedOutputGroups & 0x04) {
		result = strtok(0, delims);
		if (result == NULL)
			return VNERR_INVALID_VALUE;
		*outputGroup3Selections = (uint16_t) atoi(result);
	}

	if (*selectedOutputGroups & 0x08) {
		result = strtok(0, delims);
		if (result == NULL)
			return VNERR_INVALID_VALUE;
		*outputGroup4Selections = (uint16_t) atoi(result);
	}

	if (*selectedOutputGroups & 0x10) {
		result = strtok(0, delims);
		if (result == NULL)
			return VNERR_INVALID_VALUE;
		*outputGroup5Selections = (uint16_t) atoi(result);
	}

	if (*selectedOutputGroups & 0x20) {
		result = strtok(0, delims);
		if (result == NULL)
			return VNERR_INVALID_VALUE;
		*outputGroup5Selections = (uint16_t) atoi(result);
	}

	if (*selectedOutputGroups & 0x40) {
		result = strtok(0, delims);
		if (result == NULL)
			return VNERR_INVALID_VALUE;
		*outputGroup6Selections = (uint16_t) atoi(result);
	}

	return VNERR_NO_ERROR;
}

DLL_EXPORT VN_ERROR_CODE vndevice_setBinaryOutputConfiguration(
	VnDevice* vndevice,
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
	int errorCode;
	uint16_t outputGroups = 0;
	int curBufLoc = 0;
	const char* responseMatch = "VNWRG,";
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,%u",
		binaryOutputRegisterId + 74);
	cmdToSendBuilder[curBufLoc] = '\0';

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%u", asyncMode);

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%u", rateDivisor);

	/* Determine which output groups are selected. */
	if (outputGroup1Selections != 0) {
		outputGroups |= 0x01;
	}
	if (outputGroup2Selections != 0) {
		outputGroups |= 0x02;
	}
	if (outputGroup3Selections != 0) {
		outputGroups |= 0x04;
	}
	if (outputGroup4Selections != 0) {
		outputGroups |= 0x08;
	}
	if (outputGroup5Selections != 0) {
		outputGroups |= 0x10;
	}
	if (outputGroup6Selections != 0) {
		outputGroups |= 0x20;
	}

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%X", outputGroups);

	/* Now output the available group selections. */
	if (outputGroup1Selections != 0) {
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%X", outputGroup1Selections);
	}
	if (outputGroup2Selections != 0) {
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%X", outputGroup2Selections);
	}
	if (outputGroup3Selections != 0) {
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%X", outputGroup3Selections);
	}
	if (outputGroup4Selections != 0) {
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%X", outputGroup4Selections);
	}
	if (outputGroup5Selections != 0) {
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%X", outputGroup5Selections);
	}
	if (outputGroup6Selections != 0) {
		curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",%X", outputGroup6Selections);
	}

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, responseMatch);
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	return VNERR_NO_ERROR;
}


VN_ERROR_CODE vndevice_getSynchronizationControl(
	VnDevice* vndevice,
	uint8_t* syncInMode,
	uint8_t* syncInEdge,
	uint16_t* syncInSkipFactor,
	uint8_t* syncOutMode,
	uint8_t* syncOutPolarity,
	uint16_t* syncOutSkipFactor,
	uint32_t* syncOutPulseWidth)
{
	const char* cmdToSend = "$VNRRG,32";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInEdge = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInSkipFactor = (unsigned short) atoi(result);
	result = strtok(0, delims);	/* Placeholder for the reserved0 result. */
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutPolarity = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutSkipFactor = (unsigned short) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutPulseWidth = (unsigned int) atoi(result);
	result = strtok(0, delims); /* Placeholder for the reserved1 result. */
	if (result == NULL)
		return VNERR_INVALID_VALUE;

	return VNERR_NO_ERROR;
}


VN_ERROR_CODE vndevice_setSynchronizationControl(
	VnDevice* vndevice,
	uint8_t syncInMode,
	uint8_t syncInEdge,
	uint16_t syncInSkipFactor,
	uint8_t syncOutMode,
	uint8_t syncOutPolarity,
	uint16_t syncOutSkipFactor,
	uint32_t syncOutPulseWidth,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,32,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInEdge);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInSkipFactor);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", 0);	/* Placeholder for reserved0 field. */
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutPolarity);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutSkipFactor);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutPulseWidth);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", 0);	/* Placeholder for reserved1 field. */

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getSynchronizationStatus(
	VnDevice* vndevice,
	uint32_t* syncInCount,
	uint32_t* syncInTime,
	uint32_t* syncOutCount)
{
	const char* cmdToSend = "$VNRRG,33";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInCount = (unsigned int) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncInTime = (unsigned int) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*syncOutCount = (unsigned int) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setSynchronizationStatus(
	VnDevice* vndevice,
	uint32_t syncInCount,
	uint32_t syncInTime,
	uint32_t syncOutCount,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,33,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInCount);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncInTime);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", syncOutCount);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getImuMeasurements(
	VnDevice* vndevice,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate,
	float* temperature,
	float* pressure)
{
	const char* cmdToSend = "$VNRRG,54";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
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
	*temperature = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*pressure = (float) atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getDeltaThetaAndDeltaVelocity(
	VnDevice* vndevice,
	float* deltaTime,
	VnVector3* deltaTheta,
	VnVector3* deltaVelocity)
{
	const char* cmdToSend = "$VNRRG,80";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*deltaTime = (float) atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	deltaTheta->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	deltaTheta->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	deltaTheta->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	deltaVelocity->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	deltaVelocity->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	deltaVelocity->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getMagneticCompensation(
	VnDevice* vndevice,
	VnMatrix3x3* c,
	VnVector3* b)
{
	const char* cmdToSend = "$VNRRG,23";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setMagneticCompensation(
	VnDevice* vndevice,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,23,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f", c.c00, c.c01, c.c02, c.c10, c.c11, c.c12, c.c20, c.c21, c.c22);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", b.c0, b.c1, b.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getAccelerationCompensation(
	VnDevice* vndevice,
	VnMatrix3x3* c,
	VnVector3* b)
{
	const char* cmdToSend = "$VNRRG,25";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setAccelerationCompensation(
	VnDevice* vndevice,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,25,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f", c.c00, c.c01, c.c02, c.c10, c.c11, c.c12, c.c20, c.c21, c.c22);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", b.c0, b.c1, b.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getGyroCompensation(
	VnDevice* vndevice,
	VnMatrix3x3* c,
	VnVector3* b)
{
	const char* cmdToSend = "$VNRRG,84";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setGyroCompensation(
	VnDevice* vndevice,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,84,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f", c.c00, c.c01, c.c02, c.c10, c.c11, c.c12, c.c20, c.c21, c.c22);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", b.c0, b.c1, b.c2);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getReferenceFrameRotation(
	VnDevice* vndevice,
	VnMatrix3x3* c)
{
	const char* cmdToSend = "$VNRRG,26";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setReferenceFrameRotation(
	VnDevice* vndevice,
	VnMatrix3x3 c,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,26,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f,%+09.6f", c.c00, c.c01, c.c02, c.c10, c.c11, c.c12, c.c20, c.c21, c.c22);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

DLL_EXPORT VN_ERROR_CODE vndevice_getImuFilteringConfiguration(
	VnDevice* vndevice,
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
	const char* cmdToSend = "$VNRRG,85";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magWindowSize = (uint16_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelWindowSize = (uint16_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gyroWindowSize = (uint16_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*tempWindowSize = (uint16_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*presWindowSize = (uint16_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*magFilterMode = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelFilterMode = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gyroFilterMode = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*tempFilterMode = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*presFilterMode = (uint8_t) atoi(result);

	return VNERR_NO_ERROR;
}

DLL_EXPORT VN_ERROR_CODE vndevice_setImuFilteringConfiguration(
	VnDevice* vndevice,
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
	int errorCode;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	sprintf(cmdToSendBuilder, "$VNWRG,85,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
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

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getDeltaThetaAndDeltaVelocityConfiguration(
	VnDevice* vndevice,
	uint8_t* integrationFrame,
	uint8_t* gyroCompensation,
	uint8_t* accelCompensation)
{
	const char* cmdToSend = "$VNRRG,82";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*integrationFrame = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*gyroCompensation = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*accelCompensation = (uint8_t) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setDeltaThetaAndDeltaVelocityConfiguration(
	VnDevice* vndevice,
	uint8_t integrationFrame,
	uint8_t gyroCompensation,
	uint8_t accelCompensation,
	bool waitForResponse)
{
	int errorCode;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	sprintf(cmdToSendBuilder, "$VNWRG,82,%d,%d,%d,0,0",
		integrationFrame,
		gyroCompensation,
		accelCompensation);

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getUserTag(
	VnDevice* vndevice,
	char* userTagBuffer,
	uint32_t userTagBufferLength)
{
	const char* cmdToSend = "$VNRRG,0";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	/* Verify the provided buffer is large enough. */
	if (userTagBufferLength < 21)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 20)
		return VNERR_UNKNOWN_ERROR;
	strcpy(userTagBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setUserTag(
	VnDevice* vndevice,
	char* userTagData,
	uint32_t userTagDataLength,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	/* Verify the provided data is small enough. */
	if (userTagDataLength > 20)
		return VNERR_UNKNOWN_ERROR;

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,0,");

	memcpy(cmdToSendBuilder + curBufLoc, userTagData, userTagDataLength);
	curBufLoc += userTagDataLength;

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getModelNumber(
	VnDevice* vndevice,
	char* modelBuffer,
	uint32_t modelBufferLength)
{
	const char* cmdToSend = "$VNRRG,1";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	/* Verify the provided buffer is large enough. */
	if (modelBufferLength < 25)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 24)
		return VNERR_UNKNOWN_ERROR;
	strcpy(modelBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getHardwareRevision(
	VnDevice* vndevice,
	int32_t* hardwareRevision)
{
	const char* cmdToSend = "$VNRRG,2";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*hardwareRevision = atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getSerialNumber(
	VnDevice* vndevice,
	char* serialNumberBuffer,
	uint32_t serialNumberBufferLength)
{
	const char* cmdToSend = "$VNRRG,3";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	/* Verify the provided buffer is large enough. */
	if (serialNumberBufferLength < 13)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 12)
		return VNERR_UNKNOWN_ERROR;
	strcpy(serialNumberBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getFirmwareVersion(
	VnDevice* vndevice,
	char* firmwareVersionBuffer,
	uint32_t firmwareVersionBufferLength)
{
	const char* cmdToSend = "$VNRRG,4";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	/* Verify the provided buffer is large enough. */
	if (firmwareVersionBufferLength < 16)
		return VNERR_UNKNOWN_ERROR;

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (strlen(result) > 15)
		return VNERR_UNKNOWN_ERROR;
	strcpy(firmwareVersionBuffer, result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getSerialBaudRate(
	VnDevice* vndevice,
	uint32_t* serialBaudrate)
{
	const char* cmdToSend = "$VNRRG,5";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialBaudrate = (uint32_t) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setSerialBaudRate(
	VnDevice* vndevice,
	uint32_t serialBaudrate,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,5,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialBaudrate);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getAsynchronousDataOutputType(
	VnDevice* vndevice,
	uint32_t* asyncDataOutputType)
{
	const char* cmdToSend = "$VNRRG,6";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*asyncDataOutputType = (uint32_t) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setAsynchronousDataOutputType(
	VnDevice* vndevice,
	uint32_t asyncDataOutputType,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,6,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", asyncDataOutputType);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getAsynchronousDataOutputFrequency(
	VnDevice* vndevice,
	uint32_t* asyncDataOutputFrequency)
{
	const char* cmdToSend = "$VNRRG,7";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*asyncDataOutputFrequency = (uint32_t) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setAsynchronousDataOutputFrequency(
	VnDevice* vndevice,
	uint32_t asyncDataOutputFrequency,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,7,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", asyncDataOutputFrequency);

		cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getQuaternion(VnDevice* vndevice, VnQuaternion* attitude)
{
	const char* cmdToSend = "$VNRRG,9";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getQuaternionMagneticAccelerationAngularRate(
	VnDevice* vndevice,
	VnQuaternion* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,15";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vndevice_getMagnetic(
	VnDevice* vndevice,
	VnVector3* magnetic)
{
	const char* cmdToSend = "$VNRRG,17";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
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

VN_ERROR_CODE vndevice_getAcceleration(
	VnDevice* vndevice,
	VnVector3* acceleration)
{
	const char* cmdToSend = "$VNRRG,18";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
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

VN_ERROR_CODE vndevice_getAngularRate(
	VnDevice* vndevice,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,19";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
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

VN_ERROR_CODE vndevice_getMagneticAccelerationAngularRate(
	VnDevice* vndevice,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,20";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
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

VN_ERROR_CODE vndevice_getYawPitchRoll(
	VnDevice* vndevice,
	VnYpr* attitude)
{
	const char* cmdToSend = "$VNRRG,8";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getYawPitchRollMagneticAccelerationAngularRate(
	VnDevice* vndevice,
	VnYpr* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,27";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vndevice_getYawPitchRollTrueBodyAccelerationAngularRate(
	VnDevice* vndevice,
	VnYpr* attitude,
	VnVector3* bodyAcceleration,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,239";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
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
	bodyAcceleration->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	bodyAcceleration->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	bodyAcceleration->c2 = atof(result);
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

VN_ERROR_CODE vndevice_getYawPitchRollTrueInertialAccelerationAngularRate(
	VnDevice* vndevice,
	VnYpr* attitude,
	VnVector3* inertialAcceleration,
	VnVector3* angularRate)
{
	const char* cmdToSend = "$VNRRG,240";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
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

VN_ERROR_CODE vndevice_getVpeControl(
	VnDevice* vndevice,
	uint8_t* enable,
	uint8_t* headingMode,
	uint8_t* filteringMode,
	uint8_t* tuningMode)
{
	const char* cmdToSend = "$VNRRG,35";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*enable = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*headingMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*filteringMode = (unsigned char) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*tuningMode = (unsigned char) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setVpeControl(
	VnDevice* vndevice,
	uint8_t enable,
	uint8_t headingMode,
	uint8_t filteringMode,
	uint8_t tuningMode,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,35,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", enable);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", headingMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", filteringMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", tuningMode);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getVpeMagnetometerBasicTuning(
	VnDevice* vndevice,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning,
	VnVector3* adaptiveFiltering)
{
	const char* cmdToSend = "$VNRRG,36";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
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
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setVpeMagnetometerBasicTuning(
	VnDevice* vndevice,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,36,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveFiltering.c0, adaptiveFiltering.c1, adaptiveFiltering.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getVpeAccelerometerBasicTuning(
	VnDevice* vndevice,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning,
	VnVector3* adaptiveFiltering)
{
	const char* cmdToSend = "$VNRRG,38";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
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
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	adaptiveFiltering->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setVpeAccelerometerBasicTuning(
	VnDevice* vndevice,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,38,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", baseTuning.c0, baseTuning.c1, baseTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveTuning.c0, adaptiveTuning.c1, adaptiveTuning.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", adaptiveFiltering.c0, adaptiveFiltering.c1, adaptiveFiltering.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getMagnetometerCalibrationControl(
	VnDevice* vndevice,
	uint8_t* hsiMode,
	uint8_t* hsiOutput,
	uint8_t* convergeRate)
{
	const char* cmdToSend = "$VNRRG,44";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*hsiMode = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*hsiOutput = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*convergeRate = (uint8_t) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setMagnetometerCalibrationControl(
	VnDevice* vndevice,
	uint8_t hsiMode,
	uint8_t hsiOutput,
	uint8_t convergeRate,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,44,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", hsiMode);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", hsiOutput);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", convergeRate);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getCalculatedMagnetometerCalibration(
	VnDevice* vndevice,
	VnMatrix3x3* c,
	VnVector3* b)
{
	const char* cmdToSend = "$VNRRG,47";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c00 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c01 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c02 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c10 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c11 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c12 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c20 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c21 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	c->c22 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	b->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_getMagneticGravityReferenceVectors(
	VnDevice* vndevice,
	VnVector3* magneticReference,
	VnVector3* gravityReference)
{
	const char* cmdToSend = "$VNRRG,21";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticReference->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticReference->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	magneticReference->c2 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gravityReference->c0 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gravityReference->c1 = atof(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	gravityReference->c2 = atof(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setMagneticGravityReferenceVectors(
	VnDevice* vndevice,
	VnVector3 magneticReference,
	VnVector3 gravityReference,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,21,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", magneticReference.c0, magneticReference.c1, magneticReference.c2);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%+09.6f,%+09.6f,%+09.6f", gravityReference.c0, gravityReference.c1, gravityReference.c2);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getCommunicationProtocolControl(
	VnDevice* vndevice,
	uint8_t* serialCount,
	uint8_t* serialStatus,
	uint8_t* spiCount,
	uint8_t* spiStatus,
	uint8_t* serialChecksum,
	uint8_t* spiChecksum,
	uint8_t* errorMode)
{
	const char* cmdToSend = "$VNRRG,30";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialCount = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialStatus = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*spiCount = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*spiStatus = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*serialChecksum = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*spiChecksum = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*errorMode = (uint8_t) atoi(result);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setCommunicationProtocolControl(
	VnDevice* vndevice,
	uint8_t serialCount,
	uint8_t serialStatus,
	uint8_t spiCount,
	uint8_t spiStatus,
	uint8_t serialChecksum,
	uint8_t spiChecksum,
	uint8_t errorMode,
	bool waitForResponse)
{
	int errorCode;
	int curBufLoc = 0;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	curBufLoc = sprintf(cmdToSendBuilder, "$VNWRG,30,");

	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialCount);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialStatus);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", spiCount);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", spiStatus);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", serialChecksum);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", spiChecksum);
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, ",");
	curBufLoc += sprintf(cmdToSendBuilder + curBufLoc, "%d", errorMode);

	cmdToSendBuilder[curBufLoc] = '\0';

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_getReferenceVectorConfiguration(
	VnDevice* vndevice,
	uint8_t* useMagModel,
	uint8_t* useGravityModel,
	uint32_t* recalcThreshold,
	float* year,
	VnVector3* lla)
{
	const char* cmdToSend = "$VNRRG,83";
	char delims[] = ",*";
	char* result;
	int errorCode;
	const char* responseMatch = "VNRRG,";

	errorCode = vndevice_transaction(vndevice, cmdToSend, responseMatch);

	if (errorCode != VNERR_NO_ERROR)
		return errorCode;

	result = strtok(vndevice->cmdResponseBuffer, delims);  /* Returns VNRRG */
	result = strtok(0, delims);                            /* Returns register ID */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*useMagModel = (uint8_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*useGravityModel = (uint8_t) atoi(result);
	result = strtok(0, delims);	/* Placeholder for Resv1 field. */
	result = strtok(0, delims); /* Placeholder for Resv2 field. */
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*recalcThreshold = (uint32_t) atoi(result);
	result = strtok(0, delims);
	if (result == NULL)
		return VNERR_INVALID_VALUE;
	*year = (float) atof(result);
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

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vndevice_setReferenceVectorConfiguration(
	VnDevice* vndevice,
	uint8_t useMagModel,
	uint8_t useGravityModel,
	uint32_t recalcThreshold,
	float year,
	VnVector3 lla,
	bool waitForResponse)
{
	int errorCode;
	char cmdToSendBuilder[VN_MAX_COMMAND_SIZE];

	sprintf(cmdToSendBuilder, "$VNWRG,83,%d,%d,%d,0,0,%+09.6f,%+09.6f,%+09.6f,%+09.6f",
		useMagModel,
		useGravityModel,
		recalcThreshold,
		year,
		lla.c0,
		lla.c1,
		lla.c2);

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSendBuilder, "VNWRG,");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSendBuilder);

	return errorCode;
}

VN_ERROR_CODE vndevice_pauseAsyncOutputs(
	VnDevice* vndevice,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNASY,0";

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSend, "VNASY");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSend);

	return errorCode;
}

VN_ERROR_CODE vndevice_resumeAsyncOutputs(
	VnDevice* vndevice,
	bool waitForResponse)
{
	int errorCode;
	const char* cmdToSend = "$VNASY,1";

	if (waitForResponse)
		errorCode = vndevice_transaction(vndevice, cmdToSend, "VNASY");
	else
		errorCode = vndevice_writeOutCommand(vndevice, cmdToSend);

	return errorCode;
}

/** \endcond */
