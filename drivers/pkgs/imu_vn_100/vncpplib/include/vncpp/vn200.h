/**
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
 * This header file provides access to devices based on VectorNav's VN-200
 * family of orientation sensors.
 */
#ifndef _VN200_H_
#define _VN200_H_

#if defined(_MSC_VER) && _MSC_VER <= 1500
	/* Visual Studio 2008 and earlier do not include the stdint.h header file. */
	#include "vnint.h"
#else
	#include <stdint.h>
#endif

#include "vndevice.h"
#include "vncp_services.h"
#include "vn_kinematics.h"
#include "vn_linearAlgebra.h"

#if defined(EXPORT_TO_DLL)
	#define DLL_EXPORT __declspec(dllexport)
#else
	/** Don't compile the library with support for DLL function export. */
	#define DLL_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Holds connection information for accessing a VN-200 device.
 */
typedef struct {
	char*				portName;		/**< The name of the serial port. */
	int					baudRate;		/**< The baudrate of the serial port. */
	bool				isConnected;	/**< Inidicates if the serial port is open. */
	VnDevice	vndevice;		/**< Pointer to internally used data. */
} Vn200;

/**
 * \brief Connects to a VectorNav VN-200 device.
 *
 * \param[out]	newVn200	An uninitialized Vn200 control object should be passed in.
 * \param[in]	portName	The name of the COM port to connect to.
 * \param[in]	baudrate	The baudrate to connect at.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_connect(
	Vn200* newVn200,
	const char* portName,
	int baudrate);

/**
 * \brief Disconnects from the VN-200 device and disposes of any internal resources.
 *
 * \param vn200 Pointer to the Vn200 control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_disconnect(
	Vn200* vn200);

/**
 * \brief Allows registering a function which will be called whenever a new
 * asynchronous data packet is received from the VN-200 module.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] listener The function pointer to be called when asynchronous data
 *     is received.
 * 
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_registerAsyncDataReceivedListener(
	Vn200* vn200,
	VnDeviceNewAsyncDataReceivedListener listener);

/**
 * \brief Unregisters an already registered function for recieving notifications
 * of when new asynchronous data is received.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] listener The function pointer to unregister.
 * 
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_unregisterAsyncDataReceivedListener(
	Vn200* vn200,
	VnDeviceNewAsyncDataReceivedListener listener);

/**
 * \brief Allows registering a function which will be called whenever an error
 * code is received from the VN-200 module.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] listener The function pointer to be called when an error code
 *     is received.
 * 
 * \return VectorNav error code.
 */
VN_ERROR_CODE vn200_registerErrorCodeReceivedListener(
	Vn200* vn200,
	VnDeviceErrorCodeReceivedListener listener);

/**
 * \brief Unregisters an already registered function for recieving notifications
 * of when error codes from the VN-200 module are received.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] listener The function pointer to unregister.
 * 
 * \return VectorNav error code.
 */
VN_ERROR_CODE vn200_unregisterErrorCodeReceivedListener(
	Vn200* vn200,
	VnDeviceErrorCodeReceivedListener listener);

/**
 * \brief Checks if we are able to send and receive communication with the VN-200 sensor.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 *
 * \return VN_TRUE if the library was able to send and receive a valid response from the VN-200 sensor; otherwise VN_FALSE.
 */
DLL_EXPORT bool vn200_verifyConnectivity(
	Vn200* vn200);

/**
 * \brief Commands the VN-200 to copy the current filter bias estimates into
 *        register 74.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setFilterBias(
	Vn200* vn200,
	bool waitForResponse);

/**
 * \brief Gets the current configuration of the requested binary output register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in]	binaryOutputRegisterId The ID of the binary output register to query for its configuration. Must be 1 - 3.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	selectedOutputGroups The selected output groups.
 * \param[out]	outputGroup1Selections The configured output data types for output group 1.
 * \param[out]	outputGroup2Selections The configured output data types for output group 2.
 * \param[out]	outputGroup3Selections The configured output data types for output group 3.
 * \param[out]	outputGroup4Selections The configured output data types for output group 4.
 * \param[out]	outputGroup5Selections The configured output data types for output group 5.
 * \param[out]	outputGroup6Selections The configured output data types for output group 6.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getBinaryOutputConfiguration(
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
	uint16_t* outputGroup6Selections);

/**
 * \brief Gets the current configuration of the binary output register 1.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	selectedOutputGroups The selected output groups.
 * \param[out]	outputGroup1Selections The configured output data types for output group 1.
 * \param[out]	outputGroup2Selections The configured output data types for output group 2.
 * \param[out]	outputGroup3Selections The configured output data types for output group 3.
 * \param[out]	outputGroup4Selections The configured output data types for output group 4.
 * \param[out]	outputGroup5Selections The configured output data types for output group 5.
 * \param[out]	outputGroup6Selections The configured output data types for output group 6.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getBinaryOutput1Configuration(
	Vn200* vn200,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup2Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup4Selections,
	uint16_t* outputGroup5Selections,
	uint16_t* outputGroup6Selections);

/**
 * \brief Gets the current configuration of the binary output register 2.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	selectedOutputGroups The selected output groups.
 * \param[out]	outputGroup1Selections The configured output data types for output group 1.
 * \param[out]	outputGroup2Selections The configured output data types for output group 2.
 * \param[out]	outputGroup3Selections The configured output data types for output group 3.
 * \param[out]	outputGroup4Selections The configured output data types for output group 4.
 * \param[out]	outputGroup5Selections The configured output data types for output group 5.
 * \param[out]	outputGroup6Selections The configured output data types for output group 6.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getBinaryOutput2Configuration(
	Vn200* vn200,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup2Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup4Selections,
	uint16_t* outputGroup5Selections,
	uint16_t* outputGroup6Selections);

/**
 * \brief Gets the current configuration of the binary output register 3.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	selectedOutputGroups The selected output groups.
 * \param[out]	outputGroup1Selections The configured output data types for output group 1.
 * \param[out]	outputGroup2Selections The configured output data types for output group 2.
 * \param[out]	outputGroup3Selections The configured output data types for output group 3.
 * \param[out]	outputGroup4Selections The configured output data types for output group 4.
 * \param[out]	outputGroup5Selections The configured output data types for output group 5.
 * \param[out]	outputGroup6Selections The configured output data types for output group 6.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getBinaryOutput3Configuration(
	Vn200* vn200,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup2Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup4Selections,
	uint16_t* outputGroup5Selections,
	uint16_t* outputGroup6Selections);

/**
 * \brief Sets the configuration of the requested binary output register. Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in]	binaryOutputRegisterId The ID of the binary output register to set its configuration. Must be 1 - 3.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	outputGroup1Selections The output data types for output group 1.
 * \param[out]	outputGroup2Selections The output data types for output group 2.
 * \param[out]	outputGroup3Selections The output data types for output group 3.
 * \param[out]	outputGroup4Selections The output data types for output group 4.
 * \param[out]	outputGroup5Selections The output data types for output group 5.
 * \param[out]	outputGroup6Selections The output data types for output group 6.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setBinaryOutputConfiguration(
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
	bool waitForResponse);

/**
 * \brief Sets the current configuration of the binary output register 1.  Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	outputGroup1Selections The output data types for output group 1.
 * \param[out]	outputGroup2Selections The output data types for output group 2.
 * \param[out]	outputGroup3Selections The output data types for output group 3.
 * \param[out]	outputGroup4Selections The output data types for output group 4.
 * \param[out]	outputGroup5Selections The output data types for output group 5.
 * \param[out]	outputGroup6Selections The output data types for output group 6.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setBinaryOutput1Configuration(
	Vn200* vn200,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup2Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup4Selections,
	uint16_t outputGroup5Selections,
	uint16_t outputGroup6Selections,
	bool waitForResponse);

/**
 * \brief Sets the current configuration of the binary output register 2.  Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	outputGroup1Selections The output data types for output group 1.
 * \param[out]	outputGroup2Selections The output data types for output group 2.
 * \param[out]	outputGroup3Selections The output data types for output group 3.
 * \param[out]	outputGroup4Selections The output data types for output group 4.
 * \param[out]	outputGroup5Selections The output data types for output group 5.
 * \param[out]	outputGroup6Selections The output data types for output group 6.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setBinaryOutput2Configuration(
	Vn200* vn200,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup2Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup4Selections,
	uint16_t outputGroup5Selections,
	uint16_t outputGroup6Selections,
	bool waitForResponse);

/**
 * \brief Sets the current configuration of the binary output register 3.  Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	outputGroup1Selections The output data types for output group 1.
 * \param[out]	outputGroup2Selections The output data types for output group 2.
 * \param[out]	outputGroup3Selections The output data types for output group 3.
 * \param[out]	outputGroup4Selections The output data types for output group 4.
 * \param[out]	outputGroup5Selections The output data types for output group 5.
 * \param[out]	outputGroup6Selections The output data types for output group 6.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setBinaryOutput3Configuration(
	Vn200* vn200,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup2Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup4Selections,
	uint16_t outputGroup5Selections,
	uint16_t outputGroup6Selections,
	bool waitForResponse);

/**
 * \brief Gets the values in the GPS Configuration register.
 *
 * \note This function is only suitable for VN-200 devices with firmware versions
 *       earlier than v1.0.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] mode The mode value of the sensor.
 * \param[out] nmeaSerial1 The NMEA_Serial1 value of the sensor.
 * \param[out] nmeaSerial2 The NMEA_Serial2 value of the sensor.
 * \param[out] nmeaRate The NMEA_Rate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGpsConfiguration_preFirmwareVersion1d0(
	Vn200* vn200,
	unsigned char* mode,
	unsigned char* nmeaSerial1,
	unsigned char* nmeaSerial2,
	unsigned char* nmeaRate);

/**
 * \brief Sets the values of the GPS Configuration register.
 *
 * \note This function is only suitable for VN-200 devices with firmware versions
 *       earlier than v1.0.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] mode Value for the mode field.
 * \param[in] nmeaSerial1 Value for the NMEA_Serial1 field.
 * \param[in] nmeaSerial2 Value for the NMEA_Serial2 field.
 * \param[in] nmeaRate Value for the NMEA_Rate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setGpsConfiguration_preFirmwareVersion1d0(
	Vn200* vn200,
	unsigned char mode,
	unsigned char nmeaSerial1,
	unsigned char nmeaSerial2,
	unsigned char nmeaRate,
	bool waitForResponse);

/**
 * \brief Gets the values in the GPS Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] mode The mode value of the sensor.
 * \param[out] ppsSource GPS PPS mode.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGpsConfiguration(
	Vn200* vn200,
	unsigned char* mode,
	unsigned char* ppsSource);

/**
 * \brief Sets the values of the GPS Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] mode Value for the mode field.
 * \param[in] ppsSource Value for the PpsSource field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setGpsConfiguration(
	Vn200* vn200,
	unsigned char mode,
	unsigned char ppsSource,
	bool waitForResponse);


/**
 * \brief Gets the values in the GPS Antenna Offset register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] position The current sensor relative postion of GPS antenna (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGpsAntennaOffset(
	Vn200* vn200,
	VnVector3* position);

/**
 * \brief Sets the values of the GPS Antenna Offset register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] position The relative postion of GPS antenna (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setGpsAntennaOffset(
	Vn200* vn200,
	VnVector3 position,
	bool waitForResponse);

/**
 * \brief Gets the values in the GPS Solution - LLA register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] gpsTime The GPS time of week in seconds value of the sensor.
 * \param[out] gpsWeek The GPS week value of the sensor.
 * \param[out] gpsFix The GPS fix type value of the sensor.
 * \param[out] numberOfSatellites The number of GPS satellites used in solution value of the sensor.
 * \param[out] lattitudeLongitudeAltitude The current sensor latitude, longitude, and altitude values.
 * \param[out] nedVelocity The current sensor velocity measurements (X,Y,Z) in north, east, down directions values.
 * \param[out] positionAccuracy The current sensor position accuracy (X,Y,Z) values.
 * \param[out] speedAccuracy The speed accuracy estimate value of the sensor.
 * \param[out] timeAccuracy The time accuracy estimate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGpsSolutionLla(
	Vn200* vn200,
	double* gpsTime,
	unsigned short* gpsWeek,
	unsigned char* gpsFix,
	unsigned char* numberOfSatellites,
	VnVector3* lattitudeLongitudeAltitude,
	VnVector3* nedVelocity,
	VnVector3* positionAccuracy,
	float* speedAccuracy,
	float* timeAccuracy);

/**
 * \brief Gets the values in the GPS Solution - ECEF register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] gpsTime The GPS time of week in seconds value of the sensor.
 * \param[out] gpsWeek The GPS week value of the sensor.
 * \param[out] gpsFix The GPS fix type value of the sensor.
 * \param[out] numberOfSatellites The number of GPS satellites used in solution value of the sensor.
 * \param[out] position The current sensor's ECEF values.
 * \param[out] velocity The current sensor velocity measurements (X,Y,Z) in ECEF.
 * \param[out] positionAccuracy The current sensor position accuracy (X,Y,Z) in ECEF.
 * \param[out] speedAccuracy The speed accuracy estimate value of the sensor.
 * \param[out] timeAccuracy The time accuracy estimate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGpsSolutionEcef(
	Vn200* vn200,
	double* gpsTime,
	unsigned short* gpsWeek,
	unsigned char* gpsFix,
	unsigned char* numberOfSatellites,
	VnVector3* position,
	VnVector3* velocity,
	VnVector3* positionAccuracy,
	float* speedAccuracy,
	float* timeAccuracy);

/**
 * \brief Gets the values in the INS Solution - LLA register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] gpsTime The GPS time of week in seconds value of the sensor.
 * \param[out] gpsWeek The GPS week value of the sensor.
 * \param[out] status The status flags for the INS filter value of the sensor.
 * \param[out] ypr The current sensor heading, pitch, and roll values.
 * \param[out] lattitudeLongitudeAltitude The current sensor latitude, longitude, and altitude values.
 * \param[out] nedVelocity The current sensor velocity measurements (X,Y,Z) in north, east, down directions values.
 * \param[out] attitudeUncertainty The attitude uncertainty value of the sensor.
 * \param[out] positionUncertainty The position uncertainty value of the sensor.
 * \param[out] velocityUncertainty The velocity uncertainty value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getInsSolutionLla(
	Vn200* vn200,
	double* gpsTime,
	unsigned short* gpsWeek,
	unsigned short* status,
	VnVector3* ypr,
	VnVector3* lattitudeLongitudeAltitude,
	VnVector3* nedVelocity,
	float* attitudeUncertainty,
	float* positionUncertainty,
	float* velocityUncertainty);

/**
 * \brief Gets the values in the INS Solution - ECEF register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] gpsTime The GPS time of week in seconds value of the sensor.
 * \param[out] gpsWeek The GPS week value of the sensor.
 * \param[out] status The status flags for the INS filter value of the sensor.
 * \param[out] ypr The current sensor heading, pitch, and roll values.
 * \param[out] position The current sensor position in the ECEF frame.
 * \param[out] velocity The current sensor velocity.
 * \param[out] attitudeUncertainty The attitude uncertainty value of the sensor.
 * \param[out] positionUncertainty The position uncertainty value of the sensor.
 * \param[out] velocityUncertainty The velocity uncertainty value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getInsSolutionEcef(
	Vn200* vn200,
	double* gpsTime,
	uint16_t* gpsWeek,
	uint16_t* status,
	VnVector3* ypr,
	VnVector3* position,
	VnVector3* velocity,
	float* attitudeUncertainty,
	float* positionUncertainty,
	float* velocityUncertainty);

/**
 * \brief Gets the values in the INS State - LLA register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] ypr The current sensor heading, pitch, and roll values.
 * \param[out] lla The estimated position in latitude, longitude, and altitude.
 * \param[out] velocity The current sensor velocity.
 * \param[out] accel The estimated acceleration in body frame.
 * \param[out] angularRate The estimate angular rate in body frame.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getInsStateLla(
	Vn200* vn200,
	VnVector3* ypr,
	VnVector3* lla,
	VnVector3* velocity,
	VnVector3* accel,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the INS State - ECEF register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] ypr The current sensor heading, pitch, and roll values.
 * \param[out] position The estimated position in ECEF.
 * \param[out] velocity The current sensor velocity.
 * \param[out] accel The estimated acceleration in body frame.
 * \param[out] angularRate The estimate angular rate in body frame.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getInsStateEcef(
	Vn200* vn200,
	VnVector3* ypr,
	VnVector3* position,
	VnVector3* velocity,
	VnVector3* accel,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the INS Basic Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] scenario INS mode.
 * \param[out] ahrsAiding Enables AHRS attitude aiding.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getInsBasicConfiguration(
	Vn200* vn200,
	uint8_t* scenario,
	uint8_t* ahrsAiding);

/**
 * \brief Sets the values of the INS Basic Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] scenario INS mode.
 * \param[in] ahrsAiding Enables AHRS attitude aiding.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setInsBasicConfiguration(
	Vn200* vn200,
	uint8_t scenario,
	uint8_t ahrsAiding,
	bool waitForResponse);

/**
 * \brief Gets the values in the Startup Filter Bias Estimate register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] gyroBias Gyro bias field.
 * \param[out] accelBias Accelerometer bias field.
 * \param[out] pressureBias Pressure bias field.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getStartupFilterBiasEstimate(
	Vn200* vn200,
	VnVector3* gyroBias,
	VnVector3* accelBias,
	float* pressureBias);

/**
 * \brief Sets the values of the Startup Filter Bias Estimate register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] gyroBias Gyro bias field.
 * \param[in] accelBias Accelerometer bias field.
 * \param[in] pressureBias Pressure bias field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setStartupFilterBiasEstimate(
	Vn200* vn200,
	VnVector3 gyroBias,
	VnVector3 accelBias,
	float pressureBias,
	bool waitForResponse);

/**
 * \brief Retrieves the associated timeout value for the Vn200 object.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 *
 * \return The timeout value in milliseconds. -1 indicates that timeouts are
 * not used.
 */
DLL_EXPORT int vn200_get_timeout(
	Vn200* vn200);

/**
 * \brief Sets the timeout value for the reading values from the VectorNav sensor.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] timeout The timeout value in milliseconds. Specify -1 to not use
 * any timeouts.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_set_timeout(
	Vn200* vn200,
	int timeout);

/**
 * \brief Retrieves the most recent stored asynchronous data.
 *
 * \param[in] vn200 Pointer to the Vn200 object.
 * \param[out] curData Returned pointer current asychronous data.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getCurrentAsyncData(
	Vn200* vn200,
	VnDeviceCompositeData* curData);

/**
 * \brief Commands the VectorNav unit to write its current register setting to
 * non-volatile memory.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_writeSettings(
	Vn200* vn200,
	bool waitForResponse);

/**
 * \brief Commands the VectorNav unit to revert its settings to factory defaults.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_restoreFactorySettings(
	Vn200* vn200,
	bool waitForResponse);

/**
 * \brief Commands the VectorNav module to reset itself.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_reset(
	Vn200* vn200);

/**
 * \brief Gets the values in the User Tag register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] userTagBuffer Buffer to store the response. Must have a length of at least 21 characters.
 * \param[in] userTagBufferLength Length of the provided userTagBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getUserTag(
	Vn200* vn200,
	char* userTagBuffer,
	uint32_t userTagBufferLength);

/**
 * \brief Sets the values of the User Tag register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] userTagData Array containg the data to send. Length must be equal to or less than 20 characters.
 * \param[in] userTagDataLength Length of the data to send in the userTagData array.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setUserTag(
	Vn200* vn200,
	char* userTagData,
	uint32_t userTagDataLength,
	bool waitForResponse);

/**
 * \brief Gets the values in the Model Number register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] modelBuffer Buffer to store the response. Must have a length of at least 25 characters.
 * \param[in] modelBufferLength Length of the provided modelBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getModelNumber(
	Vn200* vn200,
	char* modelBuffer,
	uint32_t modelBufferLength);

/**
 * \brief Gets the values in the Hardware Revision register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] hardwareRevision The hardware revision value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getHardwareRevision(
	Vn200* vn200,
	int32_t* hardwareRevision);

/**
 * \brief Gets the values in the Serial Number register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] serialNumberBuffer Buffer to store the response. Must have a length of at least 13 characters.
 * \param[in] serialNumberBufferLength Length of the provided serialNumberBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getSerialNumber(
	Vn200* vn200,
	char* serialNumberBuffer,
	uint32_t serialNumberBufferLength);

/**
 * \brief Gets the value in the Firmware Version register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] firmwareVersionBuffer Buffer to store the response. Must have a length of at least 16 characters.
 * \param[in] firmwareVersionBufferLength Length of the provided firmwareVersionBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getFirmwareVersion(
	Vn200* vn200,
	char* firmwareVersionBuffer,
	uint32_t firmwareVersionBufferLength);

/**
 * \brief Gets the values in the Serial Baud Rate register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] serialBaudrate The serial baudrate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getSerialBaudRate(
	Vn200* vn200,
	uint32_t* serialBaudrate);

/**
 * \brief Sets the values of the Serial Baud Rate register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] serialBaudrate Value for the serial baudrate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setSerialBaudRate(
	Vn200* vn200,
	uint32_t serialBaudrate,
	bool waitForResponse);

/**
 * \brief Gets the values in the Asynchronous Data Output Type register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] asyncDataOutputType The asynchronous data output type value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getAsynchronousDataOutputType(
	Vn200* vn200,
	uint32_t* asyncDataOutputType);

/**
 * \brief Sets the values of the Asynchronous Data Output Type register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] asyncDataOutputType Value for the asynchronous data output type field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setAsynchronousDataOutputType(
	Vn200* vn200,
	uint32_t asyncDataOutputType,
	bool waitForResponse);

/**
 * \brief Gets the values in the Asynchronous Data Output Frequency register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] asyncDataOutputFrequency The asynchronous data output frequency value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getAsynchronousDataOutputFrequency(
	Vn200* vn200,
	uint32_t* asyncDataOutputFrequency);

/**
 * \brief Sets the values of the Asynchronous Data Output Frequency register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] asyncDataOutputFrequency Value for the asynchronous data output frequency field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setAsynchronousDataOutputFrequency(
	Vn200* vn200,
	uint32_t asyncDataOutputFrequency,
	bool waitForResponse);

/**
 * \brief Gets the values in the Yaw Pitch Roll register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getYawPitchRoll(
	Vn200* vn200,
	VnYpr* attitude);

/**
 * \brief Gets the values in the Attitude Quaternion register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getQuaternion(
	Vn200* vn200,
	VnQuaternion* attitude);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, Magentic, Accleration, and Angular Rates register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor uncompensated angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getYawPitchRollMagneticAccelerationAngularRate(
	Vn200* vn200,
	VnYpr* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Quaternion, Magnetic, Acceleration and Angular
 * Rates register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getQuaternionMagneticAccelerationAngularRate(
	Vn200* vn200,
	VnQuaternion* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Magnetic Measurements register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getMagnetic(
	Vn200* vn200,
	VnVector3* magnetic);

/**
 * \brief Gets the values in the Acceleration Measurements register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getAcceleration(
	Vn200* vn200,
	VnVector3* acceleration);

/**
 * \brief Gets the values in the Angular Rate Measurements register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getAngularRate(
	Vn200* vn200,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Magnetic, Acceleration and Angular Rates register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getMagneticAccelerationAngularRate(
	Vn200* vn200,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, True Body Acceleration and Angular Rates register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] bodyAcceleration The current sensor body acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getYawPitchRollTrueBodyAccelerationAngularRate(
	Vn200* vn200,
	VnYpr* attitude,
	VnVector3* bodyAcceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, True Inertial Acceleration and Angular Rates register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] inertialAcceleration The current sensor inertial acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getYawPitchRollTrueInertialAccelerationAngularRate(
	Vn200* vn200,
	VnYpr* attitude,
	VnVector3* inertialAcceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the VPE Basic Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] enable The enable/disable value of the sensor.
 * \param[out] headingMode The heading mode value of the sensor.
 * \param[out] filteringMode The filtering mode value of the sensor.
 * \param[out] tuningMode The tuning mode value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getVpeControl(
	Vn200* vn200,
	uint8_t* enable,
	uint8_t* headingMode,
	uint8_t* filteringMode,
	uint8_t* tuningMode);

/**
 * \brief Sets the values of the VPE Basic Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] enable Value for the enable/disable field.
 * \param[in] headingMode Value for the heading mode field.
 * \param[in] filteringMode Value for the filtering mode field.
 * \param[in] tuningMode Value for the tuning mode field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setVpeControl(
	Vn200* vn200,
	uint8_t enable,
	uint8_t headingMode,
	uint8_t filteringMode,
	uint8_t tuningMode,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Magnetometer Basic Tuning register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] baseTuning The current sensor magnetometer base tuning (X,Y,Z) values.
 * \param[out] adaptiveTuning The current sensor magnetometer adaptive tuning (X,Y,Z) values.
 * \param[out] adaptiveFiltering The current sensor magnetometer adaptive filtering (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getVpeMagnetometerBasicTuning(
	Vn200* vn200,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning,
	VnVector3* adaptiveFiltering);

/**
 * \brief Sets the values of the VPE Magnetometer Basic Tuning register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] baseTuning The magnetometer base tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveTuning The magnetometer adaptive tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveFiltering The magnetometer adaptive filtering (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setVpeMagnetometerBasicTuning(
	Vn200* vn200,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Accelerometer Basic Tuning register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] baseTuning The current sensor accelerometer base tuning (X,Y,Z) values.
 * \param[out] adaptiveTuning The current sensor accelerometer adaptive tuning (X,Y,Z) values.
 * \param[out] adaptiveFiltering The current sensor accelerometer adaptive filtering (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getVpeAccelerometerBasicTuning(
	Vn200* vn200,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning,
	VnVector3* adaptiveFiltering);

/**
 * \brief Sets the values of the VPE Accelerometer Basic Tuning register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] baseTuning The accelerometer base tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveTuning The accelerometer adaptive tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveFiltering The accelerometer adaptive filtering (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setVpeAccelerometerBasicTuning(
	Vn200* vn200,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering,
	bool waitForResponse);

/**
 * \brief Gets the values in the IMU Measurements register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] magnetic The current sensor uncompensated magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor uncompensated acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor uncompensated angular rate (X,Y,Z) values.
 * \param[out] temperature The temperature value of the sensor.
 * \param[out] pressure The pressure value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getImuMeasurements(
	Vn200* vn200,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate,
	float* temperature,
	float* pressure);

/**
 * \brief Gets the values in the Reference Frame Rotation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] c The current sensor C matrix values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getReferenceFrameRotation(
	Vn200* vn200,
	VnMatrix3x3* c);

/**
 * \brief Sets the values of the Reference Frame Rotation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setReferenceFrameRotation(
	Vn200* vn200,
	VnMatrix3x3 c,
	bool waitForResponse);

/**
 * \brief Gets the values in the Synchronization Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] syncInMode The input signal synchronization mode value of the sensor.
 * \param[out] syncInEdge The input signal synchronization edge selection value of the sensor.
 * \param[out] syncInSkipFactor The input signal trigger skip factor value of the sensor.
 * \param[out] syncOutMode The output signal synchronization mode value of the sensor.
 * \param[out] syncOutPolarity The output signal synchronization polarity value of the sensor.
 * \param[out] syncOutSkipFactor The output synchronization signal skip factor value of the sensor.
 * \param[out] syncOutPulseWidth The output synchronization signal pulse width value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getSynchronizationControl(
	Vn200* vn200,
	uint8_t* syncInMode,
	uint8_t* syncInEdge,
	uint16_t* syncInSkipFactor,
	uint8_t* syncOutMode,
	uint8_t* syncOutPolarity,
	uint16_t* syncOutSkipFactor,
	uint32_t* syncOutPulseWidth);

/**
 * \brief Sets the values of the Synchronization Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] syncInMode Value for the input signal synchronization mode field.
 * \param[in] syncInEdge Value for the input signal synchronization edge selection field.
 * \param[in] syncInSkipFactor Value for the input signal trigger skip factor field.
 * \param[in] syncOutMode Value for the output signal synchronization mode field.
 * \param[in] syncOutPolarity Value for the output signal synchronization polarity field.
 * \param[in] syncOutSkipFactor Value for the output synchronization signal skip factor field.
 * \param[in] syncOutPulseWidth Value for the output synchronization signal pulse width field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setSynchronizationControl(
	Vn200* vn200,
	uint8_t syncInMode,
	uint8_t syncInEdge,
	uint16_t syncInSkipFactor,
	uint8_t syncOutMode,
	uint8_t syncOutPolarity,
	uint16_t syncOutSkipFactor,
	uint32_t syncOutPulseWidth,
	bool waitForResponse);

/**
 * \brief Gets the values in the Synchronization Status register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] syncInCount The synchronization in count value of the sensor.
 * \param[out] syncInTime The synchronization in time value of the sensor.
 * \param[out] syncOutCount The synchronization out count value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getSynchronizationStatus(
	Vn200* vn200,
	uint32_t* syncInCount,
	uint32_t* syncInTime,
	uint32_t* syncOutCount);

/**
 * \brief Sets the values of the Synchronization Status register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] syncInCount Value for the synchronization in count field.
 * \param[in] syncInTime Value for the synchronization in time field.
 * \param[in] syncOutCount Value for the synchronization out count field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setSynchronizationStatus(
	Vn200* vn200,
	uint32_t syncInCount,
	uint32_t syncInTime,
	uint32_t syncOutCount,
	bool waitForResponse);

/**
 * \brief Gets the contents of the Delta Theta and Delta Velocity register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] deltaTime Delta time for the integration interval.
 * \param[out] deltaTheta Delta rotation vector.
 * \param[out] deltaVelocity Delta velocity vector.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getDeltaThetaAndDeltaVelocity(
	Vn200* vn200,
	float* deltaTime,
	VnVector3* deltaTheta,
	VnVector3* deltaVelocity);

/**
 * \brief Gets the values in the Acceleration Compensation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getAccelerationCompensation(
	Vn200* vn200,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Acceleration Compensation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setAccelerationCompensation(
	Vn200* vn200,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Gets the values in the Magnetic Compensation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getMagneticCompensation(
	Vn200* vn200,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Magnetic Compensation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setMagneticCompensation(
	Vn200* vn200,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Gets the values in the Gyro Compensation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGyroCompensation(
	Vn200* vn200,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Gyro Compensation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setGyroCompensation(
	Vn200* vn200,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Retreives the current values of the IMU Filtering Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] magWindowSize Number of previous measurements averaged for magnetic measurements.
 * \param[out] accelWindowSize Number of previous measurements averaged for acceleration measurements.
 * \param[out] gyroWindowSize Number of previous measurements averaged for gyro measurements.
 * \param[out] tempWindowSize Number of previous measurements averaged for temperature measurements.
 * \param[out] presWindowSize Number of previous measurements averaged for pressure measurements.
 * \param[out] magFilterMode Filtering mode for magnetic measurements.
 * \param[out] accelFilterMode Filtering mode for acceleration measurements.
 * \param[out] gyroFilterMode Filtering mode for gyro measurements.
 * \param[out] tempFilterMode Filtering mode for temperature measurements.
 * \param[out] presFilterMode Filtering mode for pressure measurements.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getImuFilteringConfiguration(
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
	uint8_t* presFilterMode);

/**
 * \brief Sets the values of the IMU Filtering Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] magWindowSize Number of previous measurements averaged for magnetic measurements.
 * \param[in] accelWindowSize Number of previous measurements averaged for acceleration measurements.
 * \param[in] gyroWindowSize Number of previous measurements averaged for gyro measurements.
 * \param[in] tempWindowSize Number of previous measurements averaged for temperature measurements.
 * \param[in] presWindowSize Number of previous measurements averaged for pressure measurements.
 * \param[in] magFilterMode Filtering mode for magnetic measurements.
 * \param[in] accelFilterMode Filtering mode for acceleration measurements.
 * \param[in] gyroFilterMode Filtering mode for gyro measurements.
 * \param[in] tempFilterMode Filtering mode for temperature measurements.
 * \param[in] presFilterMode Filtering mode for pressure measurements.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setImuFilteringConfiguration(
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
	bool waitForResponse);

/**
 * \brief Retreives the current values of the Delta Theta and Delta Velocity Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] integrationFrame Output frame for delta velocity quantities.
 * \param[out] gyroCompensation Compensation to apply to angular rate.
 * \param[out] accelCompensation Compensation to apply to accelerations.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getDeltaThetaAndDeltaVelocityConfiguration(
	Vn200* vn200,
	uint8_t* integrationFrame,
	uint8_t* gyroCompensation,
	uint8_t* accelCompensation);

/**
 * \brief Sets the values of the Delta Theta and Delta Velocity Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] integrationFrame Output frame for delta velocity quantities.
 * \param[in] gyroCompensation Compensation to apply to angular rate.
 * \param[in] accelCompensation Compensation to apply to accelerations.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setDeltaThetaAndDeltaVelocityConfiguration(
	Vn200* vn200,
	uint8_t integrationFrame,
	uint8_t gyroCompensation,
	uint8_t accelCompensation,
	bool waitForResponse);

/**
 * \brief Gets the values in the Magnetometer Calibration Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] hsiMode The HSIMode value of the sensor.
 * \param[out] hsiOutput The HSIOutput value of the sensor.
 * \param[out] convergeRate The ConvergeRate value of the sensor.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getMagnetometerCalibrationControl(
	Vn200* vn200,
	uint8_t* hsiMode,
	uint8_t* hsiOutput,
	uint8_t* convergeRate);

/**
 * \brief Sets the values of the Magnetometer Calibration Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] hsiMode Value for the HSIMode field.
 * \param[in] hsiOutput Value for the HSIOutput field.
 * \param[in] convergeRate Value for the ConvergeRate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setMagnetometerCalibrationControl(
	Vn200* vn200,
	uint8_t hsiMode,
	uint8_t hsiOutput,
	uint8_t convergeRate,
	bool waitForResponse);

/**
 * \brief Gets the values in the Calculated Magnetometer Calibration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getCalculatedMagnetometerCalibration(
	Vn200* vn200,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Gets the values in the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] magneticReference The current sensor magnetic reference vector (X,Y,Z) values.
 * \param[out] gravityReference The current sensor gravity reference vector (X,Y,Z) values.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getMagneticGravityReferenceVectors(
	Vn200* vn200,
	VnVector3* magneticReference,
	VnVector3* gravityReference);

/**
 * \brief Sets the values of the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] magneticReference The magnetic reference vector (X,Y,Z) values to write to the sensor.
 * \param[in] gravityReference The gravity reference vector (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setMagneticGravityReferenceVectors(
	Vn200* vn200,
	VnVector3 magneticReference,
	VnVector3 gravityReference,
	bool waitForResponse);

/**
 * \brief Gets the values in the Communication Protocol Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] serialCount The serial count value of the sensor.
 * \param[out] serialStatus The serial status value of the sensor.
 * \param[out] spiCount The SPI count value of the sensor.
 * \param[out] spiStatus The SPI status value of the sensor.
 * \param[out] serialChecksum The serial checksum value of the sensor.
 * \param[out] spiChecksum The SPI checksum value of the sensor.
 * \param[out] errorMode The error mode value of the sensor.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getCommunicationProtocolControl(
	Vn200* vn200,
	uint8_t* serialCount,
	uint8_t* serialStatus,
	uint8_t* spiCount,
	uint8_t* spiStatus,
	uint8_t* serialChecksum,
	uint8_t* spiChecksum,
	uint8_t* errorMode);

/**
 * \brief Sets the values of the Communication Protocol Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] serialCount Value for the serial count field.
 * \param[in] serialStatus Value for the serial status field.
 * \param[in] spiCount Value for the SPI count field.
 * \param[in] spiStatus Value for the SPI status field.
 * \param[in] serialChecksum Value for the serial checksum field.
 * \param[in] spiChecksum Value for the SPI checksum field.
 * \param[in] errorMode Value for the error mode field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setCommunicationProtocolControl(
	Vn200* vn200,
	uint8_t serialCount,
	uint8_t serialStatus,
	uint8_t spiCount,
	uint8_t spiStatus,
	uint8_t serialChecksum,
	uint8_t spiChecksum,
	uint8_t errorMode,
	bool waitForResponse);

/**
 * \brief Gets the values in the Reference Vector Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] useMagModel The UseMagModel field.
 * \param[out] useGravityModel The UseGravityModel field.
 * \param[out] recalcThreshold The RecalcThreshold field.
 * \param[out] year The Year field.
 * \param[out] lla The Lattitude, Longitude, Altitude fields.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getReferenceVectorConfiguration(
	Vn200* vn200,
	uint8_t* useMagModel,
	uint8_t* useGravityModel,
	uint32_t* recalcThreshold,
	float* year,
	VnVector3* lla);

/**
 * \brief Sets the values in the Reference Vector Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] useMagModel The UseMagModel field.
 * \param[in] useGravityModel The UseGravityModel field.
 * \param[in] recalcThreshold The RecalcThreshold field.
 * \param[in] year The Year field.
 * \param[in] lla The Lattitude, Longitude, Altitude fields.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setReferenceVectorConfiguration(
	Vn200* vn200,
	uint8_t useMagModel,
	uint8_t useGravityModel,
	uint32_t recalcThreshold,
	float year,
	VnVector3 lla,
	bool waitForResponse);

/**
 * \brief Pauses the asynchronous data output.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_pauseAsyncOutputs(
	Vn200* vn200,
	bool waitForResponse);

/**
 * \brief Resumes the asynchronous data output.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_resumeAsyncOutputs(
	Vn200* vn200,
	bool waitForResponse);

#ifdef __cplusplus
}
#endif

#endif /* _VN200_H_ */
