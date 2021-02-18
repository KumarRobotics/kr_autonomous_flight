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
 * This header file provides access to devices based on VectorNav's VN-100
 * family of orientation sensors.
 */
#ifndef _VN100_H_
#define _VN100_H_

/* Disable some unecessary warnings for compiling using Visual Studio with -Wall. */
#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4668)	/* Preprocessor macro not defined warning. */
#endif

#if defined(_MSC_VER) && _MSC_VER <= 1500
	/* Visual Studio 2008 and earlier do not include the stdint.h header file. */
	#include "vnint.h"
#else
	#include <stdint.h>
#endif

#if defined(_MSC_VER)
	#pragma warning(pop)
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

/* Disable some unecessary warnings for compiling using Visual Studio with -Wall. */
#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4820)	/* Padding added in structures warning. */
#endif

/**
 * \brief Holds connection information for accessing a VN-100 device.
 */
typedef struct {
	char*				portName;		/**< The name of the serial port. */
	int					baudRate;		/**< The baudrate of the serial port. */
	bool				isConnected;	/**< Inidicates if the serial port is open. */
	VnDevice	vndevice;		/**< Pointer to internally used data. */
} Vn100;

#if defined(_MSC_VER)
	#pragma warning(pop)
#endif

/**
 * \brief Connects to a VectorNav VN-100 device.
 *
 * \param[out]	newVn100	An uninitialized Vn100 control object should be passed in.
 * \param[in]	portName	The name of the COM port to connect to.
 * \param[in]	baudrate	The baudrate to connect at.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_connect(
	Vn100* newVn100,
	const char* portName,
	int baudrate);

/**
 * \brief Disconnects from the VN-100 device and disposes of any internal resources.
 *
 * \param vn100 Pointer to the Vn100 control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_disconnect(
	Vn100* vn100);

/**
 * \brief Commands the VN-100 module to zero out its current orientation.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_tare(
	Vn100* vn100,
	bool waitForResponse);

/**
 * \brief Notifies the VN-100 module if a known magnetic disturbance is present.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] isDisturbancePresent True if a known magnetic disturbance is present. False if not.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_knownMagneticDisturbance(
	Vn100* vn100,
	bool isDisturbancePresent,
	bool waitForResponse);

/**
 * \brief Notifies the VN-100 module if a known acceleration disturbance is present.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] isDisturbancePresent True if a known acceleration disturbance is present. False if not.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_knownAccelerationDisturbance(
	Vn100* vn100,
	bool isDisturbancePresent,
	bool waitForResponse);

/**
 * \brief Commands the VN-100 module to save the current gyro bias estimate to
 * memory for use on the next module startup.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setGyroBias(
	Vn100* vn100,
	bool waitForResponse);

/**
 * \brief Allows registering a function which will be called whenever a new
 * asynchronous data packet is received from the VN-100 module.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] listener The function pointer to be called when asynchronous data
 *     is received.
 * 
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_registerAsyncDataReceivedListener(
	Vn100* vn100,
	VnDeviceNewAsyncDataReceivedListener listener);

/**
 * \brief Unregisters an already registered function for recieving notifications
 * of when new asynchronous data is received.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] listener The function pointer to unregister.
 * 
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_unregisterAsyncDataReceivedListener(
	Vn100* vn100,
	VnDeviceNewAsyncDataReceivedListener listener);

/**
 * \brief Allows registering a function which will be called whenever an error
 * code is received from the VN-100 module.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] listener The function pointer to be called when an error code
 *     is received.
 * 
 * \return VectorNav error code.
 */
VN_ERROR_CODE vn100_registerErrorCodeReceivedListener(
	Vn100* vn100,
	VnDeviceErrorCodeReceivedListener listener);

/**
 * \brief Unregisters an already registered function for recieving notifications
 * of when error codes from the VN-100 module are received.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] listener The function pointer to unregister.
 * 
 * \return VectorNav error code.
 */
VN_ERROR_CODE vn100_unregisterErrorCodeReceivedListener(
	Vn100* vn100,
	VnDeviceErrorCodeReceivedListener listener);

/**
 * \brief Checks if we are able to send and receive communication with the VN-100 sensor.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 *
 * \return VN_TRUE if the library was able to send and receive a valid response from the VN-100 sensor; otherwise VN_FALSE.
 */
DLL_EXPORT bool vn100_verifyConnectivity(
	Vn100* vn100);

/**
 * \brief Gets the current configuration of the requested binary output register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in]	binaryOutputRegisterId The ID of the binary output register to query for its configuration. Must be 1 - 3.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	selectedOutputGroups The selected output groups.
 * \param[out]	outputGroup1Selections The configured output data types for output group 1.
 * \param[out]	outputGroup3Selections The configured output data types for output group 3.
 * \param[out]	outputGroup5Selections The configured output data types for output group 5.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getBinaryOutputConfiguration(
	Vn100* vn100,
	uint8_t binaryOutputRegisterId,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup5Selections);

/**
 * \brief Gets the current configuration of the binary output register 1.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	selectedOutputGroups The selected output groups.
 * \param[out]	outputGroup1Selections The configured output data types for output group 1.
 * \param[out]	outputGroup3Selections The configured output data types for output group 3.
 * \param[out]	outputGroup5Selections The configured output data types for output group 5.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getBinaryOutput1Configuration(
	Vn100* vn100,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup5Selections);

/**
 * \brief Gets the current configuration of the binary output register 2.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	selectedOutputGroups The selected output groups.
 * \param[out]	outputGroup1Selections The configured output data types for output group 1.
 * \param[out]	outputGroup3Selections The configured output data types for output group 3.
 * \param[out]	outputGroup5Selections The configured output data types for output group 5.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getBinaryOutput2Configuration(
	Vn100* vn100,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup5Selections);

/**
 * \brief Gets the current configuration of the binary output register 3.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	selectedOutputGroups The selected output groups.
 * \param[out]	outputGroup1Selections The configured output data types for output group 1.
 * \param[out]	outputGroup3Selections The configured output data types for output group 3.
 * \param[out]	outputGroup5Selections The configured output data types for output group 5.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getBinaryOutput3Configuration(
	Vn100* vn100,
	uint16_t* asyncMode,
	uint16_t* rateDivisor,
	uint16_t* selectedOutputGroups,
	uint16_t* outputGroup1Selections,
	uint16_t* outputGroup3Selections,
	uint16_t* outputGroup5Selections);

/**
 * \brief Sets the configuration of the requested binary output register. Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in]	binaryOutputRegisterId The ID of the binary output register to set its configuration. Must be 1 - 3.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	outputGroup1Selections The output data types for output group 1.
 * \param[out]	outputGroup3Selections The output data types for output group 3.
 * \param[out]	outputGroup5Selections The output data types for output group 5.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setBinaryOutputConfiguration(
	Vn100* vn100,
	uint8_t binaryOutputRegisterId,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup5Selections,
	bool waitForResponse);

/**
 * \brief Sets the current configuration of the binary output register 1.  Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	outputGroup1Selections The output data types for output group 1.
 * \param[out]	outputGroup3Selections The output data types for output group 3.
 * \param[out]	outputGroup5Selections The output data types for output group 5.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setBinaryOutput1Configuration(
	Vn100* vn100,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup5Selections,
	bool waitForResponse);

/**
 * \brief Sets the current configuration of the binary output register 2.  Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	outputGroup1Selections The output data types for output group 1.
 * \param[out]	outputGroup3Selections The output data types for output group 3.
 * \param[out]	outputGroup5Selections The output data types for output group 5.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setBinaryOutput2Configuration(
	Vn100* vn100,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup5Selections,
	bool waitForResponse);

/**
 * \brief Sets the current configuration of the binary output register 3.  Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out]	asyncMode The selected asyncMode for the binary output register.
 * \param[out]	rateDivisor The rate divisor.
 * \param[out]	outputGroup1Selections The output data types for output group 1.
 * \param[out]	outputGroup3Selections The output data types for output group 3.
 * \param[out]	outputGroup5Selections The output data types for output group 5.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setBinaryOutput3Configuration(
	Vn100* vn100,
	uint16_t asyncMode,
	uint16_t rateDivisor,
	uint16_t outputGroup1Selections,
	uint16_t outputGroup3Selections,
	uint16_t outputGroup5Selections,
	bool waitForResponse);


/**
 * \brief Gets the values in the Quaternion and Magnetic register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware. Please use the function vn100_getQuaternionMagneticAccelerationAngularRate
 *             instead.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getQuaternionMagnetic(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* magnetic);

/**
 * \brief Gets the values in the Quaternion Acceleration register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware. Please use the function vn100_getQuaternionMagneticAccelerationAngularRate
 *             instead.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getQuaternionAcceleration(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* acceleration);

/**
 * \brief Gets the values in the Quaternion Angular Rates register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware. Please use the function vn100_getQuaternionMagneticAccelerationAngularRate
 *             instead.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getQuaternionAngularRate(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Quaternion, Magnetic and Acceleration register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware. Please use the function vn100_getQuaternionMagneticAccelerationAngularRate
 *             instead.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getQuaternionMagneticAcceleration(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration);

/**
 * \brief Gets the values in the Quaternion, Acceleration and Angular Rates register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware. Please use the function vn100_getQuaternionMagneticAccelerationAngularRate
 *             instead.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getQuaternionAccelerationAngularRate(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* magnetic,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Attitude (Directional Cosine Matrix) register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware. Please use the function vn100_getQuaternion
 *             or vn100_getYawPitchRoll instead.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor attitude (DCM) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getDirectionCosineMatrix(
	Vn100* vn100,
	VnMatrix3x3* attitude);

/**
 * \brief Gets the values in the Filter Measurement Variance Parameters register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] angularWalkVariance The angular walk variance value of the sensor.
 * \param[out] angularRateVariance The current sensor angular rate variance (X,Y,Z) values.
 * \param[out] magneticVariance The current sensor magnetic variance (X,Y,Z) values.
 * \param[out] accelerationVariance The current sensor acceleration variance (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getFilterMeasurementVarianceParameters(
	Vn100* vn100,
	double* angularWalkVariance,
	VnVector3* angularRateVariance,
	VnVector3* magneticVariance,
	VnVector3* accelerationVariance);

/**
 * \brief Sets the values of the Filter Measurement Variance Parameters register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] angularWalkVariance Value for the angular walk variance field.
 * \param[in] angularRateVariance The angular rate variance (X,Y,Z) values to write to the sensor.
 * \param[in] magneticVariance The magnetic variance (X,Y,Z) values to write to the sensor.
 * \param[in] accelerationVariance The acceleration variance (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setFilterMeasurementVarianceParameters(
	Vn100* vn100,
	double angularWalkVariance,
	VnVector3 angularRateVariance,
	VnVector3 magneticVariance,
	VnVector3 accelerationVariance,
	bool waitForResponse);

/**
 * \brief Gets the values in the Filter Active Tuning Parameters register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] magneticGain The magnetic disturbance gain value of the sensor.
 * \param[out] accelerationGain The acceleration disturbance gain value of the sensor.
 * \param[out] magneticMemory The magnetic disturbance memory value of the sensor.
 * \param[out] accelerationMemory The acceleration disturbance memory value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getFilterActiveTuningParameters(
	Vn100* vn100,
	double* magneticGain,
	double* accelerationGain,
	double* magneticMemory,
	double* accelerationMemory);

/**
 * \brief Sets the values of the Filter Active Tuning Parameters register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] magneticGain Value for the magnetic disturbance gain field.
 * \param[in] accelerationGain Value for the acceleration disturbance gain field.
 * \param[in] magneticMemory Value for the magnetic disturbance memory field.
 * \param[in] accelerationMemory Value for the acceleration disturbance memory field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setFilterActiveTuningParameters(
	Vn100* vn100,
	double magneticGain,
	double accelerationGain,
	double magneticMemory,
	double accelerationMemory,
	bool waitForResponse);

/**
 * \brief Gets the values in the Accelerometer Gain register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] accelerometerGain The accelerometer gain mode value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getAccelerometerGain(
	Vn100* vn100,
	unsigned int* accelerometerGain);

/**
 * \brief Sets the values of the Accelerometer Gain register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] accelerometerGain Value for the accelerometer gain mode field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setAccelerometerGain(
	Vn100* vn100,
	unsigned int accelerometerGain,
	bool waitForResponse);

/**
 * \brief Gets the values in the Communication Protocol Status register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] numOfParsedSerialMessages The number of successfully parsed serial messages received value of the sensor.
 * \param[out] numOfParsedSpiMessages The number of successfully parsed SPI messages received value of the sensor.
 * \param[out] maxUsageSerialRxBuffer The maximum percent usage of serial incoming buffer value of the sensor.
 * \param[out] maxUsageSerialTxBuffer The maximum percent usage of serial outgoing buffer value of the sensor.
 * \param[out] maxUsageSpiRxBuffer The maximum percent usage of SPI incoming buffer value of the sensor.
 * \param[out] maxUsageSpiTxBuffer The maximum percent usage of SPI outgoing buffer value of the sensor.
 * \param[out] systemError0 The SystemError 0 value of the sensor.
 * \param[out] systemError1 The SystemError 1 value of the sensor.
 * \param[out] systemError2 The SystemError 2 value of the sensor.
 * \param[out] systemError3 The SystemError 3 value of the sensor.
 * \param[out] systemError4 The SystemError 4 value of the sensor.
 * \param[out] systemError5 The SystemError 5 value of the sensor.
 * \param[out] systemError6 The SystemError 6 value of the sensor.
 * \param[out] systemError7 The SystemError 7 value of the sensor.
 * \param[out] systemError8 The SystemError 8 value of the sensor.
 * \param[out] systemError9 The SystemError 9 value of the sensor.
 * \param[out] systemError10 The SystemError 10 value of the sensor.
 * \param[out] systemError11 The SystemError 11 value of the sensor.
 * \param[out] systemError12 The SystemError 12 value of the sensor.
 * \param[out] systemError13 The SystemError 13 value of the sensor.
 * \param[out] systemError14 The SystemError 14 value of the sensor.
 * \param[out] systemError15 The SystemError 15 value of the sensor.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getCommunicationProtocolStatus(
	Vn100* vn100,
	unsigned int* numOfParsedSerialMessages,
	unsigned int* numOfParsedSpiMessages,
	unsigned char* maxUsageSerialRxBuffer,
	unsigned char* maxUsageSerialTxBuffer,
	unsigned char* maxUsageSpiRxBuffer,
	unsigned char* maxUsageSpiTxBuffer,
	unsigned short* systemError0,
	unsigned short* systemError1,
	unsigned short* systemError2,
	unsigned short* systemError3,
	unsigned short* systemError4,
	unsigned short* systemError5,
	unsigned short* systemError6,
	unsigned short* systemError7,
	unsigned short* systemError8,
	unsigned short* systemError9,
	unsigned short* systemError10,
	unsigned short* systemError11,
	unsigned short* systemError12,
	unsigned short* systemError13,
	unsigned short* systemError14,
	unsigned short* systemError15);

/**
 * \brief Sets the values of the Communication Protocol Status register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] numOfParsedSerialMessages Value for the number of successfully parsed serial messages received field.
 * \param[in] numOfParsedSpiMessages Value for the number of successfully parsed SPI messages received field.
 * \param[in] maxUsageSerialRxBuffer Value for the maximum percent usage of serial incoming buffer field.
 * \param[in] maxUsageSerialTxBuffer Value for the maximum percent usage of serial outgoing buffer field.
 * \param[in] maxUsageSpiRxBuffer Value for the maximum percent usage of SPI incoming buffer field.
 * \param[in] maxUsageSpiTxBuffer Value for the maximum percent usage of SPI outgoing buffer field.
 * \param[in] systemError0 Value for the SystemError 0 field.
 * \param[in] systemError1 Value for the SystemError 1 field.
 * \param[in] systemError2 Value for the SystemError 2 field.
 * \param[in] systemError3 Value for the SystemError 3 field.
 * \param[in] systemError4 Value for the SystemError 4 field.
 * \param[in] systemError5 Value for the SystemError 5 field.
 * \param[in] systemError6 Value for the SystemError 6 field.
 * \param[in] systemError7 Value for the SystemError 7 field.
 * \param[in] systemError8 Value for the SystemError 8 field.
 * \param[in] systemError9 Value for the SystemError 9 field.
 * \param[in] systemError10 Value for the SystemError 10 field.
 * \param[in] systemError11 Value for the SystemError 11 field.
 * \param[in] systemError12 Value for the SystemError 12 field.
 * \param[in] systemError13 Value for the SystemError 13 field.
 * \param[in] systemError14 Value for the SystemError 14 field.
 * \param[in] systemError15 Value for the SystemError 15 field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setCommunicationProtocolStatus(
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
	bool waitForResponse);

/**
 * \brief Gets the values in the Filter Basic Control register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] magneticMode The magnetic mode value of the sensor.
 * \param[out] externalMagnetometerMode The external magnetometer mode value of the sensor.
 * \param[out] externalAccelerometerMode The external accelerometer mode value of the sensor.
 * \param[out] externalGyroscopeMode The external gyroscope mode value of the sensor.
 * \param[out] angularRateLimit The current sensor angular rate saturation liimit (X,Y,Z) values.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getFilterBasicControl(
	Vn100* vn100,
	unsigned char* magneticMode,
	unsigned char* externalMagnetometerMode,
	unsigned char* externalAccelerometerMode,
	unsigned char* externalGyroscopeMode,
	VnVector3* angularRateLimit);

/**
 * \brief Sets the values of the Filter Basic Control register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] magneticMode Value for the magnetic mode field.
 * \param[in] externalMagnetometerMode Value for the external magnetometer mode field.
 * \param[in] externalAccelerometerMode Value for the external accelerometer mode field.
 * \param[in] externalGyroscopeMode Value for the external gyroscope mode field.
 * \param[in] angularRateLimit The angular rate saturation liimit (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setFilterBasicControl(
	Vn100* vn100,
	unsigned char magneticMode,
	unsigned char externalMagnetometerMode,
	unsigned char externalAccelerometerMode,
	unsigned char externalGyroscopeMode,
	VnVector3 angularRateLimit,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Magnetometer Advanced Tuning register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] minimumFiltering The current sensor minimum allowed level of filtering (X,Y,Z) values.
 * \param[out] maximumFiltering The current sensor maximum allowed level of filtering (X,Y,Z) values.
 * \param[out] maximumAdaptRate The MaxAdaptRate value of the sensor.
 * \param[out] disturbanceWindow The DisturbanceWindow value of the sensor.
 * \param[out] maximumTuning The MaxTuning value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getVpeMagnetometerAdvancedTuning(
	Vn100* vn100,
	VnVector3* minimumFiltering,
	VnVector3* maximumFiltering,
	float* maximumAdaptRate,
	float* disturbanceWindow,
	float* maximumTuning);

/**
 * \brief Sets the values of the VPE Magnetometer Advanced Tuning register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] minimumFiltering The minimum allowed level of filtering (X,Y,Z) values to write to the sensor.
 * \param[in] maximumFiltering The maximum allowed level of filtering (X,Y,Z) values to write to the sensor.
 * \param[in] maximumAdaptRate Value for the MaxAdaptRate field.
 * \param[in] disturbanceWindow Value for the DisturbanceWindow field.
 * \param[in] maximumTuning Value for the MaxTuning field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setVpeMagnetometerAdvancedTuning(
	Vn100* vn100,
	VnVector3 minimumFiltering,
	VnVector3 maximumFiltering,
	float maximumAdaptRate,
	float disturbanceWindow,
	float maximumTuning,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Accelerometer Advanced Tuning register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] minimumFiltering The current sensor minimum allowed level of filtering (X,Y,Z) values.
 * \param[out] maximumFiltering The current sensor maximum allowed level of filtering (X,Y,Z) values.
 * \param[out] maximumAdaptRate The MaxAdaptRate value of the sensor.
 * \param[out] disturbanceWindow The DisturbanceWindow value of the sensor.
 * \param[out] maximumTuning The MaxTuning value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getVpeAccelerometerAdvancedTuning(
	Vn100* vn100,
	VnVector3* minimumFiltering,
	VnVector3* maximumFiltering,
	float* maximumAdaptRate,
	float* disturbanceWindow,
	float* maximumTuning);

/**
 * \brief Sets the values of the VPE Accelerometer Advanced Tuning register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] minimumFiltering The minimum allowed level of filtering (X,Y,Z) values to write to the sensor.
 * \param[in] maximumFiltering The maximum allowed level of filtering (X,Y,Z) values to write to the sensor.
 * \param[in] maximumAdaptRate Value for the MaxAdaptRate field.
 * \param[in] disturbanceWindow Value for the DisturbanceWindow field.
 * \param[in] maximumTuning Value for the MaxTuning field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setVpeAccelerometerAdvancedTuning(
	Vn100* vn100,
	VnVector3 minimumFiltering,
	VnVector3 maximumFiltering,
	float maximumAdaptRate,
	float disturbanceWindow,
	float maximumTuning,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Gyro Basic Tuning register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] varianceAngularWalk The current sensor gyroscope angular walk variance (X,Y,Z) values.
 * \param[out] baseTuning The current sensor gyroscope base tuning (X,Y,Z) values.
 * \param[out] adaptiveTuning The current sensor gyroscope adaptive tuning (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getVpeGyroBasicTuning(
	Vn100* vn100,
	VnVector3* varianceAngularWalk,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning);

/**
 * \brief Sets the values of the VPE Gyro Basic Tuning register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] varianceAngularWalk The gyroscope angular walk variance (X,Y,Z) values to write to the sensor.
 * \param[in] baseTuning The gyroscope base tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveTuning The gyroscope adaptive tuning (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setVpeGyroBasicTuning(
	Vn100* vn100,
	VnVector3 varianceAngularWalk,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	bool waitForResponse);

/**
 * \brief Gets the values in the Filter Status register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] solutionStatus The solution status bitfield value of the sensor.
 * \param[out] yawUncertainty The YawUncertainty value of the sensor.
 * \param[out] pitchUncertainty The PitchUncertainty value of the sensor.
 * \param[out] rollUncertainty The RollUncertainty value of the sensor.
 * \param[out] gyroBiasUncertainty The GyroBiasUncertainty value of the sensor.
 * \param[out] magUncertainty The MagUncertainty value of the sensor.
 * \param[out] accelUncertainty The AccelUncertainty value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getFilterStatus(
	Vn100* vn100,
	unsigned short* solutionStatus,
	float* yawUncertainty,
	float* pitchUncertainty,
	float* rollUncertainty,
	float* gyroBiasUncertainty,
	float* magUncertainty,
	float* accelUncertainty);

/**
 * \brief Gets the values in the Filter Startup Gyro Bias register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] gyroBias The current sensor gyroscope startup bias (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getFilterStartupGyroBias(
	Vn100* vn100,
	VnVector3* gyroBias);

/**
 * \brief Sets the values of the Filter Startup Gyro Bias register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] gyroBias The gyroscope startup bias (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setFilterStartupGyroBias(
	Vn100* vn100,
	VnVector3 gyroBias,
	bool waitForResponse);

/**
 * \brief Gets the values in the Magnetometer Calibration Status register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] lastBin The LastBin value of the sensor.
 * \param[out] numOfMeasurements The NumMeas value of the sensor.
 * \param[out] avgResidual The AvgResidual value of the sensor.
 * \param[out] lastMeasurement The current sensor last measurement (X,Y,Z) values.
 * \param[out] bin0 The number of measurements in bin 1 value of the sensor.
 * \param[out] bin1 The number of measurements in bin 2 value of the sensor.
 * \param[out] bin2 The number of measurements in bin 3 value of the sensor.
 * \param[out] bin3 The number of measurements in bin 4 value of the sensor.
 * \param[out] bin4 The number of measurements in bin 5 value of the sensor.
 * \param[out] bin5 The number of measurements in bin 6 value of the sensor.
 * \param[out] bin6 The number of measurements in bin 7 value of the sensor.
 * \param[out] bin7 The number of measurements in bin 8 value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getMagnetometerCalibrationStatus(
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
	unsigned char* bin7);

/**
 * \brief Gets the values in the Indoor Heading Mode Control register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] maxRateError The MaxRateError value of the sensor.
 * \param[out] reserved The reserved value of the sensor.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getIndoorHeadingModeControl(
	Vn100* vn100,
	float* maxRateError,
	float* reserved);

/**
 * \brief Sets the values of the Indoor Heading Mode Control register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] maxRateError Value for the MaxRateError field.
 * \param[in] reserved Value for the reserved field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setIndoorHeadingModeControl(
	Vn100* vn100,
	float maxRateError,
	float reserved,
	bool waitForResponse);

/**
 * \brief Gets the values in the Velocity Compenstation Control register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] mode The Mode field.
 * \param[out] velocityTuning The VelocityTuning field.
 * \param[out] rateTuning The RateTuning field.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getVelocityCompenstationControl(
	Vn100* vn100,
	uint8_t* mode,
	float* velocityTuning,
	float* rateTuning);

/**
 * \brief Sets the values of the Velocity Compenstation Control register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] mode The Mode field.
 * \param[in] velocityTuning The VelocityTuning field.
 * \param[in] rateTuning The RateTuning field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setVelocityCompenstationControl(
	Vn100* vn100,
	uint8_t mode,
	float velocityTuning,
	float rateTuning,
	bool waitForResponse);

/**
 * \brief Gets the values in the Velocity Compenstation Measurement register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] velocity The Velocity field.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getVelocityCompenstationMeasurement(
	Vn100* vn100,
	VnVector3* velocity);

/**
 * \brief Sets the values of the Velocity Compenstation Measurement register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] velocity The Velocity field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setVelocityCompenstationMeasurement(
	Vn100* vn100,
	VnVector3 velocity,
	bool waitForResponse);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll and Inertial Calibrated Measurements register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] inertialMagnetic The current sensor inertial magnetic (X,Y,Z) values.
 * \param[out] inertialAcceleration The current sensor inertial acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getYawPitchRollInertialCalibratedMeasurements(
	Vn100* vn100,
	VnYpr* attitude,
	VnVector3* inertialMagnetic,
	VnVector3* inertialAcceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Raw Voltage Measurements register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] magnetometer The current sensor magnetometer raw voltages (X,Y,Z) values.
 * \param[out] accelerometer The current sensor accelerometer raw voltages (X,Y,Z) values.
 * \param[out] gyroscope The current sensor gyroscope raw voltages (X,Y,Z) values.
 * \param[out] temperature The temperature raw voltages value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getRawVoltageMeasurements(
	Vn100* vn100,
	VnVector3* magnetometer,
	VnVector3* accelerometer,
	VnVector3* gyroscope,
	float* temperature);

/**
 * \brief Gets the values in the Kalman Filter State Vector register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] gyroscopeBias The current sensor gyroscope bias (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getKalmanFilterStateVector(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* gyroscopeBias);

/**
 * \brief Gets the values in the Kalman Filter Covariance Matrix Diagonal register.
 * \deprecated This function is deprecated in version 2.0 and greater of the
 *             VN-100 firmware.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] p00 The P[0,0] value of the sensor.
 * \param[out] p11 The P[1,1] value of the sensor.
 * \param[out] p22 The P[2,2] value of the sensor.
 * \param[out] p33 The P[3,3] value of the sensor.
 * \param[out] p44 The P[4,4] value of the sensor.
 * \param[out] p55 The P[5,5] value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getKalmanFilterCovarianceMatrixDiagonal(
	Vn100* vn100,
	float* p00,
	float* p11,
	float* p22,
	float* p33,
	float* p44,
	float* p55);

/**
 * \brief Retrieves the associated timeout value for the Vn100 object.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 *
 * \return The timeout value in milliseconds. -1 indicates that timeouts are
 * not used.
 */
DLL_EXPORT int vn100_get_timeout(
	Vn100* vn100);

/**
 * \brief Sets the timeout value for the reading values from the VectorNav sensor.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] timeout The timeout value in milliseconds. Specify -1 to not use
 * any timeouts.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_set_timeout(
	Vn100* vn100,
	int timeout);

/**
 * \brief Retrieves the most recent stored asynchronous data.
 *
 * \param[in] vn100 Pointer to the Vn100 object.
 * \param[out] curData Returned pointer current asychronous data.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getCurrentAsyncData(
	Vn100* vn100,
	VnDeviceCompositeData* curData);

/**
 * \brief Commands the VectorNav unit to write its current register setting to
 * non-volatile memory.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_writeSettings(
	Vn100* vn100,
	bool waitForResponse);

/**
 * \brief Commands the VectorNav unit to revert its settings to factory defaults.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_restoreFactorySettings(
	Vn100* vn100,
	bool waitForResponse);

/**
 * \brief Commands the VectorNav module to reset itself.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_reset(
	Vn100* vn100);

/**
 * \brief Gets the values in the User Tag register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] userTagBuffer Buffer to store the response. Must have a length of at least 21 characters.
 * \param[in] userTagBufferLength Length of the provided userTagBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getUserTag(
	Vn100* vn100,
	char* userTagBuffer,
	uint32_t userTagBufferLength);

/**
 * \brief Sets the values of the User Tag register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] userTagData Array containg the data to send. Length must be equal to or less than 20 characters.
 * \param[in] userTagDataLength Length of the data to send in the userTagData array.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setUserTag(
	Vn100* vn100,
	char* userTagData,
	uint32_t userTagDataLength,
	bool waitForResponse);

/**
 * \brief Gets the values in the Model Number register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] modelBuffer Buffer to store the response. Must have a length of at least 25 characters.
 * \param[in] modelBufferLength Length of the provided modelBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getModelNumber(
	Vn100* vn100,
	char* modelBuffer,
	uint32_t modelBufferLength);

/**
 * \brief Gets the values in the Hardware Revision register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] hardwareRevision The hardware revision value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getHardwareRevision(
	Vn100* vn100,
	int32_t* hardwareRevision);

/**
 * \brief Gets the values in the Serial Number register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] serialNumberBuffer Buffer to store the response. Must have a length of at least 13 characters.
 * \param[in] serialNumberBufferLength Length of the provided serialNumberBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getSerialNumber(
	Vn100* vn100,
	char* serialNumberBuffer,
	uint32_t serialNumberBufferLength);

/**
 * \brief Gets the value in the Firmware Version register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] firmwareVersionBuffer Buffer to store the response. Must have a length of at least 16 characters.
 * \param[in] firmwareVersionBufferLength Length of the provided firmwareVersionBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getFirmwareVersion(
	Vn100* vn100,
	char* firmwareVersionBuffer,
	uint32_t firmwareVersionBufferLength);

/**
 * \brief Gets the values in the Serial Baud Rate register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] serialBaudrate The serial baudrate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getSerialBaudRate(
	Vn100* vn100,
	uint32_t* serialBaudrate);

/**
 * \brief Sets the values of the Serial Baud Rate register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] serialBaudrate Value for the serial baudrate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setSerialBaudRate(
	Vn100* vn100,
	uint32_t serialBaudrate,
	bool waitForResponse);

/**
 * \brief Gets the values in the Asynchronous Data Output Type register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] asyncDataOutputType The asynchronous data output type value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getAsynchronousDataOutputType(
	Vn100* vn100,
	uint32_t* asyncDataOutputType);

/**
 * \brief Sets the values of the Asynchronous Data Output Type register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] asyncDataOutputType Value for the asynchronous data output type field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setAsynchronousDataOutputType(
	Vn100* vn100,
	uint32_t asyncDataOutputType,
	bool waitForResponse);

/**
 * \brief Gets the values in the Asynchronous Data Output Frequency register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] asyncDataOutputFrequency The asynchronous data output frequency value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getAsynchronousDataOutputFrequency(
	Vn100* vn100,
	uint32_t* asyncDataOutputFrequency);

/**
 * \brief Sets the values of the Asynchronous Data Output Frequency register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] asyncDataOutputFrequency Value for the asynchronous data output frequency field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setAsynchronousDataOutputFrequency(
	Vn100* vn100,
	uint32_t asyncDataOutputFrequency,
	bool waitForResponse);

/**
 * \brief Gets the values in the Yaw Pitch Roll register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getYawPitchRoll(
	Vn100* vn100,
	VnYpr* attitude);

/**
 * \brief Gets the values in the Attitude Quaternion register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getQuaternion(
	Vn100* vn100,
	VnQuaternion* attitude);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, Magentic, Accleration, and Angular Rates register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor uncompensated angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getYawPitchRollMagneticAccelerationAngularRate(
	Vn100* vn100,
	VnYpr* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Quaternion, Magnetic, Acceleration and Angular
 * Rates register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getQuaternionMagneticAccelerationAngularRate(
	Vn100* vn100,
	VnQuaternion* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Magnetic Measurements register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getMagnetic(
	Vn100* vn100,
	VnVector3* magnetic);

/**
 * \brief Gets the values in the Acceleration Measurements register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getAcceleration(
	Vn100* vn100,
	VnVector3* acceleration);

/**
 * \brief Gets the values in the Angular Rate Measurements register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getAngularRate(
	Vn100* vn100,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Magnetic, Acceleration and Angular Rates register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getMagneticAccelerationAngularRate(
	Vn100* vn100,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, True Body Acceleration and Angular Rates register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] bodyAcceleration The current sensor body acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getYawPitchRollTrueBodyAccelerationAngularRate(
	Vn100* vn100,
	VnYpr* attitude,
	VnVector3* bodyAcceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, True Inertial Acceleration and Angular Rates register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] inertialAcceleration The current sensor inertial acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getYawPitchRollTrueInertialAccelerationAngularRate(
	Vn100* vn100,
	VnYpr* attitude,
	VnVector3* inertialAcceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the VPE Basic Control register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] enable The enable/disable value of the sensor.
 * \param[out] headingMode The heading mode value of the sensor.
 * \param[out] filteringMode The filtering mode value of the sensor.
 * \param[out] tuningMode The tuning mode value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getVpeControl(
	Vn100* vn100,
	uint8_t* enable,
	uint8_t* headingMode,
	uint8_t* filteringMode,
	uint8_t* tuningMode);

/**
 * \brief Sets the values of the VPE Basic Control register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] enable Value for the enable/disable field.
 * \param[in] headingMode Value for the heading mode field.
 * \param[in] filteringMode Value for the filtering mode field.
 * \param[in] tuningMode Value for the tuning mode field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setVpeControl(
	Vn100* vn100,
	uint8_t enable,
	uint8_t headingMode,
	uint8_t filteringMode,
	uint8_t tuningMode,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Magnetometer Basic Tuning register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] baseTuning The current sensor magnetometer base tuning (X,Y,Z) values.
 * \param[out] adaptiveTuning The current sensor magnetometer adaptive tuning (X,Y,Z) values.
 * \param[out] adaptiveFiltering The current sensor magnetometer adaptive filtering (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getVpeMagnetometerBasicTuning(
	Vn100* vn100,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning,
	VnVector3* adaptiveFiltering);

/**
 * \brief Sets the values of the VPE Magnetometer Basic Tuning register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] baseTuning The magnetometer base tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveTuning The magnetometer adaptive tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveFiltering The magnetometer adaptive filtering (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setVpeMagnetometerBasicTuning(
	Vn100* vn100,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Accelerometer Basic Tuning register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] baseTuning The current sensor accelerometer base tuning (X,Y,Z) values.
 * \param[out] adaptiveTuning The current sensor accelerometer adaptive tuning (X,Y,Z) values.
 * \param[out] adaptiveFiltering The current sensor accelerometer adaptive filtering (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getVpeAccelerometerBasicTuning(
	Vn100* vn100,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning,
	VnVector3* adaptiveFiltering);

/**
 * \brief Sets the values of the VPE Accelerometer Basic Tuning register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] baseTuning The accelerometer base tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveTuning The accelerometer adaptive tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveFiltering The accelerometer adaptive filtering (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setVpeAccelerometerBasicTuning(
	Vn100* vn100,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering,
	bool waitForResponse);

/**
 * \brief Gets the values in the IMU Measurements register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] magnetic The current sensor uncompensated magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor uncompensated acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor uncompensated angular rate (X,Y,Z) values.
 * \param[out] temperature The temperature value of the sensor.
 * \param[out] pressure The pressure value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getImuMeasurements(
	Vn100* vn100,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate,
	float* temperature,
	float* pressure);

/**
 * \brief Gets the values in the Reference Frame Rotation register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] c The current sensor C matrix values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getReferenceFrameRotation(
	Vn100* vn100,
	VnMatrix3x3* c);

/**
 * \brief Sets the values of the Reference Frame Rotation register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setReferenceFrameRotation(
	Vn100* vn100,
	VnMatrix3x3 c,
	bool waitForResponse);

/**
 * \brief Gets the values in the Synchronization Control register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] syncInMode The input signal synchronization mode value of the sensor.
 * \param[out] syncInEdge The input signal synchronization edge selection value of the sensor.
 * \param[out] syncInSkipFactor The input signal trigger skip factor value of the sensor.
 * \param[out] syncOutMode The output signal synchronization mode value of the sensor.
 * \param[out] syncOutPolarity The output signal synchronization polarity value of the sensor.
 * \param[out] syncOutSkipFactor The output synchronization signal skip factor value of the sensor.
 * \param[out] syncOutPulseWidth The output synchronization signal pulse width value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getSynchronizationControl(
	Vn100* vn100,
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
 * \param[in] vn100 Pointer to the Vn100 control object.
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
DLL_EXPORT VN_ERROR_CODE vn100_setSynchronizationControl(
	Vn100* vn100,
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
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] syncInCount The synchronization in count value of the sensor.
 * \param[out] syncInTime The synchronization in time value of the sensor.
 * \param[out] syncOutCount The synchronization out count value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getSynchronizationStatus(
	Vn100* vn100,
	uint32_t* syncInCount,
	uint32_t* syncInTime,
	uint32_t* syncOutCount);

/**
 * \brief Sets the values of the Synchronization Status register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] syncInCount Value for the synchronization in count field.
 * \param[in] syncInTime Value for the synchronization in time field.
 * \param[in] syncOutCount Value for the synchronization out count field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setSynchronizationStatus(
	Vn100* vn100,
	uint32_t syncInCount,
	uint32_t syncInTime,
	uint32_t syncOutCount,
	bool waitForResponse);

/**
 * \brief Gets the contents of the Delta Theta and Delta Velocity register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] deltaTime Delta time for the integration interval.
 * \param[out] deltaTheta Delta rotation vector.
 * \param[out] deltaVelocity Delta velocity vector.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getDeltaThetaAndDeltaVelocity(
	Vn100* vn100,
	float* deltaTime,
	VnVector3* deltaTheta,
	VnVector3* deltaVelocity);

/**
 * \brief Gets the values in the Acceleration Compensation register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getAccelerationCompensation(
	Vn100* vn100,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Acceleration Compensation register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setAccelerationCompensation(
	Vn100* vn100,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Gets the values in the Magnetic Compensation register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getMagneticCompensation(
	Vn100* vn100,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Magnetic Compensation register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setMagneticCompensation(
	Vn100* vn100,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Gets the values in the Gyro Compensation register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getGyroCompensation(
	Vn100* vn100,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Gyro Compensation register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setGyroCompensation(
	Vn100* vn100,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Retreives the current values of the IMU Filtering Configuration register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
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
DLL_EXPORT VN_ERROR_CODE vn100_getImuFilteringConfiguration(
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
	uint8_t* presFilterMode);

/**
 * \brief Sets the values of the IMU Filtering Configuration register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
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
DLL_EXPORT VN_ERROR_CODE vn100_setImuFilteringConfiguration(
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
	bool waitForResponse);

/**
 * \brief Retreives the current values of the Delta Theta and Delta Velocity Configuration register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] integrationFrame Output frame for delta velocity quantities.
 * \param[out] gyroCompensation Compensation to apply to angular rate.
 * \param[out] accelCompensation Compensation to apply to accelerations.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getDeltaThetaAndDeltaVelocityConfiguration(
	Vn100* vn100,
	uint8_t* integrationFrame,
	uint8_t* gyroCompensation,
	uint8_t* accelCompensation);

/**
 * \brief Sets the values of the Delta Theta and Delta Velocity Configuration register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] integrationFrame Output frame for delta velocity quantities.
 * \param[in] gyroCompensation Compensation to apply to angular rate.
 * \param[in] accelCompensation Compensation to apply to accelerations.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setDeltaThetaAndDeltaVelocityConfiguration(
	Vn100* vn100,
	uint8_t integrationFrame,
	uint8_t gyroCompensation,
	uint8_t accelCompensation,
	bool waitForResponse);

/**
 * \brief Gets the values in the Magnetometer Calibration Control register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] hsiMode The HSIMode value of the sensor.
 * \param[out] hsiOutput The HSIOutput value of the sensor.
 * \param[out] convergeRate The ConvergeRate value of the sensor.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getMagnetometerCalibrationControl(
	Vn100* vn100,
	uint8_t* hsiMode,
	uint8_t* hsiOutput,
	uint8_t* convergeRate);

/**
 * \brief Sets the values of the Magnetometer Calibration Control register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] hsiMode Value for the HSIMode field.
 * \param[in] hsiOutput Value for the HSIOutput field.
 * \param[in] convergeRate Value for the ConvergeRate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setMagnetometerCalibrationControl(
	Vn100* vn100,
	uint8_t hsiMode,
	uint8_t hsiOutput,
	uint8_t convergeRate,
	bool waitForResponse);

/**
 * \brief Gets the values in the Calculated Magnetometer Calibration register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getCalculatedMagnetometerCalibration(
	Vn100* vn100,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Gets the values in the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] magneticReference The current sensor magnetic reference vector (X,Y,Z) values.
 * \param[out] gravityReference The current sensor gravity reference vector (X,Y,Z) values.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getMagneticGravityReferenceVectors(
	Vn100* vn100,
	VnVector3* magneticReference,
	VnVector3* gravityReference);

/**
 * \brief Sets the values of the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] magneticReference The magnetic reference vector (X,Y,Z) values to write to the sensor.
 * \param[in] gravityReference The gravity reference vector (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_setMagneticGravityReferenceVectors(
	Vn100* vn100,
	VnVector3 magneticReference,
	VnVector3 gravityReference,
	bool waitForResponse);

/**
 * \brief Gets the values in the Communication Protocol Control register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
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
DLL_EXPORT VN_ERROR_CODE vn100_getCommunicationProtocolControl(
	Vn100* vn100,
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
 * \param[in] vn100 Pointer to the Vn100 control object.
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
DLL_EXPORT VN_ERROR_CODE vn100_setCommunicationProtocolControl(
	Vn100* vn100,
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
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[out] useMagModel The UseMagModel field.
 * \param[out] useGravityModel The UseGravityModel field.
 * \param[out] recalcThreshold The RecalcThreshold field.
 * \param[out] year The Year field.
 * \param[out] lla The Lattitude, Longitude, Altitude fields.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_getReferenceVectorConfiguration(
	Vn100* vn100,
	uint8_t* useMagModel,
	uint8_t* useGravityModel,
	uint32_t* recalcThreshold,
	float* year,
	VnVector3* lla);

/**
 * \brief Sets the values in the Reference Vector Configuration register.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
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
DLL_EXPORT VN_ERROR_CODE vn100_setReferenceVectorConfiguration(
	Vn100* vn100,
	uint8_t useMagModel,
	uint8_t useGravityModel,
	uint32_t recalcThreshold,
	float year,
	VnVector3 lla,
	bool waitForResponse);

/**
 * \brief Pauses the asynchronous data output.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_pauseAsyncOutputs(
	Vn100* vn100,
	bool waitForResponse);

/**
 * \brief Resumes the asynchronous data output.
 *
 * \param[in] vn100 Pointer to the Vn100 control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn100_resumeAsyncOutputs(
	Vn100* vn100,
	bool waitForResponse);

#ifdef __cplusplus
}
#endif

#endif /* _VN100_H_ */
