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
 * This header file provides access to common functionality amongst VectorNav
 * devices.
 */
#ifndef _VNDEVICE_H_
#define _VNDEVICE_H_

/* Disable some unnecessary warnings for compiling using Visual Studio with -Wall. */
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
 * @defgroup ASYNC_OUTPUTS Asynchronous Output Types
 *
 * @{
 */
#define VNASYNC_OFF		0		/**< Asynchronous output is turned off. */
#define VNASYNC_VNYPR	1		/**< Asynchronous output type is Yaw, Pitch, Roll. */
#define VNASYNC_VNQTN	2		/**< Asynchronous output type is Quaternion. */
#define VNASYNC_VNQTM	3		/**< Asynchronous output type is Quaternion and Magnetic. */
#define VNASYNC_VNQTA	4		/**< Asynchronous output type is Quaternion and Acceleration. */
#define VNASYNC_VNQTR	5		/**< Asynchronous output type is Quaternion and Angular Rates. */
#define VNASYNC_VNQMA	6		/**< Asynchronous output type is Quaternion, Magnetic and Acceleration. */
#define VNASYNC_VNQAR	7		/**< Asynchronous output type is Quaternion, Acceleration and Angular Rates. */
#define VNASYNC_VNQMR	8		/**< Asynchronous output type is Quaternion, Magnetic, Acceleration and Angular Rates. */
#define VNASYNC_VNDCM	9		/**< Asynchronous output type is Directional Cosine Orientation Matrix. */
#define VNASYNC_VNMAG	10		/**< Asynchronous output type is Magnetic Measurements. */
#define VNASYNC_VNACC	11		/**< Asynchronous output type is Acceleration Measurements. */
#define VNASYNC_VNGYR	12		/**< Asynchronous output type is Angular Rate Measurements. */
#define VNASYNC_VNMAR	13		/**< Asynchronous output type is Magnetic, Acceleration, and Angular Rate Measurements. */
#define VNASYNC_VNYMR	14		/**< Asynchronous output type is Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements. */
#define VNASYNC_VNYCM	15		/**< Asynchronous output type is Yaw, Pitch, Roll, and Calibrated Measurements. */
#define VNASYNC_VNYBA	16		/**< Asynchronous output type is Yaw, Pitch, Roll, Body True Acceleration. */
#define VNASYNC_VNYIA	17		/**< Asynchronous output type is Yaw, Pitch, Roll, Inertial True Acceleration. */
#define VNASYNC_VNICM	18		/**< Asynchronous output type is Yaw, Pitch, Roll, Inertial Magnetic/Acceleration, and Angular Rate Measurements. */
#define VNASYNC_VNIMU	19		/**< Asynchronous output type is Calibrated Interial Measurements. */
#define VNASYNC_VNGPS	20		/**< Asynchronous output type is GPS Measurements. */
#define VNASYNC_VNINS	22		/**< Asynchronous output type is INS Solution. */
#define VNASYNC_VNRAW	252		/**< Asynchronous output type is Raw Voltage Measurements. */
#define VNASYNC_VNCMV	253		/**< Asynchronous output type is Calibrated Measurements. */
#define VNASYNC_VNSTV	254		/**< Asynchronous output type is Kalman Filter State Vector. */
#define VNASYNC_VNCOV	255		/**< Asynchronous output type is Kalman Filter Covariance Matrix Diagonal. */
/** @} */

/**
 * @defgroup BINARY_ASYNC_MODES Binary Asynchronous Modes
 *
 * @{
 */
#define BINARY_ASYNC_MODE_NONE				0	/**< No output of binary async data. */
#define BINARY_ASYNC_MODE_SERIAL_1			1	/**< Binary messages are sent out serial port 1. */
#define BINARY_ASYNC_MODE_SERIAL_2			2	/**< Binary messages are sent out serial port 2. */
#define BINARY_ASYNC_MODE_SERIAL_1_AND_2	3	/**< Binary messages are sent out serial ports 1 and 2. */
/** @} */

/**
 * @defgroup BG1_SELECTIONS Binary Group 1 Selections
 *
 * @{
 */
#define BG1_NONE			0x0000	/**< No ouput. */
#define BG1_TIME_STARTUP	0x0001	/**< Time since startup. */
#define BG1_TIME_GPS		0x0002	/**< GPS time. */
#define BG1_TIME_SYNC_IN	0x0004	/**< Time since the last SyncIn trigger. */
#define BG1_YPR				0x0008	/**< Estimated attitude as yaw, pitch, roll in degrees. */
#define BG1_QTN				0x0010	/**< Estimated attitude as a quaternion. */
#define BG1_ANGULAR_RATE	0x0020	/**< Compensated angular rate. */
#define BG1_POSITION		0x0040	/**< Estimated position given as latitude, longitude and altitude. */
#define BG1_VELOCITY		0x0080	/**< Estimated velocity. */
#define BG1_ACCEL			0x0100	/**< Estimated acceleration. */
#define BG1_IMU				0x0200	/**< Calibrated uncompensated angular rate and acceleration measurements. */
#define BG1_MAG_PRES		0x0400	/**< Calibrated magnetic, temperature and pressure measurements. */
#define BG1_DELTA_THETA		0x0800	/**< Delta time, theta and velocity. */
#define BG1_INS_STATUS		0x1000	/**< INS status. */
#define BG1_SYNC_IN_CNT		0x2000	/**< SyncIn count. */
/** @} */

/**
 * @defgroup BG2_SELECTIONS Binary Group 2 Selections
 *
 * @{
 */
#define BG2_NONE			0x0000	/**< No output. */
#define BG2_TIME_STARTUP	0x0001	/**< Time since startup. */
#define BG2_TIME_GPS		0x0002	/**< Absolute GPS time. */
#define BG2_GPS_TOW			0x0004	/**< Time since start of GPS week. */
#define BG2_GPS_WEEK		0x0008	/**< GPS week. */
#define BG2_TIME_SYNC_IN	0x0010	/**< Time since the last SyncIn trigger. */
#define BG2_TIME_PPS		0x0020	/**< Time since the last GPS PPS trigger. */
#define BG2_TIME_UTC		0x0040	/**< UTC time. */
#define BG2_SYNC_IN_CNT		0x0080	/**< SyncIn trigger count. */
/** @} */

/**
 * @defgroup BG3_SELECTIONS Binary Group 3 Selections
 *
 * @{
 */
#define BG3_NONE			0x0000	/**< No output. */
#define BG3_UNCOMP_MAG		0x0002	/**< Uncompensated magnetic measurements. */
#define BG3_UNCOMP_ACCEL	0x0004	/**< Uncompensated acceleration measurements. */
#define BG3_UNCOMP_GYRO		0x0008	/**< Uncompensated angular rate measurements. */
#define BG3_TEMP			0x0010	/**< Temperature measurements. */
#define BG3_PRES			0x0020	/**< Pressure measurements. */
#define BG3_DELTA_THETA		0x0040	/**< Delta theta angles. */
#define BG3_DELTA_V			0x0080	/**< Delta velocity. */
#define BG3_MAG				0x0100	/**< Compensated magnetic measurements. */
#define BG3_ACCEL			0x0200	/**< Compensated acceleration measurements. */
#define BG3_GYRO			0x0400	/**< Compensated angular rate measurements. */
#define BG3_SENS_SAT		0x0800	/**< Sensor saturation bit field. */
/** @} */

/**
 * @defgroup BG4_SELECTIONS Binary Group 4 Selections
 *
 * @{
 */
#define BG4_NONE			0x0000	/**< No output. */
#define BG4_UTC				0x0001	/**< GPS UTC time. */
#define BG4_TOW				0x0002	/**< GPS time of week. */
#define BG4_WEEK			0x0004	/**< GPS week. */
#define BG4_NUM_SATS		0x0008	/**< Number of tracked satellites. */
#define BG4_FIX				0x0010	/**< GPS fix. */
#define BG4_POS_LLA			0x0020	/**< GPS position in latitude, longitude and altitude. */
#define BG4_POS_ECEF		0x0040	/**< GPS position in Earth-Centered, Earth-Fixed (ECEF) frame. */
#define BG4_VEL_NED			0x0080	/**< GPS velocity in North, East, Down (NED) frame. */
#define BG4_VEL_ECEF		0x0100	/**< GPS velocity in Earth-Centered, Earth-Fixed (ECEF) frame. */
#define BG4_POS_U			0x0200	/**< GPS position uncertainty in North, East, Down (NED) frame. */
#define BG4_VEL_U			0x0400	/**< GPS velocity uncertainty. */
#define BG4_TIME_U			0x0800	/**< GPS time uncertainty. */
/** @} */

/**
 * @defgroup BG5_SELECTIONS Binary Group 5 Selections
 *
 * @{
 */
#define BG5_NONE				0x0000	/**< No output. */
#define BG5_VPE_STATUS			0x0001	/**< VPE status. */
#define BG5_YPR					0x0002	/**< Yaw, pitch, roll. */
#define BG5_QUATERNION			0x0004	/**< Quaternion. */
#define BG5_DCM					0x0008	/**< Directon cosine matrix. */
#define BG5_MAG_NED				0x0010	/**< Compensated magnetic in North, East, Down (NED) frame. */
#define BG5_ACCEL_NED			0x0020	/**< Compensated acceleration in North, East, Down (NED) frame. */
#define BG5_LINEAR_ACCEL_BODY	0x0040	/**< Compensated linear acceleration (no gravity) in the body frame. */
#define BG5_LINEAR_ACCEL_NED	0x0080	/**< Compensated linear acceleration (no gravity) in the North, East, Down (NED) frame. */
#define BG5_YPR_U				0x0100	/**< Yaw, pitch, roll uncertainty. */
/** @} */

/**
 * @defgroup BG6_SELECTIONS Binary Group 6 Selections
 *
 * @{
 */
#define BG6_NONE				0x0000	/**< No output. */
#define BG6_INS_STATUS			0x0001	/**< INS status. */
#define BG6_POS_LLA				0x0002	/**< INS position in latitude, longitude and altitude. */
#define BG6_POS_ECEF			0x0004	/**< INS position in Earth-Centered, Earth-Fixed (ECEF) frame. */
#define BG6_VEL_BODY			0x0008	/**< INS velocity in body frame. */
#define BG6_VEL_NED				0x0010	/**< INS velocity in North, East, Down (NED) frame. */
#define BG6_VEL_ECEF			0x0020	/**< INS velocity in Earth-Centered, Earth-Fixed (ECEF) frame. */
#define BG6_MAG_ECEF			0x0040	/**< Compensated magnetic in Earth-Centered, Earth-Fixed (ECEF) frame. */
#define BG6_ACCEL_ECEF			0x0080	/**< Compensated acceleration in Earth-Centered, Earth-Fixed (ECEF) frame. */
#define BG6_LINEAR_ACCEL_ECEF	0x0100	/**< Compensated linear acceleration (no gravity) in Earth-Centered, Earth-Fixed (ECEF) frame. */
#define BG6_POS_U				0x0200	/**< INS position uncertainty. */
#define BG6_VEL_U				0x0400	/**< INS velocity uncertainty. */
/** @} */

/**
 * @defgroup Synchronization control register
 *
 * @{
 */
#define SYNCINMODE_COUNT            3
#define SYNCINMODE_IMU              4
#define SYNCINMODE_ASYNC            5
#define SYNCINEDGE_RISING           0
#define SYNCINEDGE_FALLING          1
#define SYNCOUTMODE_NONE            0
#define SYNCOUTMODE_IMU_START       1
#define SYNCOUTMODE_IMU_READY       2
#define SYNCOUTMODE_AHRS            3
#define SYNCOUTPOLARITY_NEGATIVE 0
#define SYNCOUTPOLARITY_POSITIVE 1
/** @} */

/**
 * @defgroup Communication protocal control register
 *
 * @{
 */
#define SERIALCOUNT_NONE            0
#define SERIALCOUNT_SYNCIN_COUNT    1
#define SERIALCOUNT_SYNCIN_TIME     2
#define SERIALCOUNT_SYNCOUT_COUNT   3
#define SERIALSTATUS_OFF            0
#define SERIALSTATUS_VPE            1
#define SPICOUNT_NONE               0
#define SPICOUNT_SYNCIN_COUNT       1
#define SPICOUNT_SYNCIN_TIME        2
#define SPICOUNT_SYNCOUT_COUNT      3
#define SPISTATUS_OFF               0
#define SPISTATUS_VPE               1
#define SERIALCHECKSUM_8BIT         1
#define SERIALCHECKSUM_16BIT        3
#define SPICHECKSUM_OFF             0
#define SPICHECKSUM_8BIT            1
#define SPICHECKSUM_16BIT           3
#define ERRORMODE_IGNORE            0
#define ERRORMODE_SEND              1
#define ERRORMODE_ADOR              2
/** @} */

#define VN_RESPONSE_MATCH_SIZE			10		/**< Size to match for the response. */
#define VN_MAX_RESPONSE_SIZE			256		/**< Maximum response size from sensor. */
#define VN_MAX_COMMAND_SIZE				256		/**< Maximum size of command to send to the sensor. */

/**
 * \brief Represents a UTC time.
 */
typedef struct {
	int8_t		year;	/**< Year. */
	uint8_t		month;	/**< Month. */
	uint8_t		day;	/**< Day. */
	uint8_t		hour;	/**< Hour. */
	uint8_t		min;	/**< Minute. */
	uint8_t		sec;	/**< Second. */
	uint16_t	ms;		/**< Millisecond. */
} UtcTime;

/* Disable some unecessary warnings for compiling using Visual Studio with -Wall. */
#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4820)	/* Padding added in structures warning. */
#endif

/**
 * \brief Composite structure of the various asynchronous data VectorNav devices
 * are capable of outputting.
 */
typedef struct {
	VnYpr			ypr;						/**< Yaw, pitch, roll. */
	VnQuaternion	quaternion;					/**< Quaternion. */
	VnVector3		magnetic;					/**< Magnetic measurements. */
	VnVector3		acceleration;				/**< Acceleration measurements. */
	VnVector3		angularRate;				/**< Angular rate / gyro measurements. */
	VnMatrix3x3		dcm;						/**< Direction cosine matrix. */
	double			temperature;				/**< Temperature. */
	VnVector3		magneticVoltage;			/**< Magnetic sensor voltages. */
	VnVector3		accelerationVoltage;		/**< Accleration sensor voltages. */
	VnVector3		angularRateVoltage;			/**< Angular rate sensor voltages. */
	double			temperatureVoltage;			/**< Temperatue sensor voltages. */
	VnVector3		angularRateBias;			/**< Angular rate estimated biases. */
	VnVector3		attitudeVariance;			/**< Variance for the computed attitude. */
	VnVector3		angularRateBiasVariance;	/**< Angular rate bias variance. */
	uint64_t		timeStartup;				/**< Time since startup. */
	uint64_t		timeGps;					/**< GPS time. */
	uint64_t		timeSyncIn;					/**< Time since the last SyncIn trigger. */
	VnVector3		latitudeLongitudeAltitude;	/**< The estimated latitude, longitude and altitude. */
	VnVector3		velocity;					/**< Velocity measurements. */
	VnVector3		angularRateUncompensated;	/**< Uncompensated angular rate measurements. */
	VnVector3		accelerationUncompensated;	/**< Uncompensated acceleration measurements. */
	VnVector3		magneticUncompensated;		/**< Uncompensated magnetic measurements. */
	double			pressure;					/**< Pressure measurements. */
	double			deltaTime;					/**< Time interval that the delta angles and velocities are integrated over. */
	VnVector3		deltaTheta;					/**< The delta rotation angles due to rotation since the values were last output by the device. */
	VnVector3		deltaVelocity;				/**< The delta velocity due to motion since the values were last output by the device. */
	uint16_t		insStatus;					/**< Status flags for the INS filter. */
	uint32_t		syncInCnt;					/**< Number of SyncIn trigger events. */
	uint64_t		timeGpsPps;					/**< Time since the last GPS PPS trigger event. */
	double			gpsTowSec;					/**< GPS time of week in seconds. */
	uint64_t		gpsTowNs;					/**< GPS time of week in nanoseconds. */
	uint16_t		gpsWeek;					/**< GPS week. */
	UtcTime			timeUtc;					/**< UTC time. */
	uint16_t		sensSat;					/**< Flags for identifying sensor saturation. */
	uint8_t			numSats;					/**< Number of tracked GPS satellites. */
	uint8_t			gpsFix;						/**< The current GPS fix. */
	VnVector3		gpsPosEcef;					/**< The current GPS position given in Earth-Centered, Earth-Fixed (ECEF) frame. */
	VnVector3		gpsVelEcef;					/**< The current GPS velocity in the Earth-Centered, Earth-Fixed (ECEF) frame. */
	VnVector3		gpsPosU;					/**< The current GPS position uncertainty in the North, East, Down (NED) frame. */
	double			gpsVelU;					/**< The current GPS velocity uncertainty in m/s. */
	uint32_t		timeU;						/**< The current GPS time uncertainty given in nanoseconds. */
	float			timeAccSec;					/**< The current GPS time accuracy given in seconds. */
	uint16_t		vpeStatus;					/**< The VPE status bitfield. */
	VnVector3		magNed;						/**< The estimated magnetic field in North, East, Down (NED) frame. */
	VnVector3		accelNed;					/**< The estimated acceleration (with gravity) given in the North, East, Down (NED) frame. */
	VnVector3		linearAccelBody;			/**< The estimated linear acceleration (without gravity) given in the body frame. */
	VnVector3		linearAccelNed;				/**< The estimated linear acceleration (without gravity) givein in the North, East, Down (NED) frame. */
	VnVector3		yprU;						/**< The estimated attitude (Yaw, Pitch, Roll) uncertainty. */
	VnVector3		velBody;					/**< The estimated velocity in the body frame. */
	VnVector3		velNed;						/**< The estimated velocity in the North, East, Down (NED) frame. */
	VnVector3		gpsPosLla;					/**< The current GPS possition given in latitude, longitude, and altitude. */
	VnVector3		gpsVelocity;				/**< The current GPS velocity. */
	VnVector3		posEcef;					/**< The estimated position given in Earth-Centered, Earth-Fixed (ECEF) frame. */
	VnVector3		velEcef;					/**< The estimated velocity given in Earth-Centered, Earth-Fixed (ECEF) frame. */
	VnVector3		magEcef;					/**< The compensated magnetic measurement givein in Earth-Centered, Earth-Fixed (ECEF) frame. */
	VnVector3		accelEcef;					/**< The estimated acceleration (with gravity) givin in Earth-Centered, Earth-Fixed (ECEF) frame. */
	VnVector3		linearAccelEcef;			/**< The estimated linear acceleration (without gravity) givin in Earth-Centered, Earth-Fixed (ECEF) frame. */
	double			posU;						/**< The estimated uncertainty of the current position estimate in meters. */
	double			velU;						/**< The estimated uncertainty of the current velocity estimate in m/s. */
	float			attitudeUncertainty;		/**< Uncertainty in attitude estimate. */
} VnDeviceCompositeData;

#if defined(_MSC_VER)
	#pragma warning(pop)
#endif

/**
 * \brief Fuction type used for receiving notifications of when new asynchronous
 * data is received.
 *
 * \param[in]	sender	The device that sent the notification.
 * \param[in]	newData	Pointer to the new data.
 */
typedef void (*VnDeviceNewAsyncDataReceivedListener)(void* sender, VnDeviceCompositeData* newData);

/**
 * \brief Function type used for receiving notification of errors received from the sensor.
 *
 * \param[in] sender The device that sent the notification.
 * \param[in] errorCode The error code received from the sensor converted into the library's VN_ERROR_CODE format.
 */
typedef void (*VnDeviceErrorCodeReceivedListener) (void* sender, VN_ERROR_CODE errorCode);

/* Disable some unecessary warnings for compiling using Visual Studio with -Wall. */
#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4820)	/* Padding added in structures warning. */
#endif

/**
 * Internally used data structure for the VectorNav object.
 */
typedef struct {

	/**
	 * Handle to the comPortServiceThread.
	 */
	VN_HANDLE				comPortServiceThreadHandle;

	/**
	 * Handle to the COM port.
	 */
	VN_HANDLE				comPortHandle;

	VN_HANDLE				waitForThreadToStopServicingComPortEvent;
	VN_HANDLE				waitForThreadToStartServicingComPortEvent;

	/**
	 * Used by the user thread to wait until the comPortServiceThread receives
	 * a command response.
	 */
	VN_HANDLE				waitForCommandResponseEvent;

	/**
	 * Critical section for communicating over the COM port.
	 */
	VN_CRITICAL_SECTION		critSecForComPort;

	/**
	 * Critical section for accessing the response match fields.
	 */
	VN_CRITICAL_SECTION		critSecForResponseMatchAccess;

	/**
	 * Critical section for accessing the latestAsyncData field.
	 */
	VN_CRITICAL_SECTION		critSecForLatestAsyncDataAccess;

	/**
	 * Signals to the comPortServiceThread if it should continue servicing the
	 * COM port.
	 */
	bool					continueServicingComPort;

	/**
	 * This field is used to signal to the comPortServiceThread that it should
	 * be checking to a command response comming from the VN-100 device. The
	 * user thread can toggle this field after setting the cmdResponseMatch
	 * field so the comPortServiceThread differeniate between the various
	 * output data of the VN-100. This field should only be accessed in the
	 * functions vn100_shouldCheckForResponse_threadSafe,
	 * vn100_enableResponseChecking_threadSafe, and
	 * vn100_disableResponseChecking_threadSafe.
	 */
	bool					checkForResponse;

	/**
	 * This field contains the string the comPortServiceThread will use to
	 * check if a data packet from the VN-100 is a match for the command sent.
	 * This field should only be accessed in the functions
	 * vn100_shouldCheckForResponse_threadSafe, vn100_enableResponseChecking_threadSafe,
	 * and vn100_disableResponseChecking_threadSafe.
	 */
	char					cmdResponseMatchBuffer[VN_RESPONSE_MATCH_SIZE + 1];

	/**
	 * This field is used by the comPortServiceThread to place responses
	 * received from commands sent to the VN-100 device. The responses
	 * placed in this buffer will be null-terminated and of the form
	 * "$VNRRG,1,VN-100" where the checksum is stripped since this will
	 * have already been checked by the thread comPortServiceThread.
	 */
	char					cmdResponseBuffer[VN_MAX_RESPONSE_SIZE + 1];

	VnDeviceCompositeData	lastestAsyncData;

	/**
	 * This field specifies the number of milliseconds to wait for a response
	 * from sensor before timing out.
	 */
	int						timeout;

	/**
	 * Holds pointer to a listener for async data recieved.
	 */
	VnDeviceNewAsyncDataReceivedListener asyncDataListener;

	/**
	 * Holds any error code received from the sensor.
	 */
	VN_ERROR_CODE sensorError;

	/**
	 * Holds pointer to a listener for error codes received from the VectorNav module.
	 */
	VnDeviceErrorCodeReceivedListener errorCodeListener;

	/**
	 * Mask for the end user on what the public struct for this device.
	 */
	void* deviceMask;

} VnDevice;

#if defined(_MSC_VER)
	#pragma warning(pop)
#endif

/* Function declarations. ****************************************************/

/**
 * \brief Performs a send command and then receive response transaction.
 *
 * Takes a command of the form "$VNRRG,1" and transmits it to the VectorNav device.
 * The function will then wait until the response is received. The response
 * will be located in the VnDevice->cmdResponseBuffer field and will be
 * null-terminated.
 *
 * \param[in]	vndevice	Pointer to the VnDevice control object.
 *
 * \param[in]	responseMatch
 * Null-terminated string which will be used by the comPortServiceThread to
 * determine if a received data packet is a match for the command sent.
 *
 * \param[in]	cmdToSend
 * Pointer to the command data to transmit to the VectorNav device. Should be
 * null-terminated.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_transaction(
	VnDevice* vndevice,
	const char* cmdToSend,
	const char* responseMatch);


/**
 * \brief Allows registering a function which will be called whenever an error
 * code is received from the VectorNav module.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] listener The function pointer to be called when an error code
 *     is received.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_registerErrorCodeReceivedListener(
	VnDevice* vndevice,
	VnDeviceErrorCodeReceivedListener listener);

/**
 * \brief Unregisters an already registered function for recieving notifications
 * of when error codes from the VectorNav module are received.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] listener The function pointer to unregister.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_unregisterErrorCodeReceivedListener(
	VnDevice* vndevice,
	VnDeviceErrorCodeReceivedListener listener);

/**
 * \brief Computes the CRC8 checksum for the provided command.
 *
 * \param[in]  cmdToCheck Null-terminated string of the form "VNRRG,1".
 *
 * \return The computed checksum number.
 */
DLL_EXPORT uint8_t vndevice_checksum_computeCrc8FromCommand(
	const char* cmdToCheck);

/**
 * \brief Computes the CRC16 checksum for the provided command.
 *
 * \param[in]  data The array of data to perform the CRC16 checksum on.
 * \param[in]  length The number of bytes in the data array to compute the
 *             checksum over.
 *
 * \return The computed checksum number.
 */
DLL_EXPORT uint16_t vndevice_checksum_computeCrc16(
	const char data[],
	uint32_t length);

/**
 * \brief Computes the CRC16 checksum for the provided command.
 *
 * \param[in]  cmdToCheck Null-terminated string of the form "VNRRG,1".
 *
 * \return The computed checksum number.
 */
DLL_EXPORT uint16_t vndevice_checksum_computeCrc16FromCommand(
	const char* cmdToCheck);

/**
 * \brief Computes the checksum for the provided command and returns it as a
 * two character string representing it in hexidecimal.
 *
 * \param[in]	cmdToCheck
 * Null-terminated string of the form "VNRRG,1".
 *
 * \param[out]	checksum
 * A character array of length 2 which the computed checksum will be placed.
 */
DLL_EXPORT void vndevice_checksum_computeAndReturnAsHex(
	const char* cmdToCheck,
	char* checksum);

/**
 * \brief Computes the expected length of a received binary packet.
 *
 * \param[in] groupIndex The current index of the group field settings.
 * \param[in] groupField The value of the group field.
 *
 * \return Unknown.
 */
int vndevice_computeLengthOfBinaryGroupPayload(
	unsigned char groupIndex,
	uint16_t groupField);

/**
 * \brief Process a received and verified binary packet.
 *
 * \param[in] vndevice Pointer to the VnDevice object.
 * \param[in] buffer Pointer to the buffer containing the start of the received binary packet.
 */
void vndevice_processReceivedBinaryPacket(
	VnDevice* vndevice,
	char* buffer);

/**
 * \brief Provides initialization to the VnDevice data structure.
 *
 * \param[in] vndevice Pointer to an uninitialized VnDevice structure.
 * \param[in] portName The name of the port to connect to.
 * \param[in] baudrate The baudrate to connect at.
 * \param[in] deviceMask Used to mimic the top-level device structure (i.e. Vn100 or Vn200).
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_initializeVnDevice(
	VnDevice* vndevice,
	const char* portName,
	int baudrate,
	void* deviceMask);

/**
 * \brief Deinitalized the VnDevice object.
 *
 * \param[in] vndevice Pointer to the VnDevice structure to deinitialize.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_deinitializeVnDevice(
	VnDevice* vndevice);

/**
 * \brief Allows registering a function which will be called whenever a new
 * asynchronous data packet is received from the VectorNav module.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] listener The function pointer to be called when asynchronous data
 *     is received.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_registerAsyncDataReceivedListener(
	VnDevice* vndevice,
	VnDeviceNewAsyncDataReceivedListener listener);

/**
 * \brief Unregisters an already registered function for recieving notifications
 * of when new asynchronous data is received.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] listener The function pointer to unregister.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_unregisterAsyncDataReceivedListener(
	VnDevice* vndevice,
	VnDeviceNewAsyncDataReceivedListener listener);

/**
 * \brief Commands the VnDevice control object to start processing communcation.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_startHandlingCommunication(
	VnDevice* vndevice);

/**
 * \brief Waits for the communication handler thread to start processing data.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_waitForThreadToStartHandlingCommunicationPort(
	VnDevice* vndevice);

/**
 * \brief Writes out the provided command to the COM port.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] cmdToSend Pointer to the command to send out.
 *
 * \return VectorNav error code.
 */
VN_ERROR_CODE vndevice_writeOutCommand(
	VnDevice* vndevice,
	const char* cmdToSend);

/**
 * \brief Retrieves a pointer to the VnDevice response buffer.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 *
 * \return The pointer to the response buffer.
 */
char* vndevice_getResponseBuffer(
	VnDevice* vndevice);

/**
 * \brief Gets the current configuration of the requested binary output register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
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
DLL_EXPORT VN_ERROR_CODE vndevice_getBinaryOutputConfiguration(
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
	uint16_t* outputGroup6Selections);

/**
 * \brief Sets the configuration of the requested binary output register. Note
 * that you do not have to provide the selected output groups option since this
 * will be determined from the provided configurations for the individual output
 * groups.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
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
	bool waitForResponse);


/**
 * \brief Retrieves the associated timeout value for the VnDevice object.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 *
 * \return The timeout value in milliseconds. -1 indicates that timeouts are
 * not used.
 */
DLL_EXPORT int vndevice_get_timeout(
	VnDevice* vndevice);

/**
 * \brief Sets the timeout value for the reading values from the VectorNav sensor.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] timeout The timeout value in milliseconds. Specify -1 to not use
 * any timeouts.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_set_timeout(
	VnDevice* vndevice,
	int timeout);

/**
 * \brief Retrieves the most recent stored asynchronous data.
 *
 * \param[in] vndevice Pointer to the VnDevice object.
 * \param[out] curData Returned pointer current asychronous data.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getCurrentAsyncData(
	VnDevice* vndevice,
	VnDeviceCompositeData* curData);

/**
 * \brief Commands the VectorNav unit to write its current register setting to
 * non-volatile memory.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_writeSettings(
	VnDevice* vndevice,
	bool waitForResponse);

/**
 * \brief Commands the VectorNav unit to revert its settings to factory defaults.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_restoreFactorySettings(
	VnDevice* vndevice,
	bool waitForResponse);

/**
 * \brief Commands the VectorNav module to reset itself.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_reset(
	VnDevice* vndevice);

/**
 * \brief Gets the values in the User Tag register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] userTagBuffer Buffer to store the response. Must have a length of at least 21 characters.
 * \param[in] userTagBufferLength Length of the provided userTagBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getUserTag(
	VnDevice* vndevice,
	char* userTagBuffer,
	uint32_t userTagBufferLength);

/**
 * \brief Sets the values of the User Tag register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] userTagData Array containg the data to send. Length must be equal to or less than 20 characters.
 * \param[in] userTagDataLength Length of the data to send in the userTagData array.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setUserTag(
	VnDevice* vndevice,
	char* userTagData,
	uint32_t userTagDataLength,
	bool waitForResponse);

/**
 * \brief Gets the values in the Model Number register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] modelBuffer Buffer to store the response. Must have a length of at least 25 characters.
 * \param[in] modelBufferLength Length of the provided modelBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getModelNumber(
	VnDevice* vndevice,
	char* modelBuffer,
	uint32_t modelBufferLength);

/**
 * \brief Gets the values in the Hardware Revision register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] hardwareRevision The hardware revision value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getHardwareRevision(
	VnDevice* vndevice,
	int32_t* hardwareRevision);

/**
 * \brief Gets the values in the Serial Number register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] serialNumberBuffer Buffer to store the response. Must have a length of at least 13 characters.
 * \param[in] serialNumberBufferLength Length of the provided serialNumberBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getSerialNumber(
	VnDevice* vndevice,
	char* serialNumberBuffer,
	uint32_t serialNumberBufferLength);

/**
 * \brief Gets the value in the Firmware Version register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] firmwareVersionBuffer Buffer to store the response. Must have a length of at least 16 characters.
 * \param[in] firmwareVersionBufferLength Length of the provided firmwareVersionBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getFirmwareVersion(
	VnDevice* vndevice,
	char* firmwareVersionBuffer,
	uint32_t firmwareVersionBufferLength);

/**
 * \brief Gets the values in the Serial Baud Rate register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] serialBaudrate The serial baudrate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getSerialBaudRate(
	VnDevice* vndevice,
	uint32_t* serialBaudrate);

/**
 * \brief Sets the values of the Serial Baud Rate register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] serialBaudrate Value for the serial baudrate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setSerialBaudRate(
	VnDevice* vndevice,
	uint32_t serialBaudrate,
	bool waitForResponse);

/**
 * \brief Gets the values in the Asynchronous Data Output Type register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] asyncDataOutputType The asynchronous data output type value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getAsynchronousDataOutputType(
	VnDevice* vndevice,
	uint32_t* asyncDataOutputType);

/**
 * \brief Sets the values of the Asynchronous Data Output Type register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] asyncDataOutputType Value for the asynchronous data output type field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setAsynchronousDataOutputType(
	VnDevice* vndevice,
	uint32_t asyncDataOutputType,
	bool waitForResponse);

/**
 * \brief Gets the values in the Asynchronous Data Output Frequency register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] asyncDataOutputFrequency The asynchronous data output frequency value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getAsynchronousDataOutputFrequency(
	VnDevice* vndevice,
	uint32_t* asyncDataOutputFrequency);

/**
 * \brief Sets the values of the Asynchronous Data Output Frequency register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] asyncDataOutputFrequency Value for the asynchronous data output frequency field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setAsynchronousDataOutputFrequency(
	VnDevice* vndevice,
	uint32_t asyncDataOutputFrequency,
	bool waitForResponse);

/**
 * \brief Gets the values in the Yaw Pitch Roll register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getYawPitchRoll(
	VnDevice* vndevice,
	VnYpr* attitude);

/**
 * \brief Gets the values in the Attitude Quaternion register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getQuaternion(
	VnDevice* vndevice,
	VnQuaternion* attitude);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, Magentic, Accleration, and Angular Rates register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor uncompensated angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getYawPitchRollMagneticAccelerationAngularRate(
	VnDevice* vndevice,
	VnYpr* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Quaternion, Magnetic, Acceleration and Angular
 * Rates register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] attitude The current sensor Quaterion values.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getQuaternionMagneticAccelerationAngularRate(
	VnDevice* vndevice,
	VnQuaternion* attitude,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Magnetic Measurements register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getMagnetic(
	VnDevice* vndevice,
	VnVector3* magnetic);

/**
 * \brief Gets the values in the Acceleration Measurements register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getAcceleration(
	VnDevice* vndevice,
	VnVector3* acceleration);

/**
 * \brief Gets the values in the Angular Rate Measurements register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getAngularRate(
	VnDevice* vndevice,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Magnetic, Acceleration and Angular Rates register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getMagneticAccelerationAngularRate(
	VnDevice* vndevice,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, True Body Acceleration and Angular Rates register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] bodyAcceleration The current sensor body acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getYawPitchRollTrueBodyAccelerationAngularRate(
	VnDevice* vndevice,
	VnYpr* attitude,
	VnVector3* bodyAcceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the Yaw,Pitch,Roll, True Inertial Acceleration and Angular Rates register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] attitude The current sensor YawPitchRoll values.
 * \param[out] inertialAcceleration The current sensor inertial acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getYawPitchRollTrueInertialAccelerationAngularRate(
	VnDevice* vndevice,
	VnYpr* attitude,
	VnVector3* inertialAcceleration,
	VnVector3* angularRate);

/**
 * \brief Gets the values in the VPE Basic Control register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] enable The enable/disable value of the sensor.
 * \param[out] headingMode The heading mode value of the sensor.
 * \param[out] filteringMode The filtering mode value of the sensor.
 * \param[out] tuningMode The tuning mode value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getVpeControl(
	VnDevice* vndevice,
	uint8_t* enable,
	uint8_t* headingMode,
	uint8_t* filteringMode,
	uint8_t* tuningMode);

/**
 * \brief Sets the values of the VPE Basic Control register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] enable Value for the enable/disable field.
 * \param[in] headingMode Value for the heading mode field.
 * \param[in] filteringMode Value for the filtering mode field.
 * \param[in] tuningMode Value for the tuning mode field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setVpeControl(
	VnDevice* vndevice,
	uint8_t enable,
	uint8_t headingMode,
	uint8_t filteringMode,
	uint8_t tuningMode,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Magnetometer Basic Tuning register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] baseTuning The current sensor magnetometer base tuning (X,Y,Z) values.
 * \param[out] adaptiveTuning The current sensor magnetometer adaptive tuning (X,Y,Z) values.
 * \param[out] adaptiveFiltering The current sensor magnetometer adaptive filtering (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getVpeMagnetometerBasicTuning(
	VnDevice* vndevice,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning,
	VnVector3* adaptiveFiltering);

/**
 * \brief Sets the values of the VPE Magnetometer Basic Tuning register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] baseTuning The magnetometer base tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveTuning The magnetometer adaptive tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveFiltering The magnetometer adaptive filtering (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setVpeMagnetometerBasicTuning(
	VnDevice* vndevice,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering,
	bool waitForResponse);

/**
 * \brief Gets the values in the VPE Accelerometer Basic Tuning register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] baseTuning The current sensor accelerometer base tuning (X,Y,Z) values.
 * \param[out] adaptiveTuning The current sensor accelerometer adaptive tuning (X,Y,Z) values.
 * \param[out] adaptiveFiltering The current sensor accelerometer adaptive filtering (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getVpeAccelerometerBasicTuning(
	VnDevice* vndevice,
	VnVector3* baseTuning,
	VnVector3* adaptiveTuning,
	VnVector3* adaptiveFiltering);

/**
 * \brief Sets the values of the VPE Accelerometer Basic Tuning register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] baseTuning The accelerometer base tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveTuning The accelerometer adaptive tuning (X,Y,Z) values to write to the sensor.
 * \param[in] adaptiveFiltering The accelerometer adaptive filtering (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setVpeAccelerometerBasicTuning(
	VnDevice* vndevice,
	VnVector3 baseTuning,
	VnVector3 adaptiveTuning,
	VnVector3 adaptiveFiltering,
	bool waitForResponse);

/**
 * \brief Gets the values in the IMU Measurements register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] magnetic The current sensor uncompensated magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor uncompensated acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor uncompensated angular rate (X,Y,Z) values.
 * \param[out] temperature The temperature value of the sensor.
 * \param[out] pressure The pressure value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getImuMeasurements(
	VnDevice* vndevice,
	VnVector3* magnetic,
	VnVector3* acceleration,
	VnVector3* angularRate,
	float* temperature,
	float* pressure);

/**
 * \brief Gets the values in the Reference Frame Rotation register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] c The current sensor C matrix values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getReferenceFrameRotation(
	VnDevice* vndevice,
	VnMatrix3x3* c);

/**
 * \brief Sets the values of the Reference Frame Rotation register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setReferenceFrameRotation(
	VnDevice* vndevice,
	VnMatrix3x3 c,
	bool waitForResponse);

/**
 * \brief Gets the values in the Synchronization Control register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] syncInMode The input signal synchronization mode value of the sensor.
 * \param[out] syncInEdge The input signal synchronization edge selection value of the sensor.
 * \param[out] syncInSkipFactor The input signal trigger skip factor value of the sensor.
 * \param[out] syncOutMode The output signal synchronization mode value of the sensor.
 * \param[out] syncOutPolarity The output signal synchronization polarity value of the sensor.
 * \param[out] syncOutSkipFactor The output synchronization signal skip factor value of the sensor.
 * \param[out] syncOutPulseWidth The output synchronization signal pulse width value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getSynchronizationControl(
	VnDevice* vndevice,
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
 * \param[in] vndevice Pointer to the VnDevice control object.
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
DLL_EXPORT VN_ERROR_CODE vndevice_setSynchronizationControl(
	VnDevice* vndevice,
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
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] syncInCount The synchronization in count value of the sensor.
 * \param[out] syncInTime The synchronization in time value of the sensor.
 * \param[out] syncOutCount The synchronization out count value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getSynchronizationStatus(
	VnDevice* vndevice,
	uint32_t* syncInCount,
	uint32_t* syncInTime,
	uint32_t* syncOutCount);

/**
 * \brief Sets the values of the Synchronization Status register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] syncInCount Value for the synchronization in count field.
 * \param[in] syncInTime Value for the synchronization in time field.
 * \param[in] syncOutCount Value for the synchronization out count field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setSynchronizationStatus(
	VnDevice* vndevice,
	uint32_t syncInCount,
	uint32_t syncInTime,
	uint32_t syncOutCount,
	bool waitForResponse);

/**
 * \brief Gets the contents of the Delta Theta and Delta Velocity register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] deltaTime Delta time for the integration interval.
 * \param[out] deltaTheta Delta rotation vector.
 * \param[out] deltaVelocity Delta velocity vector.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getDeltaThetaAndDeltaVelocity(
	VnDevice* vndevice,
	float* deltaTime,
	VnVector3* deltaTheta,
	VnVector3* deltaVelocity);

/**
 * \brief Gets the values in the Acceleration Compensation register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getAccelerationCompensation(
	VnDevice* vndevice,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Acceleration Compensation register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setAccelerationCompensation(
	VnDevice* vndevice,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Gets the values in the Magnetic Compensation register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getMagneticCompensation(
	VnDevice* vndevice,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Magnetic Compensation register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setMagneticCompensation(
	VnDevice* vndevice,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Gets the values in the Gyro Compensation register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getGyroCompensation(
	VnDevice* vndevice,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Sets the values of the Gyro Compensation register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] b The B vector values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setGyroCompensation(
	VnDevice* vndevice,
	VnMatrix3x3 c,
	VnVector3 b,
	bool waitForResponse);

/**
 * \brief Retreives the current values of the IMU Filtering Configuration register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
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
	uint8_t* presFilterMode);

/**
 * \brief Sets the values of the IMU Filtering Configuration register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
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
	bool waitForResponse);

/**
 * \brief Retreives the current values of the Delta Theta and Delta Velocity Configuration register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] integrationFrame Output frame for delta velocity quantities.
 * \param[out] gyroCompensation Compensation to apply to angular rate.
 * \param[out] accelCompensation Compensation to apply to accelerations.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getDeltaThetaAndDeltaVelocityConfiguration(
	VnDevice* vndevice,
	uint8_t* integrationFrame,
	uint8_t* gyroCompensation,
	uint8_t* accelCompensation);

/**
 * \brief Sets the values of the Delta Theta and Delta Velocity Configuration register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] integrationFrame Output frame for delta velocity quantities.
 * \param[in] gyroCompensation Compensation to apply to angular rate.
 * \param[in] accelCompensation Compensation to apply to accelerations.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setDeltaThetaAndDeltaVelocityConfiguration(
	VnDevice* vndevice,
	uint8_t integrationFrame,
	uint8_t gyroCompensation,
	uint8_t accelCompensation,
	bool waitForResponse);

/**
 * \brief Gets the values in the Magnetometer Calibration Control register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] hsiMode The HSIMode value of the sensor.
 * \param[out] hsiOutput The HSIOutput value of the sensor.
 * \param[out] convergeRate The ConvergeRate value of the sensor.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getMagnetometerCalibrationControl(
	VnDevice* vndevice,
	uint8_t* hsiMode,
	uint8_t* hsiOutput,
	uint8_t* convergeRate);

/**
 * \brief Sets the values of the Magnetometer Calibration Control register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] hsiMode Value for the HSIMode field.
 * \param[in] hsiOutput Value for the HSIOutput field.
 * \param[in] convergeRate Value for the ConvergeRate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setMagnetometerCalibrationControl(
	VnDevice* vndevice,
	uint8_t hsiMode,
	uint8_t hsiOutput,
	uint8_t convergeRate,
	bool waitForResponse);

/**
 * \brief Gets the values in the Calculated Magnetometer Calibration register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] c The current sensor C matrix values.
 * \param[out] b The current sensor B vector values.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getCalculatedMagnetometerCalibration(
	VnDevice* vndevice,
	VnMatrix3x3* c,
	VnVector3* b);

/**
 * \brief Gets the values in the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] magneticReference The current sensor magnetic reference vector (X,Y,Z) values.
 * \param[out] gravityReference The current sensor gravity reference vector (X,Y,Z) values.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getMagneticGravityReferenceVectors(
	VnDevice* vndevice,
	VnVector3* magneticReference,
	VnVector3* gravityReference);

/**
 * \brief Sets the values of the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] magneticReference The magnetic reference vector (X,Y,Z) values to write to the sensor.
 * \param[in] gravityReference The gravity reference vector (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_setMagneticGravityReferenceVectors(
	VnDevice* vndevice,
	VnVector3 magneticReference,
	VnVector3 gravityReference,
	bool waitForResponse);

/**
 * \brief Gets the values in the Communication Protocol Control register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
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
DLL_EXPORT VN_ERROR_CODE vndevice_getCommunicationProtocolControl(
	VnDevice* vndevice,
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
 * \param[in] vndevice Pointer to the VnDevice control object.
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
DLL_EXPORT VN_ERROR_CODE vndevice_setCommunicationProtocolControl(
	VnDevice* vndevice,
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
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[out] useMagModel The UseMagModel field.
 * \param[out] useGravityModel The UseGravityModel field.
 * \param[out] recalcThreshold The RecalcThreshold field.
 * \param[out] year The Year field.
 * \param[out] lla The Lattitude, Longitude, Altitude fields.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_getReferenceVectorConfiguration(
	VnDevice* vndevice,
	uint8_t* useMagModel,
	uint8_t* useGravityModel,
	uint32_t* recalcThreshold,
	float* year,
	VnVector3* lla);

/**
 * \brief Sets the values in the Reference Vector Configuration register.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
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
DLL_EXPORT VN_ERROR_CODE vndevice_setReferenceVectorConfiguration(
	VnDevice* vndevice,
	uint8_t useMagModel,
	uint8_t useGravityModel,
	uint32_t recalcThreshold,
	float year,
	VnVector3 lla,
	bool waitForResponse);

/**
 * \brief Pauses the asynchronous data output.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_pauseAsyncOutputs(
	VnDevice* vndevice,
	bool waitForResponse);

/**
 * \brief Resumes the asynchronous data output.
 *
 * \param[in] vndevice Pointer to the VnDevice control object.
 * \param[in] waitForResponse Signals if the function should block until a response is
 *     received from the sensor. TRUE to block for a response; FALSE to immediately
 *     return after sending out the command.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vndevice_resumeAsyncOutputs(
	VnDevice* vndevice,
	bool waitForResponse);

#ifdef __cplusplus
}
#endif

#endif /* _VNDEVICE_H_ */
