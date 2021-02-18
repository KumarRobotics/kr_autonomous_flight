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
 * This header file defines the error codes used within the VectorNav C/C++
 * Library.
 */
#ifndef _VN_ERRORCODES_H_
#define _VN_ERRORCODES_H_

#if defined(_MSC_VER) && _MSC_VER <= 1500
	/* Visual Studio 2008 and earlier do not include the stdint.h header file. */
	#include "vnint.h"
#else
	#include <stdint.h>
#endif

/** Type define for VectorNav error codes. */
typedef uint32_t VN_ERROR_CODE;

/**
 * @defgroup VN_ERROR_CODE VectorNav Error Code Definitions
 *
 * @{
 */
#define VNERR_NO_ERROR							0	/**< No error occurred. */
#define VNERR_UNKNOWN_ERROR						1	/**< An unknown error occurred. */
#define VNERR_NOT_IMPLEMENTED					2	/**< The operation is not implemented. */
#define VNERR_TIMEOUT							3	/**< Operation timed out. */
#define VNERR_INVALID_VALUE						4	/**< Invalid value was provided. */
#define VNERR_FILE_NOT_FOUND					5	/**< The file was not found. */
#define VNERR_NOT_CONNECTED						6	/**< Not connected to the sensor. */
#define VNERR_PERMISSION_DENIED					7	/**< Permission is denied. */
#define VNERR_SENSOR_HARD_FAULT					8	/**< Sensor experienced a hard fault error. */
#define VNERR_SENSOR_SERIAL_BUFFER_OVERFLOW		9	/**< Sensor experienced a serial buffer overflow error. */
#define VNERR_SENSOR_INVALID_CHECKSUM			10	/**< Sensor reported an invalid checksum error. */
#define VNERR_SENSOR_INVALID_COMMAND			11	/**< Sensor reported an invalid command error. */
#define VNERR_SENSOR_NOT_ENOUGH_PARAMETERS		12	/**< Sensor reported a not enough parameters error. */
#define VNERR_SENSOR_TOO_MANY_PARAMETERS		13	/**< Sensor reported a too many parameters error. */
#define VNERR_SENSOR_INVALID_PARAMETER			14	/**< Sensor reported an invalid parameter error. */
#define VNERR_SENSOR_UNAUTHORIZED_ACCESS		15	/**< Sensor reported an unauthorized access error. */
#define VNERR_SENSOR_WATCHDOG_RESET				16	/**< Sensor reported a watchdog reset error. */
#define	VNERR_SENSOR_OUTPUT_BUFFER_OVERFLOW		17	/**< Sensor reported an output buffer overflow error. */
#define	VNERR_SENSOR_INSUFFICIENT_BAUD_RATE		18	/**< Sensor reported an insufficient baudrate error. */
#define VNERR_SENSOR_ERROR_BUFFER_OVERFLOW		19	/**< Sensor reported an error buffer overflow error. */
/** @} */

#endif /* _VN_ERRORCODES_H_ */
