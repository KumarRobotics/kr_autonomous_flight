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
 * This file supplies the cross-platform services when on a Linux machine.
 */
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include "vncp_services.h"
#include "vn_errorCodes.h"

/* Private type declarations. ************************************************/
typedef struct {
	VN_THREAD_START_ROUTINE	startRoutine;
	void*					routineData;
} VncpThreadStartData;

typedef struct {
	pthread_mutex_t		mutex;
	pthread_cond_t		condition;
	bool				isTriggered;
} VncpConditionAndMutex;

/* Private function declarations. ********************************************/
void* vncp_thread_startRoutine(void* threadStartData);
VN_ERROR_CODE vncp_convertNativeToVnErrorCode(int nativeErrorCode);

/**
 * \brief Determines what the baudrate flag should be for the provide baudrate.
 *
 * \param[in]	baudrate	Desired baudrate.
 * \return The appropriate baudrate flag to set in the termios.c_cflag field to
 * get the desired baudrate. If the provided baudrate is invalid, the value B0
 * will be returned.
 */
tcflag_t vncp_determineBaudrateFlag(unsigned int baudrate);

/* Private variables. */
double _clockStart = -1.0;

VN_ERROR_CODE vncp_thread_startNew(VN_HANDLE* newThreadHandle, VN_THREAD_START_ROUTINE startRoutine, void* routineData)
{
	int errorCode;

	VncpThreadStartData* data = (VncpThreadStartData*) malloc(sizeof(VncpThreadStartData));
	
	data->startRoutine = startRoutine;
	data->routineData = routineData;

	errorCode = pthread_create(&newThreadHandle->pThreadHandle, NULL, vncp_thread_startRoutine, data);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_open(VN_HANDLE* newComPortHandle, char const* portName, unsigned int baudrate)
{
	struct termios portSettings;
	int portFd = -1;
	tcflag_t baudrateFlag;

	portFd = open(portName, O_RDWR | O_NOCTTY);
	if (portFd == -1)
		return vncp_convertNativeToVnErrorCode(errno);

	/* clear struct for new port settings */
	memset(&portSettings, 0, sizeof(portSettings));

	baudrateFlag = vncp_determineBaudrateFlag(baudrate);
	if (baudrateFlag == B0)
		return VNERR_UNKNOWN_ERROR;

	/* Set baudrate, 8n1, no modem control, enable receiving characters. */
	portSettings.c_cflag = baudrateFlag | CS8 | CLOCAL | CREAD;

	portSettings.c_iflag = IGNPAR;		/* Ignore bytes with parity errors. */
	portSettings.c_oflag = 0;			/* Enable raw data output. */

	portSettings.c_cc[VTIME]    = 0;	/* Do not use inter-character timer. */
	portSettings.c_cc[VMIN]     = 0;	/* Block on reads until 0 character is received. */

    /* Clear the COM port buffers. */
	if (tcflush(portFd, TCIFLUSH) != 0)
		return vncp_convertNativeToVnErrorCode(errno);

	if (tcsetattr(portFd, TCSANOW, &portSettings) != 0)
		return vncp_convertNativeToVnErrorCode(errno);

	newComPortHandle->comPortHandle = portFd;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_writeData(VN_HANDLE comPortHandle, char const* dataToWrite, unsigned int numOfBytesToWrite)
{
	int errorCode;

	errorCode = write(comPortHandle.comPortHandle, dataToWrite, numOfBytesToWrite);

	if (errorCode == -1)
		return vncp_convertNativeToVnErrorCode(errno);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_readData(VN_HANDLE comPortHandle, char* readBuffer, unsigned int numOfBytesToRead, unsigned int* numOfBytesActuallyRead)
{
	int numOfBytesRead;

	*numOfBytesActuallyRead = 0;

	numOfBytesRead = read(comPortHandle.comPortHandle, readBuffer, numOfBytesToRead);
	
	if (numOfBytesRead == -1)
		return vncp_convertNativeToVnErrorCode(errno);

	*numOfBytesActuallyRead = (unsigned int) numOfBytesRead;

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_close(VN_HANDLE comPortHandle)
{
	if (close(comPortHandle.comPortHandle) == -1)
		return vncp_convertNativeToVnErrorCode(errno);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_isOptimized(
	char const* portName,
	bool* isOptimized)
{
	/* Currently Linux USB COM ports don't need any optimization. */
	*isOptimized = true;
	
	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_comPort_optimize(
	char const* portName)
{
	/* Nothing necessary to do on Linux machines. */
	return VNERR_NO_ERROR;
}

#if defined(__APPLE__)
  #define B9600 9600
  #define B19200 19200
  #define B38400 38400
  #define B57600 57600
  #define B115200 115200
  #define B230400 230400
  #define B460800 460800
  #define B921600 921600
#endif

tcflag_t vncp_determineBaudrateFlag(unsigned int baudrate)
{
	switch (baudrate) {
		case 9600:		return B9600;
		case 19200:		return B19200;
		case 38400:		return B38400;
		case 57600:		return B57600;
		case 115200:	return B115200;
		#if !defined(__QNXNTO__) /* QNX does not have higher baudrates defined. */
		case 230400:	return B230400;
		case 460800:	return B460800;
		case 921600:	return B921600;
		#endif
		default:		return B0;
	}
}

VN_ERROR_CODE vncp_event_create(VN_HANDLE* newEventHandle)
{
	int errorCode;
	VncpConditionAndMutex* cm;

	cm = (VncpConditionAndMutex*) malloc(sizeof(VncpConditionAndMutex));
	
	errorCode = pthread_mutex_init(&cm->mutex, NULL);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);
		
	errorCode = pthread_cond_init(&cm->condition, NULL);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	cm->isTriggered = false;

	newEventHandle->conditionAndMutexStruct = cm;
		
	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_event_waitFor(VN_HANDLE eventHandle, int timeout)
{
	int errorCode, loopErrorCode;
	VncpConditionAndMutex* cm;
	struct timespec delta;
	struct timespec abstime;
	
	// Compute our timeout.
	if (timeout != -1) {

		clock_gettime(CLOCK_REALTIME, &abstime);

		int nano = abstime.tv_nsec + (timeout % 1000) * 1000000;

		delta.tv_nsec = nano % 1000000000;
		delta.tv_sec = abstime.tv_sec + timeout / 1000 + nano / 1000000000;
	}

	cm = (VncpConditionAndMutex*) eventHandle.conditionAndMutexStruct;

	errorCode = pthread_mutex_lock(&cm->mutex);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	while (!cm->isTriggered) {
		
		if (timeout == -1) {
			loopErrorCode = pthread_cond_wait(&cm->condition, &cm->mutex);

			if (loopErrorCode != 0)
				return vncp_convertNativeToVnErrorCode(loopErrorCode);
		}
		else {
			loopErrorCode = pthread_cond_timedwait(&cm->condition, &cm->mutex, &delta);

			if (loopErrorCode == ETIMEDOUT) {
				cm->isTriggered = false;
				errorCode = pthread_mutex_unlock(&cm->mutex);
	
				return VNERR_TIMEOUT;
			}
		}
	}

	cm->isTriggered = false;

	errorCode = pthread_mutex_unlock(&cm->mutex);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_event_signal(VN_HANDLE eventHandle)
{
	int errorCode;
	VncpConditionAndMutex* cm;

	cm = (VncpConditionAndMutex*) eventHandle.conditionAndMutexStruct;

	errorCode = pthread_mutex_lock(&cm->mutex);

	cm->isTriggered = true;

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	errorCode = pthread_cond_signal(&cm->condition);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	errorCode = pthread_mutex_unlock(&cm->mutex);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_criticalSection_initialize(VN_CRITICAL_SECTION* criticalSection)
{
	int errorCode;

	errorCode = pthread_mutex_init(criticalSection, NULL);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);
		
	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_criticalSection_enter(VN_CRITICAL_SECTION* criticalSection)
{
	int errorCode;

	errorCode = pthread_mutex_lock(criticalSection);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_criticalSection_leave(VN_CRITICAL_SECTION* criticalSection)
{
	int errorCode;

	errorCode = pthread_mutex_unlock(criticalSection);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	return VNERR_NO_ERROR;
}

VN_ERROR_CODE vncp_criticalSection_dispose(VN_CRITICAL_SECTION* criticalSection)
{
	int errorCode;

	errorCode = pthread_mutex_destroy(criticalSection);

	if (errorCode != 0)
		return vncp_convertNativeToVnErrorCode(errorCode);

	return VNERR_NO_ERROR;
}

void* vncp_thread_startRoutine(void* threadStartData)
{
	VncpThreadStartData* data;

	data = (VncpThreadStartData*) threadStartData;

	/* Call the user's thread routine. */
	data->startRoutine(data->routineData);

	return 0;
}

VN_ERROR_CODE vncp_convertNativeToVnErrorCode(int nativeErrorCode)
{
	switch (nativeErrorCode) {

		case ENOENT:
			return VNERR_FILE_NOT_FOUND;

		case EACCES:
			return VNERR_PERMISSION_DENIED;

		default:
			return VNERR_UNKNOWN_ERROR;
	}
}

VN_ERROR_CODE vncp_sleepInMs(unsigned int numOfMillisecondsToSleep)
{
	usleep(numOfMillisecondsToSleep * 1000);

	return VNERR_NO_ERROR;
}

void vncp_startMsTimer()
{
	struct timespec time;
	int error;

	error = clock_gettime(CLOCK_MONOTONIC, &time);

	if (error != 0) {

	  _clockStart = -1.0;

	  return;
	}

	_clockStart = (time.tv_sec * 1000.0) + (time.tv_nsec / 1000000.0);
}

double vncp_stopMsTimer()
{
	struct timespec time;
	int error;
	double result;

	if (_clockStart < 0)
	  return -1.0;

	error = clock_gettime(CLOCK_MONOTONIC, &time);

	if (error != 0) {

	  _clockStart = -1.0;

	  return -1.0;
	}

	result = (time.tv_sec * 1000.0) + (time.tv_nsec / 1000000.0) - _clockStart;

	_clockStart = -1.0;

	return result;
}

/** \endcond */
