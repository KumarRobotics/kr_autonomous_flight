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
// $Id: Utilities.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_UTILITIES_H_
#define PGR_FC2_UTILITIES_H_

#include "FlyCapture2Platform.h"
#include "FlyCapture2Defs.h"
#include <string>

namespace FlyCapture2
{
	class Error;

	/** Possible operating systems. */
	enum OSType
	{
		WINDOWS_X86, /**< All Windows 32-bit variants. */
		WINDOWS_X64, /**< All Windows 64-bit variants. */
		LINUX_X86, /**< All Linux 32-bit variants. */
		LINUX_X64, /**< All Linux 32-bit variants. */
		MAC, /**< Mac OSX. */
		UNKNOWN_OS, /**< Unknown operating system. */
		OSTYPE_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** Possible byte orders. */
	enum ByteOrder
	{
		BYTE_ORDER_LITTLE_ENDIAN,
		BYTE_ORDER_BIG_ENDIAN,
		BYTE_ORDER_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** Description of the system. */
	struct SystemInfo
	{
		/** Operating system type as described by OSType. */
		OSType osType;

		/** Detailed description of the operating system. */
		char osDescription[sk_maxStringLength];

		/** Byte order of the system. */
		ByteOrder byteOrder;

		/** Amount of memory available on the system. */
		size_t	sysMemSize;

		/** Detailed description of the CPU. */
		char cpuDescription[sk_maxStringLength];

		/** Number of cores on all CPUs on the system. */
		size_t	numCpuCores;

		/** List of drivers used. */
		char driverList[sk_maxStringLength];

		/** List of libraries used. */
		char libraryList[sk_maxStringLength];

		/** Detailed description of the GPU. */
		char gpuDescription[sk_maxStringLength];

		/** Screen resolution width in pixels. */
		size_t screenWidth;

		/** Screen resolution height in pixels. */
		size_t screenHeight;

		/** Reserved for future use. */
		unsigned int reserved[16];

	};

	/**
	 * Async command callback function prototype. Defines the syntax of the
	 * async command function that is passed into LaunchCommandAsync().
	 */
	typedef void (*AsyncCommandCallback)( class Error retError, void* pUserData );

	/**
	 * The Utility class is generally used to query for general system
	 * information such as operating system, available memory etc.
	 * It can also be used to launch browsers, CHM viewers or terminal commands.
	 */
	class FLYCAPTURE2_API Utilities
	{
		public:

			/**
			 * Check for driver compatibility for the given camera guid.
			 *
			 * @param guid Pointer to the guid of the device to check.
			 *
			 * @return PGR_NO_ERROR if the library is compatible with the currently
			 *         loaded driver, otherwise an error indicating the type of failure.
			 */
			static Error CheckDriver(const PGRGuid* guid);

			/**
			 * Get the driver's name for a device
			 *
			 * @param guid Pointer to the guid of the device to check.
			 * @param deviceName The device name will be returned in this string
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error GetDriverDeviceName(const PGRGuid* guid, std::string& deviceName);

			/**
			 * Get system information.
			 *
			 * @param pSystemInfo Structure to receive system information.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error GetSystemInfo(
					SystemInfo* pSystemInfo );

			/**
			 * Get library version.
			 *
			 * @param pVersion Structure to receive the library version.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error GetLibraryVersion(
					FC2Version* pVersion );

			/**
			 * Launch a URL in the system default browser.
			 *
			 * @param pAddress URL to open in browser.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error LaunchBrowser(
					const char*  pAddress );

			/**
			 * Open a CHM file in the system default CHM viewer.
			 *
			 * @param pFileName Filename of CHM file to open.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error LaunchHelp(
					const char*  pFileName );

			/**
			 * Execute a command in the terminal. This is a blocking call that
			 * will return when the command completes.
			 *
			 * @param pCommand Command to execute.
			 *
			 * @see LaunchCommandAsync()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error LaunchCommand(
					const char*  pCommand );

			/**
			 * Execute a command in the terminal. This is a non-blocking call that
			 * will return immediately. The return value of the command can be
			 * retrieved in the callback.
			 *
			 * @param pCommand Command to execute.
			 * @param pCallback Callback to fire when command is complete.
			 * @param pUserData Data pointer to pass to callback.
			 *
			 * @see LaunchCommand()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error LaunchCommandAsync(
					const char*				pCommand,
					AsyncCommandCallback	pCallback,
					void*					pUserData );

		protected:
		private:
			Utilities();
			~Utilities();

			Utilities( const Utilities& other );
			Utilities& operator=( const Utilities& other );

	};
}


#endif // PGR_FC2_UTILITIES_H_

