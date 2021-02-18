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
// $Id: AVIRecorder.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_AVIRECORDER_H
#define PGR_FC2_AVIRECORDER_H

#include "FlyCapture2Platform.h"
#include "FlyCapture2Defs.h"

namespace FlyCapture2
{
	class Error;
	class Image;

	/**
	 * The AVIRecorder class provides the functionality for the user to record
	 * images to an AVI file.
	 */
	class FLYCAPTURE2_API AVIRecorder
	{
		public:

			/**
			 * Default constructor.
			 */
			AVIRecorder();

			/**
			 * Default destructor.
			 */
			virtual ~AVIRecorder();

			/**
			 * Open an AVI file in preparation for writing Images to disk.
			 * The size of AVI files is limited to 2GB. The filenames are
			 * automatically generated using the filename specified.
			 *
			 * @param pFileName The filename of the AVI file.
			 * @param pOption Options to apply to the AVI file.
			 *
			 * @see AVIClose()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error AVIOpen(
					const char* pFileName,
					AVIOption*  pOption );

			/**
			 * Open an MJPEG AVI file in preparation for writing Images to disk.
			 * The size of AVI files is limited to 2GB. The filenames are
			 * automatically generated using the filename specified.
			 *
			 * @param pFileName The filename of the AVI file.
			 * @param pOption MJPEG options to apply to the AVI file.
			 *
			 * @see AVIClose()
			 * @see MJPGOption
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error AVIOpen(
					const char* pFileName,
					MJPGOption*  pOption );


			/**
			 * Open an H264 MP4 file in preparation for writing Images to disk.
			 * The size of MP4 files is limited to 2GB. The filenames are
			 * automatically generated using the filename specified.
			 *
			 * @param pFileName The filename of the MP4 file.
			 * @param pOption H264 options to apply to the MP4 file.
			 *
			 * @see AVIClose()
			 * @see H264Option
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error AVIOpen(
					const char* pFileName,
					H264Option*  pOption );


			/**
			 * Append an image to the AVI/MP4 file.
			 *
			 * @param pImage The image to append.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error AVIAppend( Image* pImage);

			/**
			 * Close the AVI/MP4 file.
			 *
			 * @see AVIOpen()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error AVIClose( );

		private:

			AVIRecorder( const AVIRecorder& );
			AVIRecorder& operator=( const AVIRecorder& );

			struct AVIRecorderData; // Forward declaration

			AVIRecorderData* m_pAVIRecorderData;
	};
}

#endif //PGR_FC2_AVIRECORDER_H
