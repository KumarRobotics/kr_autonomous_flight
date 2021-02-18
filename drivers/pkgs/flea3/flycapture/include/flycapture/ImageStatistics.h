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
// $Id: ImageStatistics.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_IMAGESTATISTICS_H
#define PGR_FC2_IMAGESTATISTICS_H

#include "FlyCapture2Platform.h"
#include "FlyCapture2Defs.h"

namespace FlyCapture2
{
	class Error;

	/**
	 * The ImageStatistics object represents image statistics for an image.
	 */
	class FLYCAPTURE2_API ImageStatistics
	{
		public:

			/**
			 * Channels that allow statistics to be calculated.
			 */
			enum StatisticsChannel
			{
				GREY,
				RED,
				GREEN,
				BLUE,
				HUE,
				SATURATION,
				LIGHTNESS,
				NUM_STATISTICS_CHANNELS
			};

			/**
			 * Default constructor.
			 */
			ImageStatistics();

			/**
			 * Default destructor.
			 */
			virtual ~ImageStatistics();

			/**
			 * Copy constructor.
			 */
			ImageStatistics( const ImageStatistics& other );

			/**
			 * Assignment operator.
			 *
			 * @param other The ImageStatistics object to copy from.
			 */
			ImageStatistics& operator=( const ImageStatistics& other );

			/**
			 * Enable all channels.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error EnableAll();

			/**
			 * Disable all channels.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error DisableAll();

			/**
			 * Enable only the grey channel.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error EnableGreyOnly();

			/**
			 * Enable only the RGB channels.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error EnableRGBOnly();

			/**
			 * Enable only the HSL channels.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error EnableHSLOnly();

			/**
			 * Get the status of a statistics channel.
			 *
			 * @param channel The statistics channel.
			 * @param pEnabled Whether the channel is enabled.
			 *
			 * @see SetChannelStatus()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error GetChannelStatus(
					StatisticsChannel channel,
					bool* pEnabled ) const;

			/**
			 * Set the status of a statistics channel.
			 *
			 * @param channel The statistics channel.
			 * @param enabled Whether the channel should be enabled.
			 *
			 * @see GetChannelStatus()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error SetChannelStatus(
					StatisticsChannel channel,
					bool enabled );

			/**
			 * Get the range of a statistics channel. The values returned
			 * are the maximum possible values for any given pixel in the image.
			 * This is generally 0-255 for 8 bit images, and 0-65535 for
			 * 16 bit images.
			 *
			 * @param channel The statistics channel.
			 * @param pMin The minimum possible value.
			 * @param pMax The maximum possible value.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error GetRange(
					StatisticsChannel channel,
					unsigned int* pMin,
					unsigned int* pMax ) const;

			/**
			 * Get the range of a statistics channel. The values returned
			 * are the maximum values recorded for all pixels in the image.
			 *
			 * @param channel The statistics channel.
			 * @param pPixelValueMin The minimum pixel value.
			 * @param pPixelValueMax The maximum pixel value.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error GetPixelValueRange(
					StatisticsChannel channel,
					unsigned int* pPixelValueMin,
					unsigned int* pPixelValueMax ) const;

			/**
			 * Get the number of unique pixel values in the image.
			 *
			 * @param channel The statistics channel.
			 * @param pNumPixelValues The number of unique pixel values.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error GetNumPixelValues(
					StatisticsChannel channel,
					unsigned int* pNumPixelValues ) const;

			/**
			 * Get the mean of the image.
			 *
			 * @param channel The statistics channel.
			 * @param pPixelValueMean The mean of the image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error GetMean(
					StatisticsChannel channel,
					float* pPixelValueMean ) const;

			/**
			 * Get the histogram for the image.
			 *
			 * @param channel The statistics channel.
			 * @param ppHistogram Pointer to an array containing the histogram.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error GetHistogram(
					StatisticsChannel channel,
					int** ppHistogram ) const;

			/**
			 * Get all statistics for the image.
			 *
			 * @param channel The statistics channel.
			 * @param pRangeMin The minimum possible value.
			 * @param pRangeMax The maximum possible value.
			 * @param pPixelValueMin The minimum pixel value.
			 * @param pPixelValueMax The maximum pixel value.
			 * @param pNumPixelValues The number of unique pixel values.
			 * @param pPixelValueMean The mean of the image.
			 * @param ppHistogram Pointer to an array containing the histogram.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error GetStatistics(
					StatisticsChannel channel,
					unsigned int* pRangeMin = NULL,
					unsigned int* pRangeMax = NULL,
					unsigned int* pPixelValueMin = NULL,
					unsigned int* pPixelValueMax = NULL,
					unsigned int* pNumPixelValues = NULL,
					float* pPixelValueMean = NULL,
					int** ppHistogram = NULL ) const;

		private:
			friend class ImageStatsCalculator;
			struct ImageStatisticsData;
			ImageStatisticsData* m_pData;
	};
}

#endif
