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
// $Id: Image.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_IMAGE_H
#define PGR_FC2_IMAGE_H

#include "FlyCapture2Platform.h"
#include "FlyCapture2Defs.h"

namespace FlyCapture2
{
	class Error;
	class ImageStatistics;

	/**
	 * The Image class is used to retrieve images from a camera, convert
	 * between multiple pixel formats and save images to disk. Operations on
	 * Image objects are not guaranteed to be thread safe. It is recommended
	 * that operations on Image objects be protected by thread synchronization
	 * constructs such as mutexes.
	 */
	class FLYCAPTURE2_API Image
	{
		public:

			/**
			 * Set the default color processing algorithm.  This method will be
			 * used for any image with the DEFAULT algorithm set. The method used
			 * is determined at the time of the Convert() call, therefore the most
			 * recent execution of this function will take precedence. The default
			 * setting is shared within the current process.
			 *
			 * @param defaultMethod The color processing algorithm to set.
			 *
			 * @see GetDefaultColorProcessing()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error SetDefaultColorProcessing(
					ColorProcessingAlgorithm defaultMethod );

			/**
			 * Get the default color processing algorithm.
			 *
			 * @see SetDefaultColorProcessing()
			 *
			 * @return The default color processing algorithm.
			 */
			static ColorProcessingAlgorithm GetDefaultColorProcessing();

			/**
			 * Set the default output pixel format. This format will be used for any
			 * call to Convert() that does not specify an output format. The format
			 * used will be determined at the time of the Convert() call, therefore
			 * the most recent execution of this function will take precedence.
			 * The default is shared within the current process.
			 *
			 * @param format The output pixel format to set.
			 *
			 * @see GetDefaultOutputFormat()
			 *
			 * @return The default color processing algorithm.
			 */
			static Error SetDefaultOutputFormat( PixelFormat format );

			/**
			 * Get the default output pixel format.
			 *
			 * @see SetDefaultOutputFormat()
			 *
			 * @return The default pixel format.
			 */
			static PixelFormat GetDefaultOutputFormat();

			/**
			 * Calculate the bits per pixel for the specified pixel format.
			 *
			 * @param format The pixel format.
			 *
			 * @return The bits per pixel.
			 */
			static unsigned int DetermineBitsPerPixel( PixelFormat format );

			/**
			 * Default constructor.
			 */
			Image();

			/**
			 * Construct an Image object with the specified arguments.
			 * Ownership of the image buffer is not transferred to the Image object.
			 * It is the user's responsibility to delete the buffer when it is
			 * no longer in use.
			 *
			 * @param rows Rows in the image.
			 * @param cols Columns in the image.
			 * @param stride Stride of the image buffer.
			 * @param pData Pointer to the image buffer.
			 * @param dataSize Size of the image buffer.
			 * @param format Pixel format.
			 * @param bayerFormat Format of the Bayer tiled raw image.
			 */
			Image(
					unsigned int    rows,
					unsigned int    cols,
					unsigned int    stride,
					unsigned char*  pData,
					unsigned int    dataSize,
					PixelFormat     format,
					BayerTileFormat bayerFormat = NONE );

			/**
			 * Construct an Image object with the specified arguments.
			 * Ownership of the image buffer is not transferred to the Image object.
			 * It is the user's responsibility to delete the buffer when it is
			 * no longer in use.
			 *
			 * @param rows Rows in the image.
			 * @param cols Columns in the image.
			 * @param stride Stride of the image buffer.
			 * @param pData Pointer to the image buffer.
			 * @param dataSize Size of the image buffer.
			 * @param receivedDataSize Actual size of data.
			 * @param format Pixel format.
			 * @param bayerFormat Format of the Bayer tiled raw image.
			 */
			Image(
					unsigned int    rows,
					unsigned int    cols,
					unsigned int    stride,
					unsigned char*  pData,
					unsigned int    dataSize,
					unsigned int    receivedDataSize,
					PixelFormat     format,
					BayerTileFormat bayerFormat = NONE );

			/**
			 * Construct an Image object with the specified arguments.
			 * Ownership of the image buffer is not transferred to the Image object.
			 * It is the user's responsibility to delete the buffer when it is
			 * no longer in use.
			 *
			 * @param pData Pointer to the image buffer.
			 * @param dataSize Size of the image buffer.
			 */
			Image(
					unsigned char* pData,
					unsigned int   dataSize);

			/**
			 * Construct an Image object with the specified arguments.
			 *
			 * @param rows Rows in the image.
			 * @param cols Columns in the image.
			 * @param format Pixel format.
			 * @param bayerFormat Format of the Bayer tiled raw image.
			 */
			Image(
					unsigned int    rows,
					unsigned int    cols,
					PixelFormat     format,
					BayerTileFormat bayerFormat = NONE );

			/**
			 * Copy constructor. Both images will point to the same image buffer
			 * internally.
			 */
			Image( const Image& image );

			/**
			 * Default destructor. The internal image buffer will be released if
			 * there are no other Image objects holding a reference to it. This
			 * will also allow the buffer to be requeued internally.
			 */
			virtual ~Image();

			/**
			 * Assignment operator. Both images will point to the same image buffer
			 * internally. If the Image already has a buffer attached to it, it will
			 * will be released.
			 *
			 * @param image The image to copy from.
			 */
			virtual Image& operator=( const Image& image );

			/**
			 * Indexing operator.
			 *
			 * @param index The index of the byte to return.
			 *
			 * @return The address of the specified byte from the image data.
			 */
			virtual unsigned char* operator[]( unsigned int index );

			/**
			 * Indexing operator.
			 *
			 * @param row The row of the pixel to return.
			 * @param col The column of the pixel to return.
			 *
			 * @return The address of the specified byte from the image data.
			 */
			virtual unsigned char* operator()(
					unsigned int row,
					unsigned int col );

			/**
			 * Perform a deep copy of the Image. After this operation, the image
			 * contents and member variables will be the same. The Images will not
			 * share a buffer. The Image's current buffer will not be released.
			 *
			 * @param pImage The Image to copy the data from.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error DeepCopy( const Image* pImage );

			/**
			 * Sets the dimensions of the image object.
			 *
			 * @param rows Number of rows to set.
			 * @param cols Number of cols to set.
			 * @param stride Stride to set.
			 * @param pixelFormat Pixel format to set.
			 * @param bayerFormat Bayer tile format to set.
			 *
			 * @see GetDimensions()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetDimensions(
					unsigned int    rows,
					unsigned int    cols,
					unsigned int    stride,
					PixelFormat     pixelFormat,
					BayerTileFormat bayerFormat );

			/**
			 * Set the data of the Image object.
			 * Ownership of the image buffer is not transferred to the Image object.
			 * It is the user's responsibility to delete the buffer when it is
			 * no longer in use.
			 *
			 * @param pData Pointer to the image buffer.
			 * @param dataSize Size of the image buffer.
			 */
			virtual Error SetData(
					const unsigned char* pData,
					unsigned int         dataSize );

			/**
			 * Set the block id of the Image object.
			 *
			 * @param blockId The blockId to assign to the image.
			 */
			virtual Error SetBlockId( const unsigned int blockId);

			/**
			 * get the block id of the Image object.
			 *
			 * @return The blockId assigned to the image.
			 */
			virtual unsigned int GetBlockId();

			/**
			 * Get the current pixel format.
			 *
			 * @return The current pixel format.
			 */
			virtual PixelFormat GetPixelFormat() const;

			/**
			 * Get the current color processing algorithm.
			 *
			 * @see SetColorProcessing()
			 *
			 * @return The current color processing algorithm.
			 */
			virtual ColorProcessingAlgorithm GetColorProcessing() const;

			/**
			 * Set the color processing algorithm. This should be set on the
			 * input Image object.
			 *
			 * @param colorProc The color processing algorithm to use.
			 *
			 * @see GetColorProcessing()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetColorProcessing( ColorProcessingAlgorithm colorProc );

			/**
			 * Get the number of columns in the image.
			 *
			 * @return The number of columns.
			 */
			virtual unsigned int GetCols() const;

			/**
			 * Get the number of rows in the image.
			 *
			 * @return The number of rows.
			 */
			virtual unsigned int GetRows() const;

			/**
			 * Get the stride in the image.
			 *
			 * @return The stride (The number of bytes between rows of the image).
			 */
			virtual unsigned int GetStride() const;

			/**
			 * Get the bits per pixel of the image.
			 *
			 * @return The bits per pixel.
			 */
			virtual unsigned int GetBitsPerPixel() const;

			/**
			 * Get the Bayer tile format of the image.
			 *
			 * @return The Bayer tile format.
			 */
			virtual BayerTileFormat GetBayerTileFormat() const;

			/**
			 * Get the size of the buffer associated with the image, in bytes.
			 *
			 * @return The size of the buffer, in bytes.
			 */
			virtual unsigned int GetDataSize() const;

			/**
			 * Get the size of the compressed data, in bytes. A compressed image
			 * will have a maximum size equal to GetDataSize(), but may actually
			 * contain less data, depending on the compression level.
			 * For uncompressed images, a value smaller than the data size may
			 * indicate lost data.
			 *
			 * @return The size of the compressed data, in bytes. 0 when camera
			 *         not sending compressed data.
			 */
			virtual unsigned int GetReceivedDataSize() const;

			/**
			 * Get the image dimensions associated with the image.
			 *
			 * @param pRows Number of rows.
			 * @param pCols Number of columns.
			 * @param pStride The stride.
			 * @param pPixelFormat Pixel format.
			 * @param pBayerFormat Bayer tile format.
			 */
			virtual void GetDimensions(
					unsigned int*    pRows,
					unsigned int*    pCols = NULL,
					unsigned int*    pStride = NULL,
					PixelFormat*     pPixelFormat = NULL,
					BayerTileFormat* pBayerFormat = NULL ) const;

			/**
			 * Get a pointer to the data associated with the image. This function
			 * is considered unsafe. The pointer returned could be invalidated if
			 * the buffer is resized or released. The pointer may also be
			 * invalidated if the Image object is passed to
			 * Camera::RetrieveBuffer(). It is recommended that a Image::DeepCopy()
			 * be performed if a seperate copy of the Image data is required
			 * for further processing.
			 *
			 * @return A pointer to the image data.
			 */
			virtual unsigned char* GetData();

			virtual unsigned char* const GetData() const;

			/**
			 * Get the metadata associated with the image. This includes
			 * embedded image information.
			 *
			 * @return Metadata associated with the image.
			 */
			virtual ImageMetadata GetMetadata() const;

			/**
			 * Calculate statistics associated with the image. In order to collect
			 * statistics for a particular channel, the enabled flag for the
			 * channel must be set to true. Statistics can only be collected for
			 * images in Mono8, Mono16, RGB, RGBU, BGR and BGRU.
			 *
			 * @param pStatistics The ImageStatistics object to hold the statistics.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error CalculateStatistics( ImageStatistics* pStatistics );

			/**
			 * Get the timestamp data associated with the image.
			 *
			 * @return Timestamp data associated with the image.
			 */
			virtual TimeStamp GetTimeStamp() const;

			/**
			 * Save the image to the specified file name with the file format
			 * specified.
			 *
			 * @param pFilename Filename to save image with.
			 * @param format File format to save in.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Save(
					const char*     pFilename,
					ImageFileFormat format = FROM_FILE_EXT );

			/**
			 * Save the image to the specified file name with the options specified.
			 *
			 * @param pFilename Filename to save image with.
			 * @param pOption Options to use while saving image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Save(
					const char*     pFilename,
					PNGOption*      pOption );

			/**
			 * Save the image to the specified file name with the options specified.
			 *
			 * @param pFilename Filename to save image with.
			 * @param pOption Options to use while saving image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Save(
					const char*     pFilename,
					PPMOption*      pOption );

			/**
			 * Save the image to the specified file name with the options specified.
			 *
			 * @param pFilename Filename to save image with.
			 * @param pOption Options to use while saving image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Save(
					const char*     pFilename,
					PGMOption*      pOption );

			/**
			 * Save the image to the specified file name with the options specified.
			 *
			 * @param pFilename Filename to save image with.
			 * @param pOption Options to use while saving image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Save(
					const char*     pFilename,
					TIFFOption*      pOption );

			/**
			 * Save the image to the specified file name with the options specified.
			 *
			 * @param pFilename Filename to save image with.
			 * @param pOption Options to use while saving image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Save(
					const char*     pFilename,
					JPEGOption*     pOption );

			/**
			 * Save the image to the specified file name with the options specified.
			 *
			 * @param pFilename Filename to save image with.
			 * @param pOption Options to use while saving image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Save(
					const char*     pFilename,
					JPG2Option*     pOption );

			/**
			 * Save the image to the specified file name with the options specified.
			 *
			 * @param pFilename Filename to save image with.
			 * @param pOption Options to use while saving image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Save(
					const char*     pFilename,
					BMPOption*     pOption );

			/**
			 * Converts the current image buffer to the specified output format and
			 * stores the result in the specified image. The destination image
			 * does not need to be configured in any way before the call is made.
			 *
			 * @param format Output format of the converted image.
			 * @param pDestImage Destination image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Convert(
					PixelFormat format,
					Image*      pDestImage ) const;

			/**
			 * Converts the current image buffer to the specified output format and
			 * stores the result in the specified image. The destination image
			 * does not need to be configured in anyway before the call is made.
			 *
			 * @param pDestImage Destination image.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Convert( Image* pDestImage ) const;

			/**
			 * Release the buffer associated with the Image. If no buffer is
			 * associated, the function does nothing.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ReleaseBuffer();

		protected:

		private:
			friend class Iso;
			struct ImageImpl;
			ImageImpl* m_pImpl;
	};
}

#endif //PGR_FC2_IMAGE_H
