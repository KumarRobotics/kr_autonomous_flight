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
// $Id: CameraBase.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_CAMERABASE_H_
#define PGR_FC2_CAMERABASE_H_

#include "FlyCapture2Platform.h"
#include "FlyCapture2Defs.h"

namespace FlyCapture2
{
	class Error;
	class Image;

	/**
	 * Image event callback function prototype. Defines the syntax of the
	 * image callback function that is passed into StartCapture(). It is
	 * possible for this function to be called simultaneously. Therefore,
	 * users must make sure that code in the callback is thread safe.
	 */
	typedef void (*ImageEventCallback)( class Image* pImage, const void* pCallbackData );

	/**
	 * The CameraBase class is an abstract base class that defines a general
	 * interface to a camera.
	 *
	 * @nosubgrouping
	 */
	class FLYCAPTURE2_API CameraBase
	{
		public:

			/**
			 * Default constructor.
			 */
			CameraBase() {}

			/**
			 * Default destructor.
			 */
			virtual ~CameraBase() {}

			/**
			 * @name Connection and Image Retrieval
			 *
			 * These functions deal with connections and image retrieval from
			 * the camera.
			 */
			/*@{*/

			/**
			 * Connects the camera object to the camera specified by the GUID.
			 * If the guid is omitted or set to NULL, the connection will be made
			 * to the first camera detected on the PC (i.e. index = 0).
			 *
			 * @param pGuid The unique identifier for a specific camera on the PC.
			 *
			 * @see BusManager::GetCameraFromIndex()
			 * @see BusManager::GetCameraFromSerialNumber()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Connect( PGRGuid* pGuid = NULL ) = 0;

			/**
			 * Disconnects the camera object from the camera. This allows another
			 * physical camera specified by a GUID to be connected to the camera
			 * object.
			 *
			 * @see Connect()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error Disconnect() = 0;

			/**
			 * Checks if the camera object is connected to a physical camera
			 * specified by a GUID.
			 *
			 * @see Connect()
			 * @see Disconnect()
			 *
			 * @return Whether Connect() was called on the camera object.
			 */
			virtual bool IsConnected() = 0;

			/**
			 * Sets the callback data to be used on completion of image transfer.
			 * To clear the current stored callback data, pass in NULL for both
			 * arguments.
			 *
			 * @param callbackFn A function to be called when a new image is
			 *                   received.
			 * @param pCallbackData A pointer to data that can be passed to the
			 *                      callback function.
			 *
			 * @see StartCapture()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetCallback(
					ImageEventCallback callbackFn,
					const void* pCallbackData = NULL ) = 0;

			/**
			 * Starts isochronous image capture. It will use either the current
			 * video mode or the most recently set video mode of the camera.
			 * The optional callback function parameter is called on completion of
			 * image transfer.
			 * When a callback function is specified, the grab mode will determine how
			 * images are delivered.
			 * If the grab mode has not been set, or has been set to DROP_FRAMES
			 * the default behavior is to requeue images for DMA if they have
			 * not been delivered by the time the next image transfer completes.
			 * If BUFFER_FRAMES is specified, the next image in the sequence will
			 * be delivered. Note that for the BUFFER_FRAMES case, if delivery
			 * does not keep up with the DMA process, images will be lost.
			 * The default behavior is to perform DROP_FRAMES image delivery
			 * Alternatively, the callback parameter can be set to NULL
			 * and RetrieveBuffer() can be called as a blocking call to get
			 * the image data.
			 *
			 * @param callbackFn A function to be called when a new image is
			 *                   received.
			 * @param pCallbackData A pointer to data that can be passed to the
			 *                      callback function.
			 *
			 * @see RetrieveBuffer()
			 * @see StartSyncCapture()
			 * @see StopCapture()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error StartCapture(
					ImageEventCallback callbackFn = NULL,
					const void* pCallbackData = NULL ) = 0;

			/**
			 * Starts isochronous image capture on multiple cameras. On each frame,
			 * the time stamps across the cameras are aligned which means the frames
			 * are synchronized. Note that the cameras must be synchronized by
			 * external means in order for this function to work. This means that
			 * the cameras should either be on the same bus, hardware synchronized
			 * (e.g. through triggering) or Multisync is running.
			 * Note: The use of this function with GigE Cameras is not supported.
			 *
			 * @param numCameras Number of Camera objects in the ppCameras array.
			 * @param ppCameras Array of pointers to Camera objects containing the
			 *                  cameras to be started and synchronized.
			 * @param pCallbackFns Array of callback functions for each camera.
			 * @param pCallbackDataArray Array of callback data pointers.
			 *
			 * @see RetrieveBuffer()
			 * @see StartCapture()
			 * @see StopCapture()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error StartSyncCapture(
					unsigned int numCameras,
					const CameraBase **ppCameras,
					const ImageEventCallback* pCallbackFns = NULL,
					const void** pCallbackDataArray = NULL );

			/**
			 * Retrieves the the next image object containing the next image.
			 * If the grab mode has not been set, or has been set to DROP_FRAMES
			 * the default behavior is to requeue images for DMA if they have
			 * not been retrieved by the time the next image transfer completes.
			 * If BUFFER_FRAMES is specified, the next image in the sequence will
			 * be retrieved.  Note that for the BUFFER_FRAMES case, if retrieval
			 * does not keep up with the DMA process, images will be lost.
			 * The default behavior is to perform DROP_FRAMES image retrieval.
			 *
			 * @param pImage Pointer to Image object to store image data.
			 *
			 * @see StartCapture()
			 * @see StopCapture()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error RetrieveBuffer( Image* pImage ) = 0;

			/**
			 * Stops isochronous image transfer and cleans up all associated
			 * resources.
			 * If an image callback function (specified in the StartCapture() call)
			 * is currently executing, StopCapture() will not return until after
			 * the callback has completed.
			 *
			 * @see StartCapture()
			 * @see RetrieveBuffer()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error StopCapture() = 0;

			/**
			 * Retrieves the next image event containing the next part of the image.
			 *
			 * @param pImage Pointer to Image object to store image data.
			 * @param eventNumber The event number to wait for.
			 *
			 * @see StartCapture()
			 * @see RetrieveBuffer()
			 * @see StopCapture()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error WaitForBufferEvent( Image* pImage, unsigned int eventNumber ) = 0;

			/**
			 * Specify user allocated buffers to use as image data buffers.
			 * To prevent image tearing, the size of each buffer should be equal to
			 * ((unsigned int)(bufferSize + packetSize - 1)/packetSize) * packetSize.
			 * The total size should be (size * numBuffers) or larger.
			 * The packet Size that should be used differs between interfaces:
			 *	Firewire:	Use the Format7 packet size.
			 *  Usb2:		First round to Format7 packet size then round to 512 bytes.
			 *  Usb3:		Use a packet size of 1024 bytes.
			 *  GigE:		No need to do any rounding on GigE
			 *
			 * @param pMemBuffers Pointer to memory buffers to be written to.
			 * @param size The size of each buffer (in bytes).
			 * @param numBuffers Number of buffers in the array.
			 *
			 * @see StartCapture()
			 * @see RetrieveBuffer()
			 * @see StopCapture()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetUserBuffers(
					unsigned char* const    pMemBuffers,
					int                     size,
					int                     numBuffers ) = 0;

			/**
			 * Get the configuration associated with the camera object.
			 *
			 * @param pConfig Pointer to the configuration structure to be filled.
			 *
			 * @see SetConfiguration()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetConfiguration( FC2Config* pConfig ) = 0;

			/**
			 * Set the configuration associated with the camera object.
			 *
			 * @param pConfig Pointer to the configuration structure to be used.
			 *
			 * @see GetConfiguration()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetConfiguration( const FC2Config* pConfig ) = 0;

			/*@}*/

			/**
			 * @name Information and Properties
			 *
			 * These functions deal with information and properties can be
			 * retrieved from the camera.
			 */
			/*@{*/

			/**
			 * Retrieves information from the camera such as serial number, model
			 * name and other camera information.
			 *
			 * @param pCameraInfo Pointer to the camera information structure
			 *                    to be filled.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetCameraInfo( CameraInfo* pCameraInfo ) = 0;

			/**
			 * Retrieves information about the specified camera property. The
			 * property type must be specified in the PropertyInfo structure
			 * passed into the function in order for the function to succeed.
			 *
			 * @param pPropInfo Pointer to the PropertyInfo structure to be filled.
			 *
			 * @see GetProperty()
			 * @see SetProperty()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetPropertyInfo( PropertyInfo* pPropInfo ) = 0;

			/**
			 * Reads the settings for the specified property from the camera. The
			 * property type must be specified in the Property structure passed
			 * into the function in order for the function to succeed. If auto
			 * is on, the integer and abs values returned may not be consistent
			 * with each other.
			 *
			 * @param pProp Pointer to the Property structure to be filled.
			 *
			 * @see GetPropertyInfo()
			 * @see SetProperty()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetProperty( Property* pProp ) = 0;

			/**
			 * Writes the settings for the specified property to the camera. The
			 * property type must be specified in the Property structure passed
			 * into the function in order for the function to succeed.
			 * The absControl flag controls whether the absolute or integer value
			 * is written to the camera. Use GetPropertyInfo() to query which 
			 * options are available for a specific property.
			 *
			 * @param pProp Pointer to the Property structure to be used.
			 * @param broadcast Whether the action should be broadcast.
			 *
			 * @see GetPropertyInfo()
			 * @see GetProperty()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetProperty(
					const Property* pProp,
					bool            broadcast = false ) = 0;

			/*@}*/

			/**
			 * @name General Purpose Input / Output
			 *
			 * These functions deal with general GPIO pin control on the camera.
			 */
			/*@{*/

			/**
			 * Get the GPIO pin direction for the specified pin. This is not a
			 * required call when using the trigger or strobe functions as
			 * the pin direction is set automatically internally.
			 *
			 * @param pin Pin to get the direction for.
			 * @param pDirection Direction of the pin. 0 for input, 1 for output.
			 *
			 * @see SetGPIOPinDirection()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetGPIOPinDirection( unsigned int pin, unsigned int* pDirection) = 0;

			/**
			 * Set the GPIO pin direction for the specified pin. This is useful if
			 * there is a need to set the pin into an input pin (i.e. to read the
			 * voltage) off the pin without setting it as a trigger source. This
			 * is not a required call when using the trigger or strobe functions as
			 * the pin direction is set automatically internally.
			 *
			 * @param pin Pin to get the direction for.
			 * @param direction Direction of the pin. 0 for input, 1 for output.
			 * @param broadcast Whether the action should be broadcast.
			 *
			 * @see GetGPIOPinDirection()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetGPIOPinDirection( unsigned int pin, unsigned int direction, bool broadcast = false ) = 0;

			/*@}*/

			/**
			 * @name Trigger
			 *
			 * These functions deal with trigger control on the camera.
			 */
			/*@{*/

			/**
			 * Retrieve trigger information from the camera.
			 *
			 * @param pTriggerModeInfo Structure to receive trigger information.
			 *
			 * @see GetTriggerMode()
			 * @see SetTriggerMode()
			 * @see GetTriggerDelayInfo()
			 * @see GetTriggerDelay()
			 * @see SetTriggerDelay()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetTriggerModeInfo( TriggerModeInfo* pTriggerModeInfo ) = 0;

			/**
			 * Retrieve current trigger settings from the camera.
			 *
			 * @param pTriggerMode Structure to receive trigger mode settings.
			 *
			 * @see GetTriggerModeInfo()
			 * @see SetTriggerMode()
			 * @see GetTriggerDelayInfo()
			 * @see GetTriggerDelay()
			 * @see SetTriggerDelay()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetTriggerMode( TriggerMode* pTriggerMode ) = 0;

			/**
			 * Set the specified trigger settings to the camera.
			 *
			 * @param pTriggerMode Structure providing trigger mode settings.
			 * @param broadcast Whether the action should be broadcast.
			 *
			 * @see GetTriggerModeInfo()
			 * @see GetTriggerMode()
			 * @see GetTriggerDelayInfo()
			 * @see GetTriggerDelay()
			 * @see SetTriggerDelay()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetTriggerMode(
					const TriggerMode*  pTriggerMode,
					bool                broadcast = false ) = 0;

			/**
			 * Fire the software trigger according to the DCAM specifications.
			 *
			 * @param broadcast Whether the action should be broadcast.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error FireSoftwareTrigger( bool broadcast = false ) = 0;

			/**
			 * Retrieve trigger delay information from the camera.
			 *
			 * @param pTriggerDelayInfo Structure to receive trigger delay information.
			 *
			 * @see GetTriggerModeInfo()
			 * @see GetTriggerMode()
			 * @see SetTriggerMode()
			 * @see GetTriggerDelay()
			 * @see SetTriggerDelay()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetTriggerDelayInfo( TriggerDelayInfo* pTriggerDelayInfo ) = 0;

			/**
			 * Retrieve current trigger delay settings from the camera.
			 *
			 * @param pTriggerDelay Structure to receive trigger delay settings.
			 *
			 * @see GetTriggerModeInfo()
			 * @see GetTriggerMode()
			 * @see SetTriggerMode()
			 * @see GetTriggerDelayInfo()
			 * @see SetTriggerDelay()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetTriggerDelay( TriggerDelay* pTriggerDelay ) = 0;

			/**
			 * Set the specified trigger delay settings to the camera.
			 *
			 * @param pTriggerDelay Structure providing trigger delay settings.
			 * @param broadcast Whether the action should be broadcast.
			 *
			 * @see GetTriggerModeInfo()
			 * @see GetTriggerMode()
			 * @see SetTriggerMode()
			 * @see GetTriggerDelayInfo()
			 * @see GetTriggerDelay()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetTriggerDelay(
					const TriggerDelay* pTriggerDelay,
					bool                broadcast = false ) = 0;

			/*@}*/

			/**
			 * @name Strobe
			 *
			 * These functions deal with strobe control on the camera.
			 */
			/*@{*/

			/**
			 * Retrieve strobe information from the camera.
			 *
			 * @param pStrobeInfo Structure to receive strobe information.
			 *
			 * @see GetStrobe()
			 * @see SetStrobe()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetStrobeInfo( StrobeInfo* pStrobeInfo ) = 0;

			/**
			 * Retrieve current strobe settings from the camera. The strobe pin
			 * must be specified in the structure before being passed in to
			 * the function.
			 *
			 * @param pStrobeControl Structure to receive strobe settings.
			 *
			 * @see GetStrobeInfo()
			 * @see SetStrobe()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetStrobe( StrobeControl* pStrobeControl ) = 0;

			/**
			 * Set current strobe settings to the camera. The strobe pin
			 * must be specified in the structure before being passed in to
			 * the function.
			 *
			 * @param pStrobeControl Structure providing strobe settings.
			 * @param broadcast Whether the action should be broadcast.
			 *
			 * @see GetStrobeInfo()
			 * @see GetStrobe()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetStrobe(
					const StrobeControl* pStrobeControl,
					bool                 broadcast = false ) = 0;

			/*@}*/

			/**
			 * @name Look Up Table
			 *
			 * These functions deal with Look Up Table control on the camera.
			 */
			/*@{*/

			/**
			 * Query if LUT support is available on the camera. Note that some cameras
			 * may report support for the LUT and return an inputBitDepth of 0. In these
			 * cases use log2(numEntries) for the inputBitDepth.
			 *
			 * @param pData The LUT structure to be filled.
			 *
			 * @see EnableLUT()
			 * @see GetLUTChannel()
			 * @see SetLUTChannel()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetLUTInfo( LUTData* pData ) = 0;

			/**
			 * Query the read/write status of a single LUT bank.
			 *
			 * @param bank The bank to query.
			 * @param pReadSupported Whether reading from the bank is supported.
			 * @param pWriteSupported Whether writing to the bank is supported.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetLUTBankInfo(
					unsigned int bank,
					bool* pReadSupported,
					bool* pWriteSupported ) = 0;

			/**
			 * Get the LUT bank that is currently being used. For cameras with
			 * PGR LUT, the active bank is always 0.
			 *
			 * @param pActiveBank The currently active bank.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetActiveLUTBank( unsigned int* pActiveBank ) = 0;

			/**
			 * Set the LUT bank that will be used.
			 *
			 * @param activeBank The bank to be set as active.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetActiveLUTBank( unsigned int activeBank ) = 0;

			/**
			 * Enable or disable LUT functionality on the camera.
			 *
			 * @param on Whether to enable or disable LUT.
			 *
			 * @see GetLUTInfo()
			 * @see GetLUTChannel()
			 * @see SetLUTChannel()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error EnableLUT( bool on ) = 0;

			/**
			 * Get the LUT channel settings from the camera.
			 *
			 * @param bank Bank to retrieve.
			 * @param channel Channel to retrieve.
			 * @param sizeEntries Number of entries in LUT table to read.
			 * @param pEntries Array to store LUT entries.
			 *
			 * @see GetLUTInfo()
			 * @see EnableLUT()
			 * @see SetLUTChannel()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetLUTChannel(
					unsigned int  bank,
					unsigned int  channel,
					unsigned int  sizeEntries,
					unsigned int* pEntries ) = 0;

			/**
			 * Set the LUT channel settings to the camera.
			 *
			 * @param bank Bank to set.
			 * @param channel Channel to set.
			 * @param sizeEntries Number of entries in LUT table to write. This must be the
			 *					  same size as numEntries returned by GetLutInfo().
			 * @param pEntries Array containing LUT entries to write.
			 *
			 * @see GetLUTInfo()
			 * @see EnableLUT()
			 * @see GetLUTChannel()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetLUTChannel(
					unsigned int        bank,
					unsigned int        channel,
					unsigned int        sizeEntries,
					const unsigned int* pEntries ) = 0;

			/*@}*/

			/**
			 * @name Memory Channels
			 *
			 * These functions deal with memory channel control on the camera.
			 */
			/*@{*/

			/**
			 * Retrieve the current memory channel from the camera.
			 *
			 * @param pCurrentChannel Current memory channel.
			 *
			 * @see SaveToMemoryChannel()
			 * @see RestoreFromMemoryChannel()
			 * @see GetMemoryChannelInfo()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetMemoryChannel( unsigned int* pCurrentChannel ) = 0;

			/**
			 * Save the current settings to the specfied current memory channel.
			 *
			 * @param channel Memory channel to save to.
			 *
			 * @see GetMemoryChannel()
			 * @see RestoreFromMemoryChannel()
			 * @see GetMemoryChannelInfo()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SaveToMemoryChannel( unsigned int channel ) = 0;

			/**
			 * Restore the specfied current memory channel.
			 *
			 * @param channel Memory channel to restore from.
			 *
			 * @see GetMemoryChannel()
			 * @see SaveToMemoryChannel()
			 * @see GetMemoryChannelInfo()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error RestoreFromMemoryChannel( unsigned int channel ) = 0;

			/**
			 * Query the camera for memory channel support. If the number of
			 * channels is 0, then memory channel support is not available.
			 *
			 * @param pNumChannels Number of memory channels supported.
			 *
			 * @see GetMemoryChannel()
			 * @see SaveToMemoryChannel()
			 * @see RestoreFromMemoryChannel()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetMemoryChannelInfo( unsigned int* pNumChannels ) = 0;

			/*@}*/

			/**
			 * @name Embedded Image Information
			 *
			 * These functions deal with embedded image information control
			 * on the camera.
			 */
			/*@{*/

			/**
			 * Get the current status of the embedded image information register,
			 * as well as the availability of each embedded property.
			 *
			 * @param pInfo Structure to be filled.
			 *
			 * @see SetEmbeddedImageInfo()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetEmbeddedImageInfo( EmbeddedImageInfo* pInfo ) = 0;

			/**
			 * Sets the on/off values of the embedded image information structure
			 * to the camera.
			 *
			 * @param pInfo Structure to be used.
			 *
			 * @see GetEmbeddedImageInfo()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetEmbeddedImageInfo( EmbeddedImageInfo* pInfo ) = 0;

			/*@}*/

			/**
			 * @name Register Operation
			 *
			 * These functions deal with register operation on the camera.
			 */
			/*@{*/

			/**
			 * Write to the specified register on the camera.
			 *
			 * @param address DCAM address to be written to.
			 * @param value The value to be written.
			 * @param broadcast Whether the action should be broadcast.
			 *
			 * @see ReadRegister()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error WriteRegister(
					unsigned int address,
					unsigned int value,
					bool broadcast=false) = 0;

			/**
			 * Read the specified register from the camera.
			 *
			 * @param address DCAM address to be read from.
			 * @param pValue The value that is read.
			 *
			 * @see WriteRegister()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ReadRegister(
					unsigned int  address,
					unsigned int* pValue ) = 0;

			/**
			 * Write to the specified register block on the camera.
			 *
			 * @param addressHigh Top 16 bits of the 48 bit absolute address to
			 *                    write to.
			 * @param addressLow Bottom 32 bits of the 48 bits absolute address to
			 *                   write to.
			 * @param pBuffer Array containing data to be written.
			 * @param length Size of array, in quadlets.
			 *
			 * @see ReadRegisterBlock()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error WriteRegisterBlock(
					unsigned short       addressHigh,
					unsigned int         addressLow,
					const unsigned int*  pBuffer,
					unsigned int         length ) = 0;

			/**
			 * Read from the specified register block on the camera.
			 *
			 * @param addressHigh Top 16 bits of the 48 bit absolute address to
			 *                    read from.
			 * @param addressLow Bottom 32 bits of the 48 bits absolute address to
			 *                   read from.
			 * @param pBuffer Array to store read data.
			 * @param length Size of array, in quadlets.
			 *
			 * @see WriteRegisterBlock()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ReadRegisterBlock(
					unsigned short addressHigh,
					unsigned int   addressLow,
					unsigned int*  pBuffer,
					unsigned int   length ) = 0;

			/**
			 * Returns a text representation of the register value.
			 *
			 * @param registerVal The register value to query.
			 *
			 * @return The text representation of the register.
			 */
			static const char* GetRegisterString( unsigned int registerVal);

			/**
			 * Returns a Timestamp struct containing 1394 CYCLE_TIME information
			 *
			 * @param registerVal The register value to query.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetCycleTime(TimeStamp *timeStamp) = 0;

			/*
			 * Returns the camera diagnostic infomation.
			 *
			 * @param pStats Pointer to the CameraStats structure.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetStats( CameraStats* pStats ) = 0;

			/*
			 * Reset the camera diagnostic infomation.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ResetStats() = 0;

			/*
			 * Register the camera to issue a custom callback function call for a
			 * specific device event.
			 *
			 * @param pOpts Pointer to the EventOptions structure which defines the
			 *              callback function to use, the event for which to register
			 *              the device, and a pointer to user data (optional) to be
			 *              passed to the callback function.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error RegisterEvent( EventOptions* pOpts ) = 0;

			/*
			 * De-register an event previously registered with the camera.
			 *
			 * @param pOpts Pointer to the EventOptions structure which defines the
			 *              callback function to use, the event for which to register
			 *              the device, and a pointer to user data (optional) to be
			 *              passed to the callback function. The callback function and
			 *              user data elements of the EventOptions structure are ignored
			 *              in this call, and just the event name within the structure is
			 *              used with this function call.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error DeregisterEvent( EventOptions* pOpts ) = 0;
			
			/*
			 * Register the camera to issue a custom callback function call for a
			 * specific device event.
			 *
			 * @param pOpts Pointer to the EventOptions structure which defines the
			 *              callback function to use, the event for which to register
			 *              the device, and a pointer to user data (optional) to be
			 *              passed to the callback function. The event name element of
			 *              the structure is ignored with this function call. If a single
			 *              event has already been registered via RegisterEvent(), this
			 *              call will fail, as the user could accidentally change the
			 *              the internal callback function pointer for a queued event.
			 *              The user will need to de-register all registered events,
			 *              then call this function again.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error RegisterAllEvents( EventOptions* pOpts ) = 0;

			/*
			 * De-register all events registered with the camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error DeregisterAllEvents( void ) = 0;

			/*@}*/

		protected:
			struct CameraData; // Forward declaration
			CameraData* m_pCameraData;

		private:
			CameraBase( const CameraBase& );
			CameraBase& operator=( const CameraBase& );
	};
}

#endif // PGR_FC2_CAMERABASE_H_
