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
// $Id: Camera.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_CAMERA_H_
#define PGR_FC2_CAMERA_H_

#include "CameraBase.h"

namespace FlyCapture2
{
	/**
	 * The Camera object represents a physical camera that uses the IIDC
	 * register set. The object must first be connected to using Connect()
	 * before any other operations can proceed.
	 *
	 * It is possible for more than 1 Camera object to connect to a single
	 * physical camera. However, isochronous transmission to more than
	 * 1 Camera object is not supported.
	 *
	 * @nosubgrouping
	 */
	class FLYCAPTURE2_API Camera : public CameraBase
	{
		public:

			/**
			 * Default constructor.
			 */
			Camera();

			/**
			 * Default destructor.
			 */
			virtual ~Camera();

			/**
			 * @name DCAM Formats
			 *
			 * These functions deal with DCAM video mode and frame rate
			 * on the camera.
			 */
			/*@{*/

			/**
			 * Query the camera to determine if the specified video mode and
			 * frame rate is supported.
			 *
			 * @param videoMode Video mode to check.
			 * @param frameRate Frame rate to check.
			 * @param pSupported Whether the video mode and frame rate is
			 *                   supported.
			 *
			 * @see GetVideoModeAndFrameRate()
			 * @see SetVideoModeAndFrameRate()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetVideoModeAndFrameRateInfo(
					VideoMode videoMode,
					FrameRate frameRate,
					bool*     pSupported);

			/**
			 * Get the current video mode and frame rate from the camera. If
			 * the camera is in Format7, the video mode will be VIDEOMODE_FORMAT7
			 * and the frame rate will be FRAMERATE_FORMAT7.
			 *
			 * @param pVideoMode Current video mode.
			 * @param pFrameRate Current frame rate.
			 *
			 * @see GetVideoModeAndFrameRateInfo()
			 * @see SetVideoModeAndFrameRate()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetVideoModeAndFrameRate(
					VideoMode* pVideoMode,
					FrameRate* pFrameRate );

			/**
			 * Set the specified video mode and frame rate to the camera. It is
			 * not possible to set the camera to VIDEOMODE_FORMAT7 or
			 * FRAMERATE_FORMAT7. Use the Format7 functions to set the camera
			 * into Format7.
			 *
			 * @param videoMode Video mode to set to camera.
			 * @param frameRate Frame rate to set to camera.
			 *
			 * @see GetVideoModeAndFrameRateInfo()
			 * @see GetVideoModeAndFrameRate()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetVideoModeAndFrameRate(
					VideoMode videoMode,
					FrameRate frameRate );

			/*@}*/

			/**
			 * @name Format7
			 *
			 * These functions deal with Format7 custom image control on the camera.
			 */
			/*@{*/

			/**
			 * Retrieve the availability of Format7 custom image mode and the
			 * camera capabilities for the specified Format7 mode. The mode must
			 * be specified in the Format7Info structure in order for the
			 * function to succeed.
			 *
			 * @param pInfo Structure to be filled with the capabilities of the
			 *              specified mode and the current state in the specified
			 *              mode.
			 * @param pSupported Whether the specified mode is supported.
			 *
			 * @see ValidateFormat7Settings()
			 * @see GetFormat7Configuration()
			 * @see SetFormat7Configuration()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetFormat7Info(
					Format7Info*   pInfo,
					bool*          pSupported );

			/**
			 * Validates Format7ImageSettings structure and returns valid packet
			 * size information if the image settings are valid. The current
			 * image settings are cached while validation is taking place. The
			 * cached settings are restored when validation is complete.
			 *
			 * @param pImageSettings Structure containing the image settings.
			 * @param pSettingsAreValid Whether the settings are valid.
			 * @param pPacketInfo Packet size information that can be used to
			 *                    determine a valid packet size.
			 *
			 * @see GetFormat7Info()
			 * @see GetFormat7Configuration()
			 * @see SetFormat7Configuration()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ValidateFormat7Settings(
					const Format7ImageSettings* pImageSettings,
					bool*                       pSettingsAreValid,
					Format7PacketInfo*          pPacketInfo );

			/**
			 * Get the current Format7 configuration from the camera. This call
			 * will only succeed if the camera is already in Format7.
			 *
			 * @param pImageSettings Current image settings.
			 * @param pPacketSize Current packet size.
			 * @param pPercentage Current packet size as a percentage.
			 *
			 * @see GetFormat7Info()
			 * @see ValidateFormat7Settings()
			 * @see SetFormat7Configuration()
			 * @see GetVideoModeAndFrameRate()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetFormat7Configuration(
					Format7ImageSettings* pImageSettings,
					unsigned int*         pPacketSize,
					float*                pPercentage );

			/**
			 * Set the current Format7 configuration to the camera.
			 *
			 * @param pImageSettings Image settings to be written to the camera.
			 * @param packetSize Packet size to be written to the camera.
			 *
			 * @see GetFormat7Info()
			 * @see ValidateFormat7Settings()
			 * @see GetFormat7Configuration()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetFormat7Configuration(
					const Format7ImageSettings* pImageSettings,
					unsigned int                packetSize );

			/**
			 * Set the current Format7 configuration to the camera.
			 *
			 * @param pImageSettings Image settings to be written to the camera.
			 * @param percentSpeed Percentage of packet size to be written to
			 *                     the camera.
			 *
			 * @see GetFormat7Info()
			 * @see ValidateFormat7Settings()
			 * @see GetFormat7Configuration()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetFormat7Configuration(
					const Format7ImageSettings* pImageSettings,
					float                       percentSpeed );

			/*@}*/

			/**
			 * The following functions are inherited from CameraBase. See
			 * CameraBase.h for further information.
			 */

			virtual Error Connect( PGRGuid* pGuid = NULL );
			virtual Error Disconnect();
			virtual bool IsConnected();
			virtual Error SetCallback(
					ImageEventCallback callbackFn,
					const void* pCallbackData = NULL );
			virtual Error StartCapture(
					ImageEventCallback callbackFn = NULL,
					const void* pCallbackData = NULL );
			static Error StartSyncCapture(
					unsigned int numCameras,
					const Camera **ppCameras,
					const ImageEventCallback *pCallbackFns = NULL,
					const void** pCallbackDataArray = NULL );
			virtual Error RetrieveBuffer( Image* pImage );
			virtual Error StopCapture();
			virtual Error WaitForBufferEvent( Image* pImage, unsigned int eventNumber );
			virtual Error SetUserBuffers(
					unsigned char* const    pMemBuffers,
					int                     size,
					int                     numBuffers );
			virtual Error GetConfiguration( FC2Config* pConfig );
			virtual Error SetConfiguration( const FC2Config* pConfig );
			virtual Error GetCameraInfo( CameraInfo* pCameraInfo );
			virtual Error GetPropertyInfo( PropertyInfo* pPropInfo );
			virtual Error GetProperty( Property* pProp );
			virtual Error SetProperty(
					const Property* pProp,
					bool            broadcast = false );
			virtual Error GetGPIOPinDirection( unsigned int pin, unsigned int* pDirection);
			virtual Error SetGPIOPinDirection( unsigned int pin, unsigned int direction, bool broadcast = false );
			virtual Error GetTriggerModeInfo( TriggerModeInfo* pTriggerModeInfo );
			virtual Error GetTriggerMode( TriggerMode* pTriggerMode );
			virtual Error SetTriggerMode(
					const TriggerMode*  pTriggerMode,
					bool                broadcast = false );
			virtual Error FireSoftwareTrigger( bool broadcast = false );
			virtual Error GetTriggerDelayInfo( TriggerDelayInfo* pTriggerDelayInfo );
			virtual Error GetTriggerDelay( TriggerDelay* pTriggerDelay );
			virtual Error SetTriggerDelay(
					const TriggerDelay* pTriggerDelay,
					bool                broadcast = false );
			virtual Error GetStrobeInfo( StrobeInfo* pStrobeInfo );
			virtual Error GetStrobe( StrobeControl* pStrobeControl );
			virtual Error SetStrobe(
					const StrobeControl* pStrobeControl,
					bool                 broadcast = false );
			virtual Error GetLUTInfo( LUTData* pData );
			virtual Error GetLUTBankInfo(
					unsigned int bank,
					bool* pReadSupported,
					bool* pWriteSupported );
			virtual Error GetActiveLUTBank( unsigned int* pActiveBank );
			virtual Error SetActiveLUTBank( unsigned int activeBank );
			virtual Error EnableLUT( bool on );
			virtual Error GetLUTChannel(
					unsigned int  bank,
					unsigned int  channel,
					unsigned int  sizeEntries,
					unsigned int* pEntries );
			virtual Error SetLUTChannel(
					unsigned int        bank,
					unsigned int        channel,
					unsigned int        sizeEntries,
					const unsigned int* pEntries );
			virtual Error GetMemoryChannel( unsigned int* pCurrentChannel );
			virtual Error SaveToMemoryChannel( unsigned int channel );
			virtual Error RestoreFromMemoryChannel( unsigned int channel );
			virtual Error GetMemoryChannelInfo( unsigned int* pNumChannels );
			virtual Error GetEmbeddedImageInfo( EmbeddedImageInfo* pInfo );
			virtual Error SetEmbeddedImageInfo( EmbeddedImageInfo* pInfo );
			virtual Error WriteRegister(
					unsigned int address,
					unsigned int value,
					bool broadcast=false);
			virtual Error ReadRegister(
					unsigned int  address,
					unsigned int* pValue );
			virtual Error WriteRegisterBlock(
					unsigned short       addressHigh,
					unsigned int         addressLow,
					const unsigned int*  pBuffer,
					unsigned int         length );
			virtual Error ReadRegisterBlock(
					unsigned short addressHigh,
					unsigned int   addressLow,
					unsigned int*  pBuffer,
					unsigned int   length );
			static const char* GetRegisterString( unsigned int registerVal);
			virtual Error GetCycleTime(TimeStamp *timeStamp);
			virtual Error GetStats( CameraStats* pStats );
			virtual Error ResetStats();

			virtual Error RegisterEvent( EventOptions* pOpts );
			virtual Error DeregisterEvent( EventOptions* pOpts );
			virtual Error RegisterAllEvents( EventOptions* pOpts );
			virtual Error DeregisterAllEvents( void );

	};
}

#endif // PGR_FC2_CAMERA_H_
