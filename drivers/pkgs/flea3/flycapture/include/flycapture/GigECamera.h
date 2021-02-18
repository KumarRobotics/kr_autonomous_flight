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
// $Id: GigECamera.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_GIGECAMERA_H_
#define PGR_FC2_GIGECAMERA_H_

#include "CameraBase.h"

namespace FlyCapture2
{
	/**
	 * The GigECamera object represents a physical Gigabit Ethernet camera.
	 * The object must first be connected to using Connect() before any
	 * other operations can proceed.
	 *
	 * Please see Camera.h for basic functions that this class inherits from.
	 *
	 * @nosubgrouping
	 */
	class FLYCAPTURE2_API GigECamera : public CameraBase
	{
		public:

			/**
			 * Default constructor.
			 */
			GigECamera();

			/**
			 * Default destructor.
			 */
			virtual ~GigECamera();

			/**
			 * @name GVCP Register Operation
			 *
			 * These functions deal with GVCP register operation on the camera.
			 */
			/*@{*/

			/**
			 * Write a GVCP register.
			 *
			 * @param address GVCP address to be written to.
			 * @param value The value to be written.
			 * @param broadcast Whether the action should be broadcast.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error WriteGVCPRegister(
					unsigned int address,
					unsigned int value,
					bool broadcast = false);

			/**
			 * Read a GVCP register.
			 *
			 * @param address GVCP address to be read from.
			 * @param pValue The value that is read.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ReadGVCPRegister(
					unsigned int address,
					unsigned int* pValue );

			/**
			 * Write a GVCP register block.
			 *
			 * @param address GVCP address to be write to.
			 * @param pBuffer Array containing data to be written.
			 * @param length Size of array, in quadlets.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error WriteGVCPRegisterBlock(
					unsigned int address,
					const unsigned int* pBuffer,
					unsigned int length );

			/**
			 * Read a GVCP register block.
			 *
			 * @param address GVCP address to be read from.
			 * @param pBuffer Array for data to be read into.
			 * @param length Size of array, in quadlets.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ReadGVCPRegisterBlock(
					unsigned int address,
					unsigned int* pBuffer,
					unsigned int length );

			/**
			 * Write a GVCP Memory block.
			 *
			 * @param address GVCP address to be write to.
			 * @param pBuffer Array containing data to be written in increments.
			 * @param length Size of array, in quadlets.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error WriteGVCPMemory(
					unsigned int address,
					const unsigned char* pBuffer,
					unsigned int length );

			/**
			 * Read a GVCP memory block.
			 *
			 * @param address GVCP address to be read from.
			 * @param pBuffer Array for data to be read into.
			 * @param length Size of array, in quadlets.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ReadGVCPMemory(
					unsigned int address,
					unsigned char* pBuffer,
					unsigned int length );

			/*@}*/

			/**
			 * @name GigE property manipulation
			 *
			 * These functions deal with GigE properties.
			 */
			/*@{*/

			/**
			 * Get the specified GigEProperty. The GigEPropertyType field must
			 * be set in order for this function to succeed.
			 *
			 * @param pGigEProp The GigE property to get.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetGigEProperty( GigEProperty* pGigEProp );

			/**
			 * Set the specified GigEProperty. The GigEPropertyType field must
			 * be set in order for this function to succeed.
			 *
			 * @param pGigEProp The GigE property to set.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetGigEProperty( const GigEProperty* pGigEProp );

			/**
			 * Discover the largest packet size that works for the network link
			 * between the PC and the camera. This is useful in cases where
			 * there may be multiple links between the PC and the camera and
			 * there is a possiblity of a component not supporting the
			 * recommended jumbo frame packet size of 9000.
			 *
			 * @param packetSize The maximum packet size supported by the link.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error DiscoverGigEPacketSize( unsigned int* packetSize );

			/*@}*/

			/**
			 * @name GigE image settings
			 *
			 * These functions deal with GigE image setting.
			 */
			/*@{*/

			/**
			 * Check if the particular imaging mode is supported by the camera.
			 *
			 * @param mode The mode to check.
			 * @param isSupported Whether the mode is supported.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error QueryGigEImagingMode( Mode mode, bool* isSupported );

			/**
			 * Get the current imaging mode on the camera.
			 *
			 * @param mode Current imaging mode on the camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetGigEImagingMode( Mode* mode );

			/**
			 * Set the current imaging mode to the camera. This should only be
			 * done when the camera is not streaming images.
			 *
			 * @param mode Imaging mode to set to the camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetGigEImagingMode( Mode mode );

			/**
			 * Get information about the image settings possible on the camera.
			 *
			 * @param pInfo Image settings information.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetGigEImageSettingsInfo( GigEImageSettingsInfo* pInfo );

			/**
			 * Get the current image settings on the camera.
			 *
			 * @param pImageSettings Current image settings on camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetGigEImageSettings( GigEImageSettings* pImageSettings );

			/**
			 * Set the image settings specified to the camera.
			 *
			 * @param pImageSettings Image settings to set to camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetGigEImageSettings( const GigEImageSettings* pImageSettings );

			/*@}*/

			/**
			 * @name GigE image binning settings
			 *
			 * These functions deal with GigE image binning settings.
			 */
			/*@{*/

			/**
			 * Get the current binning settings on the camera.
			 *
			 * @param horzBinnningValue Current horizontal binning value.
			 * @param vertBinnningValue Current vertical binning value.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetGigEImageBinningSettings( unsigned int* horzBinnningValue, unsigned int* vertBinnningValue );

			/**
			 * Set the specified binning values to the camera. It is recommended
			 * that GetGigEImageSettingsInfo() be called after this function
			 * succeeds to retrieve the new image settings information for
			 * the new binning mode.
			 *
			 * @param horzBinnningValue Horizontal binning value.
			 * @param vertBinnningValue Vertical binning value.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetGigEImageBinningSettings( unsigned int horzBinnningValue, unsigned int vertBinnningValue );

			/*@}*/

			/**
			 * @name GigE image stream configuration
			 *
			 * These functions deal with GigE image stream configuration.
			 */
			/*@{*/

			/**
			 * Get the number of stream channels present on the camera.
			 *
			 * @param numChannels Number of stream channels present.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetNumStreamChannels( unsigned int* numChannels );

			/**
			 * Get the stream channel information for the specified channel.
			 *
			 * @param channel Channel number to use.
			 * @param pChannel Stream channel information for the specified channel.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetGigEStreamChannelInfo( unsigned int channel, GigEStreamChannel* pChannel );

			/**
			 * Set the stream channel information for the specified channel.
			 *
			 * Note that the source UDP port of the stream channel is read-only.
			 *
			 * @param channel Channel number to use.
			 * @param pChannel Stream channel information to use for the specified channel.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetGigEStreamChannelInfo( unsigned int channel, GigEStreamChannel* pChannel );

			/**
			 * Get the current gige config on the camera.
			 *
			 * @param pGigEConfig Current configuration on camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetGigEConfig( GigEConfig* pGigEConfig );

			/**
			 * Set the gige config specified to the camera.
			 *
			 * @param pGigEConfig configuration to set to camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error SetGigEConfig( const GigEConfig* pGigEConfig );

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

			/**
			 * StartSyncCapture() with GigE Cameras is not supported.
			 * This function has been deprecated and will be removed
			 * in a future version of FlyCapture.
			 */
			static Error StartSyncCapture(
					unsigned int numCameras,
					const GigECamera **ppCameras,
					const ImageEventCallback *pCallbackFns = NULL,
					const void** pCallbackDataArray = NULL );
			virtual Error RetrieveBuffer( Image* pImage );
			virtual Error StopCapture();
			virtual Error WaitForBufferEvent( Image* pImage, unsigned int eventNumber );
			virtual Error SetUserBuffers(
					unsigned char* const pMemBuffers,
					int size,
					int numBuffers );
			virtual Error GetConfiguration( FC2Config* pConfig );
			virtual Error SetConfiguration( const FC2Config* pConfig );
			virtual Error GetCameraInfo( CameraInfo* pCameraInfo );
			virtual Error GetPropertyInfo( PropertyInfo* pPropInfo );
			virtual Error GetProperty( Property* pProp );
			virtual Error SetProperty(
					const Property* pProp,
					bool broadcast = false );
			virtual Error GetGPIOPinDirection( unsigned int pin, unsigned int* pDirection);
			virtual Error SetGPIOPinDirection( unsigned int pin, unsigned int direction, bool broadcast = false );
			virtual Error GetTriggerModeInfo( TriggerModeInfo* pTriggerModeInfo );
			virtual Error GetTriggerMode( TriggerMode* pTriggerMode );
			virtual Error SetTriggerMode(
					const TriggerMode* pTriggerMode,
					bool broadcast = false );
			virtual Error FireSoftwareTrigger( bool broadcast = false );
			virtual Error GetTriggerDelayInfo( TriggerDelayInfo* pTriggerDelayInfo );
			virtual Error GetTriggerDelay( TriggerDelay* pTriggerDelay );
			virtual Error SetTriggerDelay(
					const TriggerDelay* pTriggerDelay,
					bool broadcast = false );
			virtual Error GetStrobeInfo( StrobeInfo* pStrobeInfo );
			virtual Error GetStrobe( StrobeControl* pStrobeControl );
			virtual Error SetStrobe(
					const StrobeControl* pStrobeControl,
					bool broadcast = false );
			virtual Error GetLUTInfo( LUTData* pData );
			virtual Error GetLUTBankInfo(
					unsigned int bank,
					bool* pReadSupported,
					bool* pWriteSupported );
			virtual Error GetActiveLUTBank( unsigned int* pActiveBank );
			virtual Error SetActiveLUTBank( unsigned int activeBank );
			virtual Error EnableLUT( bool on );
			virtual Error GetLUTChannel(
					unsigned int bank,
					unsigned int channel,
					unsigned int sizeEntries,
					unsigned int* pEntries );
			virtual Error SetLUTChannel(
					unsigned int bank,
					unsigned int channel,
					unsigned int sizeEntries,
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
					unsigned int address,
					unsigned int* pValue );
			virtual Error WriteRegisterBlock(
					unsigned short addressHigh,
					unsigned int addressLow,
					const unsigned int* pBuffer,
					unsigned int length );
			virtual Error ReadRegisterBlock(
					unsigned short addressHigh,
					unsigned int addressLow,
					unsigned int* pBuffer,
					unsigned int length );
			static const char* GetRegisterString( unsigned int registerVal);
			Error GetCycleTime(TimeStamp *timeStamp);
			virtual Error GetStats( CameraStats* pStats );
			virtual Error ResetStats();

			virtual Error RegisterEvent( EventOptions* pOpts );
			virtual Error DeregisterEvent( EventOptions* pOpts );
			virtual Error RegisterAllEvents( EventOptions* pOpts );
			virtual Error DeregisterAllEvents( void );
	};
}

#endif // PGR_FC2_GIGECAMERA_H_
