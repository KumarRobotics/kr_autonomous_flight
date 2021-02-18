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
// $Id: BusManager.h 317385 2017-02-28 18:28:29Z corinal $
//=============================================================================

#ifndef PGR_FC2_BUSMANAGER_H
#define PGR_FC2_BUSMANAGER_H

#include "FlyCapture2Platform.h"
#include "FlyCapture2Defs.h"

namespace FlyCapture2
{
	class Error;
	class TopologyNode;

	/**
	 * Bus event callback function prototype. Defines the syntax of the
	 * callback function that is passed into RegisterCallback() and
	 * UnregisterCallback(). It is recommended that minimal handling be
	 * performed in this callback as it will block internal processing
	 * of bus events until it returns.
	 */
	typedef void (*BusEventCallback)( void* pParameter, unsigned int serialNumber );

	/**
	 * Handle that is returned when registering a callback. It is required
	 * when unregistering the callback.
	 */
	typedef void* CallbackHandle;

	/**
	 * The BusManager class provides the functionality for the user to get an
	 * PGRGuid for a desired camera or device easily. Once the
	 * camera or device token is found, it can then be used to connect to the
	 * camera or device through the camera class or device class. In addition,
	 * the BusManager class provides the ability to be notified when a camera
	 * or device is added or removed or some event occurs on the PC.
	 */
	class FLYCAPTURE2_API BusManager
	{
		public:

			/**
			 * Default constructor.
			 */
			BusManager();

			/**
			 * Default destructor.
			 */
			virtual ~BusManager();

			/**
			 * Fire a bus reset. The actual bus reset is only fired for the
			 * specified 1394 bus, but it will effectively cause a global bus
			 * reset for the library.
			 *
			 * @param pGuid PGRGuid of the camera or the device to cause bus reset.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error FireBusReset( PGRGuid* pGuid);

			/**
			 * Gets the number of cameras attached to the PC.
			 *
			 * @param pNumCameras The number of cameras attached.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetNumOfCameras( unsigned int* pNumCameras );

			/**
			 * Gets the PGRGuid for a camera with the specified IPv4 address.
			 *
			 * @param ipAddress IP address to get GUID for.
			 * @param pGuid Unique PGRGuid for the camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetCameraFromIPAddress(
					IPAddress ipAddress,
					PGRGuid* pGuid );

			/**
			 * Gets the PGRGuid for a camera on the PC. It uniquely identifies
			 * the camera specified by the index and is used to identify the camera
			 * during a Camera::Connect() call.
			 *
			 * @param index Zero based index of camera.
			 * @param pGuid Unique PGRGuid for the camera.
			 *
			 * @see GetCameraFromSerialNumber()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetCameraFromIndex(
					unsigned int index,
					PGRGuid*     pGuid );

			/**
			 * Gets the PGRGuid for a camera on the PC. It uniquely identifies
			 * the camera specified by the serial number and is used to identify the camera
			 * during a Camera::Connect() call.
			 *
			 * @param serialNumber Serial number of camera.
			 * @param pGuid Unique PGRGuid for the camera.
			 *
			 * @see GetCameraFromIndex()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetCameraFromSerialNumber(
					unsigned int serialNumber,
					PGRGuid*     pGuid );

			/**
			 * Gets the serial number of the camera with the specified index.
			 *
			 * @param index Zero based index of desired camera.
			 * @param pSerialNumber Serial number of camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetCameraSerialNumberFromIndex(
					unsigned int  index,
					unsigned int* pSerialNumber );

			/**
			 * Gets the interface type associated with a PGRGuid. This is useful
			 * in situations where there is a need to enumerate all cameras
			 * for a particular interface.
			 *
			 * @param pGuid The PGRGuid to get the interface for.
			 * @param pInterfaceType The interface type of the PGRGuid.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetInterfaceTypeFromGuid(
					PGRGuid *pGuid,
					InterfaceType* pInterfaceType );

			/**
			 * Gets the number of devices. This may include hubs, host controllers
			 * and other hardware devices (including cameras).
			 *
			 * @param pNumDevices The number of devices found.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetNumOfDevices( unsigned int* pNumDevices );

			/**
			 * Gets the PGRGuid for a device. It uniquely identifies the device
			 * specified by the index.
			 *
			 * @param index Zero based index of device.
			 * @param pGuid Unique PGRGuid for the device.
			 *
			 * @see GetNumOfDevices()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetDeviceFromIndex(
					unsigned int index,
					PGRGuid*     pGuid );

			/**
			 * Read a phy register on the specified device. The full address
			 * to be read from is determined by the page, port and address.
			 *
			 * @param guid PGRGuid of the device to read from.
			 * @param page Page to read from.
			 * @param port Port to read from.
			 * @param address Address to read from.
			 * @param pValue Value read from the phy register.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error ReadPhyRegister(
					PGRGuid guid,
					unsigned int page,
					unsigned int port,
					unsigned int address,
					unsigned int* pValue );

			/**
			 * Write a phy register on the specified device. The full address
			 * to be written to is determined by the page, port and address.
			 *
			 * @param guid PGRGuid of the device to write to.
			 * @param page Page to write to.
			 * @param port Port to write to.
			 * @param address Address to write to.
			 * @param value Value to write to phy register.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error WritePhyRegister(
					PGRGuid guid,
					unsigned int page,
					unsigned int port,
					unsigned int address,
					unsigned int value );

			/**
			 * Read usb link info for the port that the specified device is connected to.
			 *
			 * @param guid PGRGuid of the device to read from.
			 * @param pValue Value read from the card register.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetUsbLinkInfo(
					PGRGuid guid,
					unsigned int* pValue );

			/**
			 * Read usb port status for the port that the specified device is connected to.
			 *
			 * @param guid PGRGuid of the device to read from.
			 * @param pValue Value read from the card register.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetUsbPortStatus(
					PGRGuid guid,
					unsigned int* pValue );

			/**
			 * Gets the topology information for the PC.
			 *
			 * @param pNode TopologyNode object that will contain the topology
			 *              information.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error GetTopology(
					TopologyNode* pNode );

			/**
			 * Register a callback function that will be called when the
			 * specified callback event occurs.
			 *
			 * @param busEventCallback Pointer to function that will receive
			 the callback.
			 * @param callbackType Type of callback to register for.
			 * @param pParameter Callback parameter to be passed to callback.
			 * @param pCallbackHandle Unique callback handle used for
			 *                        unregistering callback.
			 *
			 * @see UnregisterCallback()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error RegisterCallback(
					BusEventCallback    busEventCallback,
					BusCallbackType     callbackType,
					void*               pParameter,
					CallbackHandle*     pCallbackHandle );

			/**
			 * Unregister a callback function.
			 *
			 * @param callbackHandle Unique callback handle.
			 *
			 * @see RegisterCallback()
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error UnregisterCallback( CallbackHandle callbackHandle );

			/**
			 * Force a rescan of the buses. This does not trigger a bus reset.
			 * However, any current connections to a Camera object will be
			 * invalidated.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			virtual Error RescanBus();

			/**
			 * Force the camera with the specific MAC address to the specified
			 * IP address, subnet mask and default gateway. This is useful in
			 * situations where GigE Vision cameras are using IP addresses 
			 * in a subnet different from the host's subnet.
			 *
			 * @param macAddress MAC address of the camera.
			 * @param ipAddress IP address to set on the camera.
			 * @param subnetMask Subnet mask to set on the camera.
			 * @param defaultGateway Default gateway to set on the camera.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error ForceIPAddressToCamera(
					MACAddress macAddress,
					IPAddress ipAddress,
					IPAddress subnetMask,
					IPAddress defaultGateway );

			/**
			 * Force all cameras on the network to be assigned sequential IP addresses
			 * on the same subnet as the netowrk adapters that they are connected to.
			 * This is useful in situations where GigE Vision cameras are using
			 * IP addresses in a subnet different from the host's subnet.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error ForceAllIPAddressesAutomatically();

			/**
			 * Force a camera on the network to be assigned an IP address
			 * on the same subnet as the netowrk adapters that it is connected to.
			 * This is useful in situations where GigE Vision cameras are using IP
			 * addresses in a subnet different from the host's subnet.
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			static Error ForceAllIPAddressesAutomatically(unsigned int serialNumber);

			/**
			 * Discover all cameras connected to the network even if they reside
			 * on a different subnet. This is useful in situations where GigE Vision
			 * cameras are using IP addresses in a subnet different from the host's
			 * subnet. After discovering the camera, it is easy to use
			 * ForceIPAddressToCamera() to set a different IP configuration.
			 *
			 * @param gigECameras Pointer to an array of CameraInfo structures.
			 * @param arraySize Size of the array. Number of discovered cameras
			 *                  is returned in the same value.
			 *
			 * @return An Error indicating the success or failure of the function.
			 *         If the error is PGRERROR_BUFFER_TOO_SMALL then arraySize will
			 *         contain the minimum size needed for gigECameras array.
			 */
			static Error DiscoverGigECameras(
					CameraInfo* gigECameras,
					unsigned int* arraySize  );

			/**
			 * Query CCP status on camera with corresponding PGRGuid. This is
			 * useful to determine if a GigE camera can be controlled.
			 *
			 * @param pGuid PGRGuid of the camera
			 * @param pControlable Indicates whether camera is controllable
			 *
			 * @return An Error indicating the success or failure of the function.
			 */
			Error IsCameraControlable(
					PGRGuid* pGuid, bool* pControlable );

		private:

			BusManager( const BusManager& );
			BusManager& operator=( const BusManager& );

			struct BusManagerData; // Forward declaration

			BusManagerData* m_pBusManagerData;
	};
}

#endif //PGR_FC2_BUSMANAGER_H
