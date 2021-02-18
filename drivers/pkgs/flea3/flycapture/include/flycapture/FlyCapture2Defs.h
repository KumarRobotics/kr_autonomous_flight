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
// $Id: FlyCapture2Defs.h 318533 2017-03-09 22:41:16Z corinal $
//=============================================================================

#ifndef PGR_FC2_FLYCAPTURE2DEFS_H
#define PGR_FC2_FLYCAPTURE2DEFS_H

#include <memory.h>

//=============================================================================
// Definitions file for FlyCapture2.
//
// Holds structures, enumerations and other global definitions that are used
// across the entire FlyCapture2 API.
//=============================================================================

#ifndef NULL
#define NULL 0
#endif

#ifndef FULL_32BIT_VALUE
#define FULL_32BIT_VALUE 0x7FFFFFFF
#endif


namespace FlyCapture2
{
	/**
	 * @defgroup GlobalConstants Global constants
	 */

	/*@{*/

	/** The maximum length that is allocated for a string. */
	static const unsigned int sk_maxStringLength = 512;

	/** The maximum number of ports one device can have. */
	static const unsigned int sk_maxNumPorts = 32;

	/*@}*/

	/**
	 * @defgroup Enumerations Enumerations
	 */

	/*@{*/

	/** The error types returned by functions. */
	enum ErrorType
	{
		PGRERROR_UNDEFINED = -1, /**< Undefined */
		PGRERROR_OK, /**< Function returned with no errors. */
		PGRERROR_FAILED, /**< General failure. */
		PGRERROR_NOT_IMPLEMENTED, /**< Function has not been implemented. */
		PGRERROR_FAILED_BUS_MASTER_CONNECTION, /**< Could not connect to Bus Master. */
		PGRERROR_NOT_CONNECTED, /**< Camera has not been connected. */
		PGRERROR_INIT_FAILED, /**< Initialization failed. */
		PGRERROR_NOT_INTITIALIZED, /**< Camera has not been initialized. */
		PGRERROR_INVALID_PARAMETER, /**< Invalid parameter passed to function. */
		PGRERROR_INVALID_SETTINGS, /**< Setting set to camera is invalid. */
		PGRERROR_INVALID_BUS_MANAGER, /**< Invalid Bus Manager object. */
		PGRERROR_MEMORY_ALLOCATION_FAILED, /**< Could not allocate memory. */
		PGRERROR_LOW_LEVEL_FAILURE, /**< Low level error. */
		PGRERROR_NOT_FOUND, /**< Device not found. */
		PGRERROR_FAILED_GUID, /**< GUID failure. */
		PGRERROR_INVALID_PACKET_SIZE, /**< Packet size set to camera is invalid. */
		PGRERROR_INVALID_MODE, /**< Invalid mode has been passed to function. */
		PGRERROR_NOT_IN_FORMAT7, /**< Error due to not being in Format7. */
		PGRERROR_NOT_SUPPORTED, /**< This feature is unsupported. */
		PGRERROR_TIMEOUT, /**< Timeout error. */
		PGRERROR_BUS_MASTER_FAILED, /**< Bus Master Failure. */
		PGRERROR_INVALID_GENERATION, /**< Generation Count Mismatch. */
		PGRERROR_LUT_FAILED, /**< Look Up Table failure. */
		PGRERROR_IIDC_FAILED, /**< IIDC failure. */
		PGRERROR_STROBE_FAILED, /**< Strobe failure. */
		PGRERROR_TRIGGER_FAILED, /**< Trigger failure. */
		PGRERROR_PROPERTY_FAILED, /**< Property failure. */
		PGRERROR_PROPERTY_NOT_PRESENT, /**< Property is not present. */
		PGRERROR_REGISTER_FAILED, /**< Register access failed. */
		PGRERROR_READ_REGISTER_FAILED, /**< Register read failed. */
		PGRERROR_WRITE_REGISTER_FAILED, /**< Register write failed. */
		PGRERROR_ISOCH_FAILED, /**< Isochronous failure. */
		PGRERROR_ISOCH_ALREADY_STARTED, /**< Isochronous transfer has already been started. */
		PGRERROR_ISOCH_NOT_STARTED, /**< Isochronous transfer has not been started. */
		PGRERROR_ISOCH_START_FAILED, /**< Isochronous start failed. */
		PGRERROR_ISOCH_RETRIEVE_BUFFER_FAILED, /**< Isochronous retrieve buffer failed. */
		PGRERROR_ISOCH_STOP_FAILED, /**< Isochronous stop failed. */
		PGRERROR_ISOCH_SYNC_FAILED, /**< Isochronous image synchronization failed. */
		PGRERROR_ISOCH_BANDWIDTH_EXCEEDED, /**< Isochronous bandwidth exceeded. */
		PGRERROR_IMAGE_CONVERSION_FAILED, /**< Image conversion failed. */
		PGRERROR_IMAGE_LIBRARY_FAILURE, /**< Image library failure. */
		PGRERROR_BUFFER_TOO_SMALL, /**< Buffer is too small. */
		PGRERROR_IMAGE_CONSISTENCY_ERROR, /**< There is an image consistency error. */
		PGRERROR_INCOMPATIBLE_DRIVER, /**< The installed driver is not compatible with the library. */
		PGRERROR_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** The type of bus callback to register a callback function for. */
	enum BusCallbackType
	{
		BUS_RESET, /**< Register for all bus events. */
		ARRIVAL, /**< Register for arrivals only. */
		REMOVAL, /**< Register for removals only. */
		CALLBACK_TYPE_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/**
	 * The grab strategy employed during image transfer. This type controls
	 * how images that stream off the camera accumulate in a user buffer
	 * for handling.
	 */
	/*
	   @remark Unlike earlier versions of the FlyCapture SDK, it is no longer
	 * necessary to explicitly start the image grabbing process before
	 * specifying an image grabbing mode.
	 */
	enum GrabMode
	{
		/**
		 * Grabs the newest image in the user buffer each time the
		 * RetrieveBuffer() function is called. Older images are dropped
		 * instead of accumulating in the user buffer. Grabbing blocks if the
		 * camera has not finished transmitting the next available image. If
		 * the camera is transmitting images faster than the application can
		 * grab them, images may be dropped and only the most recent image
		 * is stored for grabbing. Note that this mode is the equivalent of
		 * flycaptureLockLatest in earlier versions of the FlyCapture SDK.
		 */
		DROP_FRAMES,

		/**
		 * Images accumulate in the user buffer, and the oldest image is
		 * grabbed for handling before being discarded. This member can be
		 * used to guarantee that each image is seen. However, image processing
		 * time must not exceed transmission time from the camera to the
		 * buffer. Grabbing blocks if the camera has not finished transmitting
		 * the next available image. The buffer size is controlled by the
		 * numBuffers parameter in the FC2Config struct. Note that this mode is
		 * the equivalent of flycaptureLockNext in earlier versions of the
		 * FlyCapture SDK.
		 */
		BUFFER_FRAMES,

		/**
		 * Unspecified grab mode.
		 */
		UNSPECIFIED_GRAB_MODE,
		GRAB_MODE_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** Timeout options for grabbing images. */
	enum GrabTimeout
	{
		TIMEOUT_NONE = 0,  /**<  Non-blocking wait. */
		TIMEOUT_INFINITE = -1, /**<  Wait indefinitely. */
		TIMEOUT_UNSPECIFIED = -2, /**< Unspecified timeout setting. */
		GRAB_TIMEOUT_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** Bandwidth allocation options for 1394 devices. */
	enum BandwidthAllocation
	{
		/** Do not allocate bandwidth. */
		BANDWIDTH_ALLOCATION_OFF = 0,
		/** Allocate bandwidth. This is the default setting. */
		BANDWIDTH_ALLOCATION_ON = 1,
		/**
		 * Bandwidth allocation is not supported by either the camera or
		 * operating system.
		 */
		BANDWIDTH_ALLOCATION_UNSUPPORTED = 2,
		/** Not specified. This leaves the current setting unchanged. */
		BANDWIDTH_ALLOCATION_UNSPECIFIED = 3,
		BANDWIDTH_ALLOCATION_FORCE_32BITS = FULL_32BIT_VALUE

	};

	/** Interfaces that a camera may use to communicate with a host. */
	enum InterfaceType
	{
		INTERFACE_IEEE1394, /**< IEEE-1394 (Includes 1394a and 1394b). */
		INTERFACE_USB2, /**< USB 2.0. */
		INTERFACE_USB3, /**< USB 3.0. */
		INTERFACE_GIGE, /**< GigE. */
		INTERFACE_UNKNOWN, /**< Unknown interface. */
		INTERFACE_TYPE_FORCE_32BITS = FULL_32BIT_VALUE

	};

	/**
	 * Camera properties. Not all properties may be supported, depending
	 * on the camera model.
	 */
	enum PropertyType
	{
		BRIGHTNESS, /**< Brightness. */
		AUTO_EXPOSURE, /**< Auto exposure. */
		SHARPNESS, /**< Sharpness */
		WHITE_BALANCE, /**< White balance. */
		HUE, /**< Hue. */
		SATURATION, /**< Saturation. */
		GAMMA, /**< Gamma. */
		IRIS, /**< Iris. */
		FOCUS, /**< Focus. */
		ZOOM, /**< Zoom. */
		PAN, /**< Pan. */
		TILT, /**< Tilt. */
		SHUTTER, /**< Shutter. */
		GAIN, /**< Gain. */
		TRIGGER_MODE, /**< Trigger mode. */
		TRIGGER_DELAY, /**< Trigger delay. */
		FRAME_RATE, /**< Frame rate. */
		TEMPERATURE, /**< Temperature. */
		UNSPECIFIED_PROPERTY_TYPE, /**< Unspecified property type. */
		PROPERTY_TYPE_FORCE_32BITS = FULL_32BIT_VALUE

	};

	/** Frame rates in frames per second. */
	enum FrameRate
	{
		FRAMERATE_1_875, /**< 1.875 fps. */
		FRAMERATE_3_75, /**< 3.75 fps. */
		FRAMERATE_7_5, /**< 7.5 fps. */
		FRAMERATE_15, /**< 15 fps. */
		FRAMERATE_30, /**< 30 fps. */
		FRAMERATE_60, /**< 60 fps. */
		FRAMERATE_120, /**< 120 fps. */
		FRAMERATE_240, /**< 240 fps. */
		FRAMERATE_FORMAT7, /**< Custom frame rate for Format7 functionality. */
		NUM_FRAMERATES, /**< Number of possible camera frame rates. */
		FRAMERATE_FORCE_32BITS = FULL_32BIT_VALUE

	};

	/** DCAM video modes. */
	enum VideoMode
	{
		VIDEOMODE_160x120YUV444, /**< 160x120 YUV444. */
		VIDEOMODE_320x240YUV422, /**< 320x240 YUV422. */
		VIDEOMODE_640x480YUV411, /**< 640x480 YUV411. */
		VIDEOMODE_640x480YUV422, /**< 640x480 YUV422. */
		VIDEOMODE_640x480RGB, /**< 640x480 24-bit RGB. */
		VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
		VIDEOMODE_640x480Y16, /**< 640x480 16-bit. */
		VIDEOMODE_800x600YUV422, /**< 800x600 YUV422. */
		VIDEOMODE_800x600RGB, /**< 800x600 RGB. */
		VIDEOMODE_800x600Y8, /**< 800x600 8-bit. */
		VIDEOMODE_800x600Y16, /**< 800x600 16-bit. */
		VIDEOMODE_1024x768YUV422, /**< 1024x768 YUV422. */
		VIDEOMODE_1024x768RGB, /**< 1024x768 RGB. */
		VIDEOMODE_1024x768Y8, /**< 1024x768 8-bit. */
		VIDEOMODE_1024x768Y16, /**< 1024x768 16-bit. */
		VIDEOMODE_1280x960YUV422, /**< 1280x960 YUV422. */
		VIDEOMODE_1280x960RGB, /**< 1280x960 RGB. */
		VIDEOMODE_1280x960Y8, /**< 1280x960 8-bit. */
		VIDEOMODE_1280x960Y16, /**< 1280x960 16-bit. */
		VIDEOMODE_1600x1200YUV422, /**< 1600x1200 YUV422. */
		VIDEOMODE_1600x1200RGB, /**< 1600x1200 RGB. */
		VIDEOMODE_1600x1200Y8, /**< 1600x1200 8-bit. */
		VIDEOMODE_1600x1200Y16, /**< 1600x1200 16-bit. */
		VIDEOMODE_FORMAT7, /**< Custom video mode for Format7 functionality. */
		NUM_VIDEOMODES, /**< Number of possible video modes. */
		VIDEOMODE_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** Camera modes for DCAM formats as well as Format7. */
	enum Mode
	{
		MODE_0 = 0,
		MODE_1,
		MODE_2,
		MODE_3,
		MODE_4,
		MODE_5,
		MODE_6,
		MODE_7,
		MODE_8,
		MODE_9,
		MODE_10,
		MODE_11,
		MODE_12,
		MODE_13,
		MODE_14,
		MODE_15,
		MODE_16,
		MODE_17,
		MODE_18,
		MODE_19,
		MODE_20,
		MODE_21,
		MODE_22,
		MODE_23,
		MODE_24,
		MODE_25,
		MODE_26,
		MODE_27,
		MODE_28,
		MODE_29,
		MODE_30,
		MODE_31,
		NUM_MODES, /**< Number of modes */
		MODE_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** Pixel formats available for Format7 modes. */
	enum PixelFormat
	{
		PIXEL_FORMAT_MONO8     = 0x80000000, /**< 8 bits of mono information. */
		PIXEL_FORMAT_411YUV8   = 0x40000000, /**< YUV 4:1:1. */
		PIXEL_FORMAT_422YUV8   = 0x20000000, /**< YUV 4:2:2. */
		PIXEL_FORMAT_444YUV8   = 0x10000000, /**< YUV 4:4:4. */
		PIXEL_FORMAT_RGB8      = 0x08000000, /**< R = G = B = 8 bits. */
		PIXEL_FORMAT_MONO16    = 0x04000000, /**< 16 bits of mono information. */
		PIXEL_FORMAT_RGB16     = 0x02000000, /**< R = G = B = 16 bits. */
		PIXEL_FORMAT_S_MONO16  = 0x01000000, /**< 16 bits of signed mono information. */
		PIXEL_FORMAT_S_RGB16   = 0x00800000, /**< R = G = B = 16 bits signed. */
		PIXEL_FORMAT_RAW8      = 0x00400000, /**< 8 bit raw data output of sensor. */
		PIXEL_FORMAT_RAW16     = 0x00200000, /**< 16 bit raw data output of sensor. */
		PIXEL_FORMAT_MONO12    = 0x00100000, /**< 12 bits of mono information. */
		PIXEL_FORMAT_RAW12     = 0x00080000, /**< 12 bit raw data output of sensor. */
		PIXEL_FORMAT_BGR       = 0x80000008, /**< 24 bit BGR. */
		PIXEL_FORMAT_BGRU      = 0x40000008, /**< 32 bit BGRU. */
		PIXEL_FORMAT_RGB       = PIXEL_FORMAT_RGB8, /**< 24 bit RGB. */
		PIXEL_FORMAT_RGBU      = 0x40000002, /**< 32 bit RGBU. */
		PIXEL_FORMAT_BGR16     = 0x02000001, /**< R = G = B = 16 bits. */
		PIXEL_FORMAT_BGRU16    = 0x02000002, /**< 64 bit BGRU. */
		PIXEL_FORMAT_422YUV8_JPEG      = 0x40000001, /**< JPEG compressed stream. */
		NUM_PIXEL_FORMATS	   =  20, /**< Number of pixel formats. */
		UNSPECIFIED_PIXEL_FORMAT = 0 /**< Unspecified pixel format. */
	};

	/** Bus speeds. */
	enum BusSpeed
	{
		BUSSPEED_S100, /**< 100Mbits/sec. */
		BUSSPEED_S200, /**< 200Mbits/sec. */
		BUSSPEED_S400, /**< 400Mbits/sec. */
		BUSSPEED_S480, /**< 480Mbits/sec. Only for USB2 cameras. */
		BUSSPEED_S800, /**< 800Mbits/sec. */
		BUSSPEED_S1600, /**< 1600Mbits/sec. */
		BUSSPEED_S3200, /**< 3200Mbits/sec. */
		BUSSPEED_S5000, /**< 5000Mbits/sec. Only for USB3 cameras. */
		BUSSPEED_10BASE_T, /**< 10Base-T. Only for GigE Vision cameras. */
		BUSSPEED_100BASE_T, /**< 100Base-T.  Only for GigE Vision cameras.*/
		BUSSPEED_1000BASE_T, /**< 1000Base-T (Gigabit Ethernet).  Only for GigE Vision cameras. */
		BUSSPEED_10000BASE_T, /**< 10000Base-T.  Only for GigE Vision cameras. */
		BUSSPEED_S_FASTEST, /**< The fastest speed available. */
		BUSSPEED_ANY, /**< Any speed that is available. */
		BUSSPEED_SPEED_UNKNOWN = -1, /**< Unknown bus speed. */
		BUSSPEED_FORCE_32BITS = FULL_32BIT_VALUE
	};

	enum PCIeBusSpeed
	{
		PCIE_BUSSPEED_2_5, /** 2.5 Gb/s */
		PCIE_BUSSPEED_5_0, /** 5.0 Gb/s */
		PCIE_BUSSPEED_UNKNOWN = -1, /** Speed is unknown */
		PCIE_BUSSPEED_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** Types of low level drivers that flycapture uses. */
	enum DriverType
	{
		DRIVER_1394_CAM, /**< PGRCam.sys. */
		DRIVER_1394_PRO, /**< PGR1394.sys. */
		DRIVER_1394_JUJU, /**< firewire_core. */
		DRIVER_1394_VIDEO1394, /**< video1394. */
		DRIVER_1394_RAW1394, /**< raw1394. */
		DRIVER_USB_NONE, /**< No usb driver used just BSD stack. (Linux only) */
		DRIVER_USB_CAM, /**< PGRUsbCam.sys. */
		DRIVER_USB3_PRO, /**< PGRXHCI.sys. */
		DRIVER_GIGE_NONE, /**< no gige drivers used,MS/BSD stack. */
		DRIVER_GIGE_FILTER, /**< PGRGigE.sys. */
		DRIVER_GIGE_PRO, /**< PGRGigEPro.sys. */
		DRIVER_GIGE_LWF,	/**< PgrLwf.sys. */
		DRIVER_UNKNOWN = -1, /**< Unknown driver type. */
		DRIVER_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/**
	 * Color processing algorithms. Please refer to our knowledge base at
	 * article at http://www.ptgrey.com/support/kb/index.asp?a=4&q=33 for
	 * complete details for each algorithm.
	 */
	enum ColorProcessingAlgorithm
	{
		/** Default method. */
		DEFAULT,
		/** No color processing. */
		NO_COLOR_PROCESSING,
		/**
		 * Fastest but lowest quality. Equivalent to
		 * FLYCAPTURE_NEAREST_NEIGHBOR_FAST in FlyCapture.
		 */
		NEAREST_NEIGHBOR,
		/** Weights surrounding pixels based on localized edge orientation. */
		EDGE_SENSING,
		/** Well-balanced speed and quality. */
		HQ_LINEAR,
		/** Slowest but produces good results. */
		RIGOROUS,
		/** Multithreaded with similar results to edge sensing. */
		IPP,
		/** Best quality but much faster than rigorous. */
		DIRECTIONAL_FILTER,
		/** Weighted pixel average from different directions*/
		WEIGHTED_DIRECTIONAL_FILTER,

		COLOR_PROCESSING_ALGORITHM_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** Bayer tile formats. */
	enum BayerTileFormat
	{
		NONE, /**< No bayer tile format. */
		RGGB, /**< Red-Green-Green-Blue. */
		GRBG, /**< Green-Red-Blue-Green. */
		GBRG, /**< Green-Blue-Red-Green. */
		BGGR, /**< Blue-Green-Green-Red. */
		BT_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/** File formats to be used for saving images to disk. */
	enum ImageFileFormat
	{
		FROM_FILE_EXT = -1, /**< Determine file format from file extension. */
		PGM, /**< Portable gray map. */
		PPM, /**< Portable pixmap. */
		BMP, /**< Bitmap. */
		JPEG, /**< JPEG. */
		JPEG2000, /**< JPEG 2000. */
		TIFF, /**< Tagged image file format. */
		PNG, /**< Portable network graphics. */
		RAW, /**< Raw data. */
		IMAGE_FILE_FORMAT_FORCE_32BITS = FULL_32BIT_VALUE
	};

	/*@}*/

	/**
	 * @defgroup GigEEnums GigE specific enumerations
	 *
	 * These enumerations are specific to GigE camera operation only.
	 */

	/*@{*/

	/** Possible properties that can be queried from the camera. */
	enum GigEPropertyType
	{
		HEARTBEAT,
		HEARTBEAT_TIMEOUT,
		PACKET_SIZE,
		PACKET_DELAY
	};

	/*@}*/

	/**
	 * @defgroup Structures Structures
	 */

	/*@{*/

	/** The current version of the library. */
	struct FC2Version
	{
		unsigned int major; /**< Major version number. */
		unsigned int minor; /**< Minor version number. */
		unsigned int type; /**< Type version number. */
		unsigned int build; /**< Build version number. */
	};

	/** A GUID to the camera.  It is used to uniquely identify a camera. */
	class PGRGuid
	{
		public:
			unsigned int value[4];

			/** Constructor. */
			PGRGuid() { memset( value, 0x0, 4 * sizeof(unsigned int) ); }

			/** Equality operator. */
			bool operator==( const PGRGuid& guid ) const
			{
				if ( this->value[0] == guid.value[0] &&
						this->value[1] == guid.value[1] &&
						this->value[2] == guid.value[2] &&
						this->value[3] == guid.value[3] )
				{
					return true;
				}
				else
				{
					return false;
				}
			}

			/** Inequality operator. */
			bool operator!=( const PGRGuid& guid )
			{
				return !(operator==( guid ));
			}
	};

	/**
	 * @defgroup GigEStructures GigE specific structures
	 *
	 * These structures are specific to GigE camera operation only.
	 */

	/*@{*/

	/** IPv4 address. */
	struct IPAddress
	{
		unsigned char octets[4];

		IPAddress() { memset(octets, 0x0, 4 * sizeof(unsigned char) ); }

		IPAddress( unsigned int ipAddressVal )
		{
			this->octets[0] = (unsigned char)(ipAddressVal >> 24) & 0xFF;
			this->octets[1] = (unsigned char)(ipAddressVal >> 16) & 0xFF;
			this->octets[2] = (unsigned char)(ipAddressVal >> 8) & 0xFF;
			this->octets[3] = (unsigned char)(ipAddressVal >> 0) & 0xFF;
		}

		/** Equality operator. */
		bool operator==( const IPAddress& address ) const
		{
			if ( this->octets[0] == address.octets[0] &&
					this->octets[1] == address.octets[1] &&
					this->octets[2] == address.octets[2] &&
					this->octets[3] == address.octets[3] )
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		/** Inequality operator. */
		bool operator!=( const IPAddress& address )
		{
			return !(operator==( address ));
		}
	};

	/** MAC address. */
	struct MACAddress
	{
		unsigned char octets[6];

		MACAddress() { memset(octets, 0x0, 6 * sizeof(unsigned char) ); }

		MACAddress( unsigned int macAddressValHigh, unsigned int macAddressValLow )
		{
			this->octets[0] = (unsigned char)(macAddressValHigh >> 8) & 0xFF;
			this->octets[1] = (unsigned char)(macAddressValHigh >> 0) & 0xFF;
			this->octets[2] = (unsigned char)(macAddressValLow >> 24) & 0xFF;
			this->octets[3] = (unsigned char)(macAddressValLow >> 16) & 0xFF;
			this->octets[4] = (unsigned char)(macAddressValLow >> 8) & 0xFF;
			this->octets[5] = (unsigned char)(macAddressValLow >> 0) & 0xFF;
		}

		/** Equality operator. */
		bool operator==( const MACAddress& address ) const
		{
			if ( this->octets[0] == address.octets[0] &&
					this->octets[1] == address.octets[1] &&
					this->octets[2] == address.octets[2] &&
					this->octets[3] == address.octets[3] &&
					this->octets[4] == address.octets[4] &&
					this->octets[5] == address.octets[5] )
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		/** Inequality operator. */
		bool operator!=( const MACAddress& address )
		{
			return !(operator==( address ));
		}
	};

	/** A GigE property. */
	struct GigEProperty
	{
		/** The type of property. */
		GigEPropertyType propType;
		/**
		 * Whether the property is readable. If this is false, then
		 * no other value in this structure is valid.
		 */
		bool isReadable;
		/** Whether the property is writable. */
		bool isWritable;
		/** Minimum value. */
		unsigned int min;
		/** Maximum value. */
		unsigned int max;
		/** Current value. */
		unsigned int value;
	};

	/** Information about a single GigE stream channel. */
	struct GigEStreamChannel
	{
		/** Network interface index used (or to use). */
		unsigned int networkInterfaceIndex;
		/** Host port on the PC where the camera will send the data stream. */
		unsigned int hostPort;
		/** Disable IP fragmentation of packets. */
		bool doNotFragment;
		/** Packet size, in bytes. */
		unsigned int packetSize;
		/** Inter packet delay, in timestamp counter units. */
		unsigned int interPacketDelay;
		/** Destination IP address. It can be a multicast or unicast address. */
		IPAddress destinationIpAddress;
		/** Source UDP port of the stream channel. Read only. */
		unsigned int sourcePort;

		GigEStreamChannel()
		{
			networkInterfaceIndex = 0;
			hostPort = 0;
			doNotFragment = false;
			packetSize = 0;
			interPacketDelay = 0;
			sourcePort = 0;
		}
	};

	/**
	 * Configuration for a GigE camera.  These options are options that are
	 * generally should be set before starting isochronous transfer.
	 */
	struct GigEConfig
	{
		/** Turn on/off packet resend functionality */
		bool enablePacketResend;

		/**
		 * Number of retries to perform when a register read/write timeout
		 * is received by the library. The default value is 0.
		 */
		unsigned int registerTimeoutRetries;

		/**
		 * Register read/write timeout value, in microseconds.
		 * The default value is dependent on the interface type.
		 */
		unsigned int registerTimeout;

		GigEConfig()
		{
			enablePacketResend = false;
			registerTimeoutRetries = 3;
			registerTimeout = 20000;
		}
	};

	/** Format 7 information for a single mode. */
	struct GigEImageSettingsInfo
	{
		/** Maximum image width. */
		unsigned int maxWidth;
		/** Maximum image height. */
		unsigned int maxHeight;
		/** Horizontal step size for the offset. */
		unsigned int offsetHStepSize;
		/** Vertical step size for the offset. */
		unsigned int offsetVStepSize;
		/** Horizontal step size for the image. */
		unsigned int imageHStepSize;
		/** Vertical step size for the image. */
		unsigned int imageVStepSize;
		/** Supported pixel formats in a bit field. */
		unsigned int pixelFormatBitField;
		/** Vendor unique pixel formats in a bit field. */
		unsigned int vendorPixelFormatBitField;
		/** Reserved for future use. */
		unsigned int reserved[16];

		GigEImageSettingsInfo()
		{
			maxWidth = 0;
			maxHeight = 0;
			offsetHStepSize = 0;
			offsetVStepSize = 0;
			imageHStepSize = 0;
			imageVStepSize = 0;
			pixelFormatBitField = 0;
			vendorPixelFormatBitField = 0;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** Image settings for a GigE camera. */
	struct GigEImageSettings
	{
		/** Horizontal image offset. */
		unsigned int offsetX;
		/** Vertical image offset. */
		unsigned int offsetY;
		/** Width of image. */
		unsigned int width;
		/** Height of image. */
		unsigned int height;
		/** Pixel format of image. */
		PixelFormat pixelFormat;
		/** Reserved for future use. */
		unsigned int reserved[8];

		GigEImageSettings()
		{
			offsetX = 0;
			offsetY = 0;
			width = 0;
			height = 0;
			pixelFormat = UNSPECIFIED_PIXEL_FORMAT;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/*@}*/

	/**
	 * @defgroup IIDCStructures IIDC specific structures
	 *
	 * These structures are specific to IIDC camera operation only.
	 */

	/*@{*/

	/** Format 7 image settings. */
	struct Format7ImageSettings
	{
		/** Format 7 mode. */
		Mode mode;
		/** Horizontal image offset. */
		unsigned int offsetX;
		/** Vertical image offset. */
		unsigned int offsetY;
		/** Width of image. */
		unsigned int width;
		/** Height of image. */
		unsigned int height;
		/** Pixel format of image. */
		PixelFormat pixelFormat;
		/** Reserved for future use. */
		unsigned int reserved[8];

		Format7ImageSettings()
		{
			mode = MODE_0;
			offsetX = 0;
			offsetY = 0;
			width = 0;
			height = 0;
			pixelFormat = UNSPECIFIED_PIXEL_FORMAT;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** Format 7 information for a single mode. */
	struct Format7Info
	{
		/** Format 7 mode. */
		Mode mode;

		/** Maximum image width. */
		unsigned int maxWidth;
		/** Maximum image height. */
		unsigned int maxHeight;
		/** Horizontal step size for the offset. */
		unsigned int offsetHStepSize;
		/** Vertical step size for the offset. */
		unsigned int offsetVStepSize;
		/** Horizontal step size for the image. */
		unsigned int imageHStepSize;
		/** Vertical step size for the image. */
		unsigned int imageVStepSize;
		/** Supported pixel formats in a bit field. */
		unsigned int pixelFormatBitField;
		/** Vendor unique pixel formats in a bit field. */
		unsigned int vendorPixelFormatBitField;

		/** Current packet size in bytes. */
		unsigned int packetSize;
		/** Minimum packet size in bytes for current mode. */
		unsigned int minPacketSize;
		/** Maximum packet size in bytes for current mode. */
		unsigned int maxPacketSize;
		/** Current packet size as a percentage of maximum packet size. */
		float percentage;
		/** Reserved for future use. */
		unsigned int reserved[16];

		Format7Info()
		{
			mode = MODE_0;
			maxWidth = 0;
			maxHeight = 0;
			offsetHStepSize = 0;
			offsetVStepSize = 0;
			imageHStepSize = 0;
			imageVStepSize = 0;
			pixelFormatBitField = 0;
			vendorPixelFormatBitField = 0;
			packetSize = 0;
			minPacketSize = 0;
			maxPacketSize = 0;
			percentage = 0.0f;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** Format 7 packet information. */
	struct Format7PacketInfo
	{
		/** Recommended bytes per packet. */
		unsigned int recommendedBytesPerPacket;
		/** Maximum bytes per packet. */
		unsigned int maxBytesPerPacket;
		/** Minimum bytes per packet. */
		unsigned int unitBytesPerPacket;
		/** Reserved for future use. */
		unsigned int reserved[8];

		Format7PacketInfo()
		{
			recommendedBytesPerPacket = 0;
			maxBytesPerPacket = 0;
			unitBytesPerPacket = 0;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/*@}*/

	/**
	 * Configuration for a camera. These options are options that are
	 * generally should be set before starting isochronous transfer.
	 */
	struct FC2Config
	{
		/** Number of buffers used by the FlyCapture2 library to grab images. */
		unsigned int numBuffers;

		/**
		 * Number of notifications per image. This value should only be set
		 * after the image settings to be used is set to the camera.
		 * The default number of notifications is 1.
		 *
		 * There are 4 general scenarios:
		 * - 1 notification - End of image
		 * - 2 notifications - After first packet and end of image
		 * - 3 notifications - After first packet, middle of image, end of image
		 * - x notifications - After first packet, (x -2) spread evenly, end of image
		 *
		 * Specifying zero for the number of notifications will be ignored (the current value
		 * will not be modified).
		 *
		 * Note that the event numbers start at 0. Ex. when 3 notifications are
		 * used, the three events will be 0, 1 and 2.
		 */
		unsigned int numImageNotifications;

		/**
		 *  Minimum number of notifications needed for the current image
		 *  settings on the camera. Read-only value.
		 */
		unsigned int minNumImageNotifications;

		/**
		 * Time in milliseconds that RetrieveBuffer() and WaitForBufferEvent()
		 * will wait for an image before timing out and returning.
		 */
		int grabTimeout;

		/** Grab mode for the camera. The default is DROP_FRAMES. */
		GrabMode grabMode;

		/** This parameter enables RetrieveBuffer to run in high
		 *	performance mode.  This means that any interaction
		 *  with the camera, other then grabbing the image is disabled.
		 *	Currently Retrieve buffer reads registers on the camera to
		 *	determine which embedded image information settings have been
		 *	enabled, and it reads what the bayer tile is currently set to.
		 *	When High Performance mode is on, these reads are disabled.  This
		 *	means that any changes to the Bayer Tile or to the Embedded image
		 *	info after StartCapture() will not be tracked when made using
		 *	direct register writes.  If the corresponding SetEmbededImageInfo()
		 *	and GetEmbededImageInfo() calls are used then the changes will be
		 *	appropriately reflected.  This also means that changes to embedded
		 *	image info from other processes will not be updated either.*/
		bool highPerformanceRetrieveBuffer;

		/** Isochronous bus speed. */
		BusSpeed isochBusSpeed;

		/** Asynchronous bus speed. */
		BusSpeed asyncBusSpeed;

		/**
		 * Bandwidth allocation flag that tells the camera the bandwidth
		 * allocation strategy to employ.
		 */
		BandwidthAllocation bandwidthAllocation;

		/**
		 * Number of retries to perform when a register read/write timeout
		 * is received by the library. The default value is 0.
		 */
		unsigned int registerTimeoutRetries;

		/**
		 * Register read/write timeout value, in microseconds.
		 * The default value is dependent on the interface type.
		 */
		unsigned int registerTimeout;

		/** Reserved for future use */
		unsigned int reserved[16];

		FC2Config()
		{
			numBuffers = 0;
			numImageNotifications = 0;
			minNumImageNotifications = 0;
			grabTimeout = TIMEOUT_UNSPECIFIED;
			grabMode = UNSPECIFIED_GRAB_MODE;
			isochBusSpeed = BUSSPEED_ANY;
			asyncBusSpeed = BUSSPEED_ANY;
			bandwidthAllocation = BANDWIDTH_ALLOCATION_UNSPECIFIED;
			registerTimeoutRetries = 0;
			registerTimeout = 0;
			highPerformanceRetrieveBuffer = false;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/**
	 * Information about a specific camera property. This structure is also
	 * also used as the TriggerDelayInfo structure.
	 */
	struct PropertyInfo
	{
		/** Property info type. */
		PropertyType type;
		/** Flag indicating if the property is present. */
		bool present;
		/** Flag indicating if auto is supported. */
		bool autoSupported;
		/** Flag indicating if manual is supported. */
		bool manualSupported;
		/** Flag indicating if on/off is supported. */
		bool onOffSupported;
		/** Flag indicating if one push is supported. */
		bool onePushSupported;
		/** Flag indicating if absolute mode is supported. */
		bool absValSupported;
		/** Flag indicating if property value can be read out. */
		bool readOutSupported;
		/** Minimum value (as an integer). */
		unsigned int min;
		/** Maximum value (as an integer). */
		unsigned int max;
		/** Minimum value (as a floating point value). */
		float absMin;
		/** Maximum value (as a floating point value). */
		float absMax;
		/** Textual description of units. */
		char pUnits[sk_maxStringLength];
		/** Abbreviated textual description of units. */
		char pUnitAbbr[sk_maxStringLength];
		/** Reserved for future use. */
		unsigned int reserved[8];

		PropertyInfo()
		{
			type = UNSPECIFIED_PROPERTY_TYPE;
			present = false;
			autoSupported = false;
			manualSupported = false;
			onOffSupported = false;
			onePushSupported = false;
			absValSupported = false;
			readOutSupported = false;
			min = 0;
			max = 0;
			absMin = 0.0f;
			absMax = 0.0f;
			memset( pUnits, 0, sk_maxStringLength );
			memset( pUnitAbbr, 0, sk_maxStringLength );
			memset( reserved, 0, sizeof(reserved) );
		}

		PropertyInfo( PropertyType propType )
		{
			type = propType;
			present = false;
			autoSupported = false;
			manualSupported = false;
			onOffSupported = false;
			onePushSupported = false;
			absValSupported = false;
			readOutSupported = false;
			min = 0;
			max = 0;
			absMin = 0.0f;
			absMax = 0.0f;
			memset( pUnits, 0, sk_maxStringLength );
			memset( pUnitAbbr, 0, sk_maxStringLength );
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** The TriggerDelayInfo structure is identical to PropertyInfo. */
	typedef PropertyInfo TriggerDelayInfo;

	/**
	 * A specific camera property. \n
	 * For example, to set the gain to 12dB, set the following values:
	 * - \a type - \c GAIN
	 * - \a absControl - \c true
	 * - \a onePush - \c false
	 * - \a onOff - \c true
	 * - \a autoManualMode - \c false
	 * - \a absValue - \c 12.0
	 */
	struct Property
	{
		/** Property info type. */
		PropertyType type;
		/** Flag indicating if the property is present. */
		bool present;
		/**
         * Flag controlling absolute mode (real world units)
         * or non-absolute mode (camera internal units).
         */
		bool absControl;
		/** Flag controlling one push. */
		bool onePush;
		/** Flag controlling on/off. */
		bool onOff;
		/** Flag controlling auto. */
		bool autoManualMode;
		/**
         * Value A (integer).
         * Used to configure properties in non-absolute mode.
         */
		unsigned int valueA;
		/**
         * Value B (integer). For white balance, value B applies to the blue value and
         * value A applies to the red value.
         */
		unsigned int valueB;
		/**
		* Floating point value.
		* Used to configure properties in absolute mode.
		*/
		float absValue;
		/** Reserved for future use. */
		unsigned int reserved[8];

		Property()
		{
			type = UNSPECIFIED_PROPERTY_TYPE;
			present = false;
			absControl = false;
			onePush = false;
			onOff = false;
			autoManualMode = false;
			valueA = 0;
			valueB = 0;
			absValue = 0.0f;
			memset( reserved, 0, sizeof(reserved) );
		}

		Property( PropertyType propType )
		{
			type = propType;
			present = false;
			absControl = false;
			onePush = false;
			onOff = false;
			autoManualMode = false;
			valueA = 0;
			valueB = 0;
			absValue = 0.0f;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** The TriggerDelay structure is identical to Property. */
	typedef Property TriggerDelay;

	/** Information about a camera trigger property. */
	struct TriggerModeInfo
	{
		/** Presence of trigger mode. */
		bool present;
		/** Flag indicating if trigger value can be read out. */
		bool readOutSupported;
		/** Flag indicating if on/off is supported. */
		bool onOffSupported;
		/** Flag indicating if polarity is supported. */
		bool polaritySupported;
		/** Flag indicating if the value is readable. */
		bool valueReadable;
		/** Source mask. */
		unsigned int sourceMask;
		/** Flag indicating if software trigger is supported. */
		bool softwareTriggerSupported;
		/** Mode mask. */
		unsigned int modeMask;
		/** Reserved for future use. */
		unsigned int reserved[8];

		TriggerModeInfo()
		{
			present = false;
			readOutSupported = false;
			onOffSupported = false;
			polaritySupported = false;
			valueReadable = false;
			sourceMask = 0;
			softwareTriggerSupported = false;
			modeMask = 0;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** A camera trigger. */
	struct TriggerMode
	{
		/** Flag controlling on/off. */
		bool onOff;
		/** Polarity value. */
		unsigned int polarity;
		/** Source value. */
		unsigned int source;
		/** Mode value. */
		unsigned int mode;
		/** Parameter value. */
		unsigned int parameter;
		/** Reserved for future use. */
		unsigned int reserved[8];

		TriggerMode()
		{
			onOff = false;
			polarity = 0;
			source = 0;
			mode = 0;
			parameter = 0;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** A camera strobe property. */
	struct StrobeInfo
	{
		/** Source value. */
		unsigned int source;
		/** Presence of strobe. */
		bool present;
		/** Flag indicating if strobe value can be read out. */
		bool readOutSupported;
		/** Flag indicating if on/off is supported. */
		bool onOffSupported;
		/** Flag indicating if polarity is supported. */
		bool polaritySupported;
		/** Minimum value. */
		float minValue;
		/** Maximum value. */
		float maxValue;
		/** Reserved for future use. */
		unsigned int reserved[8];

		StrobeInfo()
		{
			source = 0;
			present = false;
			readOutSupported = false;
			onOffSupported = false;
			polaritySupported = false;
			minValue = 0.0f;
			maxValue = 0.0f;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** A camera strobe. */
	struct StrobeControl
	{
		/** Source value. */
		unsigned int source;
		/** Flag controlling on/off. */
		bool onOff;
		/** Signal polarity. */
		unsigned int polarity;
		/** Signal delay (in ms). */
		float delay;
		/** Signal duration (in ms). */
		float duration;
		/** Reserved for future use. */
		unsigned int reserved[8];

		StrobeControl()
		{
			source = 0;
			onOff = false;
			polarity = 0;
			delay = 0.0f;
			duration = 0.0f;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** Timestamp information. */
	struct TimeStamp
	{
		/** Seconds. */
		long long seconds;
		/** Microseconds. */
		unsigned int microSeconds;
		/** 1394 cycle time seconds. */
		unsigned int cycleSeconds;
		/** 1394 cycle time count. */
		unsigned int cycleCount;
		/** 1394 cycle time offset. */
		unsigned int cycleOffset;
		/** Reserved for future use. */
		unsigned int reserved[8];

		TimeStamp()
		{
			seconds = 0;
			microSeconds = 0;
			cycleSeconds = 0;
			cycleCount = 0;
			cycleOffset = 0;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** Camera configuration ROM. */
	struct ConfigROM
	{
		/** Vendor ID of a node. */
		unsigned int nodeVendorId;
		/** Chip ID (high part). */
		unsigned int chipIdHi;
		/** Chip ID (low part). */
		unsigned int chipIdLo;
		/** Unit Spec ID, usually 0xa02d. */
		unsigned int unitSpecId;
		/** Unit software version. */
		unsigned int unitSWVer;
		/** Unit sub software version. */
		unsigned int unitSubSWVer;
		/** Vendor unique info 0. */
		unsigned int vendorUniqueInfo_0;
		/** Vendor unique info 1. */
		unsigned int vendorUniqueInfo_1;
		/** Vendor unique info 2. */
		unsigned int vendorUniqueInfo_2;
		/** Vendor unique info 3. */
		unsigned int vendorUniqueInfo_3;
		/** Keyword. */
		char pszKeyword[ sk_maxStringLength ];
		/** Reserved for future use. */
		unsigned int reserved[16];

		ConfigROM()
		{
			nodeVendorId = 0;
			chipIdHi = 0;
			chipIdLo = 0;
			unitSpecId = 0;
			unitSWVer = 0;
			unitSubSWVer = 0;
			vendorUniqueInfo_0 = 0;
			vendorUniqueInfo_1 = 0;
			vendorUniqueInfo_2 = 0;
			vendorUniqueInfo_3 = 0;
			memset( pszKeyword, 0, sizeof ( pszKeyword ) );
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** Camera information. */
	struct CameraInfo
	{
		/** Device serial number. */
		unsigned int serialNumber;
		/** Interface type. */
		InterfaceType interfaceType;
		/** Driver type. */
		DriverType driverType;
		/** Flag indicating if this is a color camera. */
		bool isColorCamera;
		/** Device model name. */
		char modelName[sk_maxStringLength];
		/** Device vendor name. */
		char vendorName[sk_maxStringLength];
		/** String detailing the sensor information. */
		char sensorInfo[sk_maxStringLength];
		/** String providing the sensor resolution. */
		char sensorResolution[sk_maxStringLength];
		/** Driver name of driver being used. */
		char driverName[sk_maxStringLength];
		/** Firmware version of camera. */
		char firmwareVersion[sk_maxStringLength];
		/** Firmware build time. */
		char firmwareBuildTime[sk_maxStringLength];
		/** Maximum bus speed. */
		BusSpeed maximumBusSpeed;
		/** Bayer tile format. */
		BayerTileFormat bayerTileFormat;
		/** Bus number, set to 0 for GigE and USB cameras **/
		unsigned short busNumber;
		/** ieee1394 Node number, set to 0 for GigE and USB cameras **/
		unsigned short nodeNumber;
		/** PCIe Bus Speed, set to PCIE_BUSSPEED_UNKNOWN for unsupported drivers **/
		PCIeBusSpeed pcieBusSpeed;

		/** @name IIDC specific information */
		/*@{*/

		/** DCAM version. */
		unsigned int iidcVer;
		/** Configuration ROM data. */
		ConfigROM configROM;

		/*@}*/

		/** @name GigE specific information */
		/*@{*/

		/** GigE Vision version. */
		unsigned int gigEMajorVersion;
		/** GigE Vision minor version. */
		unsigned int gigEMinorVersion;
		/** User defined name. */
		char userDefinedName[sk_maxStringLength];
		/** XML URL 1. */
		char xmlURL1[sk_maxStringLength];
		/** XML URL 2. */
		char xmlURL2[sk_maxStringLength];
		/** MAC address */
		MACAddress macAddress;
		/** IP address. */
		IPAddress ipAddress;
		/** Subnet mask. */
		IPAddress subnetMask;
		/** Default gateway. */
		IPAddress defaultGateway;
		/** Status/Content of CCP register */
		unsigned int ccpStatus;
		/** Local Application IP Address. */
		unsigned int applicationIPAddress;
		/** Local Application port. */
		unsigned int applicationPort;
		/*@}*/

		/** Reserved for future use. */
		unsigned int reserved[16];

		CameraInfo()
		{
			serialNumber = 0;
			interfaceType = INTERFACE_UNKNOWN;
			driverType = DRIVER_UNKNOWN;
			isColorCamera = false;
			memset( modelName, 0, sizeof( modelName ) );
			memset( vendorName, 0, sizeof( vendorName ) );
			memset( sensorInfo, 0, sizeof( sensorInfo ) );
			memset( sensorResolution, 0, sizeof( sensorResolution ) );
			memset( driverName, 0, sizeof(driverName) );
			memset( firmwareVersion, 0, sizeof( firmwareVersion ) );
			memset( firmwareBuildTime, 0, sizeof( firmwareBuildTime ) );
			maximumBusSpeed = BUSSPEED_SPEED_UNKNOWN;
			bayerTileFormat = NONE;
			busNumber = 0;
			nodeNumber = 0;
			pcieBusSpeed = PCIE_BUSSPEED_UNKNOWN;

			// IIDC
			iidcVer = 0;

			// GigE
			gigEMajorVersion = 0;
			gigEMinorVersion = 0;

			ccpStatus = 0;
			applicationIPAddress = 0;
			applicationPort = 0;

			memset( userDefinedName, 0x0, sizeof(userDefinedName) );
			memset( xmlURL1, 0x0, sizeof(xmlURL1) );
			memset( xmlURL2, 0x0, sizeof(xmlURL2) );

			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** Properties of a single embedded image info property. */
	struct EmbeddedImageInfoProperty
	{
		/** Whether this property is available. */
		bool available;
		/** Whether this property is on or off. */
		bool onOff;

		EmbeddedImageInfoProperty()
		{
			available = false;
			onOff = false;
		}
	};

	/** Properties of the possible embedded image information. */
	struct EmbeddedImageInfo
	{
		EmbeddedImageInfoProperty timestamp;
		EmbeddedImageInfoProperty gain;
		EmbeddedImageInfoProperty shutter;
		EmbeddedImageInfoProperty brightness;
		EmbeddedImageInfoProperty exposure;
		EmbeddedImageInfoProperty whiteBalance;
		EmbeddedImageInfoProperty frameCounter;
		EmbeddedImageInfoProperty strobePattern;
		EmbeddedImageInfoProperty GPIOPinState;
		EmbeddedImageInfoProperty ROIPosition;
	};

	/** Metadata related to an image. */
	struct ImageMetadata
	{
		/** Embedded timestamp. */
		unsigned int embeddedTimeStamp;
		/** Embedded gain. */
		unsigned int embeddedGain;
		/** Embedded shutter. */
		unsigned int embeddedShutter;
		/** Embedded brightness. */
		unsigned int embeddedBrightness;
		/** Embedded exposure. */
		unsigned int embeddedExposure;
		/** Embedded white balance. */
		unsigned int embeddedWhiteBalance;
		/** Embedded frame counter. */
		unsigned int embeddedFrameCounter;
		/** Embedded strobe pattern. */
		unsigned int embeddedStrobePattern;
		/** Embedded GPIO pin state. */
		unsigned int embeddedGPIOPinState;
		/** Embedded ROI position. */
		unsigned int embeddedROIPosition;

		/** Reserved for future use. */
		unsigned int reserved[31];

		ImageMetadata()
		{
			embeddedTimeStamp = 0;
			embeddedGain = 0;
			embeddedShutter = 0;
			embeddedBrightness = 0;
			embeddedExposure = 0;
			embeddedWhiteBalance = 0;
			embeddedFrameCounter = 0;
			embeddedStrobePattern = 0;
			embeddedGPIOPinState = 0;
			embeddedROIPosition = 0;
			memset( reserved, 0, sizeof(reserved));
		}
	};

	/** Information about the camera's look up table. */
	struct LUTData
	{
		/** Flag indicating if LUT is supported. */
		bool supported;
		/** Flag indicating if LUT is enabled. */
		bool enabled;
		/** The number of LUT banks available (Always 1 for PGR LUT). */
		unsigned int numBanks;
		/** The number of LUT channels per bank available. */
		unsigned int numChannels;
		/** The input bit depth of the LUT. */
		unsigned int inputBitDepth;
		/** The output bit depth of the LUT. */
		unsigned int outputBitDepth;
		/** The number of entries in the LUT. */
		unsigned int numEntries;
		/** Reserved for future use. */
		unsigned int reserved[8];

		LUTData()
		{
			supported = false;
			enabled = false;
			numBanks = 0;
			numChannels = 0;
			inputBitDepth = 0;
			outputBitDepth = 0;
			numEntries = 0;
			memset( reserved, 0, sizeof(reserved) );
		}
	};

	/** Camera diagnostic information. */
	struct CameraStats
	{
		/* Number of dropped images in DROP_IMAGE mode */
		unsigned int imageDropped;
		/* Number of corrupt images, such as missing packet, zero packet */
		unsigned int imageCorrupt;
		/* Number of transmissions failed of camera */
		unsigned int imageXmitFailed;
		/* Number of images dropped in driver */
		unsigned int imageDriverDropped;
		/* Errors of register reading */
		unsigned int regReadFailed;
		/* Errors of register writing */
		unsigned int regWriteFailed;
		/* Port errors */
		unsigned int portErrors;
		/* The value of the camera power register.
		 * false: Camera is powered down.
		 * true: Camera is powered up.
		 */
		bool cameraPowerUp;

		/* The voltage values of the various voltage registers
		 * supported by the camera.
		 */
		float cameraVoltages[8];
		/** The number of voltage registers available.
		 * 0: the values in cameraVoltages[] are invalid.
		 */
		unsigned int numVoltages;

		/* The current values of the various current registers
		 * supported by the camera.
		 */
		float cameraCurrents[8];
		/** The number of current registers available.
		 * 0: the values in cameraCurrents[] are invalid.
		 */
		unsigned int numCurrents;
		/* The temperature of the camera board-level components. The value is
		 * in kelvins (0°C = 273.15K) and are in one-tenths (0.1) of a kelvin.
		 */
		unsigned int temperature;
		/* Time in seconds since the camera was initialized. */
		unsigned int timeSinceInitialization;
		/* Time in seconds since the camera detected a bus reset. */
		unsigned int timeSinceBusReset;
		/* Time stamp */
		TimeStamp timeStamp;
		/* Number of packets requested for resend */
		unsigned int numResendPacketsRequested;
		/* Number of packet-resend packets received */
		unsigned int numResendPacketsReceived;
		/** Reserved for future use. */
		unsigned int reserved[16];

		CameraStats()
		{
			imageDropped = 0;
			imageCorrupt = 0;
			imageXmitFailed = 0;
			imageDriverDropped = 0;
			regReadFailed = 0;
			regWriteFailed = 0;
			portErrors = 0;
			cameraPowerUp = false;
			memset( cameraVoltages, 0, sizeof(cameraVoltages) );
			numVoltages = 0;
			memset( cameraCurrents, 0, sizeof(cameraCurrents) );
			numCurrents = 0;
			temperature = 0;
			timeSinceInitialization = 0;
			timeSinceBusReset = 0;
			memset( reserved, 0, sizeof(reserved) );
		}
	};


	/**
	 * @defgroup ImageSaveStructures Image saving structures.
	 *
	 * These structures define various parameters used for saving images.
	 */

	/*@{*/

	/** Options for saving PNG images. */
	struct PNGOption
	{
		/** Whether to save the PNG as interlaced. */
		bool interlaced;
		/** Compression level (0-9). 0 is no compression, 9 is best compression. */
		unsigned int compressionLevel;
		/** Reserved for future use. */
		unsigned int reserved[16];

		PNGOption()
		{
			interlaced = false;
			compressionLevel = 6;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/** Options for saving PPM images. */
	struct PPMOption
	{
		/** Whether to save the PPM as a binary file. */
		bool binaryFile;
		/** Reserved for future use. */
		unsigned int reserved[16];

		PPMOption()
		{
			binaryFile = true;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/** Options for saving PGM images. */
	struct PGMOption
	{
		/** Whether to save the PPM as a binary file. */
		bool binaryFile;
		/** Reserved for future use. */
		unsigned int reserved[16];

		PGMOption()
		{
			binaryFile = true;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/** Options for saving TIFF images. */
	struct TIFFOption
	{
		enum CompressionMethod
		{
			NONE = 1, /**< Save without any compression. */
			PACKBITS, /**< Save using PACKBITS compression. */
			DEFLATE, /**< Save using DEFLATE compression (ZLIB compression). */
			ADOBE_DEFLATE, /**< Save using ADOBE DEFLATE compression */
			/**
			 * Save using CCITT Group 3 fax encoding. This is only valid for
			 * 1-bit images only. Default to LZW for other bit depths.
			 */
			CCITTFAX3,
			/**
			 * Save using CCITT Group 4 fax encoding. This is only valid for
			 * 1-bit images only. Default to LZW for other bit depths.
			 */
			CCITTFAX4,
			LZW, /**< Save using LZW compression. */
			/**
			 * Save using JPEG compression. This is only valid for 8-bit
			 * greyscale and 24-bit only. Default to LZW for other bit depths.
			 */
			JPEG
		};

		/** Compression method to use for encoding TIFF images. */
		CompressionMethod compression;
		/** Reserved for future use. */
		unsigned int reserved[16];

		TIFFOption()
		{
			compression = LZW;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/** Options for saving JPEG image. */
	struct JPEGOption
	{
		/** Whether to save as a progressive JPEG file. */
		bool progressive;
		/**
		 * JPEG image quality in range (0-100).
		 * - 100 - Superb quality.
		 * - 75  - Good quality.
		 * - 50  - Normal quality.
		 * - 10  - Poor quality.
		 */
		unsigned int quality;
		/** Reserved for future use. */
		unsigned int reserved[16];

		JPEGOption()
		{
			progressive = false;
			quality = 75;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/** Options for saving JPEG2000 image. */
	struct JPG2Option
	{
		/** JPEG saving quality in range (1-512). */
		unsigned int quality;
		/** Reserved for future use. */
		unsigned int reserved[16];

		JPG2Option()
		{
			quality = 16;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/** Options for saving Bitmap image. */
	struct BMPOption
	{
		bool indexedColor_8bit;
		/** Reserved for future use. */
		unsigned int reserved[16];

		BMPOption()
		{
			indexedColor_8bit = false;
			memset(reserved, 0, sizeof(reserved));
		}
	};

	/** Options for saving MJPG files. */
	struct MJPGOption
	{
		/** Frame rate of the stream */
		float frameRate;

		/** Image quality (1-100) */
		unsigned int quality;

		unsigned int reserved[256];

		MJPGOption()
		{
			frameRate = 15.0;
			quality = 75;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/** Options for saving H264 files. */
	struct H264Option
	{
		/** Frame rate of the stream */
		float frameRate;

		/** Width of source image */
		unsigned int width;

		/** Height of source image */
		unsigned int height;

		/** Bitrate to encode at */
		unsigned int bitrate;

		/** Reserved for future use */
		unsigned int reserved[256];

		H264Option()
		{
			frameRate = 15.0;
			width = 0;
			height = 0;
			bitrate = 1000000;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/** Options for saving AVI files. */
	struct AVIOption
	{
		/** Frame rate of the stream */
		float frameRate;

		/** Reserved for future use */
		unsigned int reserved[256];

		AVIOption()
		{
			frameRate = 15.0;
			memset(reserved, 0, sizeof (reserved));
		}
	};

	/* Callback function typedef: a function accepting pointer-to-void input as its sole
	 * argument, and returns void (ie: nothing).
	 */
	typedef void (*CameraEventCallback)(void* data);

	/** Options for enabling device event registration. */
	struct EventOptions
	{
		/** Callback function pointer */
		CameraEventCallback EventCallbackFcn;

		/** Event name to register */
		const char* EventName;

		/** Pointer to callback data to be passed to the callback function */
		const void* EventUserData;

		/** Size of the underlying struct passed as eventCallbackData for sanity checks */
		size_t EventUserDataSize;
	};


	/* Callback data passed to the callback function provided when using
	 * RegisterEvent() or RegisterAllEvents().
	 */
	struct EventCallbackData
	{
		/** Pointer to the user-supplied data struct */
		void* EventUserData;

		/** Size of the user data data supplied to the RegisterEvent()
		 * function.
		 */
		size_t EventUserDataSize;

		/** The event name used to register the event. Provided so the user
		 * knows which event triggered the callback.
		 */
		const char* EventName;

		/** The device register which EventName maps to. Provides an alternate
		 * means of indexing into different event types.
		 */
		long long unsigned EventID;

		/** Timestamp indicated the time (as reported by the camera) at which
		 * the camera exposure operation completed. This can be compared with
		 * image stimestamps if there is a need to map event timestamps to
		 * specific images, if applicable.
		 */
		long long unsigned EventTimestamp;

		/** A pointer to additional data pertaining to the event which just
		 * trigger the callback function. The data may be of difference sizes
		 * or may not even be allocated, depending on the type of event which
		 * triggered the callback.
		 */
		void* EventData;

		/** The size of the structure pointed to by EventData. This value should
		 * be checked, especially if there are events which can trigger variable-
		 * length event data to be returned to the user when the callback function
		 * is issued.
		 */
		size_t EventDataSize;
	};
	/*@}*/

	/*@}*/

}

#endif // PGR_FC2_FLYCAPTURE2DEFS_H

