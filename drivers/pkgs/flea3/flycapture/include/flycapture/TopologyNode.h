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
// $Id: TopologyNode.h 316355 2017-02-20 23:02:28Z alin $
//=============================================================================

#ifndef PGR_FC2_TOPOLOGYNODE_H
#define PGR_FC2_TOPOLOGYNODE_H

#include "FlyCapture2Platform.h"
#include "FlyCapture2Defs.h"

namespace FlyCapture2
{
	class Error;

	/**
	 * The TopologyNode class contains topology information that can be
	 * used to generate a tree structure of all cameras and devices connected
	 * to a computer.
	 */
	class FLYCAPTURE2_API TopologyNode
	{
		public:

			/**
			 * Possible states of a port on a node.
			 */
			enum PortType
			{
				NOT_CONNECTED = 1,
				CONNECTED_TO_PARENT,
				CONNECTED_TO_CHILD
			};

			/**
			 * Type of node.
			 */
			enum NodeType
			{
				COMPUTER,
				BUS,
				CAMERA,
				NODE
			};

			/**
			 * Default constructor.
			 */
			TopologyNode();

			/**
			 * Constructor.
			 *
			 * @param guid The PGRGuid of the node (if applicable).
			 * @param deviceId Device ID of the node.
			 * @param nodeType Type of the node.
			 * @param interfaceType Interface type of the node.
			 */
			TopologyNode(
					PGRGuid guid,
					int deviceId,
					NodeType nodeType,
					InterfaceType interfaceType );

			/**
			 * Default destructor.
			 */
			virtual ~TopologyNode();

			/**
			 * Copy constructor.
			 */
			TopologyNode( const TopologyNode& other );

			/**
			 * Assignment operator.
			 *
			 * @param other The TopologyNode to copy from.
			 */
			virtual TopologyNode& operator=( const TopologyNode& other );

			/**
			 * Get the PGRGuid associated with the node.
			 *
			 * @return PGRGuid of the node.
			 */
			virtual PGRGuid GetGuid();

			/**
			 * Get the device ID associated with the node.
			 *
			 * @return Device ID of the node.
			 */
			virtual int GetDeviceId();

			/**
			 * Get the node type associated with the node.
			 *
			 * @return Node type of the node.
			 */
			virtual NodeType GetNodeType();

			/**
			 * Get the interface type associated with the node.
			 *
			 * @return Interface type of the node.
			 */
			virtual InterfaceType GetInterfaceType();

			/**
			 * Get the number of child nodes.
			 *
			 * @return Number of child nodes.
			 */
			virtual unsigned int GetNumChildren();

			/**
			 * Get child node located at the specified position.
			 *
			 * @param position Position of the node.
			 *
			 * @return TopologyNode at the specified position.
			 */
			virtual TopologyNode GetChild( unsigned int position );

			/**
			 * Add the specified TopologyNode as a child of the node.
			 *
			 * @param childNode The TopologyNode to add.
			 */
			virtual void AddChild( TopologyNode childNode );

			/**
			 * Get the number of ports.
			 *
			 * @return Number of ports.
			 */
			virtual unsigned int GetNumPorts();

			/**
			 * Get type of port located at the specified position.
			 *
			 * @param position Position of the port.
			 *
			 * @return PortType at the specified position.
			 */
			virtual PortType GetPortType( unsigned int position );

			/**
			 * Add the specified PortType as a port of the node.
			 *
			 * @param childPort The port to add.
			 */
			virtual void AddPortType( PortType childPort );

			/**
			 * Assign a PGRGuid and device ID to the node.
			 *
			 * @param guid PGRGuid to be assigned.
			 * @param deviceId Device ID to be assigned.
			 *
			 * @return Whether the data was successfully set to the node.
			 */
			virtual bool AssignGuidToNode( PGRGuid guid, int deviceId );

			/**
			 * Assign a PGRGuid, device ID and nodeType to the node.
			 *
			 * @param guid PGRGuid to be assigned.
			 * @param deviceId Device ID to be assigned.
			 * @param nodeType NodeType to be assigned
			 *
			 * @return Whether the data was successfully set to the node.
			 */
			virtual bool AssignGuidToNode(PGRGuid guid, int deviceId, NodeType nodeType);

		private:
			struct TopologyNodeData;
			TopologyNodeData* m_pData;
	};
}

#endif
