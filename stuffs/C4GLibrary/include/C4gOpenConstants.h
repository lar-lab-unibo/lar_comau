/*
	C4gOpenConstants.hpp

	Copyright (C) 2007-2008 Sintesi S.C.p.A.

	Developers:
		Sabino   COLONNA (2006-, s.colonna@sintesi-scpa.com)
		Giovanni IACCA   (2006-, g.iacca@sintesi-scpa.com  )
		Giovanni TOTARO  (2006-, g.totaro@sintesi-scpa.com )


	This file is part of libC4gOpen.

	libC4gOpen is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; either version 2.1 of
    the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
	@file C4gOpenConstants.hpp
	@brief Constants and enums used by #C4gOpen class.

	This file contains the definition of constants and enums referring to C4Gopen
	physical limits, C4Gopen modes, including the special ones, and error codes
	corresponding to errors occurring in the use of #C4gOpen class methods.
*/

#ifndef _C4GOPENCONSTANTS_HPP_
#define _C4GOPENCONSTANTS_HPP_


#define DEFAULT_PORT_NUMBER			1000	///< Default port number of PC-C4G socket.

#define TWO_PI						6.283185307179586

///////////////////////////////// C4G Open Constraints /////////////////////////////////

#define MAX_NUM_ARMS				4		///< Maximum number of arms.
#define MAX_NUM_AXES_PER_ARM	  	10		///< Maximum number of axes per arm.
#define MAX_NUM_OPEN_AXES		  	20		///< Maximum number of axes in 'open mode'.


///////////////////////////////////// Open modes ///////////////////////////////////////

#define C4G_OPEN_MODE_0				0
#define C4G_OPEN_MODE_0_DEBUG		10
#define C4G_OPEN_MODE_1				1
#define C4G_OPEN_MODE_1_STANDBY		11
#define C4G_OPEN_MODE_2				2
#define C4G_OPEN_MODE_2_STANDBY		12
#define C4G_OPEN_MODE_4				4
#define C4G_OPEN_MODE_4_STANDBY		14
#define C4G_OPEN_MODE_5				5
#define C4G_OPEN_MODE_5_STANDBY		15
#define C4G_OPEN_MODE_7				7
#define C4G_OPEN_MODE_7_STANDBY		17
#define C4G_OPEN_MODE_8				8
#define C4G_OPEN_MODE_9				9

/////////////////////////////////// Special Modes //////////////////////////////////////

#define C4G_OPEN_ACTIVE_FREEZING		500
#define C4G_OPEN_DRIVING_ON				501
#define C4G_OPEN_PASSIVE_FREEZING		502
#define C4G_OPEN_EXIT					504
#define C4G_OPEN_DRIVE_OFF				505
#define C4G_OPEN_CLIENT_RESTART			506
#define C4G_OPEN_FOLLOWING_ERROR		508

/// Vector of the valid open modes that the user can set.
const long validOpenModes[] = 
{
	C4G_OPEN_MODE_0,
	C4G_OPEN_MODE_0_DEBUG,
	C4G_OPEN_MODE_1,
	C4G_OPEN_MODE_1_STANDBY,
	C4G_OPEN_MODE_2,
	C4G_OPEN_MODE_2_STANDBY,
	C4G_OPEN_MODE_4,
	C4G_OPEN_MODE_4_STANDBY,
	C4G_OPEN_MODE_5,
	C4G_OPEN_MODE_5_STANDBY,
	C4G_OPEN_MODE_7,
	C4G_OPEN_MODE_7_STANDBY,
	C4G_OPEN_MODE_8,
	C4G_OPEN_MODE_9,
	C4G_OPEN_ACTIVE_FREEZING,
	C4G_OPEN_DRIVE_OFF
};


////////////////////////////////////// Arm Status ///////////////////////////////////////

#define EXIT_FROM_OPEN		1	///< Arm status used in the #Header::status field of an outgoing communication packet.
#define DRIVE_ON			5	///< Arm status used in the #Header::status field of an incoming communication packet.


///////////////////////////////////// Error codes //////////////////////////////////////

/// Error codes corresponding to errors occurring in the use of #C4gOpen class methods.
enum ErrorCodes { NO_ERROR = -1,
				  RT_TASK_INIT_ERROR = 1,			/**< Error during the initialization of RTAI task. */
				  RT_SOCKET_INIT_ERROR,				/**< Error during the initialization of RTnet socket. */
				  RT_SOCKET_OPEN_ERROR,				/**< Error while opening RTnet socket. */
				  RT_SOCKET_BINDING_ERROR,			/**< Error while binding RTnet socket. */
				  RT_SOCKET_SEND_ERROR,				/**< Error while sending a packet to C4G. */
				  RT_SOCKET_RECEIVE_ERROR,			/**< Error while receiving a packet from C4G. */
				  INIT_PACKET_UNEXPECTED_SEQ_NUM,	/**< Unexpected sequence number of the initialization packet. */
				  INIT_PACKET_RECEIVE_ERROR,		/**< Error while receiving the initialization packet from C4G. */
				  INVALID_ARM_NUMBER,				/**< Access to an invalid arm number. */
				  INVALID_AXIS_NUMBER,				/**< Access to an invalid axis number. */
				  ARM_NOT_IN_OPEN_MODE,				/**< Access to a not 'open mode' arm. */
				  AXIS_NOT_IN_OPEN_MODE,			/**< Access to a not 'open mode' axis. */
				  INVALID_OPEN_MODE,				/**< Setting of an invalid open mode. */
				  SET_MODE_NOT_ALLOWED,				/**< Call to #C4gOpen::setMode when #C4gOpen::canChangeMode is false. */
				  OPERATION_NOT_ALLOWED,			/**< Call to an operation not allowed by the actual open mode. */
				  SET_TARGET_POSITION_MISSING,		/**< Target position not set even if requested by the actual open mode. */
				  SET_TARGET_VELOCITY_MISSING,		/**< Target velocity not set even if requested by the actual open mode. */
				  SET_MEASURE_MISSING,				/**< Measure from external sensor not set even if requested by the actual open mode. */
				  SET_FFW_VELOCITY_MISSING,			/**< Feedforward velocity not set even if requested by the actual open mode. */
				  SET_FFW_CURRENT_MISSING,			/**< Feedforward current not set even if requested by the actual open mode. */
				  SET_DELTA_CURRENT_MISSING,		/**< Delta current not set even if requested by the actual open mode. */
				  FOLLOWING_ERROR_OVERCOME			/**< Following error threshold overcome. */
				};

#endif // _C4GOPENCONSTANTS_HPP_

