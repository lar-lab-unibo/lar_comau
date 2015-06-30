/*
	C4gOpen.hpp

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
	@file C4gOpen.hpp
	@brief Structs and class to allow a PC to communicate with Comau C4Gopen.
	
	This file contains declaration of:<br>
	<ul>
	<li>InitPacket, a struct that represents the inizialization packet;</li>
	<li>Header, a struct that represents the header of a communication packet;</li>
	<li>AxisDataRx, a struct that represents the axis data fields of an incoming communication packet;</li>
	<li>AxisDataTx, a struct that represents the axis data fields of an outgoing communication packet;</li>
	<li>CommPacketRx, a struct that represents an incoming communication packet;</li>
	<li>CommPacketTx, a struct that represents an outgoing communication packet;</li>
	<li>C4gOpen, a class that contains members and methods to allow a PC to communicate with Comau C4Gopen.</li>
	</ul>
*/

#ifndef _C4GOPEN_HPP_
#define _C4GOPEN_HPP_

#include <rtai_lxrt.h>
#include <rtnet.h>
#include <sys/mman.h>
#include <math.h>

#include <sys/socket.h>
#include <netinet/in.h>

#include <C4gOpenConstants.h>

#define STACK_SIZE_KB		 	1024			///< Stack size (expressed in KB) of RTAI task passed to rt_grow_and_lock_stack() function.

#define MAX_SIZE_OF_PACKETS		1500			///< Maximum size of a packet exchanged between C4G and PC.
#define WAIT_INIT_PCK_TIMEOUT	300000000000LL	///< Socket timeout while waiting for the arrival of initialization packet.
#define COMMUNICATION_TIMEOUT	1000000000LL	///< Socket timeout during normal communication between C4G and PC.

/// Structure representing the initialization packet.
/**
	This packet is sent by C4G to PC before beginning the actual communication, after the synchronization phase.
	It contains C4Gopen System informations and robot arms data.
*/
struct InitPacket	// Size = 1200 bytes
{
	long  seqNumber;										///< Sequence number of the initialization packet.
	long  numberOfFieldsPerPacket;							///< Number of header fields of a communication packet.
	long  numberOfFieldsPerAxis;							///< Number of axis data fields of a communication packet.
	long  numberOfAxesInOpenMode;							///< Number of 'open mode' axes.
	long  arm1OpenMode[MAX_NUM_AXES_PER_ARM];				///< Array of axes mode of Arm 1.
	long  arm2OpenMode[MAX_NUM_AXES_PER_ARM];				///< Array of axes mode of Arm 2.
	long  arm3OpenMode[MAX_NUM_AXES_PER_ARM];				///< Array of axes mode of Arm 3.
	long  arm4OpenMode[MAX_NUM_AXES_PER_ARM];				///< Array of axes mode of Arm 4.
	long  typeOfHeaderField1;								///< Type of 1st header field of a communication packet.
	long  typeOfHeaderField2;								///< Type of 2nd header field of a communication packet.
	long  typeOfHeaderField3;								///< Type of 3rd header field of a communication packet.
	long  typeOfAxisDataField1;								///< Type of 1st axis data field of a communication packet.
	long  typeOfAxisDataField2;								///< Type of 2nd axis data field of a communication packet.
	long  typeOfAxisDataField3;								///< Type of 3rd axis data field of a communication packet.
	long  typeOfAxisDataField4;								///< Type of 4th axis data field of a communication packet.
	long  typeOfAxisDataField5;								///< Type of 5th axis data field of a communication packet.
	long  typeOfAxisDataField6;								///< Type of 6th axis data field of a communication packet.
	long  typeOfAxisDataField7;								///< Type of 7th axis data field of a communication packet.
	long  typeOfAxisDataField8;								///< Type of 8th axis data field of a communication packet.
	long  typeOfAxisDataField9;								///< Type of 9th axis data field of a communication packet.
	long  sampleTime;										///< Communication cycle time [ms].
	long  arm1OpenAxesMap[MAX_NUM_AXES_PER_ARM];			///< Open/Logical axes map of Arm 1.
	long  arm2OpenAxesMap[MAX_NUM_AXES_PER_ARM];			///< Open/Logical axes map of Arm 2.
	long  arm3OpenAxesMap[MAX_NUM_AXES_PER_ARM];			///< Open/Logical axes map of Arm 3.
	long  arm4OpenAxesMap[MAX_NUM_AXES_PER_ARM];			///< Open/Logical axes map of Arm 4.
	float arm1CalibrationConstant[MAX_NUM_AXES_PER_ARM];	///< Calibration constants of axes of Arm 1 [gear rotations].
	float arm2CalibrationConstant[MAX_NUM_AXES_PER_ARM];	///< Calibration constants of axes of Arm 2 [gear rotations].
	float arm3CalibrationConstant[MAX_NUM_AXES_PER_ARM];	///< Calibration constants of axes of Arm 3 [gear rotations].
	float arm4CalibrationConstant[MAX_NUM_AXES_PER_ARM];	///< Calibration constants of axes of Arm 4 [gear rotations].
	float arm1CurrentLimit[MAX_NUM_AXES_PER_ARM];			///< Current limits of axes of Arm 1 [A].
	float arm2CurrentLimit[MAX_NUM_AXES_PER_ARM];			///< Current limits of axes of Arm 2 [A].
	float arm3CurrentLimit[MAX_NUM_AXES_PER_ARM];			///< Current limits of axes of Arm 3 [A].
	float arm4CurrentLimit[MAX_NUM_AXES_PER_ARM];			///< Current limits of axes of Arm 4 [A].
	long  majorNumber;										///< C4G system software major number.
	long  minorNumber;										///< C4G system software minor number.
	long  buildNumber;										///< C4G system software build number.
	float arm1KinInflCoeff[MAX_NUM_AXES_PER_ARM];			///< Kinematic Influence Coefficients of axes of Arm 1.
	float arm2KinInflCoeff[MAX_NUM_AXES_PER_ARM];			///< Kinematic influence coefficients of axes of Arm 2.
	float arm3KinInflCoeff[MAX_NUM_AXES_PER_ARM];			///< Kinematic influence coefficients of axes of Arm 3.
	float arm4KinInflCoeff[MAX_NUM_AXES_PER_ARM];			///< Kinematic influence coefficients of axes of Arm 4.
	float arm1TxRate[MAX_NUM_AXES_PER_ARM];					///< Transmission rates of axes of Arm 1.
	float arm2TxRate[MAX_NUM_AXES_PER_ARM];					///< Transmission rates of axes of Arm 2.
	float arm3TxRate[MAX_NUM_AXES_PER_ARM];					///< Transmission rates of axes of Arm 3.
	float arm4TxRate[MAX_NUM_AXES_PER_ARM];					///< Transmission rates of axes of Arm 4.
	float arm1FollowingError[MAX_NUM_AXES_PER_ARM];			///< Following error thresholds of axes of Arm 1 [gear rotations].
	float arm2FollowingError[MAX_NUM_AXES_PER_ARM];			///< Following error thresholds of axes of Arm 2 [gear rotations].
	float arm3FollowingError[MAX_NUM_AXES_PER_ARM];			///< Following error thresholds of axes of Arm 3 [gear rotations].
	float arm4FollowingError[MAX_NUM_AXES_PER_ARM];			///< Following error thresholds of axes of Arm 4 [gear rotations].
};

/// Structure representing the header of a communication packet.
struct Header
{
	long seqNumber;			///< Sequence number of the communication packet.
	long status;			///< Arm status (#DRIVE_ON or #EXIT_FROM_OPEN).
	long functionality;		///< Open mode functionality exchanged between C4G and PC.
};

/// Structure representing the axis data fields of an incoming communication packet.
/**
	Each field is always present, but its meaning and value depend on the specific open mode.
*/
struct AxisDataRx
{
	long mode;				///< Axis mode.
	float targetPosition;	///< Target position computed by C4G.
							/**< The value of this field is significant depending on the specific open mode. */
	float targetVelocity;	///< Target velocity computed by C4G.
							/**< The value of this field is significant depending on the specific open mode. */
	float actualPosition;	///< Position given by the encoder.
							/**< The value of this field is always significant, apart from the specific open mode. */
	float actualVelocity;	///< Velocity given by the encoder.
							/**< The value of this field is always significant, apart from the specific open mode. */
	float targetCurrent;	///< Target current computed by C4G.
							/**< The value of this field is significant depending on the specific open mode. */
	float extra1;			///< First extra information. 
	float extra2;			///< Second extra information.
	float extra3;			///< Third extra information.
};

/// Structure representing the axis data fields of an outgoing communication packet.
/**
	Each field is always present, but its meaning and value depend on the specific open mode.
*/
struct AxisDataTx
{
	long mode;				///< Axis mode.
	float targetPosition;	///< Target position computed by PC.
							/**< The value of this field is significant depending on the specific open mode. */
	float targetVelocity;	///< Target velocity computed by PC.
							/**< The value of this field is significant depending on the specific open mode. */
	float measure;			///< Measure given from an external sensor.
							/**< The value of this field is significant depending on the specific open mode. */
	float ffwVelocity;		///< Feedforward velocity computed by PC.
							/**< The value of this field is significant depending on the specific open mode. */
	float ffwCurrent;		///< Feedforward current computed by PC.
							/**< The value of this field is significant depending on the specific open mode. */
	float extra1;			///< First extra information. 
	float extra2;			///< Second extra information.
	float extra3;			///< Third extra information.
};

/// Structure representing an incoming communication packet.
/**
	An incoming communication has always the header field and
	as many axis data fields as the number of axes in open mode.
*/
struct CommPacketRx
{
	Header header;								///< Header of the incoming communication packet.
	AxisDataRx axisData[MAX_NUM_OPEN_AXES];		///< Data fields of the incoming communication packet.
};

/// Structure representing an outgoing communication packet.
/**
	An outgoing communication has always the header field and
	as many axis data fields as the number of axes in open mode.
*/
struct CommPacketTx
{
	Header header;								///< Header of the outgoing communication packet.
	AxisDataTx axisData[MAX_NUM_OPEN_AXES];		///< Data fields of the outgoing communication packet.
};

/// Class containing members and methods to allow a PC to communicate with Comau C4Gopen.
class C4gOpen
{
	private:
		RT_TASK *realTimeTask;				///< Pointer to RT_TASK structure.
		int realTimeSocket;					///< RTnet socket identifier.
		sockaddr_in realTimeSocketStruct;	///< Server IPv4 structure.
		sockaddr_in c4gSocketStruct;		///< Client IPv4 structure.
		unsigned int c4gSocketLength;		///< Size of the buffer associated with client socket.
		int portNumber;						///< C4G-PC UDP socket port number.
		long long communicationTimeout;		///< Socket timeout during normal communication between C4G and PC.
		
		void *startUpPacket;				///< Pointer to a packet exchanged during the initialization/synchronization phase.
		
		int logicalToOpenMap[MAX_NUM_ARMS * MAX_NUM_AXES_PER_ARM];	///< Open/Logical axes map.
																	/**< This array is filled when #mapIndices() method is called. */
		
		InitPacket initPacket;		///< Instance of an initialization packet.
		CommPacketRx commPacketRx;	///< Instance of an incoming communication packet.
		CommPacketTx commPacketTx;	///< Instance of an outgoing communication packet.
		
		long numberOfOpenAxes;		///< Number of axes in open mode.
									/**< This value is read from the #InitPacket::numberOfAxesInOpenMode field of the initialization packet. */
		long sampleTime;			///< Communication cycle time.
									/**< This value is read from #InitPacket::sampleTime field of the initialization packet. */

		ErrorCodes lastError;		///< Code of the last occurred error.

		bool followingErrorOvercome[MAX_NUM_OPEN_AXES];			///< Array of flags indicating if a following error has occurred for a certain axis.
		bool setTargetPositionDone[MAX_NUM_OPEN_AXES];			///< Array of flags indicating if target position has been set for a certain axis.
		bool setTargetVelocityDone[MAX_NUM_OPEN_AXES];			///< Array of flags indicating if target velocity has been set for a certain axis.
		bool setMeasureDone[MAX_NUM_OPEN_AXES];					///< Array of flags indicating if measure from an external sensor has been set for a certain axis.
		bool setFeedForwardVelocityDone[MAX_NUM_OPEN_AXES];		///< Array of flags indicating if feedforward velocity has been set for a certain axis.	
		bool setFeedForwardCurrentDone[MAX_NUM_OPEN_AXES];		///< Array of flags indicating if feedforward current has been set for a certain axis.
		bool setDeltaCurrentDone[MAX_NUM_OPEN_AXES];			///< Array of flags indicating if delta current has been set for a certain axis.
		bool setModeDone[MAX_NUM_ARMS];							///< Array of flags indicating if an explicit mode setting has been done for a certain arm.
		bool exitFromOpenDone[MAX_NUM_ARMS];					///< Array of flags indicating if an exit from open mode has been requested for a certain arm.
		bool canChangeMode[MAX_NUM_ARMS];						///< Array of flags indicating if the open mode of a certain arm can be changed.
																/**< The open mode of an arm can be changed only after having performed #exitFromOpen().*/

		void resetInitPacket();
		void resetCommPacketRx();
		void resetCommPacketTx();
		void resetFlags();

public:
		bool startWithRtTask();
		bool startWithoutRtTask();
		bool initRealTimeSocket();
		bool initRealTimeTask();
		bool waitInitPacket();
		bool readInitPacket();
		void closeRealTimeTask();
		void closeRealTimeSocket();
		
		int receivePacket(void *packet, int bytesToBeReceived);
		int sendPacket(void *packet, int bytesToSend);

		void mapIndices(int arm, long armOpenAxesMap[MAX_NUM_AXES_PER_ARM]);
		int getOpenIndex(int arm, int axis);
		
		bool checkArm(int arm);
		bool checkAxis(int arm, int axis);
		bool checkMode(long mode);
		bool checkFollowingError(int arm, int axis, float targetPosition);
		bool checkOperations();


		C4gOpen();
		C4gOpen(int portNumber);
		~C4gOpen();

		bool start(bool taskToInit = true);
		bool receive();
		bool send();
		void stop(bool taskToClose = true);

		long long getCommunicationTimeout();
		void setCommunicationTimeout(long long timeout);
		
		bool errorOccurred();
		ErrorCodes getLastError();
		void resetError();

		long getNumberOfOpenAxes();
		long getSampleTime();
		float getCalibrationConstant(int arm, int axis);
		float getCurrentLimit(int arm, int axis);
		float getTxRate(int arm, int axis);
		float getKinInflCoeff54(int arm);
		float getKinInflCoeff64(int arm);
		float getKinInflCoeff65(int arm);
		float getFollowingErrorThreshold(int arm, int axis);

		bool isInDriveOn(int arm);
		long getFunctionality(int arm);
		long getMode(int arm);

		float getTargetPosition(int arm, int axis);
		float getTargetVelocity(int arm, int axis);
		float getActualPosition(int arm, int axis);
		float getActualVelocity(int arm, int axis);
		float getTargetCurrent(int arm, int axis);
		float getDynamicModel(int arm, int axis);
		float getDiagonalInertia(int arm, int axis);
		float getExtra1(int arm, int axis);
		float getExtra2(int arm, int axis);
		float getExtra3(int arm, int axis);
		
		bool exitFromOpen(int arm);
		bool setMode(int arm, long mode);

		bool setTargetPosition(int arm, int axis, float targetPosition);
		bool setTargetVelocity(int arm, int axis, float targetVelocity);
		bool setMeasure(int arm, int axis, float measure);
		bool setFeedForwardVelocity(int arm, int axis, float ffwVelocity);
		bool setFeedForwardCurrent(int arm, int axis, float ffwCurrent);
		bool setDeltaCurrent(int arm, int axis, float deltaCurrent);
		bool setExtra1(int arm, int axis, float extra1);
		bool setExtra2(int arm, int axis, float extra2);
		bool setExtra3(int arm, int axis, float extra3);
};

#endif // _C4GOPEN_HPP_

