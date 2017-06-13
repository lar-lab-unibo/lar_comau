/*
	Mode5_SERVER.cpp
	
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <sstream>
#include <sys/uio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <iostream>
#include <math.h>
#include <pthread.h>
#include <signal.h>

#include <C4gOpen.hpp>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "lar_comau/MachineAlarmStatus.h"

#define ROBOT_CONTROLLER_PUBLISHER_NAME "joint_states"
#define ROBOT_CONTROLLER_SUBSCRIBER_NAME "joint_command"
#define PI 3.14159265358979
#define ARM 1

bool setpoint_in_deg = true;
double mesg[7];
bool allowMotion = false;
bool startServer;
bool serverActive;
bool IS_AXIS_SELECTED[6];
bool AXIS_ENABLED[6];

bool wasGoing = true;
bool wasGoing_old = true;

double setPoint[6] = {0, 0, 0, 0, 0, 0};
double getGears[6] = {0, 0, 0, 0, 0, 0};
double Speed[6];

//FILTER VARS
double Z[6][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
double B[6][10];
double Acc = 0; //output
double X = 0;   //input
double Fc = 0.5;
double Fs, Q, W, N, B0, B1, B2, A1, A2;
double FcMax = 0.5;
double FcMin = 0.1;
double threshold[6] = {10, 10, 10, 30, 15, 30};

double initialPositions[6];

double calibCONSTANTS[6];

bool init[6] = {1, 1, 1, 1, 1, 1};
bool sinAxisEnabled[6];


double OP_LIM[6][2] = {{-170, 170}, {-85, 175}, {-170, 0}, {-210, 210}, {-130, 130}, {-2700, 2700}};
double VT_LIM[6][2] = {{-170, 170}, {-85, 175}, {-170, 0}, {-210, 210}, {-100, 100}, {-800, 800}};

/* joint position variation limit */
double DPOS_LIM[6] = {10, 10, 10, 10, 10, 10};

double actualPositions[6];
double previousPositions[6];
double deltaPositions[6];

lar_comau::MachineAlarmStatus current_state;	

using namespace std;

const char disclaimer[] = ""; //

void updateFilter(double cutoff)
{
	W = tan(PI * cutoff / Fs);
	N = 1.0 / (pow(W, 2) + W / Q + 1);
	B0 = N * pow(W, 2);
	B1 = 2 * B0;
	B2 = B0;
	A1 = 2 * N * (pow(W, 2) - 1);
	A2 = N * (pow(W, 2) - W / Q + 1);
}

void jCallback(const sensor_msgs::JointState &msg)
{

	double degreeSetPoint;

	while (!allowMotion);

	for (int i = 0; i < 6; i++)
	{
		// if(setpoint_in_deg){
		// 	degreeSetPoint=msg.position[i];
		// }else{
		// 	degreeSetPoint=(msg.position[i]*180.0f)/M_PI;
		// }
		setPoint[i] = (msg.position[i] * 180.0f) / M_PI;

		//Operative limit control
		if (setPoint[i] < OP_LIM[i][0])
		{
			//allowMotion=0;
			setPoint[i] = OP_LIM[i][0];
			current_state.alarm |= lar_comau::MachineAlarmStatus::ALARM_SETPOINT_LIMIT;

			ROS_INFO("ALERT: SETPOINT AXIS %d UNDER LOWER OP.LIMIT", i + 1);
		} // END if (setPoint[i]<OP_LIM[i][0])
		else {
			current_state.alarm &= ~lar_comau::MachineAlarmStatus::ALARM_SETPOINT_LIMIT;

		}

		if (setPoint[i] > OP_LIM[i][1])
		{
			//allowMotion=0;
			setPoint[i] = OP_LIM[i][1];
			current_state.alarm |= lar_comau::MachineAlarmStatus::ALARM_SETPOINT_LIMIT;

			ROS_INFO("ALERT: SETPOINT AXIS %d OVER UPPER OP.LIMIT", i + 1);
		} // END if (setPoint[i]>OP_LIM[i][1])
		else {
			current_state.alarm &= ~lar_comau::MachineAlarmStatus::ALARM_SETPOINT_LIMIT;

		}


		if (abs(setPoint[i] - getGears[i]) > DPOS_LIM[i])
		{
			//allowMotion=0;
			//setPoint[i] = getGears[i];
			current_state.alarm |= lar_comau::MachineAlarmStatus::ALARM_DISTANCE_LIMIT;

			ROS_INFO("ALERT: SETPOINT DISTENCE ON AXIS %d OVER OP.LIMIT", i + 1);
		} // END if (setPoint[i]<OP_LIM[i][0])
		else {
			current_state.alarm &= ~lar_comau::MachineAlarmStatus::ALARM_DISTANCE_LIMIT;

		}
		//double realNextPoint = setPoint[i] ;
		double lastDeg = previousPositions[i];

		//Operative virtual limit control
		bool virtual_limit_control = false;
		if (virtual_limit_control)
		{
			if (setPoint[i] < (VT_LIM[i][0]))
			{
				setPoint[i] = VT_LIM[i][0];
				cout << " \n Reset Forzato- " << i + 1 << "\n";
			}

			if (setPoint[i] <= (VT_LIM[i][0] + 1) && lastDeg <= (VT_LIM[i][0] + 1) && lastDeg > setPoint[i])
			{
				double proximity = abs(VT_LIM[i][0] - lastDeg);
				if (proximity <= 0)
					proximity = 0;
				if (proximity >= 1)
					proximity = 1;
				double corrective = pow((proximity / 1), 2);
				if (corrective > 1)
					corrective = 1;
				setPoint[i] = lastDeg - corrective * abs(setPoint[i] - lastDeg);
			} // END if (setPoint[i]<=(VT_LIM[i][0]+1) ...

			if (setPoint[i] > (VT_LIM[i][1]))
			{
				setPoint[i] = VT_LIM[i][1];
				cout << " \n Reset Forzato+ " << i + 1 << "\n";
			} // END if (setPoint[i]>(VT_LIM[i][1]))

			//if (i==4) cout << " Proximity+ "<<i+1 <<": "<<(VT_LIM[i][1]-lastDeg)<<"\n";

			if (setPoint[i] >= (VT_LIM[i][1] - 1) && lastDeg >= (VT_LIM[i][1] - 1) && lastDeg < setPoint[i])
			{
				double proximity = abs(VT_LIM[i][1] - lastDeg);
				if (proximity <= 0)
					proximity = 0;
				if (proximity >= 1)
					proximity = 1;
				double corrective = pow((proximity / 1), 2);
				if (corrective > 1)
					corrective = 1;
				setPoint[i] = lastDeg + corrective * abs(setPoint[i] - lastDeg);
			} // END if (setPoint[i]>=(VT_LIM[i][1]-1) ...
		}
		//////////
		//cout << "\n allowMotion = " << allowMotion << " \n";
		//for (int h = 0; h < 6; h++) {
		//	cout << "\nsetPoint[" << h << "] = " << setPoint[h] << "\n";
		//}
	}
}

ros::Publisher joint_state_pub;
ros::Publisher comau_state_pub;
string robot_name = "comau_smart_six";

void *assignSetPoint(void *)
{

	int arg_c;
	char **arg_v;

	ros::init(arg_c, arg_v, "ComauJointStatePublisher");

	ros::NodeHandle n;

	ros::Rate loop_rate(500);

	string parameter;
	if (ros::param::get("COMAU_NAME", parameter))
		robot_name = parameter;

	string joint_command_topic = "/" + robot_name + "/" + ROBOT_CONTROLLER_SUBSCRIBER_NAME;
	string joint_state_topic = "/" + robot_name + "/" + ROBOT_CONTROLLER_PUBLISHER_NAME;

	ros::Subscriber sub = n.subscribe(joint_command_topic, 1, jCallback);
	joint_state_pub = n.advertise<sensor_msgs::JointState>(joint_state_topic, 1);

	// alarm publisher
	string comau_state_topic = "/" + robot_name + "/alarm";
	comau_state_pub = n.advertise<lar_comau::MachineAlarmStatus>(comau_state_topic, 1);

	/** ROS JOINT MESSAGE */
	sensor_msgs::JointState actual_joint_state;
	actual_joint_state.name.resize(6);
	actual_joint_state.position.resize(6);
	actual_joint_state.velocity.resize(6);
	actual_joint_state.name[0] = robot_name + "/" + "base_to_link1";
	actual_joint_state.name[1] = robot_name + "/" + "link1_to_link2";
	actual_joint_state.name[2] = robot_name + "/" + "link2_to_link3";
	actual_joint_state.name[3] = robot_name + "/" + "link3_to_link4";
	actual_joint_state.name[4] = robot_name + "/" + "link4_to_link5";
	actual_joint_state.name[5] = robot_name + "/" + "link5_to_link6";

	current_state.state = lar_comau::MachineAlarmStatus::STATE_INIT;
	ROS_INFO("comau_driver: robot not initialized");

	current_state.alarm = lar_comau::MachineAlarmStatus::ALARM_NONE;
	current_state.reset_required = false;

	while (ros::ok())
	{

		actual_joint_state.header.stamp = ros::Time::now();

		for (int i = 0; i < 6; i++)
		{
			actual_joint_state.position[i] = getGears[i] * M_PI / 180.0;
			actual_joint_state.velocity[i] = Speed[i];
		}

		switch(current_state.state){
			case lar_comau::MachineAlarmStatus::STATE_INIT:
				current_state.state = lar_comau::MachineAlarmStatus::STATE_RUN;
				for (int i = 0; i < 6; i++)	{
					if (init[i] == true) current_state.state = lar_comau::MachineAlarmStatus::STATE_INIT;
				}
				if (current_state.state == lar_comau::MachineAlarmStatus::STATE_RUN){
					ROS_INFO("comau_driver: robot initialization completed");
					for (int i = 0; i < 6; i++)	{
						ROS_INFO("-> c4g: AXIS %d CONNECTED", i + 1);
						ROS_INFO("   Initial Position %d: %f", i + 1, initialPositions[i]);
						ROS_INFO("   Calibration %d: %f", i + 1, calibCONSTANTS[i]);						
					}	
				} 
				break;

			case lar_comau::MachineAlarmStatus::STATE_RUN:
				if (wasGoing_old && !wasGoing)	{
					current_state.alarm |= lar_comau::MachineAlarmStatus::ALARM_DRIVEOFF;
					current_state.reset_required = true;
					current_state.state = lar_comau::MachineAlarmStatus::STATE_ALARM;
					ROS_INFO("comau_driver: ALARM_DRIVEOFF received");
					ROS_INFO("comau_driver: switch to STATE_ALARM");
				}	
				break;


			case lar_comau::MachineAlarmStatus::STATE_ALARM:

				if (!wasGoing_old && wasGoing){
					current_state.state = lar_comau::MachineAlarmStatus::STATE_RUN;
					current_state.alarm &= ~lar_comau::MachineAlarmStatus::ALARM_DRIVEOFF;

					ROS_INFO("comau_driver: ALARM_DRIVEON received");
					ROS_INFO("comau_driver: switch to STATE_RUN");
				}	
				break;
			default:
				break;
		}

		joint_state_pub.publish(actual_joint_state);

		comau_state_pub.publish(current_state);

		wasGoing_old = wasGoing;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return NULL;
}

int main(int argc, char *argv[])
{
	if (getuid() != 0)
	{
		cerr << "\n"
			 << disclaimer;
		cerr << "\nYou must be root (or use sudo) to run server!\n\n";
		exit(1);
	}

	double txRate[6];
	double sampleTime;

	pthread_t tid_receive;
	pthread_create(&tid_receive, NULL, assignSetPoint, 0);

	for (int i = 0; i < 6; i++)
		AXIS_ENABLED[i] = true;

	//CONFIGURAZIONE INIZIALE:
	setPoint[0] = 0.0;
	setPoint[1] = 0.0;
	setPoint[2] = -90.0;
	setPoint[3] = 0.0;
	setPoint[4] = 0.0;
	setPoint[5] = 0.0;

	for (int i = 0; i < 6; i++)
	{
		sinAxisEnabled[i] = false;
	}
	for (int i = 0; i < 6; i++)
	{
		if (AXIS_ENABLED[i])
		{
			sinAxisEnabled[i] = true;
			cout << " ..Asse " << i + 1 << " in modalità OPEN.\n";
		};
	}

	C4gOpen c4gOpen;

	cout << "\nTest started...\n";
	cout.flush();
	cout << "\n    After C4G starts:";
	cout.flush();
	cout << "\n       1. Drive On the robot";
	cout.flush();
	cout << "\n       2. Set MODE 5 via PDL2\n";
	cout.flush();
	cout << "\n    To stop the test:";
	cout.flush();
	cout << "\n       1. Drive Off the robot";
	cout.flush();
	cout << "\n       2. Set mode 504 via PDL2\n\n";
	cout.flush();

	if (c4gOpen.start())
	{
		sampleTime = (double)c4gOpen.getSampleTime() / 1000.0;

		double gearDeg[6];
		double GearRotations[6] = {0, 0, 0, 0, 0, 0};
		allowMotion = 0;

		//-- IIR FILTER LOW-PASS --
		//-- FILTERING setPoints[] --
		//buffer Z
		Fc = 0.5;
		Fs = 1 / sampleTime;
		Q = 0.5;
		updateFilter(Fc);
		//-------------------------

		for (int i = 0; i < 6; i++)
		{

			if (sinAxisEnabled[i])
			{

				txRate[i] = c4gOpen.getTxRate(ARM, i + 1);
			} // END if (sinAxisEnabled[i])
		}	 // END for (int i = 0; i < 6; i++)

		bool keepGoingOn = true;


		int val;

		while (keepGoingOn)
		{
			if (c4gOpen.receive())
			{

				long mode = c4gOpen.getMode(ARM);

				if (mode == C4G_OPEN_EXIT)
					keepGoingOn = false;
				else
				{
					if (mode == C4G_OPEN_DRIVING_ON || mode == C4G_OPEN_MODE_0 || mode == C4G_OPEN_MODE_5)
					{

						if (mode == C4G_OPEN_MODE_5)
						{
							for (int i = 0; i < 6; i++)
							{
								if (sinAxisEnabled[i])
								{

									/** ABSOLUTE POSITION CONTROL **/
									//double currentGearDeg[i] = c4gOpen.getActualPosition(ARM, i+1)*360.0/txRate[i];

									if (init[i])
									{
										//Get actual config at init
										calibCONSTANTS[i] = c4gOpen.getCalibrationConstant(ARM, i + 1) * 360 / txRate[i];
										initialPositions[i] = c4gOpen.getActualPosition(ARM, i + 1) * 360.0 / txRate[i] - calibCONSTANTS[i];
										setPoint[i] = initialPositions[i];
										previousPositions[i] = initialPositions[i];
										deltaPositions[i] = 0.0;
										init[i] = false;
										//cout << "-> c4g: ASSE " << i + 1 << " CONNESSO.\n";
									
										Z[i][3] = initialPositions[i];
										Z[i][2] = initialPositions[i];
										Z[i][1] = initialPositions[i];
										Z[i][0] = initialPositions[i];

										//setPoint[i]=((initialPositions[i])*360/txRate[i]);;
										/*cout << "   Posizione iniziale " << i + 1 << ": " << initialPositions[i] << "\n";
										cout << "         Calibrazione " << i + 1 << ": " << calibCONSTANTS[i] << "\n";*/
									} // END if (init[i])
									else
										allowMotion = 1;

									getGears[i] =  c4gOpen.getActualPosition(ARM, i + 1) * 360.0 / txRate[i] - calibCONSTANTS[i] ;									
									Speed[i] = double(txRate[i]) * double(c4gOpen.getActualVelocity(ARM, i + 1)); ///360.0;

									if (allowMotion)
									{
										//FILTER 2nd order
										X = setPoint[i];
										Acc = X * B0 + Z[i][0] * B1 + Z[i][1] * B2 - Z[i][2] * A1 - Z[i][3] * A2;
										Z[i][3] = Z[i][2];
										Z[i][2] = Acc;
										Z[i][1] = Z[i][0];
										Z[i][0] = X;
										gearDeg[i] = Acc;
										//gearDeg[i] = setPoint[i];

										//Operative limit control after filtering
										if (gearDeg[i] < OP_LIM[i][0])
										{
											//allowMotion=0;
											gearDeg[i] = OP_LIM[i][0];

											cout << " \n ALERT: FILTERED GEAR AXIS " << i + 1 << ": OVER LOW OP.LIMIT.\n";
										} // END if (gearDeg[i]<OP_LIM[i][0])

										if (gearDeg[i] > OP_LIM[i][1])
										{
											//allowMotion=0;
											gearDeg[i] = OP_LIM[i][1];
											cout << " \n ALERT: FILTERED GEAR AXIS " << i + 1 << ": OVER SUP OP.LIMIT.\n";
										} // END if (gearDeg[i]>OP_LIM[i][1])

										GearRotations[i] = gearDeg[i];
									} // END if (allowMotion)
								}	 // END if (sinAxisEnabled[i])
							}		  // END for (int i = 0; i < 6; i++)

							//SENDING
							//cout << "s";
							//to_simulinkLength = sizeof(to_simulink);

							double arrayToSimulink[13];
							double IS_NOT_ANY_AXIS_MOVING = 1.0;

							for (int i = 0; i < 6; i++)
							{
								if (sinAxisEnabled[i])
								{
									arrayToSimulink[i] = getGears[i];
									arrayToSimulink[i + 6] = Speed[i];
									if ((Speed[i]) >= 0.00005 || (Speed[i]) <= -0.00005)
										IS_NOT_ANY_AXIS_MOVING = 0.0;
								} // END if (sinAxisEnabled[i])
							}	 // END for (int i = 0; i < 6; i++)

							//arrayToSimulink[12] = IS_NOT_ANY_AXIS_MOVING;

							//size=sendto(udpSocketSend, arrayToSimulink, 13*8, 0,(struct sockaddr *)&to_simulink, to_simulinkLength);
							/*if (size == -1) {
									perror("sendto()");
									exit(1);
								}*/ // END if (size == -1)

							/*** ROS was here ***/

							//cout << "Sample Time: "<<sampleTime<<endl;
							for (int i = 0; i < 6; i++)
							{
								if (sinAxisEnabled[i])
								{
									if (c4gOpen.isInDriveOn(ARM))
									{
										wasGoing = true;
										actualPositions[i] = GearRotations[i];

										if (allowMotion)
										{
											deltaPositions[i] = actualPositions[i] - previousPositions[i];
											previousPositions[i] = actualPositions[i];
										} // END if (allowMotion)
										else
										{ //stop
											actualPositions[i] = initialPositions[i];
											deltaPositions[i] = 0.0;
										} // END else
									}	 // END if (c4gOpen.isInDriveOn(ARM))
									else if (wasGoing)
									{
										cout << "Alarm --> DRIVE OFF.\n\n Error:" << c4gOpen.getLastError() << "\n Mode: " << c4gOpen.getMode(ARM) << endl;
										cout.flush();

										wasGoing = false;
									}
									c4gOpen.setTargetPosition(ARM, i + 1, (actualPositions[i] + calibCONSTANTS[i]) * txRate[i] / 360.0);
									c4gOpen.setTargetVelocity(ARM, i + 1, deltaPositions[i] * txRate[i] / 360.0);
									//cout << i << " pos: "<<actualPositions[i]<< " set;" << setPoint[i]*txRate[i]/360.0f+calibCONSTANTS[i] <<" tx: "<<txRate[i]<< " vel: "<<deltaPositions[i]<<std::endl;

								} // END if (sinAxisEnabled[i])
							}	 // END or (int i = 0; i < 6; i++)
						}		  // END if (mode == C4G_OPEN_MODE_5)

						if (!c4gOpen.send())
							keepGoingOn = false;

						if (c4gOpen.errorOccurred())
						{
							cout << "Test error occurred: " << c4gOpen.getLastError() << endl;
							c4gOpen.resetError();
						} // END if (c4gOpen.errorOccurred())
					}	 // END if (mode == C4G_OPEN_DRIVING_ON || mode == C4G_OPEN_MODE_0 || mode == C4G_OPEN_MODE_5)
					else
					{
						cout << "Test error --> DRIVE OFF.\n\n";
						cout.flush();
						c4gOpen.setMode(ARM, C4G_OPEN_DRIVE_OFF);
						c4gOpen.send();
						keepGoingOn = false;
					} // END else
				}	 // END else --- principale
			}		  // END if (c4gOpen.receive())
			else
			{
				cout << "Test error 2.\n\n";
				cout.flush();
				keepGoingOn = false;
			} // END else
		}	 // END while (keepGoingOn)
	}		  // END if (c4gOpen.start())

	c4gOpen.stop();

	//cout << "Test5: termine del server..\n\n"; cout.flush();

	pthread_kill(tid_receive, SIGKILL);
	//pthread_join(tid,NULL);

	cout << "Node terminated.\n\n";
	cout.flush();

	return 0;
}
