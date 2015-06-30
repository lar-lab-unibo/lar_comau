/*
	Mode4_SERVER.cpp

	Developer: Davide Calzolari
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

#define PI 		3.14159265358979
#define	ARM		1

double mesg[7];
double response[6];
bool allowMotion=0;
bool startServer;
bool serverActive;
bool IS_AXIS_SELECTED[6];
bool AXIS_ENABLED[6];
float setPoint[6] = {0,0,0,0,0,0};
int DRAW=3;

using namespace std;

const char disclaimer[] = "";

						
class Server {
    public:
        static void* Run(void* me) {
            static_cast<Server*>(me)->Start();
            return NULL;
        }

    private:
        void Start() {
	  
            //printf("%p: %p is running.\n", pthread_self(), this);
	    
	int                 udpSocket;
	int                 port;
	int                 status;
	int                 size;
	socklen_t           clientLength;
	struct sockaddr_in  serverName;
	struct sockaddr_in  clientName;

	port = 3456;

	udpSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udpSocket<0) {
		perror("socket()");
		exit(1);
	}

	memset(&serverName, 0, sizeof(serverName));
	serverName.sin_family = AF_INET;
	serverName.sin_addr.s_addr = htonl(INADDR_ANY);
	serverName.sin_port = htons(port);

	status = bind(udpSocket, (struct sockaddr *)&serverName,
		      sizeof(serverName));
	if (status<0) {
		perror("bind()");
		exit(1);
	}

	while (serverActive) {
		clientLength = sizeof(clientName);
		size = recvfrom(udpSocket, mesg, 56, 0,
				(struct sockaddr *)&clientName, &clientLength);
		if (size == -1) {
			perror("recvfrom()");
			exit(1);
		}
		//printf("RCV: %d from %s\n", (int)mesg[0],
		//       inet_ntoa(clientName.sin_addr));

		double arg[6]={0,0,0,0,0,0};
		int command;

		command=(int)mesg[0];

 		for (int i = 0; i < 6; i++)
			arg[i] = mesg[i+1];
		
		//UPDATE
							  
		if (command==3) 	{allowMotion  = 0; 
							  cout << "Ricevuto segnale di STOP. \n";}
		
		else if (command==1) 	{//startServer  = 1; 
							  
							  cout << "Ricevuto segnale di START. \n";}
							  
		else if (command==6) 	{

							  
							    for (int i = 0; i < 6; i++) IS_AXIS_SELECTED[i] = false;
							    for (int i = 0; i < 6; i++) {
								//all axes selected
								if (AXIS_ENABLED[i]==1) IS_AXIS_SELECTED[i] = true;
								//degrees amount
							    	setPoint[i] = (float)(arg[i]);
								//cout << "asse "<<i+1<< ": "<< setPoint[i] << "\n";
								//cout << "\n";
								response[i] = setPoint[i];
								}
							    
							  allowMotion  = 1;
							  
							
							  //cout << "Ricevuta CONFIGURAZIONE 6 assi " << "\n";
							  }

							  
							  
		
		memset(&mesg, 0, 56);
		
		sendto(udpSocket, response, 48, 0,(struct sockaddr *)&clientName, clientLength);
		memset(&response, 0, 48);
	}
	}
};

int main(int argc, char *argv[])
{
	if (getuid() != 0)
	{
		cerr << "\n" << disclaimer;
		cerr << "\nYou must be root (or use sudo) to run TestMode4!\n\n";
		exit(1);
	}

	double txRate[6];
	bool sinAxisEnabled[6];
	double sampleTime;

	cout << "\n\n SERVER loading...\n";cout.flush();
	
	serverActive=1;
	pthread_t tid;
        Server *test = (Server*)malloc(sizeof(Server));
	
	pthread_create(&tid, NULL, Server::Run, &test);
	
	cout << " SERVER READY \n";cout.flush();
	
	allowMotion=0;
	//while (!startServer) {cout.flush();};
	for (int i = 0; i < 6; i++) AXIS_ENABLED[i]=true;

	//CONFIGURAZIONE INIZIALE:
	setPoint[0] = 0.0;
	setPoint[1] = 0.0;
	setPoint[2] = 0.0;
	setPoint[3] = 0.0;
	setPoint[4] = 0.0;
	setPoint[5] = 0.0;

	for (int i = 0; i < 6; i++) {sinAxisEnabled[i] = false;}
	for (int i = 0; i < 6; i++) {
		if (AXIS_ENABLED[i]) {sinAxisEnabled[i]=true; cout << " ..Asse " << i+1 << " in modalità OPEN.\n";}; 
	}

	
	C4gOpen c4gOpen;

	cout << "\nTestMode4 started...\n"; cout.flush();
	cout << "\n    After C4G starts:"; cout.flush();
	cout << "\n       1. Drive On the robot"; cout.flush();
	cout << "\n       2. Set mode 5 via PDL2\n"; cout.flush();
	cout << "\n    To stop the test:"; cout.flush();
	cout << "\n       1. Drive Off the robot"; cout.flush();
	cout << "\n       2. Set mode 504 via PDL2\n\n"; cout.flush();

	if (c4gOpen.start())
	{
		double initialPositions[6];
		double calibCONSTANTS[6];
		double actualPositions[6];
		double previousPositions[6];
		double deltaPositions[6];
		float OP_LIM[6][2] = {{-170,170},{-85,175},{-170,0},{-210,210},{-130,130},{-2700,2700}};

		sampleTime = (double)c4gOpen.getSampleTime() / 1000.0;

		double gearDeg[6];
		double GearRotations[6];
		
		//-- IIR FILTER LOW-PASS --
		//-- FILTERING setPoints[] --
		//buffer Z
		double Z[6][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
		double Acc=0; //output
		double X=0; //input
		float Fc,Fs,Q,W,N,B0,B1,B2,A1,A2;
		Fc = 0.3;
		Fs = 1/sampleTime;
		Q = 0.5;
		W = tan(PI*Fc/Fs);
		N = 1/(pow(W,2)+W/Q+1);
		B0 = N*pow(W,2);
		B1 = 2*B0;
		B2 = B0;
		A1 = 2*N*(pow(W,2)-1);
		A2 = N*(pow(W,2)-W/Q+1);
		//-------------------------

		for (int i = 0; i < 6; i++)	
		{
			if (sinAxisEnabled[i])
			{
				txRate[i] = c4gOpen.getTxRate(ARM, i+1);
			}
		}

		bool keepGoingOn = true;
		bool init[6] = {1,1,1,1,1,1};					

		while (keepGoingOn)
		{
			if (c4gOpen.receive())
			{
				long mode = c4gOpen.getMode(ARM);

				if (mode == C4G_OPEN_EXIT) keepGoingOn = false;
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

									//ABSOLUTE POSITION CONTROL

									if (init[i]) {
										//Get actual config at init
										initialPositions[i] = c4gOpen.getActualPosition(ARM, i+1);
										previousPositions[i] = initialPositions[i];
										deltaPositions[i] = 0.0;
										init[i]=false;
										cout << "-> c4g: ASSE "<<i+1<<" CONNESSO.\n";
								                calibCONSTANTS[i] = c4gOpen.getCalibrationConstant(ARM, i+1);
																			 											Z[i][3]=(initialPositions[i]*360.0/txRate[i]);
									 	Z[i][2]=(initialPositions[i]*360.0/txRate[i]);
									 	Z[i][1]=(initialPositions[i]*360.0/txRate[i]);
									 	Z[i][0]=(initialPositions[i]*360.0/txRate[i]);

										//setPoint[i]=((initialPositions[i])*360/txRate[i]);;
							cout << "   Posizione iniziale "<<i+1<<": "<< (initialPositions[i]*360/txRate[i])<<"\n";
							cout << "         Calibrazione "<<i+1<<": "<< (calibCONSTANTS[i]*360/txRate[i])<<"\n";
									}
									
 									//Operative limit control
									if (setPoint[i]<OP_LIM[i][0]) {
										//allowMotion=0;
										setPoint[i]=OP_LIM[i][0];
										cout << " \n ALERT: SETPOINT AXIS "<<i+1<<": OVER LOW OP.LIMIT.\n";
									}
									if (setPoint[i]>OP_LIM[i][1]) {
										//allowMotion=0;
										setPoint[i]=OP_LIM[i][1];
										cout << " \n ALERT: SETPOINT AXIS "<<i+1<<": OVER SUP OP.LIMIT.\n";
									}
									//////////

									 //FILTER 2nd
									 X = setPoint[i] + ((calibCONSTANTS[i])*360.0/txRate[i]);
									 Acc = X*B0+Z[i][0]*B1+Z[i][1]*B2-Z[i][2]*A1-Z[i][3]*A2;
									 Z[i][3]=Z[i][2];
									 Z[i][2]=Acc;
									 Z[i][1]=Z[i][0];
									 Z[i][0]=X;
									 gearDeg[i] = Acc;

									 //Operative limit control after filtering
									if (gearDeg[i]<OP_LIM[i][0]) {
										//allowMotion=0;
									 gearDeg[i]=OP_LIM[i][0];
									 cout << " \n ALERT: FILTERED GEAR AXIS "<<i+1<<": OVER LOW OP.LIMIT.\n";
									}
									if (gearDeg[i]>OP_LIM[i][1]) {
										//allowMotion=0;
									 gearDeg[i]=OP_LIM[i][1];
									 cout << " \n ALERT: FILTERED GEAR AXIS "<<i+1<<": OVER SUP OP.LIMIT.\n";
									}

									GearRotations[i] = +txRate[i]*gearDeg[i] / 360.0;
									
									}
								}
							allowMotion=1;
							for (int i = 0; i < 6; i++)
							{
								if (sinAxisEnabled[i])
								{
									if (c4gOpen.isInDriveOn(ARM))
									{
									
									actualPositions[i] =  GearRotations[i];
										
									if (allowMotion) {
									  
										 deltaPositions[i] = actualPositions[i] - previousPositions[i];
										 previousPositions[i] = actualPositions[i];
										 //allowMotion=0;
									  }

									else
									  { //stop
										 actualPositions[i] = initialPositions[i];
										 deltaPositions[i] = 0.0;
									  }
									    
									}
									
		
									c4gOpen.setTargetPosition(ARM, i+1, actualPositions[i]);
									c4gOpen.setTargetVelocity(ARM, i+1, deltaPositions[i]);
								}
							}
						}
						
						if (!c4gOpen.send()) keepGoingOn = false;
						
						if (c4gOpen.errorOccurred())
						{
						  	cout << "TestMode4 error occurred: " << c4gOpen.getLastError() << endl;
							c4gOpen.resetError();
						}
					}
					else
					{
						cout << "TestMode4 error --> DRIVE OFF.\n\n";cout.flush();
						c4gOpen.setMode(ARM, C4G_OPEN_DRIVE_OFF);
						c4gOpen.send();
						keepGoingOn = false;
					}
				}
			}
			else {
			  cout << "TestMode4 error 2.\n\n";cout.flush();
			  keepGoingOn = false;
			}
		}
	}
	
	c4gOpen.stop();
	
	//cout << "TestMode5: cg4open ricevuto comando 405\n"; cout.flush();
	cout << "TestMode4: termine del server..\n\n"; cout.flush();
	
	pthread_kill(tid,SIGKILL);
	//pthread_join(tid,NULL);
	
	cout << "TestMode4 terminated.\n\n"; cout.flush();
	
	return 0;
}

