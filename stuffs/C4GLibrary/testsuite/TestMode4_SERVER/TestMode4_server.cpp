/*
	TestMode4_SERVER.cpp

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

#include <C4gOpen.hpp>

#define PI 		3.14159265358979
#define	ARM		1

double mesg[7];
bool allowMotion=0;
bool startServer;
bool serverActive;
bool IS_AXIS_SELECTED[6];
bool AXIS_ENABLED[6];
float setPoint[6] = {0,0,0,0,0,0};
double amplitudeDeg[6] = {0,0,0,0,0,0};
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
		printf("RCV: %d from %s\n", (int)mesg[0],
		       inet_ntoa(clientName.sin_addr));

		double arg[6]={0,0,0,0,0,0};
		//bool input[6]={1,1,1,1,1,1};
		int command;

		command=(int)mesg[0];

 		for (int i = 0; i < 6; i++)
			arg[i] = mesg[i+1];

		cout << "	COMMAND: " << command <<"\n";
		
		//UPDATE
							  
		if (command==3) 	{allowMotion  = 0; 
							  cout << "Ricevuto segnale di STOP. \n";}
		
		else if (command==1) 	{startServer  = 1; 
							  for (int i = 0; i < 6; i++) if ((int)arg[i]==1) AXIS_ENABLED[i]=true;
							  cout << "Ricevuto segnale di START. \n";}
							  
		else if (command==9) 	{serverActive = 0; 
							  cout << "SERVER TERMINATO. \n";}
							  
		else if (command==6) 	{

							  
							    for (int i = 0; i < 6; i++) IS_AXIS_SELECTED[i] = false;
							    for (int i = 0; i < 6; i++) {
								//all axes selected
								if (AXIS_ENABLED[i]==1) IS_AXIS_SELECTED[i] = true;
								//degrees amount
							    	setPoint[i] = (float)(arg[i]);
								}
							    
							  allowMotion  = 1;
							  DRAW=3;
							  cout << "Ricevuta CONFIGURAZIONE 6 assi " << "\n";
							  }

							  
							  
		
		memset(&mesg, 0, 56);
		//sendto(udpSocket, mesg, size, 0,(struct sockaddr *)&clientName, clientLength);
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
	double frequencyHz;
	double sampleTime;
	
	
	//
	cout << "\n\n SERVER loading...\n";cout.flush();
	
	serverActive=1;
	pthread_t tid;
        Server *test = (Server*)malloc(sizeof(Server));
	
	pthread_create(&tid, NULL, Server::Run, &test);
	
	cout << " SERVER READY \n";cout.flush();
	
	/*
	// Parse command line parameters
	frequencyHz = atof(argv[1]);
	amplitudeDeg = atof(argv[2]);
	for (int i = 0; i < 6; i++) sinAxisEnabled[i] = false;
	
	for (unsigned int i = 0; i < strlen(argv[3]); i++)
	{
		if (argv[3][i] >= '1' && argv[3][i] <= '6') sinAxisEnabled[(argv[3][i]-'0')-1] = true;
	}
	*/
	
	//Activate SERVER and parse new command line parameters
	allowMotion=0;
	cout << " Waiting START command with arguments... \n";cout.flush();
	while (!startServer) {cout.flush();};

	frequencyHz = 0.2;

	for (int i = 0; i < 6; i++) {sinAxisEnabled[i] = false; amplitudeDeg[i] = 0; setPoint[i] = 0;}
	for (int i = 0; i < 6; i++) {
		if (AXIS_ENABLED[i]) {sinAxisEnabled[i]=true; cout << " ..Asse " << i+1 << " in modalità OPEN.\n";}; 
	}
	
	cout << "	->parametri modificati: freq:" << "0.2" << " amp:" << "0"; cout.flush();
	
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
		float initialPositions[6];
		float startPositions[6];
		float actualPositions[6];
		float previousPositions[6];
		float deltaPositions[6];

		sampleTime = (double)c4gOpen.getSampleTime() / 1000.0;

		double amplitudeGearRotations[6];
		

		for (int i = 0; i < 6; i++)	
		{
			if (sinAxisEnabled[i])
			{
				txRate[i] = c4gOpen.getTxRate(ARM, i+1);
				//amplitudeGearRotations[i] = txRate[i] * amplitudeDeg[i] / 360.0;
			}
		}

		bool keepGoingOn = true;
		bool init[6] = {1,1,1,1,1,1};
		int counter = 0;					

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
							if (counter == 0)
							{
								for (int i = 0; i < 6; i++)
								{
									if (sinAxisEnabled[i])
									{
										//ABSOLUTE POSITION CONTROL
										startPositions[i] = c4gOpen.getActualPosition(ARM, i+1);

										if (init[i]) {
											//Get actual config as 0 set point
											initialPositions[i] = startPositions[i];
											previousPositions[i] = initialPositions[i];
											deltaPositions[i] = 0.0;
											init[i]=false;
										} else 
										{
										//change rot degrees according to absolute position
										amplitudeDeg[i] = //0.5*setPoint[i];
						0.5*((double)((setPoint[i] + 360*initialPositions[i]/txRate[i] - 360*startPositions[i]/txRate[i])));

										amplitudeGearRotations[i] = -txRate[i]*amplitudeDeg[i] / 360.0;

										previousPositions[i] = startPositions[i];
										deltaPositions[i] = 0.0;
										//Debug
										if (DRAW>0) {
									cout << "Gear "<<(i+1)<<":    init: " << 360*initialPositions[i]/txRate[i] <<"\n"; 
									cout << "     "       <<":    start: " << 360*startPositions[i]/txRate[i] <<"\n"; 
										 cout << "     "       <<": setPoint: " << setPoint[i] <<"\n";
										 cout << "     "       <<":     diff: " << amplitudeDeg[i]*2<<"\n";
										 cout << "     "       <<":      tau: " << txRate[i] <<"\n\n";
										 DRAW--;
											}
										}
									}
									
								}
							}

							//cout << " startPosition 4: "<< (180/PI)*startPositions[3] <<"\n"; 

							for (int i = 0; i < 6; i++)
							{
								if (sinAxisEnabled[i])
								{
									if (c4gOpen.isInDriveOn(ARM))
									{
									double omegat = 2 * PI * frequencyHz * ((double)counter * sampleTime);
									actualPositions[i] = startPositions[i] + 
											(float)(amplitudeGearRotations[i] * cos(omegat) - amplitudeGearRotations[i]);
										
									if (allowMotion && IS_AXIS_SELECTED[i]) {
									  
									  if (counter<=(0.5*500/frequencyHz)) {
										 deltaPositions[i] = actualPositions[i] - previousPositions[i];
										 previousPositions[i] = actualPositions[i];
										} else {
										  counter=0;
										  allowMotion=0;
										  DRAW=3;
										}
									  }
									else
									  { //stop
										 actualPositions[i] = previousPositions[i];
										 deltaPositions[i] = actualPositions[i] - previousPositions[i];
									  }
									    
									}

									c4gOpen.setTargetPosition(ARM, i+1, actualPositions[i]);
									c4gOpen.setTargetVelocity(ARM, i+1, deltaPositions[i]);
								}
							}
							if (c4gOpen.isInDriveOn(ARM)) {if (allowMotion) counter++;};
						}
						
						if (!c4gOpen.send()) keepGoingOn = false;
						
						if (c4gOpen.errorOccurred())
						{
						  	cout << "TestMode4 error occurred: " << c4gOpen.getLastError() << endl;
							c4gOpen.resetError();
							counter = 0;
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
	cout << "TestMode4: attesa di termine del server(comando kill)\n\n"; cout.flush();
	
	//serverActive=0;
	pthread_join(tid,NULL);
	
	cout << "TestMode4 terminated.\n\n"; cout.flush();
	
	return 0;
}

