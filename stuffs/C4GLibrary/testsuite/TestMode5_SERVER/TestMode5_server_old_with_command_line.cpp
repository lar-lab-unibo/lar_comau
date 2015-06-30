/*
	TestMode5.cpp

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

#define MAX_MESG_SIZE 1500

#include <iostream>
#include <math.h>
#include <pthread.h>

#include <C4gOpen.hpp>

#define PI 		3.14159265358979
#define	ARM		1

char mesg[MAX_MESG_SIZE];
bool allowMotion=0;
bool startServer;
bool serverActive;
bool IS_AXIS_SELECTED[6];
double arg_frequencyHz;
double arg_amplitudeDeg;
double amplitudeDeg[6] = {0,0,0,0,0,0};
char arg_axis[10];

using namespace std;

const char disclaimer[] = "\n*********************************************************************\n"
						  "                              TestMode5                                \n\n"
						  "   C4G Open Library " VERSION_NUMBER " - Copyright (C) 2007 Sintesi S.C.p.A.\n\n"
						  "  Developers:                                                          \n"
						  "           Sabino   COLONNA (2006-, s.colonna@sintesi-scpa.com)    \n"
						  "           Giovanni IACCA   (2006-, g.iacca@sintesi-scpa.com  )    \n"
						  "           Giovanni TOTARO  (2006-, g.totaro@sintesi-scpa.com )    \n"
						  "\n*********************************************************************\n";

						
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
		size = recvfrom(udpSocket, mesg, MAX_MESG_SIZE, 0,
				(struct sockaddr *)&clientName, &clientLength);
		if (size == -1) {
			perror("recvfrom()");
			exit(1);
		}
		printf("RCV: %s from %s\n", mesg,
		       inet_ntoa(clientName.sin_addr));

		//Elaborazione Comando
		std::stringstream line (mesg);
		std::string command;
		std::string segment;
		
		std::vector<std::string> seglist;
		bool com1=0;
		double arg[6]={0,0,0,0,0,0};
		bool input[6]={0,0,0,0,0,0};
		int ia=0;
		
		cout << "	COMMAND: ";
		
		while (std::getline(line, segment, '_'))
		{
		  seglist.push_back(segment);
		  if (!com1) {com1=1;  command=segment;}
		  else
		  {
		    arg[ia]=atof(segment.c_str());
		    input[ia]=true;
		    ia++;
		  }
		  cout << segment << " ";
		}
		cout << "\n";
		
		//UPDATE
		if (command.compare("move")==0) 	{
							  
							  if (input[0] && input[1]) {
							    //axis
							    int axisline = (int)floor(arg[1]);
							    char selected_axis[10];
							    sprintf(selected_axis,"%d", axisline);
							  
							    for (int i = 0; i < 6; i++) IS_AXIS_SELECTED[i] = false;
							    for (unsigned int i = 0; i < strlen(selected_axis); i++)
							    {
							      if (selected_axis[i] >= '1' && selected_axis[i] <= '6') {
								cout << " - Asse "<< ((selected_axis[i]-'0')) << "-> selezionato.\n";
								IS_AXIS_SELECTED[(selected_axis[i]-'0')-1] = true;
								//degrees amount
							    	amplitudeDeg[(selected_axis[i]-'0')-1] = (double)(arg[0])/2.0;
							      }
							    }
								allowMotion  = 1;
							  }
							  cout << "Ricevuto MOVE " << "\n";}
							  
		else if (command.compare("stop")==0) 	{allowMotion  = 0; 
							  cout << "Ricevuto segnale di STOP. \n";}
		
		else if (command.compare("start")==0) 	{startServer  = 1; 
							  arg_frequencyHz  = arg[0];
							  arg_amplitudeDeg = arg[1];
							  int axisline = (int)floor(arg[2]);
							  sprintf(arg_axis,"%d", axisline);
							  
							  cout << "Ricevuto segnale di START. \n";}
							  
		else if (command.compare("kill")==0) 	{serverActive = 0; 
							  cout << "SERVER TERMINATO. \n";}
							  
		else if (command.compare("select")==0) 	{ int axisline = (int)floor(arg[0]);
							  char selected_axis[10];
							  sprintf(selected_axis,"%d", axisline);
							  
							  for (int i = 0; i < 6; i++) IS_AXIS_SELECTED[i] = false;
							  for (unsigned int i = 0; i < strlen(selected_axis); i++)
							  {
							    if (selected_axis[i] >= '1' && selected_axis[i] <= '6') {
							      cout << " - Asse "<< ((selected_axis[i]-'0')) << "-> selezionato.\n";
							      IS_AXIS_SELECTED[(selected_axis[i]-'0')-1] = true;
							     }
							  }
							}

		else if (command.compare("config")==0) 	{
							  if (input[0] && input[1] && input[2] && input[3] && input[4] && input[5]) {
							  
							    for (int i = 0; i < 6; i++) IS_AXIS_SELECTED[i] = false;
							    for (int i = 0; i < 6; i++) {
								//all axes selected
								if (arg[i]!=0) IS_AXIS_SELECTED[i] = true;
								//degrees amount
							    	amplitudeDeg[i] = (double)(arg[i])/2.0;
								}
							    
							  allowMotion  = 1;
							  cout << "Ricevuta CONFIGURAZIONE 6 assi " << "\n";
							  } else { cout << "--> BAD INPUT <--" << "\n";}

							  
							 }
							  
		
		memset(&mesg, 0, MAX_MESG_SIZE);
		//sendto(udpSocket, mesg, size, 0,(struct sockaddr *)&clientName, clientLength);
	}
	}
};

int main(int argc, char *argv[])
{
	if (getuid() != 0)
	{
		cerr << "\n" << disclaimer;
		cerr << "\nYou must be root (or use sudo) to run TestMode5!\n\n";
		exit(1);
	}

	double txRate[6];

	bool sinAxisEnabled[6];
	double frequencyHz;
	//double amplitudeDeg;
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

	frequencyHz = arg_frequencyHz;

	for (int i = 0; i < 6; i++) {sinAxisEnabled[i] = false; amplitudeDeg[i] = arg_amplitudeDeg;}
	for (unsigned int i = 0; i < strlen(arg_axis); i++)
	{
		if (arg_axis[i] >= '1' && arg_axis[i] <= '6') {
		  cout << " ..Asse "<< ((arg_axis[i]-'0')) << " in modalità OPEN.\n";
		  sinAxisEnabled[(arg_axis[i]-'0')-1] = true;
		  //per default
		  IS_AXIS_SELECTED[(arg_axis[i]-'0')-1] = true;
		}
	}
	
	cout << "	->parametri modificati: freq:" << arg_frequencyHz << " amp:" << arg_amplitudeDeg << " assi:" << arg_axis; cout.flush();
	
	C4gOpen c4gOpen;

	cout << "\nTestMode5 started...\n"; cout.flush();
	cout << "\n    After C4G starts:"; cout.flush();
	cout << "\n       1. Drive On the robot"; cout.flush();
	cout << "\n       2. Set mode 5 via PDL2\n"; cout.flush();
	cout << "\n    To stop the test:"; cout.flush();
	cout << "\n       1. Drive Off the robot"; cout.flush();
	cout << "\n       2. Set mode 504 via PDL2\n\n"; cout.flush();

	if (c4gOpen.start())
	{
		float initialPositions[6];
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
				amplitudeGearRotations[i] = txRate[i] * amplitudeDeg[i] / 360.0;
			}
		}

		bool keepGoingOn = true;
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
										initialPositions[i] = c4gOpen.getActualPosition(ARM, i+1);
										previousPositions[i] = initialPositions[i];
										deltaPositions[i] = 0.0;
										txRate[i] = c4gOpen.getTxRate(ARM, i+1);
										amplitudeGearRotations[i] = txRate[i] * amplitudeDeg[i] / 360.0;
									}
									
								}
							}

							for (int i = 0; i < 6; i++)
							{
								if (sinAxisEnabled[i])
								{
									if (c4gOpen.isInDriveOn(ARM))
									{
									double omegat = 2 * PI * frequencyHz * ((double)counter * sampleTime);
									actualPositions[i] = initialPositions[i] + 
											(float)(amplitudeGearRotations[i] * cos(omegat) - amplitudeGearRotations[i]);
										
									if (allowMotion && IS_AXIS_SELECTED[i]) {
									  
									  if (counter<=(0.5*500/frequencyHz)) {
										 deltaPositions[i] = actualPositions[i] - previousPositions[i];
										 previousPositions[i] = actualPositions[i];
										} else {
										  counter=0;
										  allowMotion=0;
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
							if (c4gOpen.isInDriveOn(ARM)) {counter++; if (!allowMotion) counter--;};
						}
						
						if (!c4gOpen.send()) keepGoingOn = false;
						
						if (c4gOpen.errorOccurred())
						{
						  	cout << "TestMode5 error occurred: " << c4gOpen.getLastError() << endl;
							c4gOpen.resetError();
							counter = 0;
						}
					}
					else
					{
						cout << "TestMode5 error --> DRIVE OFF.\n\n";cout.flush();
						c4gOpen.setMode(ARM, C4G_OPEN_DRIVE_OFF);
						c4gOpen.send();
						keepGoingOn = false;
					}
				}
			}
			else {
			  cout << "TestMode5 error 2.\n\n";cout.flush();
			  keepGoingOn = false;
			}
		}
	}
	
	c4gOpen.stop();
	
	//cout << "TestMode5: cg4open ricevuto comando 405\n"; cout.flush();
	cout << "TestMode5: attesa di termine del server(comando kill)\n\n"; cout.flush();
	
	//serverActive=0;
	pthread_join(tid,NULL);
	
	cout << "TestMode5 terminated.\n\n"; cout.flush();
	
	return 0;
}

