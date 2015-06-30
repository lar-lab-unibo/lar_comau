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
bool allowMotion=0;
bool startServer;
bool IS_AXIS_SELECTED[6];
bool AXIS_ENABLED[6];
float setPoint[6] = {0,0,0,0,0,0};
double getGears[6] = {1,1,0,0,0,0};
//FILTER VARS
double Z[6][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
double B[6][10];
double Acc=0; //output
double X=0; //input
float Fc = 0.5;
float Fs,Q,W,N,B0,B1,B2,A1,A2;
float FcMax = 0.5;
float FcMin= 0.1;
float threshold[6] = {10,10,10,30,15,30};

using namespace std;

const char disclaimer[] = "";

void updateFilter(float cutoff) {
	W = tan(PI*cutoff/Fs);
	N = 1.0/(pow(W,2)+W/Q+1);
	B0 = N*pow(W,2);
	B1 = 2*B0;
	B2 = B0;
	A1 = 2*N*(pow(W,2)-1);
	A2 = N*(pow(W,2)-W/Q+1);
}


/****** ex-server
double arg[6]={0,0,0,0,0,0};
int command;

command=6;

for (int i = 0; i < 6; i++) arg[i] = {0,0,-90,0,0,0};

//UPDATE
					  
if (command==3){
	if ((float)arg[0]<=10.0 && (float)arg[0]>0.1 && (float)arg[0]>(float)arg[1]) {
		FcMax = arg[0];
		FcMin = arg[1];
	}; //END if ((float)arg[0]<=10.0 && (float)arg[0]>0.1 && (float)arg[0]>(float)arg[1])

	cout << " (IIR) -> Frequenza di taglio MAX: "<<FcMax<<" MIN:"<<FcMin<<"\n";
}//END if (command==3)
					  
else if(command==6){
	//controlla se riferimento è troppo grande.....
	bool farPoint = false;
	float FcCap = FcMax;
	float cap;

	for (int i = 0; i < 6; i++) IS_AXIS_SELECTED[i] = false;
	for (int i = 0; i < 6; i++) {
		//all axes selected
		if (AXIS_ENABLED[i]==1) IS_AXIS_SELECTED[i] = true;
		//setpoint in degrees
		setPoint[i] = (float)(arg[i]);
	    	//check if point is relatively far from last given point
		float err = abs(getGears[i]-setPoint[i]);
	    	if (err >= threshold[i]) {
			farPoint=true;
			float relative = (threshold[i]/err);
			cap = FcMin+(FcMax-FcMin)*relative*relative;
			if (cap < FcCap) FcCap=cap;
		}//END if (err >= threshold[i])
	}//END for (int i = 0; i < 6; i++)
	//cut-off frequency adjustements
	if (FcCap>FcMax) FcCap=FcMax;
	if (FcCap<FcMin) FcCap=FcMin;
	if (farPoint && Fc!=FcCap) {
		Fc = FcCap;
		updateFilter(Fc);
		cout << "Fc = "<<Fc<<"\n";
	}//END if (farPoint && Fc!=FcCap)	
	if (not farPoint && Fc!=FcMax) {
		Fc = FcMax;
		updateFilter(Fc);
		cout << "Fc = " <<Fc<<"\n";
	}  //END if (not farPoint && Fc!=FcMax)

	allowMotion  = 1;

} //END else if (command==6)
//LINUX MAC 00:40:95:30:0C:1E

******/


int main(int argc, char *argv[])
{

	double txRate[6];
	bool sinAxisEnabled[6];
	double sampleTime;
	
	allowMotion=0; /**OCCHIO!**/

	for (int i = 0; i < 6; i++) AXIS_ENABLED[i]=true;

	/**
		**
		acquisire qui!!
		**
	**/

	//CONFIGURAZIONE INIZIALE:
	setPoint[0] = 0.0;
	setPoint[1] = 0.0;
	setPoint[2] = -90.0;
	setPoint[3] = 0.0;
	setPoint[4] = 0.0;
	setPoint[5] = 0.0;

	for (int i = 0; i < 6; i++){
		sinAxisEnabled[i] = false;
	}

	for (int i = 0; i < 6; i++) {
		if (AXIS_ENABLED[i]){
			sinAxisEnabled[i]=true;
			cout << " ..Asse " << i+1 << " in modalità OPEN.\n";
		}
	}
	
	C4gOpen c4gOpen;

	cout << "\nTest started...\n"; cout.flush();
	cout << "\n    After C4G starts:"; cout.flush();
	cout << "\n       1. Drive On the robot"; cout.flush();
	cout << "\n       2. Set MODE 5 via PDL2\n"; cout.flush();
	cout << "\n    To stop the test:"; cout.flush();
	cout << "\n       1. Drive Off the robot"; cout.flush();
	cout << "\n       2. Set mode 504 via PDL2\n\n"; cout.flush();


	//////////////////////////////////c4gOpen START
	if (c4gOpen.start()){

		float initialPositions[6];
		float calibCONSTANTS[6];
		float actualPositions[6];
		float previousPositions[6];
		float deltaPositions[6];
		float OP_LIM[6][2] = {{-170,170},{-85,175},{-170,0},{-210,210},{-130,130},{-2700,2700}};
		float VT_LIM[6][2] = {{-170,170},{-85,175},{-170,0},{-210,210},{-100,100},{-800,800}};

		sampleTime = (double)c4gOpen.getSampleTime() / 1000.0;

		double gearDeg[6];
		double GearRotations[6] = {0,0,0,0,0,0};
		allowMotion=0; /**OCCHIO!**/

		//-- IIR FILTER LOW-PASS --
		//-- FILTERING setPoints[] --
		//buffer Z
		Fc = 0.1;
		Fs = 1/sampleTime;
		Q = 0.5;
		updateFilter(Fc);
		//-------------------------

		for (int i = 0; i < 6; i++){
			if (sinAxisEnabled[i]){
				txRate[i] = c4gOpen.getTxRate(ARM, i+1);
			}
		}

		bool keepGoingOn = true;
		bool init[6] = {1,1,1,1,1,1};					

		while (keepGoingOn){

			if(c4gOpen.receive()){

				long mode = c4gOpen.getMode(ARM);

				if (mode == C4G_OPEN_EXIT) keepGoingOn = false;
				else{
					if (mode == C4G_OPEN_DRIVING_ON || mode == C4G_OPEN_MODE_0 || mode == C4G_OPEN_MODE_5){
						if (mode == C4G_OPEN_MODE_5){
							for (int i = 0; i < 6; i++){
								if (sinAxisEnabled[i]){

									//ABSOLUTE POSITION CONTROL

									if (init[i]){ //***calibrazione***//
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

										cout << "   Posizione iniziale "<<i+1<<": "<< (initialPositions[i]*360/txRate[i])<<"\n";
										cout << "         Calibrazione "<<i+1<<": "<< (calibCONSTANTS[i]*360/txRate[i])<<"\n";
									}// END if (init[i])

									getGears[i]= -calibCONSTANTS[i]*360.0/txRate[i] + c4gOpen.getActualPosition(ARM, i+1)*360.0/txRate[i];
								
 									//Operative limit control
									if (setPoint[i]<OP_LIM[i][0]){
										//allowMotion=0;
										setPoint[i]=OP_LIM[i][0];
										cout << " \n ALERT: SETPOINT AXIS "<<i+1<<": OVER LOW OP.LIMIT.\n";
									}// END if (setPoint[i]<OP_LIM[i][0])
									if (setPoint[i]>OP_LIM[i][1]){
										//allowMotion=0;
										setPoint[i]=OP_LIM[i][1];
										cout << " \n ALERT: SETPOINT AXIS "<<i+1<<": OVER SUP OP.LIMIT.\n";
									}// END if (setPoint[i]>OP_LIM[i][1])
									//////////

									double lastDeg = previousPositions[i]*360.0/txRate[i]-calibCONSTANTS[i]*360.0/txRate[i];

					 				//Operative virtual limit control
									if (setPoint[i]<(VT_LIM[i][0])){
										setPoint[i] = VT_LIM[i][0]; cout << " \n Reset Forzato- "<<i+1<<"\n";
									}

									if (setPoint[i]<=(VT_LIM[i][0]+1) && lastDeg<=(VT_LIM[i][0]+1) && lastDeg>setPoint[i]) {
										double proximity = abs(VT_LIM[i][0]-lastDeg);
										if (proximity <= 0) proximity=0;
										if (proximity >= 1) proximity=1;
										double corrective = pow((proximity/1),2);
										if (corrective>1) corrective=1;
										setPoint[i] = lastDeg - corrective*abs(setPoint[i]-lastDeg);
									}// END if (setPoint[i]<=(VT_LIM[i][0]+1) && lastDeg<=(VT_LIM[i][0]+1) && lastDeg>setPoint[i])
					

									if (setPoint[i]>(VT_LIM[i][1])) {setPoint[i] = VT_LIM[i][1]; cout << " \n Reset Forzato+ "<<i+1<<"\n";}
			
									if (setPoint[i]>=(VT_LIM[i][1]-1) && lastDeg>=(VT_LIM[i][1]-1) && lastDeg<setPoint[i]) {
										double proximity = abs(VT_LIM[i][1]-lastDeg);
										if (proximity <= 0) proximity=0;
										if (proximity >= 1) proximity=1;
										double corrective = pow((proximity/1),2);
										if (corrective>1) corrective=1;
										setPoint[i] = lastDeg + corrective*abs(setPoint[i]-lastDeg);
									}// END if (setPoint[i]>=(VT_LIM[i][1]-1) && lastDeg>=(VT_LIM[i][1]-1) && lastDeg<setPoint[i])
							
									//////////

									if(allowMotion) {
										 //FILTER 2nd order
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
										}// END if (gearDeg[i]<OP_LIM[i][0])

										if (gearDeg[i]>OP_LIM[i][1]) {
											//allowMotion=0;
											gearDeg[i]=OP_LIM[i][1];
											cout << " \n ALERT: FILTERED GEAR AXIS "<<i+1<<": OVER SUP OP.LIMIT.\n";
										}// END if (gearDeg[i]>OP_LIM[i][1])										

										GearRotations[i] = +txRate[i]*gearDeg[i] / 360.0;
									}// END if(allowMotion)
								}// END if (sinAxisEnabled[i])
							}// END for (int i = 0; i < 6; i++)

							//SENDING
							for (int i = 0; i < 6; i++){
								if (sinAxisEnabled[i]){
									if (c4gOpen.isInDriveOn(ARM)){
										actualPositions[i] = GearRotations[i];
										
										if (allowMotion){									  
											 deltaPositions[i] = actualPositions[i] - previousPositions[i];
											 previousPositions[i] = actualPositions[i];
										}// END if (allowMotion)
										else{ //stop
											 actualPositions[i] = initialPositions[i];
											 deltaPositions[i] = 0.0;
										}// END else
									}// END if (c4gOpen.isInDriveOn(ARM))
		
									//***INVIO DATI AL COMAU***//
									c4gOpen.setTargetPosition(ARM, i+1, actualPositions[i]);
									c4gOpen.setTargetVelocity(ARM, i+1, deltaPositions[i]);
								}// END if (sinAxisEnabled[i])
							}// END for (int i = 0; i < 6; i++)
						}// END if (mode == C4G_OPEN_MODE_5)
						
						if (!c4gOpen.send()) keepGoingOn = false;
						
						if (c4gOpen.errorOccurred()){
						  	cout << "Test error occurred: " << c4gOpen.getLastError() << endl;
							c4gOpen.resetError();
						}// END if (c4gOpen.errorOccurred())
					}// END if (mode == C4G_OPEN_DRIVING_ON || mode == C4G_OPEN_MODE_0 || mode == C4G_OPEN_MODE_5)
					else{
						cout << "Test error --> DRIVE OFF.\n\n";cout.flush();
						c4gOpen.setMode(ARM, C4G_OPEN_DRIVE_OFF);
						c4gOpen.send();
						keepGoingOn = false;
					}// END else
				}// END else
			}// END if (c4gOpen.receive())
			else{
			  cout << "Test error 2.\n\n";cout.flush();
			  keepGoingOn = false;
			}// END else
		}// END while (keepGoingOn)
	}// END if(c4gOpen.start())
	
	c4gOpen.stop();

	//////////////////////////////////c4gOpen END
	
	//cout << "TestMode5: cg4open ricevuto comando 405\n"; cout.flush();
	cout << "Test5: termine del server..\n\n"; cout.flush();

	cout << "Test terminated.\n\n"; cout.flush();
	
	return 0;
}// END main()



