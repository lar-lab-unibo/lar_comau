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

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <C4gOpen.hpp>

#define PI 		3.14159265358979
#define	ARM		1

using namespace std;

const char disclaimer[] = "\n*********************************************************************\n"
						  "                              TestMode5                                \n\n"
						  "   C4G Open Library " VERSION_NUMBER " - Copyright (C) 2007 Sintesi S.C.p.A.\n\n"
						  "  Developers:                                                          \n"
						  "           Sabino   COLONNA (2006-, s.colonna@sintesi-scpa.com)    \n"
						  "           Giovanni IACCA   (2006-, g.iacca@sintesi-scpa.com  )    \n"
						  "           Giovanni TOTARO  (2006-, g.totaro@sintesi-scpa.com )    \n"
						  "\n*********************************************************************\n";

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
	double amplitudeDeg;
	double amplitudeGearRotations[6];
		
	double sampleTime;
	
	cout << disclaimer << "\n";	

	if (argc != 4)
	{
		cout << "Usage: " << argv[0] << " freqHz amplitudeDeg [1][2][3][4][5][6]\n\n";
		cout << "Example: " << argv[0] << " 0.25 5 135\n";
		cout << "         means a sin contribute of 5 degree at 0.25 Hz for axes 1,3 and 5.\n\n";
		
		exit(1);
	}

	// Parse command line parameters
	frequencyHz = atof(argv[1]);
	amplitudeDeg = atof(argv[2]);
	for (int i = 0; i < 6; i++) sinAxisEnabled[i] = false;
	
	for (unsigned int i = 0; i < strlen(argv[3]); i++)
	{
		if (argv[3][i] >= '1' && argv[3][i] <= '6') sinAxisEnabled[(argv[3][i]-'0')-1] = true;
	}

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

		for (int i = 0; i < 6; i++)	
		{
			if (sinAxisEnabled[i])
			{
				txRate[i] = c4gOpen.getTxRate(ARM, i+1);
				amplitudeGearRotations[i] = txRate[i] * amplitudeDeg / 360.0;
			}
		}

		bool keepGoingOn = true;
		int counter = 0;					
		bool isPosOK;
		bool isVelOK;
		
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
										actualPositions[i] = initialPositions[i] + (float)(amplitudeGearRotations[i] * cos(omegat) - amplitudeGearRotations[i]);
										
										deltaPositions[i] = actualPositions[i] - previousPositions[i];
										
										previousPositions[i] = actualPositions[i];
									}

									c4gOpen.setTargetPosition(ARM, i+1, actualPositions[i]);
									c4gOpen.setTargetVelocity(ARM, i+1, deltaPositions[i]);
									//cout << "c4gopen: pos:" << isPosOK << " vel:" << isVelOK << endl;
								}
							}
							if (c4gOpen.isInDriveOn(ARM)) counter++;
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

	cout << "TestMode5 terminated.\n\n"; cout.flush();
	
	return 0;
}

