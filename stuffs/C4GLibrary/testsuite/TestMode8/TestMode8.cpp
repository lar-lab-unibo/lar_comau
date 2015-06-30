/*
	TestMode8.cpp

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
#define AXIS	1

#define SPEED_COEFF	0.0005

using namespace std;

const char disclaimer[] = "\n*********************************************************************\n"
						  "                              TestMode8                                \n\n"
						  "   C4G Open Library " VERSION_NUMBER " - Copyright (C) 2007 Sintesi S.C.p.A.\n\n"
						  "  Developers:                                                          \n"
						  "           Sabino   COLONNA   (2006-, s.colonna@sintesi-scpa.com  )    \n"
						  "           Giovanni IACCA     (2006-, g.iacca@sintesi-scpa.com    )    \n"
						  "           Giovanni TOTARO    (2006-, g.totaro@sintesi-scpa.com   )    \n"
						  "\n*********************************************************************\n";

int main(int argc, char *argv[])
{
	if (getuid() != 0)
	{
		cerr << "\n" << disclaimer;
		cerr << "\nYou must be root (or use sudo) to run TestMode8!\n\n";
		exit(1);
	}

	cout << disclaimer << "\n";	

	C4gOpen c4gOpen;

	cout << "\nTestMode8 started...\n"; cout.flush();
	cout << "\n    After C4G starts:"; cout.flush();
	cout << "\n       1. Drive On the robot"; cout.flush();
	cout << "\n       2. Set mode 8 via PDL2 (ONLY for axis 1)\n"; cout.flush();
	cout << "\n    To stop the test:"; cout.flush();
	cout << "\n       1. Drive Off the robot"; cout.flush();
	cout << "\n       2. Set mode 504 via PDL2\n\n"; cout.flush();

	if (c4gOpen.start())
	{
		bool keepGoingOn = true;

		double sampleTime = (double)c4gOpen.getSampleTime() / 1000.0;
		double frequencyHz = 1.0;

		float ffwVelocity = 0.0;
		int counter = 0;
		
		while (keepGoingOn)
		{
			if (c4gOpen.receive())
			{
				long mode = c4gOpen.getMode(ARM);

				if (mode == C4G_OPEN_EXIT) keepGoingOn = false;
				else
				{
					if (mode == C4G_OPEN_DRIVING_ON || mode == C4G_OPEN_MODE_0 || mode == C4G_OPEN_MODE_8)
					{
						if (mode == C4G_OPEN_MODE_8)
						{
							if (c4gOpen.isInDriveOn(ARM))
							{
								double omegat = 2 * PI * frequencyHz * ((double)counter * sampleTime);
								ffwVelocity = (float)(SPEED_COEFF * sin(omegat));
								counter++;
							}

							c4gOpen.setFeedForwardVelocity(ARM, AXIS, ffwVelocity);
						}

						if (!c4gOpen.send()) keepGoingOn = false;

						if (c4gOpen.errorOccurred())
						{
							c4gOpen.resetError();
							counter = 0;
						}
					}
					else
					{
						c4gOpen.setMode(ARM, C4G_OPEN_DRIVE_OFF);
						c4gOpen.send();
						keepGoingOn = false;
					}
				}
			}
			else keepGoingOn = false;
		}
	}
	
	c4gOpen.stop();
	
	cout << "TestMode8 terminated.\n\n"; cout.flush();

	return 0;
}

