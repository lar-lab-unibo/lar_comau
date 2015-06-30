/*
	TestMode0Debug.cpp

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

#include <rtai_lxrt.h>

#include <C4gOpen.hpp>

#define	ARM		1
#define TICK_TIME 1000000
#define CPUMAP 0x1

using namespace std;

static RT_TASK *Main_Task;

const char disclaimer[] = "\n*********************************************************************\n"
						  "                           TestMode0Debug                              \n\n"
						  "   C4G Open Library " VERSION_NUMBER " - Copyright (C) 2007 Sintesi S.C.p.A.\n\n"
						  "  Developers:                                                          \n"
						  "           Sabino   COLONNA (2006-, s.colonna@sintesi-scpa.com)    \n"
						  "           Giovanni IACCA   (2006-, g.iacca@sintesi-scpa.com  )    \n"
						  "           Giovanni TOTARO  (2006-, g.totaro@sintesi-scpa.com )    \n"
						  "\n*********************************************************************\n";

int main(int argc, char *argv[])
{
  bool ret;
  int hard_timer_running;
  RTIME sampling_interval;
  
	if (getuid() != 0)
	{
		cerr << "\n" << disclaimer;
		cerr << "\nYou must be root (or use sudo) to run TestMode0Debug!\n\n";
		exit(1);
	}

	cout << disclaimer << "\n";	

	C4gOpen c4gOpen;

	cout << "\nTestMode0Debug started...\n"; cout.flush();
	cout << "\n    After C4G starts:"; cout.flush();
	cout << "\n       1. Drive On the robot"; cout.flush();
	cout << "\n       2. Set mode 10 (0 Debug) via PDL2\n"; cout.flush();
	cout << "\n    To stop the test:"; cout.flush();
	cout << "\n       1. Drive Off the robot"; cout.flush();
	cout << "\n       2. Set mode 504 via PDL2\n\n"; cout.flush();

	
// 	if (!(Main_Task = rt_task_init_schmod(nam2num("MNTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
// 	  cout << "CANNOT INIT MAIN TASK"<< endl;
// 	  exit(1);
// 	}
// 
// 	if ((hard_timer_running = rt_is_hard_timer_running())){
// 	  cout << "Skip hard real_timer setting..." << endl;
// 	  sampling_interval = nano2count(TICK_TIME);
// 	    
// 	} else {
// 	  cout << "Starting real time timer..." << endl;
// 	  rt_set_oneshot_mode();
// 	  start_rt_timer(0);
// 	}

	cout << "RT_TASK initialized, starting C4GOpen communication..." << endl;

// 	cout << "test rt_dev_socket..." << endl;
// 	
// 	c4gOpen.closeRealTimeSocket();
// 	
// 	if (c4gOpen.initRealTimeSocket()){
// 	  cout << "c4gOpen.initRealTimeSocket() return true" << endl;
// 	} else {
// 	  cout << "c4gOpen.initRealTimeSocket() return false"<< endl;
//   
// 	    if(c4gOpen.errorOccurred()){
// 	      cout << "error occurred: " << c4gOpen.getLastError() << endl;
// 	      c4gOpen.resetError();
// 	    } else cout << "start failed whitout error" << endl;
// 	    cout << "last error occurred: " << c4gOpen.getLastError() << endl;
// 	}
// 	
	  
	
	
	c4gOpen.resetError();

	if (c4gOpen.start(true))
	{
	  cout << "after c4gOpen.start().\n"; cout.flush();
		bool keepGoingOn = true;

		while (keepGoingOn)
		{
			if (c4gOpen.receive())
			{
			  //cout << "after c4gOpen.receive().\n"; cout.flush();
				long mode = c4gOpen.getMode(ARM);

			//	cout << "mode == " << mode << endl; cout.flush();

						  if(c4gOpen.errorOccurred()){
						      cout << "a. error occurred: " << c4gOpen.getLastError() << endl;
						      c4gOpen.resetError();						      		     
						    } 
				
				if (mode == C4G_OPEN_EXIT){
				  keepGoingOn = false;
				  cout << "mode == C4G_OPEN_EXIT.\n"; cout.flush();
				}
				else
				{
					if (mode == C4G_OPEN_DRIVING_ON || mode == C4G_OPEN_MODE_0 || mode == C4G_OPEN_MODE_0_DEBUG)
					{
					  //cout << "mode == C4G_OPEN_DRIVING_ON || mode == C4G_OPEN_MODE_0 || mode == C4G_OPEN_MODE_0_DEBUG.\n"; cout.flush();
						if (!c4gOpen.send()) {
						   if(c4gOpen.errorOccurred()){
						      cout << "b. error occurred: " << c4gOpen.getLastError() << endl;
						      c4gOpen.resetError();
						      keepGoingOn = false;			     
						    } else cout << "send failed whitout error" << endl;						  
						}
					//	cout << "Packet sent.\n"; cout.flush();
					}
					else
					{
						c4gOpen.setMode(ARM, C4G_OPEN_DRIVE_OFF);
						cout << "after c4gOpen.setMode().\n"; cout.flush();
						c4gOpen.send();
						cout << "after c4gOpen.send().\n"; cout.flush();
						//keepGoingOn = false;
					}
				}
			}
			else {
			   if(c4gOpen.errorOccurred()){
			    cout << "c. error occurred: " << c4gOpen.getLastError() << endl;
			    c4gOpen.resetError();
			    keepGoingOn = false;			     
			  } else cout << "receive failed whitout error" << endl;
			}
		}
	} else {
	  cout << "c4gOpen return false:" << ret << endl;
  
	    if(c4gOpen.errorOccurred()){
	      cout << "d. error occurred: " << c4gOpen.getLastError() << endl;
	      c4gOpen.resetError();
	    } else cout << "start failed whitout error" << endl;
	    cout << "last error occurred: " << c4gOpen.getLastError() << endl;
	}
	
	c4gOpen.stop();
	
	cout << "TestMode0Debug terminated.\n\n"; cout.flush();

	//stop_rt_timer();
	
	//rt_task_delete(Main_Task);
	
	return 0;
}

