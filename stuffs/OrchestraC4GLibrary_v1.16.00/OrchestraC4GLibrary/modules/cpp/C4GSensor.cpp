/*
	C4GSensor.cpp

	Copyright (C) 2007-2008 Sintesi S.C.p.A.

	Use and distribution of this technology
	is subject to Orchestra Community License
	(see file license.txt).

	Developers:
		Sabino   COLONNA (2006-, s.colonna@sintesi-scpa.com)
		Giovanni IACCA   (2006-, g.iacca@sintesi-scpa.com  )
		Giovanni TOTARO  (2006-, g.totaro@sintesi-scpa.com )
*/

#include <DLCInterface.hpp>
#include <C4gOpen.hpp>

#define ARM			1
#define NUM_OF_AXES 6

AUTHOR("Sintesi S.C.p.A.");
VERSION("1.16.00");

INITIALIZE_BEGIN
	ALLOCATE_EXTRA(0, 1, C4gOpen);   // C4gOpen instance 
INITIALIZE_END

STEP_BEGIN
	int alarmCode;

	if (STEP_NUMBER == 1)
	{
		// Initialize C4gOpen instance without creating an RTAI task.
		if (!(EXTRA(0,0).start(false))) 
		{ 
			alarmCode = 1000 + EXTRA(0,0).getLastError();
			ALARM(alarmCode);
		}
		
		do
		{
			EXTRA(0,0).receive();
			if (EXTRA(0,0).errorOccurred())
			{
				alarmCode = 1000 + EXTRA(0,0).getLastError();
				ALARM(alarmCode);
			}
	
			if (EXTRA(0,0).getMode(ARM) == C4G_OPEN_DRIVING_ON || EXTRA(0,0).getMode(1) == C4G_OPEN_MODE_0)
			{	
				EXTRA(0,0).send();
				if (EXTRA(0,0).errorOccurred())
				{
					alarmCode = 1000 + EXTRA(0,0).getLastError();
					ALARM(alarmCode);
				}
			}		
		}
		while (EXTRA(0,0).getMode(ARM) == C4G_OPEN_DRIVING_ON || EXTRA(0,0).getMode(ARM) == C4G_OPEN_MODE_0);
	
		if (!EXTRA(0,0).errorOccurred()) EXTERNAL_SYNCHRO_DONE;
		else
		{
			alarmCode = 1000 + EXTRA(0,0).getLastError();
			ALARM(alarmCode);
		}
	}
	
	// Receive a packet from C4G.
	if (STEP_NUMBER > 1)	
	{	
		if (EXTRA(0,0).receive()) EXTERNAL_SYNCHRO_DONE;
		else
		{
			alarmCode = 1000 + EXTRA(0,0).getLastError();
			ALARM(alarmCode);
		}
	}
	if (EXTRA(0,0).getMode(ARM) == C4G_OPEN_EXIT) STOP_EXECUTION;
	
	// Write C4gOpen instance pointer to output #0.
	OUTPUT(0,0) = (double)(unsigned long)&EXTRA(0,0);
	
	for (unsigned int i = 0; i < NUM_OF_AXES; i++)
	{
		OUTPUT(1,i) = EXTRA(0,0).getTargetPosition(ARM, i+1);  // Write target position to output #1.
		OUTPUT(2,i) = EXTRA(0,0).getTargetVelocity(ARM, i+1);  // Write target velocity to output #2.
		OUTPUT(3,i) = EXTRA(0,0).getActualPosition(ARM, i+1);  // Write actual position to output #3.
		OUTPUT(4,i) = EXTRA(0,0).getActualVelocity(ARM, i+1);  // Write actual velocity to output #4.
		OUTPUT(5,i) = EXTRA(0,0).getTargetCurrent(ARM, i+1);   // Write target current to output #5.
		OUTPUT(6,i) = EXTRA(0,0).getDynamicModel(ARM, i+1);    // Write dynamic model to output #6.
		OUTPUT(7,i) = EXTRA(0,0).getDiagonalInertia(ARM, i+1); // Write diagonal inertia to output #7.
		
	}
STEP_END

FINALIZE_BEGIN
	EXTRA(0,0).stop(false);
	DEALLOCATE_EXTRA(0);
FINALIZE_END
