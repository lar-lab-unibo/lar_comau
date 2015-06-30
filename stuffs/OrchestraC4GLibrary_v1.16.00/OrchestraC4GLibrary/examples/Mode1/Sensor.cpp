/*
	Sensor.cpp

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

AUTHOR("Sintesi S.C.p.A.");
VERSION("1.0");

INITIALIZE_BEGIN
	ALLOCATE_EXTRA(0, 1, C4gOpen);   // C4gOpen instance
	ALLOCATE_EXTRA(1, 1, float);     // Current limit
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
		
		// Get current limit of axis 6.
		EXTRA(1,0) = EXTRA(0,0).getCurrentLimit(1,6);
		
		do
		{
			EXTRA(0,0).receive();
			if (EXTRA(0,0).errorOccurred())
			{
				alarmCode = 1000 + EXTRA(0,0).getLastError();
				ALARM(alarmCode);
			}
	
			if (EXTRA(0,0).getMode(1) == C4G_OPEN_DRIVING_ON || EXTRA(0,0).getMode(1) == C4G_OPEN_MODE_0)
			{	
				EXTRA(0,0).send();
				if (EXTRA(0,0).errorOccurred())
				{
					alarmCode = 1000 + EXTRA(0,0).getLastError();
					ALARM(alarmCode);
				}
			}		
		}
		while (EXTRA(0,0).getMode(1) == C4G_OPEN_DRIVING_ON || EXTRA(0,0).getMode(1) == C4G_OPEN_MODE_0);
	
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
	if (EXTRA(0,0).getMode(1) == C4G_OPEN_EXIT) STOP_EXECUTION;
	
	// Write C4gOpen instance pointer to output #0.
	OUTPUT(0,0) = (double)(unsigned long)&EXTRA(0,0);

	// Signal on output #1 whether ARM 1 is driven on and in mode 1.
	if (EXTRA(0,0).isInDriveOn(1) && EXTRA(0,0).getMode(1) == C4G_OPEN_MODE_1) OUTPUT(1,0) = 1;
	else OUTPUT(1,0) = -1;

	// Write axis 6 actual position (in gear rotations) to output #2.
	OUTPUT(2,0) = EXTRA(0,0).getActualPosition(1,6);

	// Write axis 6 current limit (in ampere) to output #3.
	OUTPUT(3,0) = EXTRA(1,0);
STEP_END

FINALIZE_BEGIN
	EXTRA(0,0).stop(false);
	DEALLOCATE_EXTRA(0);
	DEALLOCATE_EXTRA(1);
FINALIZE_END
