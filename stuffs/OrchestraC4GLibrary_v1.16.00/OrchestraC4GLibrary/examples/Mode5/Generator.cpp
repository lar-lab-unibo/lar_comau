/*
	Generator.cpp

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
#include <math.h>

#define PI 3.14159265358979

AUTHOR("Sintesi S.C.p.A.");
VERSION("1.0");

INITIALIZE_BEGIN
	ALLOCATE_EXTRA(0, 1, int);	// Internal counter
	EXTRA(0,0) = 0;
INITIALIZE_END

STEP_BEGIN
	float actualPositions[6];  // Actual positions (in deg).

	// Check whether ARM 1 is driven on and in mode 5.
	if (INPUT(0,0) > 0)
	{
		if (EXTRA(0,0) == 0) // First time ready to move.
		{
			for (unsigned int i = 0; i < 6; i++)
			{
				if (PARAM("enabledAxes",i) > 0)
				{
					STATUS(0,i) = INPUT(1,i);   // Store initial position (in deg).
					STATUS(1,i) = STATUS(0,i);  // Store for computing the first velocity value.
				}
			}
		}

		for (unsigned int i = 0; i < 6; i++)
		{
			if (PARAM("enabledAxes",i) > 0)
			{
				double omegat = 2 * PI * PARAM("frequency",0) * PERIOD * (double)EXTRA(0,0);
				
				// Compute target position.
				actualPositions[i] = (float)(STATUS(0,i) + PARAM("amplitude",0) * cos(omegat) - PARAM("amplitude",0));
				OUTPUT(0,i) = actualPositions[i];
				
				// Compute target velocity.
				OUTPUT(1,i) = actualPositions[i] - STATUS(1,i);
				STATUS(1,i) = actualPositions[i];
			}
			else
			{
				OUTPUT(0,i) = INPUT(1,i);
				OUTPUT(1,i) = 0.0;
			}
		}
		EXTRA(0,0)++;
	}
	else EXTRA(0,0) = 0;
STEP_END

FINALIZE_BEGIN
	DEALLOCATE_EXTRA(0);
FINALIZE_END
