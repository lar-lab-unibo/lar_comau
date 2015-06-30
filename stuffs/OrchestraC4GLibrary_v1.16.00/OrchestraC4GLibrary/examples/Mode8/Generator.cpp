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
	// Check whether ARM 1 is driven on and in mode 8.
	if (INPUT(0,0) > 0)
	{
		double omegat = 2 * PI * PARAM("frequency",0) * PERIOD * (double)EXTRA(0,0);
		
		// FFW Velocity.
		OUTPUT(0,0) = (float)(PARAM("speedCoeff",0) * sin(omegat));
		
		EXTRA(0,0)++;
	}
	else EXTRA(0,0) = 0;
STEP_END

FINALIZE_BEGIN
	DEALLOCATE_EXTRA(0);
FINALIZE_END
