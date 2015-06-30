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
	STATUS(0,0) = 1;
INITIALIZE_END

STEP_BEGIN
	// Compute test current.
	if (STEP_NUMBER == 1)
	{
		STATUS(1,0) = INPUT(2,0) * PARAM("currentCoeff",0);
	}

	// Check whether ARM 1 is driven on and in mode 1.
	if (INPUT(0,0) > 0)
	{
		if (!(STEP_NUMBER % 250))
		{
			if (STATUS(0,0)) STATUS(0,0) = 0;
			else STATUS(0,0) = 1;
		}
		
		OUTPUT(0,0) = INPUT(1,0);  // Target position.
		OUTPUT(1,0) = 0.0;         // Target velocity.
		OUTPUT(2,0) = 0.0;         // FFW velocity.
		
		// FFW current.
		if (STATUS(0,0)) OUTPUT(3,0) = STATUS(1,0);
		else OUTPUT(3,0) = -STATUS(1,0);
	}
STEP_END
