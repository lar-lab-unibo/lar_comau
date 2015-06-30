/*
	Actuator.cpp

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
	ALLOCATE_EXTRA(0, 6, double);  // Gear ratios 
INITIALIZE_END

STEP_BEGIN
	C4gOpen *c4gOpen = (C4gOpen *)(unsigned long)INPUT(0,0);
	float targetPosition[6];
	float deltaPosition[6];

	if (STEP_NUMBER == 1)
	{
		// Compute gear ratios.
		for (unsigned int i = 0; i < 6; i++) EXTRA(0,i) = c4gOpen->getTxRate(1,i+1) / 360.0;
	}
	
	if (c4gOpen->getMode(1) == C4G_OPEN_MODE_5)
	{
		for (unsigned int i = 0; i < 6; i++)
		{
			// Compute target position in gear rotations.
			targetPosition[i] = EXTRA(0,i) * INPUT(1,i);
			// Compute target velocity in delta gear rotations.
			deltaPosition[i] = EXTRA(0,i) * INPUT(2,i);
			
			c4gOpen->setTargetPosition(1, i+1, targetPosition[i]);
			c4gOpen->setTargetVelocity(1, i+1, deltaPosition[i]);
		}
	}

	c4gOpen->send();

	if (c4gOpen->errorOccurred()) c4gOpen->resetError();
STEP_END

FINALIZE_BEGIN
	DEALLOCATE_EXTRA(0);
FINALIZE_END

