/*
	C4GActuatorMode8.cpp

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

STEP_BEGIN
	// Get C4gOpen instance pointer from Sensor.
	C4gOpen *c4gOpen = (C4gOpen *)(unsigned long)INPUT(0,0);

	if (c4gOpen->getMode(ARM) == C4G_OPEN_MODE_8)
	{
		for (unsigned int i = 0; i < NUM_OF_AXES; i++)
			c4gOpen->setFeedForwardVelocity(ARM, i+1, INPUT(1,i));
	}

	c4gOpen->send();

	if (c4gOpen->errorOccurred()) c4gOpen->resetError();
STEP_END
