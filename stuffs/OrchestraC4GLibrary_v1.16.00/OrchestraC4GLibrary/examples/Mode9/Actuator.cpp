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

STEP_BEGIN
	C4gOpen *c4gOpen = (C4gOpen *)(unsigned long)INPUT(0,0);

	if (c4gOpen->getMode(1) == C4G_OPEN_MODE_9)
		c4gOpen->setDeltaCurrent(1, 1, INPUT(1,0));

	c4gOpen->send();

	if (c4gOpen->errorOccurred()) c4gOpen->resetError();
STEP_END
