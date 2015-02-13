/*
 * PIDParams.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: forsytjr
 */
#include "PIDParams.h"

PIDParams::PIDParams (float p, float i, float d, float f) {
	pGain = p;
	iGain = i;
	// iGain shows up in the denominator, so check for small values
	if (iGain < 1.0E-6) iGain = 1.0E10;
	dGain = d;
	fGain = f;
}

