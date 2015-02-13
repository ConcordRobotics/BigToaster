/*
 * PIDParams.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: forsytjr
 */
#include "ControllerLimits.h"

ControllerLimits::ControllerLimits (double pMinIn, double pMaxIn,
		double rMinIn, double rMaxIn, double oMinIn, double oMaxIn) {
	pMin = pMinIn;
	pMax = pMaxIn;
	rMin = rMinIn;
	rMax = rMaxIn;
	oMin = oMinIn;
	oMax = oMaxIn;
	pRange = pMax - pMin;
}

double ControllerLimits::ApplyRateLimits(double rate) {
	double val = rate;
	if (val > rMax) val = rMax;
	if (val < rMin) val = rMin;
	return val;
}

double ControllerLimits::ApplyPositionLimits(double pos) {
	double val = pos;
	if (val > pMax) val = pMax;
	if (val < pMin) val = pMin;
	return val;
}

double ControllerLimits::ApplyOutputLimits(double output) {
	double val = output;
	if (val > oMax) val = oMax;
	if (val < oMin) val = oMin;
	return val;
}
