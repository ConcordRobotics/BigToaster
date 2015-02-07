/*
 * singleMotor.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "cSpeedController.h"
cSpeedController::cSpeedController (SpeedController* scIn, bool reversedIn) {
	sc = scIn;
	reversed = reversedIn;
}

void cSpeedController::Set(float speed, uint8_t syncGroup = 0) {
	if (reversed) {
		sc->Set(-speed,syncGroup);
	} else {
		sc->Set(speed,syncGroup);
	}
}

void cSpeedController::PIDWrite(float speed) {
	Set(speed);
}

cSpeedController::~cSpeedController(void) {
	delete(sc);
}
