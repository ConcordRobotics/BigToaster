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

void cSpeedController::Set(float speed) {
	if (reversed) {
		sc->Set(-speed);
	} else {
		sc->Set(speed);
	}
}

float cSpeedController::Get(void) {
	float output = sc->Get();
	if (reversed) output = -output;
	return output;

}
void cSpeedController::PIDWrite(float speed) {
	Set(speed);
}

cSpeedController::~cSpeedController(void) {
	delete(sc);
}
