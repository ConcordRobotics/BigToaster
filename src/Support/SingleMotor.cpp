/*
 * singleMotor.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "SingleMotor.h"

#include <algorithm>

SingleMotor::SingleMotor (SpeedController* scIn, PIController* controllerIn, Encoder* encoderIn) {
	sc = scIn;
	controller = controllerIn;
	encoder = encoderIn;
	PIControlled = true;
	scReversed = false;
    maxEnc = 1.0;
	maxOutput = 1.0;
	power = 0.0;
};

void SingleMotor::UpdateController(void){
	if (PIControlled) controller->SetRate((encoder->GetRate())/maxEnc);
}

void SingleMotor::SetTarget(float powerIn) {
	power = std::min(powerIn,maxOutput);
	power = std::max(power,-maxOutput);
	if (PIControlled) {
		controller->SetTarget(power);
	}
}

void SingleMotor::SetPower() {
	if (PIControlled) {
		power = controller->controlOutput;
	}
	if (scReversed) {
		sc->Set(-power);
	} else {
		sc->Set(power);
	}
}
