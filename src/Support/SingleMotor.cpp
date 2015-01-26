/*
 * singleMotor.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "SingleMotor.h"
#include "SmartDashboard/SmartDashboard.h"
#include <algorithm>

SingleMotor::SingleMotor (SpeedController* scIn, PIController* controllerIn, Encoder* encoderIn) {
	sc = scIn;
	controller = controllerIn;
	encoder = encoderIn;
	scReversed = false;
};

void SingleMotor::UpdateRate(void){
	controller->SetRate(encoder->GetRate());
}

void SingleMotor::SetTargetPower(double power) {
	controller->SetTarget(power);
}

void SingleMotor::SetPower() {
	double power = controller->controlOutput;
	if (scReversed) {
		sc->Set(-power);
	} else {
		sc->Set(power);
	}
}

void SingleMotor::OutputToDashboard(std::string motorName) {
	std::string keyName;

	keyName = motorName + "/Cont/";
	controller->OutputToDashboard(keyName);
	keyName = motorName + "/Enc/rate";
	SmartDashboard::PutNumber(keyName,double(encoder->GetRate()));

}
