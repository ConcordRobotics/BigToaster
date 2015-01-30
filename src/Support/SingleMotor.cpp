/*
 * singleMotor.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "SingleMotor.h"
#include "SmartDashboard/SmartDashboard.h"
#include <algorithm>

SingleMotor::SingleMotor (SpeedController* scIn, PIRateController* controllerIn, Encoder* encoderIn) {
	sc = scIn;
	controller = controllerIn;
	encoder = encoderIn;
	scReversed = false;
};

void SingleMotor::UpdateRate(void){
	controller->SetRate(encoder->GetRate());
}


void SingleMotor::SetPower() {
	double power = controller->controlOutput;
	if (std::abs(controller->target - 0.0)<0.001) power = 0.0;
	if (scReversed) {
		sc->Set(-power);
	} else {
		sc->Set(power);
	}
}

void SingleMotor::Reset(bool rateCont) {
	controller->ResetCont();
}

void SingleMotor::OutputToDashboard(std::string motorName) {
	std::string keyName;

	//keyName = motorName + "Cont";
	//controller->OutputToDashboard(keyName);
	keyName = motorName + "EncRate";
	SmartDashboard::PutNumber(keyName,double(encoder->GetRate()));
	keyName = motorName + "EncPosition";
	SmartDashboard::PutNumber(keyName,double(encoder->GetDistance()));
}
