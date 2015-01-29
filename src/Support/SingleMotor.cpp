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

void SingleMotor::UpdatePosition(void){
	controller->SetPosition(encoder->GetDistance());
}
void SingleMotor::SetPower() {
	double power = controller->controlOutput;
	if (controller->target == 0) power = 0.0;
	if (scReversed) {
		sc->Set(-power);
	} else {
		sc->Set(power);
	}
}

void SingleMotor::Reset(bool rateCont) {
	double position = encoder->GetDistance();
	double rate = encoder->GetRate();
	controller->ResetCont(rateCont, position, rate);
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
