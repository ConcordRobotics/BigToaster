/*
 * drivePIDoutput.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "PIPositionController.h"
#include "SmartDashboard/SmartDashboard.h"

PIPositionController::PIPositionController (double pGainIn, double iGainIn, double maxOutputIn, Encoder* encoderIn) {
	pGain = pGainIn;
	iGain = iGainIn;
	maxOutput = maxOutputIn;
	encoder = encoderIn;
	intErr = 0.0;
	// Start the clock
	timer = new Timer();
	timer->Start();
	curTime = timer->Get();
	lastTime = curTime;
	controlOutput = 0.0;
	curPosition = encoder->GetDistance();
	lastPosition = curPosition;
	target = 0.0;
};

void PIPositionController::UpdateController(){
	lastPosition = curPosition;
	curPosition = encoder->GetDistance();
	lastTime = curTime;
	curTime = timer->Get();
	double delT =  (curTime - lastTime);
	CalcOutput(delT);
}



void PIPositionController::SetTarget (double targetIn){
	target = targetIn;
}

void PIPositionController::CalcOutput(double delT) {
	double error;
	error = target - curPosition;
	intErr = intErr + delT*error;

	controlOutput = pGain*error + iGain*intErr;

	if ( controlOutput > maxOutput ) {
		controlOutput = maxOutput;
		if (error > 0) intErr = intErr - delT*error;
	} else if ( controlOutput < -maxOutput) {
		controlOutput = -maxOutput;
		if (error < 0) intErr = intErr - delT*error;
	}
}

void PIPositionController::Reset() {
	// Reset the controller
	intErr = 0.0;
	controlOutput = 0.0;
}

void PIPositionController::OutputToDashboard(std::string controllerName) {
	std::string keyName;
	keyName = controllerName + "pGain";
	// Get inputs
	double input;
	input = SmartDashboard::GetNumber(keyName,pGain);
	pGain = input;
	SmartDashboard::PutNumber(keyName,pGain);
	keyName = controllerName + "iGain";
	input = SmartDashboard::GetNumber(keyName,iGain);
	iGain = input;
	SmartDashboard::PutNumber(keyName,iGain);
	keyName = controllerName + "output";
	SmartDashboard::PutNumber(keyName,double(controlOutput));
	//keyName = controllerName + "curRate";
	//SmartDashboard::PutNumber(keyName,curRate);
	keyName = controllerName + "target";
	SmartDashboard::PutNumber(keyName,double(target));
}
