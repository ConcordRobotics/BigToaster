/*
 * drivePIDoutput.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "PIRateController.h"
#include "SmartDashboard/SmartDashboard.h"
PIRateController::PIRateController (double pGainIn, double iGainIn, double maxOutputIn,
		double maxRateIn, double controlSlopeIn) {
	maxRate = maxRateIn;
	controlSlope = controlSlopeIn;
	curRate = 0.0;
	pGain = pGainIn;
	iGain = iGainIn;
	maxOutput = maxOutputIn;
	intErr = 0.0;
	// Start the clock
	timer = new Timer();
	timer->Start();
	curTime = timer->Get();
	lastTime = curTime;
	controlOutput = 0.0;
	target = 0.0;
};

void PIRateController::SetRate(double rate) {
	curRate = rate;
	lastTime = curTime;
	curTime = timer->Get();
	double delT =  (curTime - lastTime);
	CalcOutput(delT);

}


void PIRateController::SetTarget (double targetIn){
	target = targetIn;
	target = target*controlSlope;
}
void PIRateController::CalcOutput(double delT) {
	double error;

	error = target - curRate;
	intErr = intErr + delT*error;

	// Include the target for the control since this is a rate controller
	// This is known as feed-forward - see http://en.wikipedia.org/wiki/Feed_forward_(control)
	// For basic PID method see: http://en.wikipedia.org/wiki/PID_controller
	controlOutput = target/controlSlope + pGain*error + iGain*intErr;

	// Check to see if controls are maxed out and clip the output
	// Also, don't accumulate error that would drive the output further past
	// the max
	if ( controlOutput > maxOutput ) {
		controlOutput = maxOutput;
		if (error > 0) intErr = intErr - delT*error;
	} else if ( controlOutput < -maxOutput) {
		controlOutput = -maxOutput;
		if (error < 0) intErr = intErr - delT*error;
	}
}

void PIRateController::ResetCont() {
	// Reset the controller and enable switching to/from rate control vs. position
	intErr = 0.0;
	controlOutput = 0.0;
	curRate = 0.0;
}

void PIRateController::OutputToDashboard(std::string controllerName) {
	std::string keyName;
	keyName = controllerName + "pGain";
	// Get inputs
	//double input;
	//input = SmartDashboard::GetNumber(keyName,pGain);
	//pGain = input;
	SmartDashboard::PutNumber(keyName,pGain);
	keyName = controllerName + "iGain";
	//input = SmartDashboard::GetNumber(keyName,iGain);
	//iGain = input;
	SmartDashboard::PutNumber(keyName,iGain);
	keyName = controllerName + "output";
	SmartDashboard::PutNumber(keyName,double(controlOutput));
	//keyName = controllerName + "curRate";
	//SmartDashboard::PutNumber(keyName,curRate);
	keyName = controllerName + "target";
	SmartDashboard::PutNumber(keyName,double(target));
}
