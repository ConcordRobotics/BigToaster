/*
 * drivePIDoutput.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "PIController.h"
#include <algorithm>
#include "SmartDashboard/SmartDashboard.h"
PIController::PIController (double pGainIn, double iGainIn, double timeFilterIn, double maxOutputIn,
		double maxRateIn, double controlSlopeIn, double initPosition) {
	maxRate = maxRateIn;
	curPosition = initPosition;
	lastPosition = initPosition;
	controlSlope = controlSlopeIn;
	lastRate = 0.0;
	curRate = 0.0;
	pGain = pGainIn;
	iGain = iGainIn;
	timeFilter = timeFilterIn;
	maxOutput = maxOutputIn;
	intErr = 0.0;
	// Start the clock
	timer = new Timer();
	timer->Start();
	curTime = timer->Get();
	lastTime = curTime;
	controlOutput = 0.0;
	target = 0.0;
	rateController = true;
};

void PIController::SetPosition(double position){
	lastPosition = curPosition;
	curPosition = position;
	lastTime = curTime;
	curTime = timer->Get();
	lastRate = curRate;
	double delT =  (curTime - lastTime);
	curRate = (curPosition - lastPosition)/delT;
	FilterRate(delT);
	CalcOutput();
}

void PIController::SetRate(double rate) {
	lastRate = curRate;
	curRate = rate;
	lastTime = curTime;
	curTime = timer->Get();
	double delT =  (curTime - lastTime);
	FilterRate(delT);
	CalcOutput();

}

void PIController::FilterRate(double delT) {
	// Perform EMA averaging, using a time filter
	// See: http://lorien.ncl.ac.uk/ming/filter/fillpass.htm
	double alpha = delT/(delT+timeFilter);
	// ToDo Try using the filter, or averaging the encoders
	// Tried it and it seemed to cause oscillations, so disable
	// by setting weight to 1.0
	alpha = 1.0;
	curRate = (1.0-alpha)*lastRate + alpha*curRate;
}

void PIController::SetTarget (double targetIn){
	target = targetIn;
	// Scale the rate target based on expected slope of rate/power
	//if (rateController) {
		target = target*controlSlope;
	//}
}
void PIController::CalcOutput() {
	double error;
	//if (rateController) {
		error = target - curRate;
	//} else {
		//error = target - curPosition;
	//}
	// Should be no need to filter the error since we are filtering
	// the rate itself.
	intErr = intErr + error;
	// Include the target for the control since this is a rate controller
	// This is known as feed-forward - see http://en.wikipedia.org/wiki/Feed_forward_(control)
	// For basic PID method see: http://en.wikipedia.org/wiki/PID_controller
	//if (rateController) {
		controlOutput = target/controlSlope + pGain*error + iGain*intErr;
	//} else {
	//	controlOutput =  pGain*error + iGain*intErr;
	//}
	// Check to see if controls are maxed out and clip the output
	// Also, don't accumulate error that would drive the output further past
	// the max
	if ( controlOutput > maxOutput ) {
		controlOutput = maxOutput;
		if (error > 0) intErr = intErr - error;
	} else if ( controlOutput < -maxOutput) {
		controlOutput = -maxOutput;
		if (error < 0) intErr = intErr - error;
	}
}

void PIController::ResetCont(bool rateCont, double position, double rate) {
	// Reset the controller and enable switching to/from rate control vs. position
	rateController = rateCont;
	intErr = 0.0;
	controlOutput = 0.0;
	curPosition = position;
	lastPosition = position;
	curRate = rate;
	lastRate = rate;
}

void PIController::OutputToDashboard(std::string controllerName) {
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
