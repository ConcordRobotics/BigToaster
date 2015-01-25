/*
 * drivePIDoutput.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "PIController.h"
#include <algorithm>
#include "SmartDashboard/SmartDashboard.h"
PIController::PIController (double pGainIn, double iGainIn, double timeFilterIn, double maxOutputIn,  double maxRateIn, double initPosition){
	maxRate = maxRateIn;
	curPosition = initPosition;
	lastPosition = initPosition;
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
	lastTime = curTime;
	curTime = timer->Get();
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
	alpha = 1.0;
	curRate = (1.0-alpha)*lastRate + alpha*curRate;
}

void PIController::SetTarget (double targetIn){
	target = targetIn;
	// Scale the target based on maxRates
	if (rateController) {
		target = target/maxOutput*maxRate;
	}
}
void PIController::CalcOutput() {
	double error;
	if (rateController) {
		error = target - curRate;
	} else {
		error = target - curPosition;
	}
	// Should be no need to filter the error since we are filtering
	// the rate itself.
	intErr = intErr + error;
	// Include the target for the control since this is a rate controller
	// This is known as feed-forward - see http://en.wikipedia.org/wiki/Feed_forward_(control)
	// For basic PID method see: http://en.wikipedia.org/wiki/PID_controller
	if (rateController) {
		controlOutput = maxOutput*target/maxRate + pGain*error + iGain*intErr;
	} else {
		controlOutput =  pGain*error + iGain*intErr;
	}
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
