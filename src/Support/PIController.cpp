/*
 * drivePIDoutput.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "PIController.h"
#include <algorithm>

PIController::PIController (double pGainIn, double iGainIn, double timeFilterIn, double maxOutputIn, double initPosition){
	curPosition = initPosition;
	lastPosition = initPosition;
	lastRate = 0.0;
	curRate = 0.0;
	pGain = pGainIn;
	iGain = iGainIn;
	timeFilter = timeFilterIn;
	maxOutput = abs(maxOutputIn);
	intErr = 0.0;
	// Start the clock
	timer = new Timer();
	timer->Start();
	lastTime = curTime;
	curTime = timer->Get();
	controlOutput = 0.0;
	target = 0.0;
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
	curRate = alpha*lastRate + (1.0 - alpha)*curRate;
}

void PIController::SetTarget (float targetIn){
	target = targetIn;
}
void PIController::CalcOutput() {
	double error = target - curRate;
	// Should be no need to filter the error since we are filtering
	// the rate itself.
	intErr = intErr + error;
	// Include the target for the control since this is a rate controller
	// This is known as feed-forward - see http://en.wikipedia.org/wiki/Feed_forward_(control)
	// For basic PID method see: http://en.wikipedia.org/wiki/PID_controller
	controlOutput = target + pGain*error + iGain*intErr;
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
