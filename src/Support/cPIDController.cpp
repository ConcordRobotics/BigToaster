/*
 * drivePIDoutput.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "cPIDController.h"
#include "SmartDashboard/SmartDashboard.h"
#include <iostream>
#include <sstream>
#include <iomanip>

double cPIDController::PIDSampleTime = 0.005;
double cPIDController::setPointAlpha = 0.75;

cPIDController::cPIDController (PIDParams* params, ControllerLimits* pLim, PIDSource* pSource, PIDOutput* pOutput) {
	pidParams = params;
	lim = pLim;
	pidSource = pSource;
	pidOutput = pOutput;
	// Initialize default values
	timer = new Timer();
	timer->Start();
	double tempTime = timer->Get();
	double tempSensor = pidSource->PIDGet();
	for (unsigned int k = 0; k < nsave; k++) {
		setPoint[k] = 0.0;
		time[k] = tempTime;
		sensVal[k] = tempSensor;
		dSensDt[k] = 0.0;
	}
	i = 0.0;
	iNM2 = 0; iNM1 = 1; iN = 2;
	logName = "";
	mode = OFF;
};

void cPIDController::SetPIDParams(PIDParams* params) {
	pidParams = params;
}

void cPIDController::SetRate(double rateIn) {
	rate = rateIn;
}

void cPIDController::SetSetpoint(double setIn) {
	// Set the setpoint for the next time
	double set = setIn;
	set = lim->ApplyPositionLimits(set);
	setPoint[(iN+1)%nsave ] = set;

}

void cPIDController::ApplyRate(double delT) {
	// Check the rate first
	rate = lim->ApplyRateLimits(rate);
	setPoint[iN] = setPoint[iNM1] + delT*rate;
}

void cPIDController::CheckLimits(double delT) {
	// Check the position limits first
	setPoint[iN] = lim->ApplyPositionLimits(setPoint[iN]);
	// Check rate
	double tRate = (setPoint[iN] - setPoint[iNM1])/delT;
	tRate = lim->ApplyRateLimits(tRate);
	setPoint[iN] = setPoint[iNM1] + delT*rate;
	setPoint[iN] = lim->ApplyPositionLimits(setPoint[iN]);
}

void cPIDController::SetFeedForward(double fIn) {
	f = (pidParams->fGain)*fIn;
}

double cPIDController::GetSetpoint() {
	return setPoint[iN];
}
double cPIDController::UpdateController(double curOutput) {
	double t = timer->Get();
	// Only iterate if we are hitting our sample rate.  Prevents noise
	// in the derivatives
	if (t - time[iN] < PIDSampleTime) return curOutput;

	// Advance the iNex, with mod to loop it
	iNM2 = iNM1;
	iNM1 = iN;
	iN = (iN+1)%nsave;

	// Get time step information
	time[iN] = t;
	double delT = time[iN] - time[iNM1];

	// Smooth the setpoint
	//setPoint[iN] = (1.0 - setPointAlpha)*setPoint[iN] + setPointAlpha*setPoint[im1];

	// Apply the rate to set the position
	if (mode == RATE) ApplyRate(delT);
	// Check both the position and rate limits
	CheckLimits(delT);

	// Get the sensor value
	sensVal[iN] = pidSource->PIDGet();
	// Set the setPoint to the sensor value for direct mode to keep
	// Controller information current.  I.e. assume it's going exactly where it's supposed to
	if ( (mode==DIRECT) or (mode==OFF) ) setPoint[iN] = sensVal[iN];
	// Calculate needed derivatives
	double error = setPoint[iN] - sensVal[iN];
	double dSetDt = (setPoint[iN] - setPoint[iNM1])/delT;
	dSensDt[iN] = (sensVal[iN] - sensVal[iNM1])/delT;
	// Second derivative of the sensor values
	double delTMid = 0.5*(time[iN] - time[iNM2]);
	double ddSensDtsq = (dSensDt[iN] - dSensDt[iNM1])/delTMid;
	// For derivative term, use sensor value rather than error to prevent jumps
	// Could filter this but better to filter at the sensor level

	// Calculate the target output
	p = pidParams->pGain*(dSetDt - dSensDt[iN]);
	i = pidParams->pGain*error/pidParams->iGain;
	// Since derivative term is based on error, which is
	// (target - sensor), need a negative sign in front of sensor
	// values.  Derivative of target neglected to prevent jumps due to setpoint
	// changes
	d = - (pidParams->pGain)*(pidParams->dGain)*ddSensDtsq;
	// Now set the output based on the mode
	if (mode == OFF) return 0.0;
	output = curOutput;
	if (mode == DIRECT) {
		// For direct mode, output set directly through feed-forward term;
		// For setting an avsolute value of power, set curPower to 0.0 when
		// Calling update controller
		output = output + f;
	} else {
		output = output + delT*(p + i + d);
	}
	// Check output limits
	output = lim->ApplyOutputLimits(output);

	pidOutput->PIDWrite(output);
	if (logData) {
		std::cout << "CONT " << logName << " " << time[iN] << " " << setPoint[iN] <<
				" " << sensVal[iN] << " " << output << " " <<  p
				<< " " << i << " " << d << " " << f << "\n";
	}
	// Reset the feed forward term
    f = 0.0;
    return output;
}

void cPIDController::SetMode(int modeIn) {
	mode = modeIn;
}


void cPIDController::OutputToDashboard(std::string controllerName) {
	std::ostringstream buffer;
	buffer << "s: " << std::fixed << std::setprecision(5) << setPoint[iN];
    buffer << " v: " << std::fixed << std::setprecision(5) << sensVal[iN];
    buffer << " tR: " << std::fixed << std::setprecision(5) << rate;
    buffer << " sR " << std::fixed << std::setprecision(4) << dSensDt[iN];
    buffer << " o: " << std::fixed << std::setprecision(4) << output;
    buffer << " p: " << std::fixed << std::setprecision(4) << p;
    buffer << " i: " << std::fixed << std::setprecision(4) << i;
    buffer << " d: " << std::fixed << std::setprecision(4) << d;
    buffer << " f: " << std::fixed << std::setprecision(4) << f;
	std::string value = buffer.str();
	SmartDashboard::PutString(controllerName, value);
}

void cPIDController::LogData(bool active, char* logNameIn) {
	logName = logNameIn;
	logData = active;
}
