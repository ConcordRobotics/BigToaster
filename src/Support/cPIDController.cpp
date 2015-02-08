/*
 * drivePIDoutput.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "cPIDController.h"
#include "SmartDashboard/SmartDashboard.h"
cPIDController::cPIDController (float p, float i,  float d, float f, PIDSource* pSource, PIDOutput* pOutput) {
	pGain = p;
	iGain = i;
	if (iGain < 1.0E-6) iGain = 1.0E30;
	dGain = d;
	fGain = f;
	pidSource = pSource;
	pidOutput = pOutput;
	// Initialize default values
	timer = new Timer();
	timer->Start();
	double tempTime = timer->Get();
	double tempSensor = pidSource->PIDGet();
	for (unsigned int k = 0; k < nsave; k++) {
		setPoint[k] = 0.0;
		output[k] = 0.0;
		dodt[k] = 0.0;
		time[k] = tempTime;
		sensVal[k] = tempSensor;
	}
	intErr = 0.0;
	enabled = false;
	ind = 0;
	rangeOutOverIn = 1.0;
	// ToDo add ability to set ranges, and scale above accordingly
};

void cPIDController::CalcRangeRatio (void) {
	float rOut, rIn;
	rOut = outRange[1] - outRange[0];
	rIn = inRange[1] - inRange[0];
	if (rIn > 0) {
		rangeOutOverIn = rOut/rIn;
	} else rangeOutOverIn = 1.0;
}

void cPIDController::SetInputRange (float iMin, float iMax) {
	inRange[0] = iMin;
	inRange[1] = iMax;
	CalcRangeRatio();
}

void cPIDController::SetOutputRange (float oMin, float oMax) {
	inRange[0] = oMin;
	inRange[1] = oMax;
	CalcRangeRatio();
}

void cPIDController::SetSetpoint(double set) {
	// Set the setpoint for the next time
	// ToDo add checks on input levels
	// Filter the setpoint
	setPoint[(ind+1)%nsave ] = set;
}

void cPIDController::UpdateController() {
	if (enabled == false) return;
	// Advance the index, with mod to loop it
	unsigned int im1 = ind;
	ind = (ind+1)%nsave;
	// Get time step information
	time[ind] = timer->Get();
	double delT = time[ind] - time[im1];
	double rDelT = 0.0;
	if (delT > 0.0) rDelT = 1.0/delT;
	// Get the sensor value
	sensVal[ind] = pidSource->PIDGet();
	double error = setPoint[ind] - sensVal[ind];
	// For derivative, use sensor value rather than error to prevent jumps
	// Could filter this but better to filter at the sensor level
	dodt[ind] = rDelT*(sensVal[ind] - sensVal[im1]);
	// Integrate the error
	intErr = error*delT;

	// Calculate the target output
	double tempOut;
	tempOut = rangeOutOverIn* // Scale everything by range of out over in
			 (fGain*setPoint[ind] +  // Feed forward term
					 pGain*( error + // All feedback terms scaled by the pGain
							 intErr/iGain + // iGain has units of time,
							 dGain*dodt[ind] ));//dGain has units of time
	// Now check the output
	if (tempOut > outRange[1]) {
		if (error > 0 ) intErr = intErr - error*delT;
		tempOut = outRange[1];
	} else if (tempOut < outRange[0]) {
		if (error < 0) {
			if (error < 0) intErr = intErr - error*delT;
		}
	}

	// Apply the output
	output[ind] = tempOut;
	if (enabled) pidOutput->PIDWrite(output[ind]);
	if (logData) {
		fprintf(cLogFile, "%f %f %f\n",time[ind], sensVal[ind], output[ind]);
		//logFile << time[ind] << " " << sensVal[ind] << " " << output[ind] << "\n";
	}

}

void cPIDController::Reset() {
	intErr = 0.0;
}

void cPIDController::Enable() {
	enabled = true;
	intErr = 0.0;
}

void cPIDController::Disable() {
	enabled = false;
	intErr = 0.0;
}

void cPIDController::OutputToDashboard(std::string controllerName) {
	std::string keyName;
	keyName = controllerName + "pGain";
	SmartDashboard::PutNumber(keyName,pGain);
	keyName = controllerName + "iGain";
	SmartDashboard::PutNumber(keyName,1.0/iGain);
	keyName = controllerName + "dGain";
	SmartDashboard::PutNumber(keyName,dGain);
	keyName = controllerName + "fGain";
	SmartDashboard::PutNumber(keyName,fGain);
	keyName = controllerName + "ind";
	SmartDashboard::PutNumber(keyName,double(ind));
	keyName = controllerName + "sens";
	SmartDashboard::PutNumber(keyName,double(sensVal[ind]));
	keyName = controllerName + "output";
	SmartDashboard::PutNumber(keyName,double(output[ind]));
	keyName = controllerName + "target";
	SmartDashboard::PutNumber(keyName,double(setPoint[ind]));
}

void cPIDController::LogData(bool active, char* fileName) {
	char* fname = new char[strlen(fileName) + 2];
	strcpy(fname,"/");
	strcat(fname,fileName);
	if (active) {
		cLogFile = fopen(fname,"w");
	} else if (logData) {
		fclose(cLogFile);
	}
	logData = active;
	delete(fname);
}
