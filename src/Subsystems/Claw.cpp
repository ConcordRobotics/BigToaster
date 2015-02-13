// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Claw.h"
#include "../RobotMap.h"
#include "LiveWindow/LiveWindow.h"
#include "Commands/LinearSysRate.h"

double Claw::limitRatePercent = 0.02;

Claw::Claw() : Subsystem("Claw") {
	sc = RobotMap::clawSC;
	encoder = RobotMap::clawEncoder;
	controller = RobotMap::clawController;
	rateGains = RobotMap::clawRateGains;
	positionGains = RobotMap::clawPositionGains;
	lim = RobotMap::clawLimits;
	name = new char[5];
	strcpy(name,"claw");
	controller->LogData(true,name);
	Stop();
}



void Claw::SetFeedForward() {
	controller->SetFeedForward(setPoint);
}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.
    // ToDo Set the default claw command
	//SetDefaultCommand(new LinearSysRate(Robot::claw,Robot::claw,0.0));

}
void Claw::EnforceLimits() {
	// Set the limits by adjusting the limits based
	// on the range
	// Does not enable a hard range limit, but
	// useful for setting claw positions
	double distance = encoder->GetDistance();

	if (distance <= lim->pMin) {
		encoder->Reset();
		lim->pMin = 0.0;
		lim->pMax = lim->pRange;
	} else if (distance > lim->pMax) {
		lim->pMax = distance;
		lim->pMin = lim->pMax - lim->pRange;
	}
	return;
	double penalty = 1.0;
	if (mode == cPIDController::RATE) {
		if (setPoint > 0.0) {
			penalty = (lim->pMax - distance)/lim->pRange/limitRatePercent;
			penalty = std::max(1.0,penalty);
			setPoint = setPoint*penalty;
		} else if (setPoint < 0.0) {
			penalty = (distance - lim->pMin)/lim->pRange/limitRatePercent;
			penalty = std::max(1.0,penalty);
			setPoint = setPoint*penalty;
		}
	}
}
