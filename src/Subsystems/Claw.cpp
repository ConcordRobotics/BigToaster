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


Claw::Claw() : LinearSystem(), Subsystem("Claw") {
	sc = RobotMap::clawSC;
	encoder = RobotMap::clawEncoder;
	positionController = RobotMap::clawPositionController;
	rateController = RobotMap::clawRateController;
	name = new char[5];
	strcpy(name,"Claw");
}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.

	SetDefaultCommand(new LinearSysRate(Robot::claw,Robot::claw,0.0));

}
void Claw::EnforceLimits() {
	// The claw can not physically go below zero, so reset the encoder if it reads negative
	if (encoder->GetDistance() < 0.0) encoder->Reset();

	// Don't do anything for rate, applying some outward or inward pressure is OK
}
