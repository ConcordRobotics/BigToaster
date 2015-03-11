// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/Lift.h"
#include "../RobotMap.h"
#include "Robot.h"
#include "LiveWindow/LiveWindow.h"
#include "Commands/LinearSysRate.h"
#include "Commands/LinearHoldPosition.h"

// This will limit the rate if it gets to
// within the set percent of the range
// of the lift
double Lift::limitRatePercent = 0.10;

Lift::Lift() : Subsystem("Lift") {
	sc = RobotMap::liftSC;
	encoder = RobotMap::liftEncoder;
	controller = RobotMap::liftController;
	lim = RobotMap::liftLimits;
	positionGains = RobotMap::liftPositionGains;
	rateGains = RobotMap::liftRateGains;
	upperSwitch = RobotMap::liftUpperSwitch;
	lowerSwitch = RobotMap::liftLowerSwitch;
	name = new char[5];
	strcpy(name,"lift");
	SmartDashboard::PutNumber("LiftUpper",double(upperSwitch->Get()));
	SmartDashboard::PutNumber("LiftLower",double(lowerSwitch->Get()));
	mode = cPIDController::OFF;
	controller->LogData(true,name);
	Stop();
}



void Lift::SetFeedForward ( ) {
	// Set a constant to help support the weight of the lift
	controller->SetFeedForward(setPoint);
}

void Lift::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// ToDo Setup a default command for Lift, perhaps hold position
	//SetDefaultCommand(new LinearSysRate(Robot::lift,Robot::lift,0.0));
	SetDefaultCommand(new LinearHoldPosition(Robot::lift, Robot::lift));
}

void Lift::UpdateController() {
		EnforceLimits();
		LinearSystem::UpdateController();
}

void Lift::EnforceLimits() {
	// Add something for the limit switch
	// Don't reset distance to zero since the lift can unwind past zero
	int atBottom = lowerSwitch->Get();
	int atTop = upperSwitch->Get();
	
	SmartDashboard::PutNumber("LiftUpper",double(upperSwitch->Get()));
	SmartDashboard::PutNumber("LiftLower",double(lowerSwitch->Get()));
	// Reset the encoder if it has hit the bottom
// To Do Re-enable Reset once limit switches verified.
	/*
	*double distance = encoder->GetDistance();
	if ( (atBottom == 1) or (distance < 0.0)) {
		encoder->Reset();
		lim->pMin = 0.0;
		lim->pMax = lim->pRange - 1.0;
	}
//

	if (atTop) {
		// If at the top update the limits
		lim->pMax = distance;
		lim->pMin = lim->pMax - lim->pRange;
	}
 */
	//Create some soft limits for the rate controller - slow it as it approaches the top limit
/* ToDo enable soft limits once encoder verified, and positions of switches
	if (mode == cPIDController::RATE) {
		double penalty = 1.0;
		if (setPoint > 0.0) {
			// Ramp the rate down as you get closer
			penalty = (lim->pMax - distance)/lim->pRange/limitRatePercent;
			penalty = std::max(1.0,penalty);
			if (penalty < 0.1) penalty = 0.1;
		} else if (setPoint < 0.0) {
			penalty = (distance - lim->pMin)/lim->pRange/limitRatePercent;
			penalty = std::max(1.0,penalty);
			if(penalty < 0.1) penalty = 0.1;

		}
		setPoint = setPoint*penalty;
	}
	*/
	// Don't do anything for position - we shouldn't be commanding a negative position.
}
