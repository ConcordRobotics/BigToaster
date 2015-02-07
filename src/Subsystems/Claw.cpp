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
#include "SmartDashboard/SmartDashboard.h"
#include "LiveWindow/LiveWindow.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PID
Claw::Claw() : Subsystem("Claw") {

	sc = RobotMap::clawSC;
	encoder = RobotMap::clawEncoder;
	controller = RobotMap::clawPositionController;
	mode = OFF;
	// ToDo: Enable PID controller on claw, once tuned.
	// Need to find the zero position
}

void Claw::SetPositionMode() {
	mode = POSITION;
	// Reset the controller to zero the integral error.
	// Change the PID source parameter to position
	controller=RobotMap::clawPositionController;
	// Reset the controller
	// Set the target to the current position to be safe
	controller->SetSetpoint(encoder->GetDistance());
	controller->Enable();
}

void Claw::SetRateMode() {
	mode = RATE;
	// Reset the controller to zero the integral error.
	// Change the PID source parameter to position
	controller=RobotMap::clawRateController;
	// Reset the controller
	// Set the target zero
	controller->SetSetpoint(0.0);
	controller->Enable();
}

void Claw::UpdateController() {
	// Enforce a lower limit of zero
	if (encoder->GetDistance() < 0) encoder->Reset();
	if ( mode <> OFF) {
		controller->UpdateController();
	}
}

void Claw::Stop() {
	mode = OFF;
	controller->Reset();
	sc->Set(0.0);

}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	//setDefaultCommand(new ClawInTelop());
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}
