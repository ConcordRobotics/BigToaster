// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.



#include <Subsystems/LinearSystem.h>
#include "../RobotMap.h"
#include "LiveWindow/LiveWindow.h"

LinearSystem::LinearSystem() {
	sc = NULL;
	encoder = NULL;
	positionController = NULL;
	rateController = NULL;
	mode = OFF;
}

LinearSystem::LinearSystem(SpeedController* scIn, Encoder* encIn,
		cPIDController* pController, cPIDController* rController)  {

	sc = scIn;
	encoder = encIn;
	positionController = pController;
	rateController = rController;
	mode = OFF;
	// ToDo: Enable PID controller on claw, once tuned.
	// Need to find the zero position
}

void LinearSystem::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	//setDefaultCommand(new ClawInTelop());
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void LinearSystem::SetPositionMode() {
	mode = POSITION;
	// Reset the controller to zero the integral error.
	// Change the PID source parameter to position
	positionController->Reset();
	// Reset the controller
	// Set the target to the current position to be safe
	positionController->SetSetpoint(encoder->GetDistance());
	positionController->Enable();
	rateController->Disable();
}

void LinearSystem::SetRateMode() {
	mode = RATE;
	// Reset the controller to zero the integral error.
	rateController->Reset();
	// Set the target zero
	rateController->SetSetpoint(0.0);
	rateController->Enable();
	positionController->Disable();
}

void LinearSystem::UpdateController() {

	EnforceLimits();
	if ( mode == RATE) {
		rateController->UpdateController();
		rateController->OutputToDashboard(name);
	} else if (mode == POSITION) {
		positionController->UpdateController();
		positionController->OutputToDashboard(name);
	} else sc->Set(0.0);
	Wait(0.005); // wait 5ms to avoid hogging CPU cycles

}

void LinearSystem::SetLimits(double min, double max) {
	limits[0] = min;
	limits[1] = max;
	range = max - min;
}

double LinearSystem::PositionError(double target) {
	return (std::abs(target - encoder->GetDistance() - distanceOffset)/range);
}

void LinearSystem::SetSetpoint(double setPointIn) {

	if ( mode == RATE) {
		rateController->SetSetpoint(setPointIn);
	} else if (mode == POSITION) {
		positionController->SetSetpoint(setPointIn);
	}
	setPoint = setPointIn;
}

void LinearSystem::Stop() {
	mode = OFF;
	sc->Set(0.0);
}


