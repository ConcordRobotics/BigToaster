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
	controller = RobotMap::clawPIDController;
	mode = OFF;
	Stop();
	// ToDo: Enable PID controller on claw, once tuned.
	// Need to find the zero position
}

void Claw::SetPositionMode() {
	mode = POSITION;
	// Reset the controller to zero the integral error.
	controller->Reset();
	// Change the PID source parameter to position
	encoder->SetPIDSourceParameter(PIDSource::kDistance);
	// Adjust gains here
	float* p;
	p = RobotMap::clawPositionGains;
	controller->SetPID(p[0], p[1], p[2], p[3]);
	// Set the target to the current position to be safe
	controller->SetSetpoint(encoder->GetDistance());
	controller->Reset();
	controller->Enable();
}

void Claw::SetRateMode() {
	mode = RATE;
	// Reset the controller to zero the integral error.
	controller->Reset();
	// Change the PID source parameter to position
	encoder->SetPIDSourceParameter(Encoder::kRate);
	// Adjust gains here
	float* p;
	p = RobotMap::clawRateGains;
	controller->SetPID(p[0], p[1], p[2], p[3]);
	// Set the target to a rate of zero to be safe
	controller->SetSetpoint(0.0);
	controller->Enable();
}

void Claw::UpdateController() {
	// Enforce a lower limit of zero
	//if (encoder->GetDistance() < 0) encoder->Reset();
	// ToDo: May need to add code to prevent
	// buildup of integration error in the controller when
	// holding things.
	sc->Set(controller->Get());
}
void Claw::Stop() {
	mode = OFF;
	controller->Disable();
	sc->Set(0.0);

}

void Claw::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	//setDefaultCommand(new ClawInTelop());
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}
