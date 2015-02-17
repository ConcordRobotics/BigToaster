// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
#include "Commands/LinearSysRate.h"
#include "Support/cPIDController.h"
LinearSysRate::LinearSysRate(Subsystem* sysIn, LinearSystem* linSys, float rateIn) {
	// Use requires() here to declare subsystem dependencies
	Requires(sysIn);
	sys = linSys;
	rate = rateIn;
}

// Called just before this Command runs the first time
void LinearSysRate::Initialize() {
	sys->SetMode(cPIDController::RATE);
	// Reset the controller to target the current position
	sys->controller->Reset(sys->encoder->GetDistance());
	sys->SetSetpoint(rate);
}

// Called repeatedly when this Command is scheduled to run
void LinearSysRate::Execute() {
	sys->SetSetpoint(rate);
	sys->UpdateController();
	//ToDo Add limit checks in the Claw class
}

// Make this return true when this Command no longer needs to run execute()
bool LinearSysRate::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void LinearSysRate::End() {
	sys->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LinearSysRate::Interrupted() {
	sys->Stop();
}
