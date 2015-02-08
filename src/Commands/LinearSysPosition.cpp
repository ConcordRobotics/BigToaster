// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
#include "Commands/LinearSysPosition.h"

LinearSysPosition::LinearSysPosition(LinearSystem* sysIn, double positionIn) {
	// Use requires() here to declare subsystem dependencies
	Requires(sysIn);
	sys = sysIn;
	position = positionIn;
	sys->SetPositionMode();
	sys->SetSetpoint(position);
}

// Called just before this Command runs the first time
void LinearSysPosition::Initialize() {
	
}

// Called repeatedly when this Command is scheduled to run
void LinearSysPosition::Execute() {
	sys->SetSetpoint(position);
	sys->UpdateController();
	//ToDo Add limit checks in the Claw class
}

// Make this return true when this Command no longer needs to run execute()
bool LinearSysPosition::IsFinished() {
	return (sys->PositionError(position) < tolerance);
}

// Called once after isFinished returns true
void LinearSysPosition::End() {
	sys->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LinearSysPosition::Interrupted() {
	sys->Stop();
}
