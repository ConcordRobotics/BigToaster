// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "ClawRate.h"
#include "Subsystems/Claw.h"

ClawRate::ClawRate(float rate) {
	// Use requires() here to declare subsystem dependencies
	Requires(Robot::claw);
	Robot::claw->SetRateMode();
	Robot::claw->controller->SetSetpoint(rate);
}

// Called just before this Command runs the first time
void ClawRate::Initialize() {
	
}

// Called repeatedly when this Command is scheduled to run
void ClawRate::Execute() {
	Robot::claw->UpdateController();
	//ToDo Add limit checks in the Claw class
}

// Make this return true when this Command no longer needs to run execute()
bool ClawRate::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ClawRate::End() {
	Robot::claw->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClawRate::Interrupted() {
	Robot::claw->Stop();
}
