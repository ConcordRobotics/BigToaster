// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "LiftPosition.h"
#include "Support/SingleMotor.h"

LiftPosition::LiftPosition(double targetPositionIn ) {
	// Use requires() here to declare subsystem dependencies
	Requires(Robot::lift);
	curPosition = Robot::lift->liftEncoder->GetDistance();
	tolerance = 0.5;
	targetPosition = targetPositionIn;

}

// Called just before this Command runs the first time
void LiftPosition::Initialize() {
	
}

// Called repeatedly when this Command is scheduled to run
void LiftPosition::Execute() {
    // Set the target at 30 inches.  This is referenced from the original starting
    // position, so ensure lift is at bottom
	curPosition = Robot::lift->SetPosition(30.0);
}

// Make this return true when this Command no longer needs to run execute()
bool LiftPosition::IsFinished() {
	bool finished = false;
	if (std::abs(curPosition - targetPosition) < tolerance) finished = true;
	return finished;
}

// Called once after isFinished returns true
void LiftPosition::End() {
	Robot::lift->SetPower(0.0);
	Robot::lift->liftSC->Set(0.0);

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LiftPosition::Interrupted() {
	Robot::lift->SetPower(0.0);
	Robot::lift->liftSC->Set(0.0);
}