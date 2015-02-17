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
#include "RobotMap.h"

LinearSysPosition::LinearSysPosition(Subsystem* sysIn, LinearSystem* linSys, double percentIn, double tol) {
	// Use requires() here to declare subsystem dependencies
	Requires(sysIn);
	sys = linSys;
	percentRange = percentIn;
	position = percentRange*(sys->lim->pRange) + sys->lim->pMin;
	// default tolerance will mean the command will hold until another command is requested.
	tolerance = tol;
	timer = RobotMap::timer;
}


// Called just before this Command runs the first time
void LinearSysPosition::Initialize() {
	position = percentRange*(sys->lim->pRange) + sys->lim->pMin;
	sys->SetMode(cPIDController::POSITION);
	sys->controller->Reset(sys->encoder->GetDistance());
	sys->SetSetpoint(position);
	inPosition = false;
}

// Called repeatedly when this Command is scheduled to run
void LinearSysPosition::Execute() {
	position = percentRange*(sys->lim->pRange) + sys->lim->pMin;
	sys->SetSetpoint(position);
	sys->UpdateController();
	//ToDo Add limit checks in the Claw class
	bool inP = (sys->PositionError(position) < tolerance);
	if (inPosition) {
		// Has previously been flagged as being in position.
		if (!inP) inPosition = false;
	} else if (inP) {
		// It wasn't in position, but now it is, so start the timer
		inPosition = true;
		startTime = timer->Get();
	}
}

// Make this return true when this Command no longer needs to run execute()
bool LinearSysPosition::IsFinished() {
	// Require it to be in position for one second before it is flagged as done
	return (inPosition and (timer->Get() - startTime > 1.0));
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
