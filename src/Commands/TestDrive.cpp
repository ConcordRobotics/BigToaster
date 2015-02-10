// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "TestDrive.h"
#include "SmartDashboard/SmartDashboard.h"
#include "RobotMap.h"

TestDrive::TestDrive() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveMotors);
	timer = new Timer();
	timer->Start();
	rate = 0.333;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

// Called just before this Command runs the first time
void TestDrive::Initialize() {
	
}

// Called repeatedly when this Command is scheduled to run
void TestDrive::Execute() {
	float x,y,z;
	x = 0.0;
	y = rate;
	z = 0.0;
	Robot::driveMotors->ArcadeDrive(x,y,z);
}

// Make this return true when this Command no longer needs to run execute()
bool TestDrive::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void TestDrive::End() {
	// Add something to kill the motors
	Robot::driveMotors->ArcadeDrive(0.0,0.0,0.0);
	Robot::driveMotors->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestDrive::Interrupted() {
	Robot::driveMotors->ArcadeDrive(0.0,0.0,0.0);
	Robot::driveMotors->Stop();
}
