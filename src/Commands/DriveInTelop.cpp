// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "DriveInTelop.h"
#include <tgmath.h>
#include "Support/AdvancedJoystick.h"
DriveInTelop::DriveInTelop(int mode) {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveMotors);
	gyroMode = mode;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

// Called just before this Command runs the first time
void DriveInTelop::Initialize() {
	Robot::driveMotors->SetGyroMode(gyroMode);
	RobotMap::gyroController->Reset(RobotMap::gyro->GetAngle());
}

// Called repeatedly when this Command is scheduled to run
void DriveInTelop::Execute() {
	AdvancedJoystick* stick = Robot::oi->getJoystick1();
	float x = stick->aGetX();
	// Flip sign on y axis since forward is negative for the joystick
	float y = -stick->aGetY();
    float z = stick->aGetTwist();
	Robot::driveMotors->ArcadeDrive(x,y,z);

}

// Make this return true when this Command no longer needs to run execute()
bool DriveInTelop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveInTelop::End() {
	// Kill the motors
	Robot::driveMotors->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveInTelop::Interrupted() {
	// Kill the motors
	Robot::driveMotors->Stop();
}
