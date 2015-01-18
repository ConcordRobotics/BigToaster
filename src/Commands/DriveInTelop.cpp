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

float xGain = 1.0;
float yGain = 1.0;
float zGain = 0.5;
float zDeadBand = 0.0;

DriveInTelop::DriveInTelop() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveMotors);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

// Called just before this Command runs the first time
void DriveInTelop::Initialize() {
	
}

// Called repeatedly when this Command is scheduled to run
void DriveInTelop::Execute() {
	Joystick* stick = Robot::oi->getJoystick1();
	float x = stick->GetX();
	float y = stick->GetY();
    float z = stick->GetTwist();
    if (z > zDeadBand) {
    	z = (z - zDeadBand)/(1.0 - zDeadBand);
    } else if (z < -zDeadBand) {
    	z = (z + zDeadBand)/(1.0 - zDeadBand);
    } else {
    	z = 0;
    }
    x = exp(xGain*abs(x))/exp(xGain)*x;
    y = exp(yGain*abs(y))/exp(yGain)*y;
    z = exp(zGain*abs(z))/exp(zGain)*z;

	Robot::driveMotors->arcadeDrive(x,y,z);

}

// Make this return true when this Command no longer needs to run execute()
bool DriveInTelop::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveInTelop::End() {
	// Add something to kill the motors
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveInTelop::Interrupted() {

}
