// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "DriveToHeading.h"
#include <tgmath.h>
#include "Support/AdvancedJoystick.h"
DriveToHeading::DriveToHeading(double headingIn, double tolIn) {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveMotors);
	heading = headingIn;
	tol = tolIn;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

// Called just before this Command runs the first time
void DriveToHeading::Initialize() {
	// Tell the drive system to hold the current angle
	Robot::driveMotors->SetGyroMode(cPIDController::POSITION);
	// Set the target angle to the current angle
	RobotMap::gyroController->Reset(RobotMap::gyro->GetAngle());
	Robot::driveMotors->SetHeadingTarget(heading);
#ifdef OUTPUT
	std::cout << "COM START " << "drive " << " Heading " << RobotMap::timer->Get() << "\n";
#endif
}

// Called repeatedly when this Command is scheduled to run
void DriveToHeading::Execute() {

	Robot::driveMotors->ArcadeDrive(0,0,0);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveToHeading::IsFinished() {
	float delHeading = RobotMap::gyro->GetAngle() - heading;
	int nRevs = floor(delHeading/360.0);
	delHeading = delHeading - nRevs*360.0;
    return (delHeading < tol);
}

// Called once after isFinished returns true
void DriveToHeading::End() {
	// Kill the motors
	Robot::driveMotors->Stop();
	std::cout << "COM END " << "drive " << " Heading " << RobotMap::timer->Get() << "\n";
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveToHeading::Interrupted() {
	// Kill the motors
	Robot::driveMotors->Stop();
	std::cout << "COM END " << "drive " << " Heading " << RobotMap::timer->Get() << "\n";
}
