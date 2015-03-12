// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "DriveForward.h"
#include <tgmath.h>
#include "Support/AdvancedJoystick.h"

DriveForward::DriveForward(float rateIn, float maxTwistIn, double disIn, double tolIn, double angleIn) {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveMotors);
	heading = RobotMap::gyro->GetAngle();
	distance= disIn;
	pGain = 1.0;
	tol = tolIn;
	maxTwist = maxTwistIn;
	// Flip sign on rate
	rate = std::abs(rateIn);
	angle = angleIn;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}

// Called just before this Command runs the first time
void DriveForward::Initialize() {
	// Tell the drive system to hold the current angle
	Robot::driveMotors->SetGyroMode(cPIDController::POSITION, maxTwist);
	Robot::driveMotors->SetDriveMode(cPIDController::RATE, rate);
	// Set the target angle to the current angle
	Robot::driveMotors->SetHeadingTarget(angle);
    for (int i = 0; i < 4; i++) {
    	wheelStart[i] = Robot::driveMotors->encoders[i]->GetDistance();
    }
    totDist = 0.0;
    /*
	for (int i = 0; i < 4; i++) {
		Robot::driveMotors->controllers[i]->Reset(Robot::driveMotors->encoders[i]->GetDistance());
	}

	*/
	//std::cout << "COM START " << "drive " << " Forward " << RobotMap::timer->Get() << "\n";
}

// Called repeatedly when this Command is scheduled to run
void DriveForward::Execute() {
    totDist = 0.0;
    for (int i = 0; i < 4; i++) {
    	totDist = totDist + RobotMap::distPerRev*
    			(Robot::driveMotors->encoders[i]->GetDistance() - wheelStart[i]);
    }
    totDist = totDist/4.0;
    // PID Control based on total distance, which is the average of the wheels
    double tRate = pGain*( distance - totDist);
    if (tRate > rate) tRate = rate;
    if (tRate < -rate) tRate = -rate;

	Robot::driveMotors->ArcadeDrive(0,tRate,0);

}

// Make this return true when this Command no longer needs to run execute()
bool DriveForward::IsFinished() {
    return (std::abs(totDist - distance) < tol);
}

// Called once after isFinished returns true
void DriveForward::End() {
	// Kill the motors
	Robot::driveMotors->Stop();
	//std::cout << "COM END " << "drive " << " Forward " << RobotMap::timer->Get() << "\n";
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveForward::Interrupted() {
	// Kill the motors
	Robot::driveMotors->Stop();
	//std::cout << "COM INT " << "drive " << " Forward " << RobotMap::timer->Get() << "\n";
}
