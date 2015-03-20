#include "AutonomousCommandGroup.h"
#include "Commands/ClawGrip.h"
#include "Commands/LinearSysPosition.h"
#include "Commands/LinearHoldPosition.h"
#include "Commands/DriveForward.h"
#include "Commands/DriveToHeading.h"
#include "Commands/ClawRate.h"


#include "Robot.h"

AutonomousCommandGroup::AutonomousCommandGroup(int autoModeIn)
{
	autoMode = autoModeIn;
	if (autoMode == RobotMap::OFF) return;

	double angle;
	RobotMap::gyro->Reset();
	//Robot::driveMotors->SetDriveMode(cPIDController::DIRECT);
	//Grip the container
	//AddSequential(new ClawGrip(0.01));
	// Once it is gripped, continue to hold the grip
	AddParallel(new ClawRate(-1.0));
	// Lift the container
	AddSequential( new LinearSysPosition(Robot::lift, Robot::lift, 0.30, 0.10));
	//AddParallel(new LinearHoldPosition(Robot::lift, Robot::lift));
	angle = 90.0;
	AddSequential(new DriveToHeading(angle, 3.0));
	AddSequential(new DriveForward(0.6, 5.0, 0.5, angle));
    if (autoMode == RobotMap::LEFT) {
    	angle = 180.0;
    	AddSequential(new DriveToHeading(angle, 3.0));
    } else if (autoMode == RobotMap::RIGHT) {
    	angle = 0.0;
    	AddSequential(new DriveToHeading(angle, 3.0));
    } else if (autoMode == RobotMap::CENTER) {
    	angle = 90.0;
    }
	//AddSequential(new LinearSysPosition(Robot::lift, Robot::lift, 0.0, 0.15));
	//AddParallel(new ClawRate(1.0));
	//AddSequential(new DriveForward(0.6, -1.0, 0.1, angle));
	//AddSequential(new ClawRate(0.0));

}
