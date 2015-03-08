#include "AutonomousCommandGroup.h"
#include "Commands/ClawGrip.h"
#include "Commands/LinearSysPosition.h"
#include "Commands/LinearHoldPosition.h"
#include "Commands/DriveForward.h"
#include "Commands/DriveToHeading.h"
#include "Commands/ClawRate.h"


#include "Robot.h"

AutonomousCommandGroup::AutonomousCommandGroup()
{
	double heading = 0.0;
	RobotMap::gyro->Reset();
	// Grip the container
	//AddSequential(new ClawGrip(0.01));
	// Once it is gripped, continue to hold the grip
	AddParallel(new ClawRate(-1.0));
	// Lift the container
	AddSequential( new LinearSysPosition(Robot::lift, Robot::lift, 0.30, 0.10));
	//AddParallel(new LinearHoldPosition(Robot::lift, Robot::lift));
	AddParallel(new ClawRate(-0.2));
	heading = 90.0;
	AddSequential(new DriveToHeading(heading, 5.0));
	AddSequential(new DriveForward(1.0, 10.0, 0.5, heading));
	AddSequential(new DriveToHeading(0.0, 5.0));
	heading = 180.0;
	AddSequential(new DriveToHeading(heading, 1.0));
	AddSequential(new DriveForward(8.0, 5.0, 0.5, heading));
	AddSequential(new LinearSysPosition(Robot::lift, Robot::lift, 0.15, 0.10));
	AddParallel(new ClawRate(1.0));
	AddSequential(new DriveForward(0.6, -1.0, 0.5, heading));
	AddSequential(new ClawRate(0.0));

}
