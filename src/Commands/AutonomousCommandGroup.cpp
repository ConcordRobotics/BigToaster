#include "AutonomousCommandGroup.h"
#include "Commands/ClawGrip.h"
#include "Commands/LinearSysPosition.h"
#include "Commands/LinearHoldPosition.h"
#include "Commands/DriveForward.h"
#include "Commands/DriveToHeading.h"

#include "Robot.h"

AutonomousCommandGroup::AutonomousCommandGroup()
{
	RobotMap::gyro->Reset();
	// Grip the container
	AddSequential(new ClawGrip(0.01));
	// Once it is gripped, continue to hold the grip
	AddParallel(new LinearSysPosition(Robot::claw, Robot::claw, -1.0, -1.0));
	// Lift the container
	AddSequential( new LinearSysPosition(Robot::lift, Robot::lift, 30.0, 3.0));
	AddParallel(new LinearHoldPosition(Robot::lift, Robot::lift));
	AddSequential(new DriveForward(5.0, -3.0, 0.1));
	AddSequential(new DriveToHeading(90.0, 1.0));
	AddSequential(new DriveToHeading(180.0, 1.0));
	AddSequential(new DriveForward(5.0, 10.0, 0.1));
	AddSequential(new LinearSysPosition(Robot::lift, Robot::lift, 10.0, 3.0));
	AddSequential(new LinearSysPosition(Robot::claw, Robot::claw, 1.0, 0.05));
	AddSequential(new DriveForward(5.0, -3.0, 0.1));

}
