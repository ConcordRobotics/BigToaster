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
	AddSequential( new LinearSysPosition(Robot::lift, Robot::lift, 0.50, 0.03));
	AddParallel(new LinearHoldPosition(Robot::lift, Robot::lift));
	AddSequential(new DriveForward(3.0, -3.0, 0.1));
	AddSequential(new DriveToHeading(90.0, 1.0));
	AddSequential(new DriveToHeading(180.0, 1.0));
	AddSequential(new DriveForward(8.0, 5.0, 0.1));
	AddSequential(new LinearSysPosition(Robot::lift, Robot::lift, 0.1, 0.03));
	AddSequential(new LinearSysPosition(Robot::claw, Robot::claw, 1.0, 0.05));
	AddSequential(new DriveForward(3.0, -2.0, 0.1));

}
